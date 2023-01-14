/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-05-12 12:49:56
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-05-24 09:39:17
 * @FilePath: \Kiri2D\demos\dem2d_scene1\src\uni_dem_example.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#include <uni_dem_example.h>
#include <kiri2d_pbs_utils.h>

namespace KIRI2D
{

    void UniDEM2DExample::init()
    {
        mWorldSize = Vector2F(2.f);
        mRenderOffset = Vector2F(WINDOW_WIDTH - mWorldSize.x * PARTICLES_RENDER_SCALE, WINDOW_HEIGHT - mWorldSize.y * PARTICLES_RENDER_SCALE) / 2.f;

        mScene = std::make_shared<KiriScene2D>((auto)WINDOW_WIDTH, (auto)WINDOW_HEIGHT);
        mRenderer = std::make_shared<KiriRenderer2D>(mScene);
    }

    void UniDEM2DExample::setupParams()
    {
        KIRI_LOG_DEBUG("DEM: setupParams");

        // app
        CUDA_DEM_APP_PARAMS.run = true;
        CUDA_DEM_APP_PARAMS.max_num = 400;
        CUDA_DEM_APP_PARAMS.bgeo_export = false;
        strcpy(CUDA_DEM_APP_PARAMS.bgeo_export_folder, (String(EXPORT_PATH) + "bgeo/dem").c_str());

        // mScene config
        auto cuda_lowest_point = make_float2(0.f);
        auto cuda_highest_point = make_float2(mWorldSize.x, mWorldSize.y);
        auto cuda_world_size = cuda_highest_point - cuda_lowest_point;
        auto cuda_world_center = (cuda_highest_point + cuda_lowest_point) / 2.f;

        // dem params
        CUDA_DEM_PARAMS.rest_density = 1700.f;
        CUDA_DEM_PARAMS.particle_radius = 0.01f;
        CUDA_DEM_PARAMS.rest_mass = CUDA_DEM_PARAMS.rest_density * std::powf(CUDA_DEM_PARAMS.particle_radius * 2.f, 2.f);

        CUDA_DEM_PARAMS.kernel_radius = 4.f * CUDA_DEM_PARAMS.particle_radius;

        CUDA_DEM_PARAMS.young = 1e8f;
        CUDA_DEM_PARAMS.poisson = 0.3f;
        CUDA_DEM_PARAMS.tan_friction_angle = 0.5f;
        CUDA_DEM_PARAMS.c0 = 0.f;

        CUDA_DEM_PARAMS.gravity = make_float2(0.0f, -9.8f);

        CUDA_DEM_PARAMS.damping = 0.4f;
        CUDA_DEM_PARAMS.dt = 1e-4f;

        // mScene data
        CUDA_BOUNDARY_PARAMS.lowest_point = cuda_lowest_point;
        CUDA_BOUNDARY_PARAMS.highest_point = cuda_highest_point;
        CUDA_BOUNDARY_PARAMS.world_size = cuda_world_size;
        CUDA_BOUNDARY_PARAMS.world_center = cuda_world_center;

        CUDA_BOUNDARY_PARAMS.kernel_radius = CUDA_DEM_PARAMS.kernel_radius;
        CUDA_BOUNDARY_PARAMS.grid_size = make_int2((CUDA_BOUNDARY_PARAMS.highest_point - CUDA_BOUNDARY_PARAMS.lowest_point) / CUDA_BOUNDARY_PARAMS.kernel_radius);

        // boundary sampling
        BoundaryData boundary_data;
        auto boundary_emitter = std::make_shared<CudaBoundaryEmitter>();

        boundary_emitter->BuildWorldBoundary(boundary_data, CUDA_BOUNDARY_PARAMS.lowest_point, CUDA_BOUNDARY_PARAMS.highest_point, CUDA_DEM_PARAMS.particle_radius);
        // draw boundary
        mBoundaryRect = KiriRect2(mRenderOffset - Vector2F(CUDA_DEM_PARAMS.particle_radius) * PARTICLES_RENDER_SCALE, (mWorldSize + Vector2F(CUDA_DEM_PARAMS.particle_radius)) * PARTICLES_RENDER_SCALE);

        // volume sampling
        auto volume_emitter = std::make_shared<CudaVolumeEmitter>();
        MRDemVolumeData volume_data;
        int2 box_size = make_int2(50, 50);
        volume_emitter->BuildUniDemVolume(
            volume_data,
            CUDA_BOUNDARY_PARAMS.world_center - make_float2(box_size.x * CUDA_DEM_PARAMS.particle_radius, box_size.y * CUDA_DEM_PARAMS.particle_radius),
            box_size,
            CUDA_DEM_PARAMS.particle_radius,
            make_float3(0.88f, 0.79552f, 0.5984f),
            CUDA_DEM_PARAMS.rest_mass,
            0.001f * CUDA_DEM_PARAMS.particle_radius);

        KIRI_LOG_DEBUG("particles size={0}", volume_data.pos.size());

        // spatial searcher & particles
        CudaDemParticlesPtr particles =
            std::make_shared<CudaMRDemParticles>(
                volume_data.pos,
                volume_data.col,
                volume_data.radius,
                volume_data.mass);

        CudaGNSearcherPtr searcher;
        searcher = std::make_shared<CudaGNSearcher>(
            CUDA_BOUNDARY_PARAMS.lowest_point,
            CUDA_BOUNDARY_PARAMS.highest_point,
            particles->MaxSize(),
            CUDA_BOUNDARY_PARAMS.kernel_radius,
            SearcherParticleType::MRDEM);

        auto boundary_particles = std::make_shared<CudaBoundaryParticles>(boundary_data.pos, boundary_data.label);
        KIRI_LOG_INFO("Number of Boundary Particles = {0}", boundary_particles->Size());

        CudaGNBoundarySearcherPtr boundary_searcher = std::make_shared<CudaGNBoundarySearcher>(
            CUDA_BOUNDARY_PARAMS.lowest_point,
            CUDA_BOUNDARY_PARAMS.highest_point,
            boundary_particles->MaxSize(),
            CUDA_BOUNDARY_PARAMS.kernel_radius);

        CudaDemSolverPtr solver = std::make_shared<CudaDemSolver>(particles->Size());

        mSystem = std::make_shared<CudaDemSystem>(
            particles,
            boundary_particles,
            solver,
            searcher,
            boundary_searcher);
    }

    void UniDEM2DExample::update()
    {
        if (CUDA_DEM_APP_PARAMS.run)
        {
            auto substep_num = mSystem->GetNumOfSubTimeSteps();

            mPerFrameTimer.restart();
            for (auto i = 0; i < substep_num; i++)
                mSystem->UpdateSystem(mRenderInterval);

            // KIRI_LOG_INFO("Time Per Frame={0}", mPerFrameTimer.elapsed());
            mTotalFrameTime += mPerFrameTimer.elapsed();

            mSystem->UpdateSystem(mRenderInterval);
            auto particles = std::dynamic_pointer_cast<CudaMRDemParticles>(mSystem->GetParticles());
            auto particles_num = particles->Size();
            auto float_bytes = particles_num * sizeof(float);
            auto float2_bytes = particles_num * sizeof(float2);
            float2 *particle_positions = (float2 *)malloc(float2_bytes);
            float *particle_radius = (float *)malloc(float_bytes);
            cudaMemcpy(particle_positions, particles->GetPosPtr(), float2_bytes, cudaMemcpyDeviceToHost);
            cudaMemcpy(particle_radius, particles->GetRadiusPtr(), float_bytes, cudaMemcpyDeviceToHost);

            std::vector<KiriCircle2> render_particles;
            for (auto i = 0; i < particles_num; i++)
            {
                auto p = KiriCircle2(Vector2F(particle_positions[i].x, particle_positions[i].y) * PARTICLES_RENDER_SCALE + mRenderOffset, Vector3F(0.88f, 0.79552f, 0.5984f), particle_radius[i] * PARTICLES_RENDER_SCALE);
                render_particles.emplace_back(p);
            }

            renderScene(render_particles);
        }
        else if (CUDA_DEM_APP_PARAMS.run)
        {
            CUDA_DEM_APP_PARAMS.run = false;
        }
    }

    void UniDEM2DExample::renderScene(std::vector<KiriCircle2> particles)
    {
        mScene->AddCircles(particles);
        mScene->addRect(mBoundaryRect);

        mRenderer->DrawCanvas();
        mRenderer->SaveImages2File();

        cv::imshow("DEM", mRenderer->canvas());
        cv::waitKey(5);

        mRenderer->ClearCanvas();
        mScene->clear();
    }
}