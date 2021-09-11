/*** 
 * @Author: Xu.WANG
 * @Date: 2021-09-03 09:27:54
 * @LastEditTime: 2021-09-11 21:58:34
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dExamples\src\main_dem.cpp
 */

#include <kiri_utils.h>

using namespace KIRI;

// global params
const UInt RunLiquidNumber = 0;
const UInt TotalFrameNumber = 300;
UInt SimCount = 0;
float TotalFrameTime = 0.f;
float RenderInterval = 1.f / 30.f;

KiriTimer PerFrameTimer;
CudaDemSystemPtr DEMSystem;

float radius = 0.015f;

void SetupParams()
{
    KIRI_LOG_DEBUG("DEM: SetupParams");

    // app
    CUDA_DEM_APP_PARAMS.max_num = 400;
    strcpy(CUDA_DEM_APP_PARAMS.bgeo_export_folder, (String(EXPORT_PATH) + "bgeo/dem").c_str());

    // scene config
    auto cuda_lowest_point = make_float2(0.f);
    auto cuda_highest_point = make_float2(1.f, 1.f);
    auto cuda_world_size = cuda_highest_point - cuda_lowest_point;
    auto cuda_world_center = (cuda_highest_point + cuda_lowest_point) / 2.f;

    // dem params
    CUDA_DEM_PARAMS.rest_density = 2000.f;
    CUDA_DEM_PARAMS.particle_radius = radius;
    CUDA_DEM_PARAMS.rest_mass = CUDA_DEM_PARAMS.rest_density * std::powf(CUDA_DEM_PARAMS.particle_radius * 2.f, 2.f);

    CUDA_DEM_PARAMS.kernel_radius = 4.f * CUDA_DEM_PARAMS.particle_radius;

    CUDA_DEM_PARAMS.young = 1e9f;
    CUDA_DEM_PARAMS.poisson = 0.3f;
    CUDA_DEM_PARAMS.tan_friction_angle = 0.5f;

    CUDA_DEM_PARAMS.gravity = make_float2(0.0f, -9.8f);

    CUDA_DEM_PARAMS.damping = 0.4f;
    //CUDA_DEM_PARAMS.dt = 0.5f * CUDA_DEM_PARAMS.particle_radius / std::sqrtf(CUDA_DEM_PARAMS.young / CUDA_DEM_PARAMS.rest_density);
    CUDA_DEM_PARAMS.dt = 1e-4f;

    // scene data
    CUDA_BOUNDARY_PARAMS.lowest_point = cuda_lowest_point;
    CUDA_BOUNDARY_PARAMS.highest_point = cuda_highest_point;
    CUDA_BOUNDARY_PARAMS.world_size = cuda_world_size;
    CUDA_BOUNDARY_PARAMS.world_center = cuda_world_center;

    CUDA_BOUNDARY_PARAMS.kernel_radius = CUDA_DEM_PARAMS.kernel_radius;
    CUDA_BOUNDARY_PARAMS.grid_size = make_int2((CUDA_BOUNDARY_PARAMS.highest_point - CUDA_BOUNDARY_PARAMS.lowest_point) / CUDA_BOUNDARY_PARAMS.kernel_radius);

    // boundary sampling
    BoundaryData boundaryData;
    auto boundaryEmitter = std::make_shared<CudaBoundaryEmitter>();

    boundaryEmitter->BuildWorldBoundary(boundaryData, CUDA_BOUNDARY_PARAMS.lowest_point, CUDA_BOUNDARY_PARAMS.highest_point, CUDA_DEM_PARAMS.particle_radius);

    int2 vbox = make_int2(20, 20);
    // volume sampling
    DemVolumeData volumeData;
    auto volumeEmitter = std::make_shared<CudaVolumeEmitter>();
    volumeEmitter->BuildUniDemVolume(
        volumeData,
        CUDA_BOUNDARY_PARAMS.world_center - make_float2(vbox.x * CUDA_DEM_PARAMS.particle_radius, vbox.y * CUDA_DEM_PARAMS.particle_radius),
        vbox,
        CUDA_DEM_PARAMS.particle_radius,
        make_float3(0.88f, 0.79552f, 0.5984f),
        CUDA_DEM_PARAMS.rest_mass,
        0.001f * CUDA_DEM_PARAMS.particle_radius);

    // spatial searcher & particles
    CudaDemParticlesPtr particles =
        std::make_shared<CudaDemParticles>(
            volumeData.pos,
            volumeData.col,
            volumeData.mass,
            volumeData.id);

    CudaGNSearcherPtr searcher;
    searcher = std::make_shared<CudaGNSearcher>(
        CUDA_BOUNDARY_PARAMS.lowest_point,
        CUDA_BOUNDARY_PARAMS.highest_point,
        particles->MaxSize(),
        CUDA_BOUNDARY_PARAMS.kernel_radius,
        SearcherParticleType::DEM);

    auto boundary_particles = std::make_shared<CudaBoundaryParticles>(boundaryData.pos, boundaryData.label);
    KIRI_LOG_INFO("Number of Boundary Particles = {0}", boundary_particles->Size());
    // ExportBgeoFileCUDA(
    //     CUDA_DEM_APP_PARAMS.bgeo_export_folder,
    //     "boundary",
    //     boundary_particles->GetPosPtr(),
    //     make_float3(0.f, 0.f, 1.f),
    //     radius,
    //     0,
    //     boundary_particles->Size());

    CudaGNBoundarySearcherPtr boundary_searcher = std::make_shared<CudaGNBoundarySearcher>(
        CUDA_BOUNDARY_PARAMS.lowest_point,
        CUDA_BOUNDARY_PARAMS.highest_point,
        boundary_particles->MaxSize(),
        CUDA_BOUNDARY_PARAMS.kernel_radius);

    CudaDemSolverPtr pSolver = std::make_shared<CudaDemSolver>(particles->Size());

    DEMSystem = std::make_shared<CudaDemSystem>(
        particles,
        boundary_particles,
        pSolver,
        searcher,
        boundary_searcher);

    //DEMSystem->UpdateSystem(RenderInterval);
}

void Update()
{
    if (CUDA_DEM_APP_PARAMS.run && SimCount < TotalFrameNumber)
    {

        auto numOfSubTimeSteps = DEMSystem->GetNumOfSubTimeSteps();
        KIRI_LOG_INFO("Simulation Frame={0}, Sub-Simulation Total Number={1}", ++SimCount, numOfSubTimeSteps);

        PerFrameTimer.Restart();
        for (size_t i = 0; i < numOfSubTimeSteps; i++)
        {
            KIRI_LOG_INFO("Current Sub-Simulation/ Total Number ={0}/{1}", i + 1, numOfSubTimeSteps);
            DEMSystem->UpdateSystem(RenderInterval);
        }

        KIRI_LOG_INFO("Time Per Frame={0}", PerFrameTimer.Elapsed());
        TotalFrameTime += PerFrameTimer.Elapsed();
        if (CUDA_DEM_APP_PARAMS.bgeo_export)
        {
            auto particles = DEMSystem->GetParticles();
            ExportBgeoFileCUDA(
                CUDA_DEM_APP_PARAMS.bgeo_export_folder,
                UInt2Str4Digit(SimCount - RunLiquidNumber),
                particles->GetPosPtr(),
                particles->GetColPtr(),
                radius,
                1,
                particles->Size());
        }
    }
    else if (CUDA_DEM_APP_PARAMS.run)
    {
        CUDA_DEM_APP_PARAMS.run = false;
    }
}

void main()
{
    KiriLog::Init();

    SetupParams();

    CUDA_DEM_APP_PARAMS.run = true;

    while (CUDA_DEM_APP_PARAMS.run)
        Update();

    return;
}
