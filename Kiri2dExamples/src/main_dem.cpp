/*** 
 * @Author: Xu.WANG
 * @Date: 2021-09-03 09:27:54
 * @LastEditTime: 2021-09-26 16:13:40
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dExamples\src\main_dem.cpp
 */

#include <kiri_utils.h>
#include <kiri2d/renderer/renderer.h>

using namespace KIRI;
using namespace KIRI2D;

// scene config
float windowheight = 1080.f;
float windowwidth = 800.f;
auto particle_scale = 500.f;
auto world_size = make_float2(1.5f, 1.5f);

auto offsetvec2 = Vector2F(windowwidth - world_size.x * particle_scale, windowheight - world_size.y * particle_scale) / 2.f;

auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
auto renderer = std::make_shared<KiriRenderer2D>(scene);

KIRI::Vector<KiriCircle2> boundaries;

// global params
const UInt RunLiquidNumber = 0;
const UInt TotalFrameNumber = 300;
UInt SimCount = 0;
float TotalFrameTime = 0.f;
float RenderInterval = 1.f / 60.f;

KiriTimer PerFrameTimer;
CudaDemSystemPtr DEMSystem;

float radius = 0.015f;

String ReadFileIntoString(const String &path)
{
    auto ss = std::ostringstream{};
    std::ifstream input_file(path);
    if (!input_file.is_open())
        exit(EXIT_FAILURE);

    ss << input_file.rdbuf();
    return ss.str();
}

Vec_Float3 LoadCSVFile2ShapeSamplers(const String fileName)
{

    String filePath = String(EXPORT_PATH) + "csv/" + fileName;
    KIRI_LOG_DEBUG("Load File Path={0}", filePath);

    char delimiter = ',';

    auto file_contents = ReadFileIntoString(filePath);
    std::istringstream sstream(file_contents);
    KIRI::Vector<String> row;
    String record;

    Vec_Float3 samplers;

    // skip first header
    std::getline(sstream, record);

    // read
    while (std::getline(sstream, record))
    {
        std::istringstream line(record);
        while (std::getline(line, record, delimiter))
            row.push_back(record);

        samplers.emplace_back(make_float3(
            std::stof(row[0]),
            std::stof(row[1]),
            std::stof(row[2])));

        row.clear();
    }
    KIRI_LOG_DEBUG("Load File Finished");
    return samplers;
}

void DEM_SetupParams()
{
    KIRI_LOG_DEBUG("DEM: SetupParams");

    // app
    CUDA_DEM_APP_PARAMS.max_num = 400;
    CUDA_DEM_APP_PARAMS.bgeo_export = false;
    strcpy(CUDA_DEM_APP_PARAMS.bgeo_export_folder, (String(EXPORT_PATH) + "bgeo/dem").c_str());

    // scene config
    auto cuda_lowest_point = make_float2(0.f);
    auto cuda_highest_point = world_size;
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
    CUDA_DEM_PARAMS.c0 = 150.f;

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
    for (size_t i = 0; i < boundaryData.pos.size(); i++)
    {
        auto pb = KiriCircle2(Vector2F(boundaryData.pos[i].x, boundaryData.pos[i].y) * particle_scale + offsetvec2, Vector3F(0.f, 0.f, 1.f), radius * particle_scale);
        boundaries.emplace_back(pb);
    }

    auto volumeEmitter = std::make_shared<CudaVolumeEmitter>();

    // volume sampling
    // DemVolumeData volumeData;
    // int2 vbox = make_int2(20, 20);
    // volumeEmitter->BuildUniDemVolume(
    //     volumeData,
    //     CUDA_BOUNDARY_PARAMS.world_center - make_float2(vbox.x * CUDA_DEM_PARAMS.particle_radius, vbox.y * CUDA_DEM_PARAMS.particle_radius),
    //     vbox,
    //     CUDA_DEM_PARAMS.particle_radius,
    //     make_float3(0.88f, 0.79552f, 0.5984f),
    //     CUDA_DEM_PARAMS.rest_mass,
    //     0.001f * CUDA_DEM_PARAMS.particle_radius);

    DemShapeVolumeData volumeData;
    auto shape = LoadCSVFile2ShapeSamplers("uni_bunny_samplers_0000.csv");
    volumeEmitter->BuildDemUniShapeVolume(
        volumeData,
        shape,
        make_float3(0.88f, 0.79552f, 0.5984f),
        CUDA_DEM_PARAMS.rest_mass,
        make_float2(0.25f, 0.f));
    CUDA_DEM_APP_PARAMS.max_num = volumeData.pos.size();

    // spatial searcher & particles
    CudaDemParticlesPtr particles =
        std::make_shared<CudaDemParticles>(
            volumeData.pos,
            volumeData.col,
            volumeData.mass);

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

void MRDEM_SetupParams()
{
    KIRI_LOG_DEBUG("MRDEM: SetupParams");

    // app
    CUDA_DEM_APP_PARAMS.bgeo_export = false;
    strcpy(CUDA_DEM_APP_PARAMS.bgeo_export_folder, (String(EXPORT_PATH) + "bgeo/dem").c_str());

    // scene config
    auto cuda_lowest_point = make_float2(0.f);
    auto cuda_highest_point = world_size;
    auto cuda_world_size = cuda_highest_point - cuda_lowest_point;
    auto cuda_world_center = (cuda_highest_point + cuda_lowest_point) / 2.f;

    // dem params
    CUDA_DEM_PARAMS.rest_density = 2000.f;

    CUDA_DEM_PARAMS.young = 1e9f;
    CUDA_DEM_PARAMS.poisson = 0.3f;
    CUDA_DEM_PARAMS.tan_friction_angle = 0.5f;
    CUDA_DEM_PARAMS.c0 = 0.f;

    CUDA_DEM_PARAMS.gravity = make_float2(0.0f, -9.8f);

    CUDA_DEM_PARAMS.damping = 0.4f;
    //CUDA_DEM_PARAMS.dt = 0.5f * CUDA_DEM_PARAMS.particle_radius / std::sqrtf(CUDA_DEM_PARAMS.young / CUDA_DEM_PARAMS.rest_density);
    CUDA_DEM_PARAMS.dt = 5e-5f;

    // scene data
    CUDA_BOUNDARY_PARAMS.lowest_point = cuda_lowest_point;
    CUDA_BOUNDARY_PARAMS.highest_point = cuda_highest_point;
    CUDA_BOUNDARY_PARAMS.world_size = cuda_world_size;
    CUDA_BOUNDARY_PARAMS.world_center = cuda_world_center;

    // volume sampling
    auto volumeEmitter = std::make_shared<CudaVolumeEmitter>();
    DemShapeVolumeData volumeData;
    auto shape = LoadCSVFile2ShapeSamplers("bunny_samplers_1800.csv");
    volumeEmitter->BuildMRDemShapeVolume(
        volumeData,
        CUDA_DEM_PARAMS.rest_density,
        shape,
        make_float3(0.88f, 0.79552f, 0.5984f),
        make_float2(0.1f, 0.f));

    KIRI_LOG_DEBUG("max radius={0}, min radius={1}", volumeData.maxRadius, volumeData.minRadius);
    CUDA_DEM_PARAMS.kernel_radius = 4.f * volumeData.maxRadius;
    CUDA_DEM_APP_PARAMS.max_num = volumeData.pos.size();
    CUDA_BOUNDARY_PARAMS.kernel_radius = CUDA_DEM_PARAMS.kernel_radius;
    CUDA_BOUNDARY_PARAMS.grid_size = make_int2((CUDA_BOUNDARY_PARAMS.highest_point - CUDA_BOUNDARY_PARAMS.lowest_point) / CUDA_BOUNDARY_PARAMS.kernel_radius);

    // boundary sampling
    BoundaryData boundaryData;
    auto boundaryEmitter = std::make_shared<CudaBoundaryEmitter>();

    boundaryEmitter->BuildWorldBoundary(boundaryData, CUDA_BOUNDARY_PARAMS.lowest_point, CUDA_BOUNDARY_PARAMS.highest_point, volumeData.minRadius);
    for (size_t i = 0; i < boundaryData.pos.size(); i++)
    {
        auto pb = KiriCircle2(Vector2F(boundaryData.pos[i].x, boundaryData.pos[i].y) * particle_scale + offsetvec2, Vector3F(0.f, 0.f, 1.f), volumeData.minRadius * particle_scale);
        boundaries.emplace_back(pb);
    }

    // spatial searcher & particles
    CudaDemParticlesPtr particles =
        std::make_shared<CudaMRDemParticles>(
            volumeData.pos,
            volumeData.col,
            volumeData.radius,
            volumeData.mass);

    KIRI_LOG_DEBUG("max size={0}", particles->MaxSize());

    CudaGNSearcherPtr searcher;
    searcher = std::make_shared<CudaGNSearcher>(
        CUDA_BOUNDARY_PARAMS.lowest_point,
        CUDA_BOUNDARY_PARAMS.highest_point,
        particles->MaxSize(),
        CUDA_BOUNDARY_PARAMS.kernel_radius,
        SearcherParticleType::MRDEM);

    auto boundary_particles = std::make_shared<CudaBoundaryParticles>(boundaryData.pos, boundaryData.label);
    KIRI_LOG_INFO("Number of Boundary Particles = {0}", boundary_particles->Size());

    CudaGNBoundarySearcherPtr boundary_searcher = std::make_shared<CudaGNBoundarySearcher>(
        CUDA_BOUNDARY_PARAMS.lowest_point,
        CUDA_BOUNDARY_PARAMS.highest_point,
        boundary_particles->MaxSize(),
        CUDA_BOUNDARY_PARAMS.kernel_radius);

    CudaDemSolverPtr pSolver = std::make_shared<CudaMRDemSolver>(particles->Size());

    DEMSystem = std::make_shared<CudaDemSystem>(
        particles,
        boundary_particles,
        pSolver,
        searcher,
        boundary_searcher);
}

void UpdateScene(const KIRI::Vector<KiriCircle2> &circles)
{

    scene->AddCircles(boundaries);
    scene->AddCircles(circles);

    renderer->DrawCanvas();
    renderer->SaveImages2File();

    cv::imshow("DEM", renderer->GetCanvas());
    cv::waitKey(5);

    renderer->ClearCanvas();
    scene->Clear();
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
            //KIRI_LOG_INFO("Current Sub-Simulation/ Total Number ={0}/{1}", i + 1, numOfSubTimeSteps);
            DEMSystem->UpdateSystem(RenderInterval);
        }

        //KIRI_LOG_INFO("Time Per Frame={0}", PerFrameTimer.Elapsed());
        TotalFrameTime += PerFrameTimer.Elapsed();
        auto particles = DEMSystem->GetParticles();

        if (CUDA_DEM_APP_PARAMS.bgeo_export)
        {
            ExportBgeoFileCUDA(
                CUDA_DEM_APP_PARAMS.bgeo_export_folder,
                UInt2Str4Digit(SimCount - RunLiquidNumber),
                particles->GetPosPtr(),
                particles->GetColPtr(),
                radius,
                1,
                particles->Size());
        }
        else
        {

            KIRI::Vector<KiriCircle2> circles;
            DEMSystem->UpdateSystem(RenderInterval);
            auto particles = std::dynamic_pointer_cast<CudaMRDemParticles>(DEMSystem->GetParticles());
            auto particles_num = particles->Size();
            size_t fbytes = particles_num * sizeof(float);
            size_t f2bytes = particles_num * sizeof(float2);
            float2 *particle_positions = (float2 *)malloc(f2bytes);
            float *particle_radius = (float *)malloc(fbytes);
            cudaMemcpy(particle_positions, particles->GetPosPtr(), f2bytes, cudaMemcpyDeviceToHost);
            cudaMemcpy(particle_radius, particles->GetRadiusPtr(), fbytes, cudaMemcpyDeviceToHost);

            for (size_t i = 0; i < particles_num; i++)
            {
                auto p = KiriCircle2(Vector2F(particle_positions[i].x, particle_positions[i].y) * particle_scale + offsetvec2, Vector3F(0.88f, 0.79552f, 0.5984f), particle_radius[i] * particle_scale);
                circles.emplace_back(p);
            }

            UpdateScene(circles);
        }
    }
    else if (CUDA_DEM_APP_PARAMS.run)
    {
        CUDA_DEM_APP_PARAMS.run = false;
    }
}

void UpdateRealTime()
{
    DEMSystem->UpdateSystem(RenderInterval);

    auto particles = DEMSystem->GetParticles();

    KIRI::Vector<KiriCircle2> circles;

    auto particles_num = particles->Size();
    size_t f2bytes = particles_num * sizeof(float2);
    float2 *particle_positions = (float2 *)malloc(f2bytes);
    cudaMemcpy(particle_positions, particles->GetPosPtr(), f2bytes, cudaMemcpyDeviceToHost);

    for (size_t i = 0; i < particles_num; i++)
    {
        auto p = KiriCircle2(Vector2F(particle_positions[i].x, particle_positions[i].y) * particle_scale + offsetvec2, Vector3F(1.f, 0.f, 0.f), radius * particle_scale);
        circles.emplace_back(p);
    }

    UpdateScene(circles);
}

void main1()
{
    KiriLog::Init();

    // DEM_SetupParams();
    MRDEM_SetupParams();

    CUDA_DEM_APP_PARAMS.run = true;

    while (CUDA_DEM_APP_PARAMS.run)
        Update();

    return;
}
