/***
 * @Author: Xu.WANG
 * @Date: 2021-09-03 09:27:54
 * @LastEditTime: 2021-11-10 01:16:34
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dExamples\src\main_dem.cpp
 */

#if defined (KIRI_WINDOWS) && defined (ENABLE_CUDA)  

#include <kiri_utils.h>
#include <kiri2d/renderer/renderer.h>

using namespace KIRI;
using namespace KIRI2D;

String readFileIntoString1(const String &path)
{
    auto ss = std::ostringstream{};
    std::ifstream input_file(path);
    if (!input_file.is_open())
        exit(EXIT_FAILURE);

    ss << input_file.rdbuf();
    return ss.str();
}

std::vector<std::string> split_str(const std::string &s, char delim)
{
    std::vector<std::string> result;
    std::stringstream ss(s);
    std::string item;

    while (getline(ss, item, delim))
    {
        result.push_back(item);
    }

    return result;
}

std::vector<NSPack> LoadCSVFile2NSPack(const String fileName)
{
    String filePath = String(EXPORT_PATH) + "csv/" + fileName;

    char delimiter = ',';

    auto file_contents = readFileIntoString1(filePath);
    std::istringstream sstream(file_contents);
    std::vector<String> row;
    String record;

    std::vector<NSPack> ns_packs;

    while (std::getline(sstream, record))
    {
        std::istringstream line(record);
        while (std::getline(line, record, delimiter))
            row.push_back(record);

        NSPack ns_pack;
        auto pos_data = split_str(row[0], ';');
        auto rad_data = split_str(row[1], ';');
        auto col_data = split_str(row[2], ':');

        for (size_t i = 0; i < pos_data.size(); i++)
        {
            auto pos_str = split_str(pos_data[i], ':');
            ns_pack.AppendSubParticles(make_float2(std::stof(pos_str[0]), std::stof(pos_str[1])), std::stof(rad_data[i]));
        }
        ns_pack.SetColor(make_float3(std::stof(col_data[0]), std::stof(col_data[1]), std::stof(col_data[2])));
        ns_packs.emplace_back(ns_pack);

        row.clear();
    }

    return ns_packs;
}

// scene config
float windowheight = 1080.f;
float windowwidth = 800.f;
auto world_size = make_float2(1.5f, 1.5f);

// float windowheight = 1080;
// float windowwidth = 1920;
// auto world_size = make_float2(900, 800);

auto particle_scale = 500.f;

auto offsetvec2 = Vector2F(windowwidth - world_size.x * particle_scale, windowheight - world_size.y * particle_scale) / 2.f;

auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
auto renderer = std::make_shared<KiriRenderer2D>(scene);

std::vector<KiriCircle2> boundaries;

// global params
const UInt RunLiquidNumber = 0;
const UInt TotalFrameNumber = 300;
UInt SimCount = 0;
float TotalFrameTime = 0.f;
float RenderInterval = 1.f / 30.f;

KiriTimer PerFrameTimer;
CudaDemSystemPtr DEMSystem;

CudaDemNSSystemPtr DEMNSSystem;

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
    std::vector<String> row;
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
    // CUDA_DEM_PARAMS.dt = 0.5f * CUDA_DEM_PARAMS.particle_radius / std::sqrtf(CUDA_DEM_PARAMS.young / CUDA_DEM_PARAMS.rest_density);
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
    DemVolumeData volumeData;
    int2 vbox = make_int2(20, 20);
    volumeEmitter->BuildUniDemVolume(
        volumeData,
        CUDA_BOUNDARY_PARAMS.world_center - make_float2(vbox.x * CUDA_DEM_PARAMS.particle_radius, vbox.y * CUDA_DEM_PARAMS.particle_radius),
        vbox,
        CUDA_DEM_PARAMS.particle_radius,
        make_float3(0.88f, 0.79552f, 0.5984f),
        CUDA_DEM_PARAMS.rest_mass,
        0.001f * CUDA_DEM_PARAMS.particle_radius);

    // DemShapeVolumeData volumeData;
    // auto shape = LoadCSVFile2ShapeSamplers("uni_bunny_samplers_0000.csv");
    // volumeEmitter->BuildDemUniShapeVolume(
    //     volumeData,
    //     shape,
    //     make_float3(0.88f, 0.79552f, 0.5984f),
    //     CUDA_DEM_PARAMS.rest_mass,
    //     make_float2(0.25f, 0.f));
    // CUDA_DEM_APP_PARAMS.max_num = volumeData.pos.size();

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

    // DEMSystem->UpdateSystem(RenderInterval);
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

    CUDA_DEM_PARAMS.damping = 0.5f;
    // CUDA_DEM_PARAMS.dt = 0.5f * CUDA_DEM_PARAMS.particle_radius / std::sqrtf(CUDA_DEM_PARAMS.young / CUDA_DEM_PARAMS.rest_density);
    CUDA_DEM_PARAMS.dt = 5e-5f;

    // scene data
    CUDA_BOUNDARY_PARAMS.lowest_point = cuda_lowest_point;
    CUDA_BOUNDARY_PARAMS.highest_point = cuda_highest_point;
    CUDA_BOUNDARY_PARAMS.world_size = cuda_world_size;
    CUDA_BOUNDARY_PARAMS.world_center = cuda_world_center;

    // volume sampling
    auto volumeEmitter = std::make_shared<CudaVolumeEmitter>();
    DemShapeVolumeData volumeData;
    // auto shape = LoadCSVFile2ShapeSamplers("bunny_samplers_7001.csv");
    auto shape = LoadCSVFile2ShapeSamplers("jiang2015.csv");
    volumeEmitter->BuildMRDemShapeVolume(
        volumeData,
        CUDA_DEM_PARAMS.rest_density,
        shape,
        make_float3(0.88f, 0.79552f, 0.5984f),
        make_float2(0.2f, 0.2f));
    // make_float2(-0.1f, -0.1f));

    KIRI_LOG_DEBUG("max radius={0}, min radius={1}", volumeData.maxRadius, volumeData.minRadius);
    CUDA_BOUNDARY_PARAMS.min_radius = volumeData.minRadius;
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

void UpdateScene(const std::vector<KiriCircle2> &circles)
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
            // KIRI_LOG_INFO("Current Sub-Simulation/ Total Number ={0}/{1}", i + 1, numOfSubTimeSteps);
            DEMSystem->UpdateSystem(RenderInterval);
        }

        // KIRI_LOG_INFO("Time Per Frame={0}", PerFrameTimer.Elapsed());
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

            std::vector<KiriCircle2> circles;
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

    std::vector<KiriCircle2> circles;

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

void NSDEM_SetupParams()
{
    KIRI_LOG_DEBUG("NSDEM: SetupParams");

    // app
    CUDA_DEM_APP_PARAMS.bgeo_export = false;
    strcpy(CUDA_DEM_APP_PARAMS.bgeo_export_folder, (String(EXPORT_PATH) + "bgeo/dem").c_str());

    // windowheight = 1080.f;
    // windowwidth = 1920.f;
    // world_size = make_float2(900, 800);

    // scene config
    auto cuda_lowest_point = make_float2(0.f);
    auto cuda_highest_point = world_size;
    auto cuda_world_size = cuda_highest_point - cuda_lowest_point;
    auto cuda_world_center = (cuda_highest_point + cuda_lowest_point) / 2.f;

    offsetvec2 = Vector2F(windowwidth - world_size.x, windowheight - world_size.y) / 2.f;

    // scene = std::make_shared<KiriScene2D>((size_t)windowheight, (size_t)windowheight);
    // renderer = std::make_shared<KiriRenderer2D>(scene);

    // dem params
    CUDA_DEM_NS_PARAMS.rest_density = 2000.f;

    CUDA_DEM_NS_PARAMS.young = 1e9f;
    CUDA_DEM_NS_PARAMS.poisson = 0.3f;
    CUDA_DEM_NS_PARAMS.tan_friction_angle = 0.5f;

    CUDA_DEM_NS_PARAMS.gravity = make_float2(0.0f, -98.f);

    CUDA_DEM_NS_PARAMS.damping = 0.4f;
    // CUDA_DEM_PARAMS.dt = 0.5f * CUDA_DEM_PARAMS.particle_radius / std::sqrtf(CUDA_DEM_PARAMS.young / CUDA_DEM_PARAMS.rest_density);
    CUDA_DEM_NS_PARAMS.dt = 5e-5f;

    // scene data
    CUDA_BOUNDARY_PARAMS.lowest_point = cuda_lowest_point;
    CUDA_BOUNDARY_PARAMS.highest_point = cuda_highest_point;
    CUDA_BOUNDARY_PARAMS.world_size = cuda_world_size;
    CUDA_BOUNDARY_PARAMS.world_center = cuda_world_center;

    // volume sampling
    auto volumeEmitter = std::make_shared<CudaVolumeEmitter>();

    DemNSBoxVolumeData volumeData;
    std::vector<NSPackPtr> pack_types;

    // test volume data
    // pack_types.emplace_back(std::make_shared<NSPack>(MSM_L2, 10.f));
    // pack_types.emplace_back(std::make_shared<NSPack>(MSM_L3, 10.f));

    // volumeEmitter->BuildRndNSDemBoxVolume(volumeData, cuda_lowest_point + make_float2(200.f, 10.f),
    //                                       cuda_highest_point - make_float2(200.f), 10.f, 0.f,
    //                                       3000,
    //                                       pack_types);

    auto ns_packs_data = LoadCSVFile2NSPack("ns_data.csv");

    volumeEmitter->BuildNsDemVolume(volumeData, ns_packs_data, 0.15f, make_float2(80.f, -50.f));

    KIRI_LOG_DEBUG("max radius={0}, min radius={1}", volumeData.max_radius, volumeData.min_radius);
    CUDA_BOUNDARY_PARAMS.min_radius = volumeData.min_radius;
    CUDA_DEM_NS_PARAMS.kernel_radius = 4.f * volumeData.max_radius;
    CUDA_DEM_APP_PARAMS.max_num = volumeData.sphere_data.size();
    CUDA_BOUNDARY_PARAMS.kernel_radius = CUDA_DEM_NS_PARAMS.kernel_radius;
    CUDA_BOUNDARY_PARAMS.grid_size = make_int2((CUDA_BOUNDARY_PARAMS.highest_point - CUDA_BOUNDARY_PARAMS.lowest_point) / CUDA_BOUNDARY_PARAMS.kernel_radius);

    // boundary sampling
    BoundaryData boundaryData;
    auto boundaryEmitter = std::make_shared<CudaBoundaryEmitter>();

    boundaryEmitter->BuildWorldBoundary(boundaryData, CUDA_BOUNDARY_PARAMS.lowest_point, CUDA_BOUNDARY_PARAMS.highest_point, volumeData.min_radius);
    for (size_t i = 0; i < boundaryData.pos.size(); i++)
    {
        auto pb = KiriCircle2(Vector2F(boundaryData.pos[i].x, boundaryData.pos[i].y) + offsetvec2, Vector3F(0.f, 0.f, 1.f), volumeData.min_radius);
        boundaries.emplace_back(pb);
    }

    // cvt data
    std::vector<float> rad;
    std::vector<float2> pos;
    std::vector<float3> col;

    for (size_t i = 0; i < volumeData.sphere_data.size(); i++)
    {
        rad.emplace_back(volumeData.sphere_data[i].radius);
        pos.emplace_back(volumeData.sphere_data[i].center);
        col.emplace_back(volumeData.sphere_data[i].color);
    }

    // spatial searcher & particles
    auto particles =
        std::make_shared<CudaNonSphericalParticles>(
            pos,
            col,
            rad,
            volumeData.ns_data,
            volumeData.map_data);

    KIRI_LOG_DEBUG("max size={0}", particles->MaxSize());

    CudaGNSearcherPtr searcher;
    searcher = std::make_shared<CudaGNSearcher>(
        CUDA_BOUNDARY_PARAMS.lowest_point,
        CUDA_BOUNDARY_PARAMS.highest_point,
        particles->MaxSize(),
        CUDA_BOUNDARY_PARAMS.kernel_radius,
        SearcherParticleType::NON_SPHERICAL);

    auto boundary_particles = std::make_shared<CudaBoundaryParticles>(boundaryData.pos, boundaryData.label);
    KIRI_LOG_INFO("Number of Boundary Particles = {0}", boundary_particles->Size());

    CudaGNBoundarySearcherPtr boundary_searcher = std::make_shared<CudaGNBoundarySearcher>(
        CUDA_BOUNDARY_PARAMS.lowest_point,
        CUDA_BOUNDARY_PARAMS.highest_point,
        boundary_particles->MaxSize(),
        CUDA_BOUNDARY_PARAMS.kernel_radius);

    // group num
    CUDA_DEM_NS_PARAMS.group_num = volumeData.ns_data.size();
    KIRI_LOG_DEBUG("group num size={0}", volumeData.ns_data.size());

    // debug group
    // KIRI_LOG_DEBUG("group num={0}", volumeData.ns_data.size());
    // for (size_t i = 0; i < volumeData.ns_data.size(); i++)
    // {
    //     auto g = volumeData.ns_data[i];
    //     KIRI_LOG_DEBUG("id={0},mass={1},interia={2},ang_acc={3},ang_vel={4};subnum={5}", g.ns_id, g.mass, g.inertia, g.angle_acc, g.angle_vel, g.sub_num);

    //     for (size_t j = 0; j < g.sub_num; j++)
    //     {
    //         KIRI_LOG_DEBUG("pos={0},{1}", g.sub_pos_list[j].x, g.sub_pos_list[j].y);
    //     }
    // }

    auto pSolver = std::make_shared<CudaNonSphericalSolver>(particles->Size(), volumeData.ns_data.size());

    DEMNSSystem = std::make_shared<CudaDemNSSystem>(
        particles,
        boundary_particles,
        pSolver,
        searcher,
        boundary_searcher);
}

void UpdateNSSystem()
{
    if (CUDA_DEM_APP_PARAMS.run && SimCount < TotalFrameNumber)
    {

        auto numOfSubTimeSteps = DEMNSSystem->GetNumOfSubTimeSteps();
        KIRI_LOG_INFO("Simulation Frame={0}, Sub-Simulation Total Number={1}", ++SimCount, numOfSubTimeSteps);

        PerFrameTimer.Restart();
        for (size_t i = 0; i < numOfSubTimeSteps; i++)
        {
            KIRI_LOG_INFO("Current Sub-Simulation/ Total Number ={0}/{1}", i + 1, numOfSubTimeSteps);
            DEMNSSystem->UpdateSystem(RenderInterval);
        }

        // KIRI_LOG_INFO("Time Per Frame={0}", PerFrameTimer.Elapsed());
        TotalFrameTime += PerFrameTimer.Elapsed();
        auto particles = DEMNSSystem->GetParticles();

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

            std::vector<KiriCircle2> circles;
            auto particles_num = particles->Size();
            size_t fbytes = particles_num * sizeof(float);
            size_t f2bytes = particles_num * sizeof(float2);
            size_t f3bytes = particles_num * sizeof(float3);

            float *particle_radius = (float *)malloc(fbytes);
            float2 *particle_positions = (float2 *)malloc(f2bytes);
            float3 *particle_colors = (float3 *)malloc(f3bytes);

            cudaMemcpy(particle_radius, particles->GetRadiusPtr(), fbytes, cudaMemcpyDeviceToHost);
            cudaMemcpy(particle_positions, particles->GetPosPtr(), f2bytes, cudaMemcpyDeviceToHost);
            cudaMemcpy(particle_colors, particles->GetColPtr(), f3bytes, cudaMemcpyDeviceToHost);

            for (size_t i = 0; i < particles_num; i++)
            {
                auto p = KiriCircle2(Vector2F(particle_positions[i].x, particle_positions[i].y) + offsetvec2, Vector3F(particle_colors[i].x, particle_colors[i].y, particle_colors[i].z), particle_radius[i]);
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

void UpdateNSSystemRealTime()
{

    auto numOfSubTimeSteps = DEMNSSystem->GetNumOfSubTimeSteps();
    KIRI_LOG_INFO("Simulation Frame={0}, Sub-Simulation Total Number={1}", ++SimCount, numOfSubTimeSteps);

    PerFrameTimer.Restart();
    for (size_t i = 0; i < numOfSubTimeSteps; i++)
    {
        KIRI_LOG_INFO("Current Sub-Simulation/ Total Number ={0}/{1}", i + 1, numOfSubTimeSteps);
        DEMNSSystem->UpdateSystem(RenderInterval);
    }

    // KIRI_LOG_INFO("Time Per Frame={0}", PerFrameTimer.Elapsed());
    TotalFrameTime += PerFrameTimer.Elapsed();
    auto particles = DEMNSSystem->GetParticles();

    std::vector<KiriCircle2> circles;
    auto particles_num = particles->Size();
    size_t fbytes = particles_num * sizeof(float);
    size_t f2bytes = particles_num * sizeof(float2);
    size_t f3bytes = particles_num * sizeof(float3);

    float *particle_radius = (float *)malloc(fbytes);
    float2 *particle_positions = (float2 *)malloc(f2bytes);
    float3 *particle_colors = (float3 *)malloc(f3bytes);

    cudaMemcpy(particle_radius, particles->GetRadiusPtr(), fbytes, cudaMemcpyDeviceToHost);
    cudaMemcpy(particle_positions, particles->GetPosPtr(), f2bytes, cudaMemcpyDeviceToHost);
    cudaMemcpy(particle_colors, particles->GetColPtr(), f3bytes, cudaMemcpyDeviceToHost);

    for (size_t i = 0; i < particles_num; i++)
    {
        auto pos = Vector2F(particle_positions[i].x, particle_positions[i].y) + offsetvec2;
        auto p = KiriCircle2(pos, Vector3F(particle_colors[i].x, particle_colors[i].y, particle_colors[i].z), particle_radius[i]);
        circles.emplace_back(p);

        // KIRI_LOG_DEBUG("pos={0},{1}", pos.x, pos.y);
    }

    UpdateScene(circles);
}

void main_dem()
{
    KiriLog::Init();

    CUDA_DEM_APP_PARAMS.run = true;

    // DEM_SetupParams();
    MRDEM_SetupParams();
    while (CUDA_DEM_APP_PARAMS.run)
        Update();

    // NSDEM_SetupParams();
    // while (CUDA_DEM_APP_PARAMS.run)
    //     UpdateNSSystemRealTime();

    return;
}

#endif