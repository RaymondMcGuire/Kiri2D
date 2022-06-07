#include <kiri2d/renderer/renderer.h>
#include <kiri2d/sdf/sdf_poly_2d.h>
#include <root_directory.h>
#include <random>

#include <tinycolormap.hpp>

using namespace KIRI2D;

String UInt2Str4Digit(UInt Input)
{
    char output[5];
    snprintf(output, 5, "%04d", Input);
    return String(output);
};

#include <kiri2d/hdv_toolkit/sampler/ms_sampler2.h>
void QuickHullVoronoi2d()
{
    using namespace HDV;

    auto pd2 = std::make_shared<Voronoi::PowerDiagram2D>();

    std::random_device seed;
    std::default_random_engine engine(seed());
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    auto scale_size = 200.0;
    auto sampler_num = 100;

    for (auto i = 0; i < sampler_num; i++)
    {
        auto x = dist(engine) * scale_size;
        auto y = dist(engine) * scale_size;

        pd2->addSite(std::make_shared<Voronoi::VoronoiSite2>(x, y, i));
    }

    // clip boundary
    auto BoundaryPolygon = std::make_shared<Voronoi::VoronoiPolygon2>();
    BoundaryPolygon->add(Vector2D(-scale_size, -scale_size));
    BoundaryPolygon->add(Vector2D(-scale_size, scale_size));
    BoundaryPolygon->add(Vector2D(scale_size, scale_size));
    BoundaryPolygon->add(Vector2D(scale_size, -scale_size));
    pd2->setBoundary(BoundaryPolygon);

    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    Vector2F offset = Vector2F(windowwidth, windowheight) / 2.f;

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    pd2->compute();

    while (1)
    {
        // KIRI_LOG_DEBUG("-----------------new----------------------------------");

        std::vector<KiriLine2> precompute_lines;
        std::vector<Vector2F> precompute_points;

        pd2->lloydIteration();

        auto sites = pd2->sites();

        for (size_t i = 0; i < sites.size(); i++)
        {
            auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
            if (site->isBoundaryVertex())
                continue;

            auto cell_polygon = site->polygon();
            for (size_t j = 0; j < cell_polygon->positions().size(); j++)
            {
                auto vert = cell_polygon->positions()[j];
                auto vert1 = cell_polygon->positions()[(j + 1) % (cell_polygon->positions().size())];
                auto line = KiriLine2(Vector2F(vert.x, vert.y) + offset, Vector2F(vert1.x, vert1.y) + offset);
                line.thick = 1.f;
                precompute_lines.emplace_back(line);

                // KIRI_LOG_DEBUG("vert={0},{1}-----vert1={2},{3}", vert.x, vert.y, vert1.x, vert1.y);
            }
            // KIRI_LOG_DEBUG("site={0},size={1}", site->id(), cell_polygon->Verts.size());
            precompute_points.emplace_back(Vector2F(site->x(), site->y()));

            // KIRI_LOG_DEBUG("pd2->AddSite(std::make_shared<Voronoi::VoronoiSite2>({0}f, {1}f, {2}));", site->x()(), site->y()(), i);
        }

        std::vector<KiriLine2> lines;
        std::vector<KiriPoint2> points;
        for (size_t i = 0; i < precompute_points.size(); i++)
        {
            points.emplace_back(KiriPoint2(precompute_points[i] + offset, Vector3F(1.f, 0.f, 0.f)));
        }

        for (auto i = 0; i < precompute_lines.size(); ++i)
        {
            lines.emplace_back(precompute_lines[i]);
        }

        scene->AddLines(lines);
        scene->AddParticles(points);

        renderer->DrawCanvas();
        // renderer->SaveImages2File();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->clearCanvas();
        scene->clear();
    }
}

void MSSampler2D()
{
    using namespace HDV;

    // omp_set_num_threads(20);
    // KIRI_LOG_INFO("OpenMP max threads number ={0}; ", omp_get_max_threads());

    auto load_polygon2d = [](std::vector<Vector2F> &points, size_t &num, const char *filePath)
    {
        std::ifstream file(filePath);
        file >> num;
        for (int i = 0; i < num; ++i)
        {
            Vector2F xy;
            file >> xy.x >> xy.y;
            points.emplace_back(xy);
        }

        file.close();
    };

    auto ComputeRMSPE = [](const Vec_Double &predict, const Vec_Double &real)
    {
        auto sum = 0.0;
        auto n = predict.size();
        for (size_t i = 0; i < n; i++)
            sum += std::pow((predict[i] - real[i]) / real[i], 2.0);

        return std::sqrt(sum / n);
    };

    // scene renderer config
    float windowheight = 4000.f;
    float windowwidth = 4000.f;

    // Vector2F offset = Vector2F(windowwidth, windowheight) / 2.f;
    Vector2F offset = Vector2F(500.f);

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    auto multiSizeSampler = std::make_shared<Sampler::MultiSizedSampler2D>();

    auto scale_size = 1000.0;

    std::random_device seed;
    std::default_random_engine dengine(seed());
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    std::uniform_real_distribution<double> rdist(-1.0, 1.0);

    std::vector<double> radiusRange;
    radiusRange.push_back(20.0);
    radiusRange.push_back(30.0);
    radiusRange.push_back(80.0);
    radiusRange.push_back(150.0);

    std::vector<double> radiusRangeProb;
    radiusRangeProb.push_back(0.5);
    radiusRangeProb.push_back(0.4);
    radiusRangeProb.push_back(0.1);

    multiSizeSampler->setRadiusDist(radiusRange);
    multiSizeSampler->setRadiusDistProb(radiusRangeProb);

    std::random_device engine;
    std::mt19937 gen(engine());
    std::piecewise_constant_distribution<double> pcdis{std::begin(radiusRange), std::end(radiusRange), std::begin(radiusRangeProb)};

    Vec_Double ary1, ary2;
    for (size_t i = 0; i < 4; i++)
    {
        ary1.emplace_back(radiusRange[i]);
    }

    ary2.emplace_back(0.0);
    for (size_t i = 0; i < 3; i++)
    {
        ary2.emplace_back(radiusRangeProb[i]);
    }

    auto total_sum = 0.0;
    for (size_t i = 0; i < 3; i++)
    {
        auto m = 0.5 * (ary2[i + 1] - ary2[i]) / (ary1[i + 1] - ary1[i]);
        auto b = (ary2[i] * ary1[i + 1] - ary1[i] * ary2[i + 1]) / (ary1[i + 1] - ary1[i]);
        total_sum += m * (ary1[i + 1] * ary1[i + 1] - ary1[i] * ary1[i]) + b * (ary1[i + 1] - ary1[i]);
    }

    // clip boundary : box
    auto BoundaryPolygon = std::make_shared<Voronoi::VoronoiPolygon2>();
    // BoundaryPolygon->add(Vector2D(-scale_size, -scale_size));
    // BoundaryPolygon->add(Vector2D(-scale_size, scale_size));
    // BoundaryPolygon->add(Vector2D(scale_size, scale_size));
    // BoundaryPolygon->add(Vector2D(scale_size, -scale_size));

    // auto boundary_name_list = std::vector<String>
    //{"bunny", "alligator", "beast", "cheburashka", "cow", "homer", "horse", "lucy", "nefertiti", "spot", "teapot", "woody", "xyzrgb_dragon"};

    String boundaryFileName = "woody";

    String filePath = String(RESOURCES_PATH) + "alpha_shapes/" + boundaryFileName + ".xy";
    std::vector<Vector2F> bunny2d;
    size_t bunnyNum;
    load_polygon2d(bunny2d, bunnyNum, filePath.c_str());

    for (size_t i = 0; i < bunny2d.size(); i++)
    {
        auto newPos = Vector2D(bunny2d[i].x, bunny2d[i].y) * scale_size * 3.0;
        BoundaryPolygon->add(newPos);
    }

    multiSizeSampler->setBoundary(BoundaryPolygon);

    auto total_area = BoundaryPolygon->area();
    auto total_num = total_area / (kiri_math_mini::pi<double>() * total_sum * total_sum);
    total_num *= 1.5f;

    KIRI_LOG_DEBUG("avg_radius={0},total_area={1},total_num={2}", total_sum, total_area, total_num);

    auto maxcnt = 100;
    for (size_t i = 0; i < maxcnt; i++)
    {
        auto pos = BoundaryPolygon->rndInnerPoint();
        auto radius = pcdis(gen);
        multiSizeSampler->addSite(pos.x, pos.y, radius);
    }

    multiSizeSampler->setMaxiumNum(static_cast<int>(total_num * 1.5));

    multiSizeSampler->init();

    std::vector<float> errorArray, porosityArray, radiusErrorArray;
    std::vector<Vector4D> lastMaxCircle;
    auto minRadius = std::numeric_limits<double>::max();
    auto maxRadius = std::numeric_limits<double>::min();

    auto min_rmsp = std::numeric_limits<double>::max();
    for (size_t idx = 0; idx < 3000; idx++)
    {
        auto porosity = multiSizeSampler->compute();

        std::vector<KiriLine2> precompute_lines;
        std::vector<Vector2F> precompute_points;

        auto sites = multiSizeSampler->sites();

        for (size_t i = 0; i < sites.size(); i++)
        {
            auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
            if (site->isBoundaryVertex())
                continue;

            auto cell_polygon = site->polygon();
            if (cell_polygon)
            {
                for (size_t j = 0; j < cell_polygon->positions().size(); j++)
                {
                    auto vert = cell_polygon->positions()[j];
                    auto vert1 = cell_polygon->positions()[(j + 1) % (cell_polygon->positions().size())];
                    auto line = KiriLine2(Vector2F(vert.x, vert.y) + offset, Vector2F(vert1.x, vert1.y) + offset);
                    line.thick = 1.f;
                    precompute_lines.emplace_back(line);

                    // KIRI_LOG_DEBUG("vert={0},{1}-----vert1={2},{3}", vert.x, vert.y, vert1.x, vert1.y);
                }
            }

            // KIRI_LOG_DEBUG("site={0},size={1}", site->id(), cell_polygon->positions().size());
            precompute_points.emplace_back(Vector2F(site->x(), site->y()));

            // KIRI_LOG_DEBUG("pd2->AddSite(std::make_shared<Voronoi::VoronoiSite2>({0}f, {1}f, {2}));", site->x()(), site->y()(), i);
        }

        std::vector<KiriLine2> lines;
        std::vector<KiriPoint2> points;
        std::vector<KiriCircle2> circles;
        for (size_t i = 0; i < precompute_points.size(); i++)
        {
            points.emplace_back(KiriPoint2(precompute_points[i] + offset, Vector3F(1.f, 0.f, 0.f)));
        }

        for (auto i = 0; i < precompute_lines.size(); ++i)
        {
            lines.emplace_back(precompute_lines[i]);
        }

        auto maxIC = multiSizeSampler->computeMICBySSkel();
        lastMaxCircle = maxIC;

        Vec_Double predictRadiusArray, realRadiusArray;
        for (size_t i = 0; i < maxIC.size(); i++)
        {
            // auto maxCir2 = KiriCircle2(Transform2Original(Vector2F(maxIC[i].x, maxIC[i].y) * 10.f, height) + offsetVec2, Vector3F(1.f, 0.f, 0.f), maxIC[i].z * 10.f);
            auto maxCir2 = KiriCircle2(Vector2F(maxIC[i].x, maxIC[i].y) + offset, Vector3F(1.f, 0.f, 0.f), maxIC[i].z);

            circles.emplace_back(maxCir2);

            minRadius = std::min(minRadius, maxIC[i].z);
            maxRadius = std::max(maxRadius, maxIC[i].z);

            predictRadiusArray.emplace_back(maxIC[i].z);
            realRadiusArray.emplace_back(maxIC[i].w);
        }

        // re-color
        for (size_t i = 0; i < maxIC.size(); i++)
        {
            auto rad = (maxIC[i].z - minRadius) / (maxRadius - minRadius);
            const tinycolormap::Color color = tinycolormap::GetColor(rad, tinycolormap::ColormapType::Plasma);
            circles[i].col = Vector3F(color.r(), color.g(), color.b());
        }

        scene->AddLines(lines);
        scene->AddParticles(points);
        scene->AddCircles(circles);

        renderer->DrawCanvas();

        renderer->SaveImages2FileWithPrefix(boundaryFileName);
        renderer->clearCanvas();
        scene->clear();

        auto cur_rmsp = ComputeRMSPE(predictRadiusArray, realRadiusArray);
        min_rmsp = std::min(min_rmsp, cur_rmsp);

        KIRI_LOG_DEBUG("name={0}, idx={1}, porosity={2},minimum rmsp={3}", boundaryFileName, idx, porosity, min_rmsp);
    }
}

void ExportParticleRadiusDist()
{
    using namespace HDV;

    std::vector<double> radiusRange;
    radiusRange.push_back(10.0);
    radiusRange.push_back(30.0);
    radiusRange.push_back(50.0);
    radiusRange.push_back(100.0);

    std::vector<double> radiusRangeProb;
    radiusRangeProb.push_back(0.45);
    radiusRangeProb.push_back(0.45);
    radiusRangeProb.push_back(0.1);

    std::random_device engine;
    std::mt19937 gen(engine());
    std::piecewise_constant_distribution<double> pcdis{std::begin(radiusRange), std::end(radiusRange), std::begin(radiusRangeProb)};

    std::vector<double> radius_array;
    for (size_t i = 0; i < 1000; i++)
    {
        radius_array.emplace_back(pcdis(gen));
    }

    auto ExportSamplingRadius2CSVFile = [](const String fileName, const Vector<double> radius)
    {
        String filePath = String(EXPORT_PATH) + "csv/" + fileName;
        std::fstream file;
        file.open(filePath.c_str(), std::ios_base::out);
        file << "rad"
             << std::endl;
        for (int i = 0; i < radius.size(); i++)
            file << radius[i] << std::endl;

        file.close();
    };

    ExportSamplingRadius2CSVFile("test1", radius_array);
}

#include <kiri2d/sph/sph_solver.h>
void Sph2dExample()
{
    // sph config
    const float particleScale = 350.f;
    const float timeStep = 0.0001f;
    const Vector2F worldSize = Vector2F(3.f, 2.f);
    const float radius = 1.f / 140.f;
    const Vector2F boxVolumeSize = Vector2F(35.f, 70.f);

    // scene renderer config
    float windowheight = 720.f;
    float windowwidth = 1280.f;

    Vector2F offset = (Vector2F(windowwidth, windowheight) - worldSize * particleScale) / 2.f;

    SPH::SPHSolver sphSolver = SPH::SPHSolver(worldSize);

    sphSolver.initWithBoxVolume(boxVolumeSize, radius);

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    // draw boundary
    auto boundaryRect = KiriRect2(offset - Vector2F(radius) * 2.f * particleScale, (worldSize + 4.f * Vector2F(radius)) * particleScale);

    while (1)
    {
        // KIRI_LOG_DEBUG("-----------------new----------------------------------");

        std::vector<KiriPoint2> points;
        sphSolver.update(timeStep);
        auto particles = sphSolver.GetParticles();
        for (auto i = 0; i < particles.size(); i++)
        {
            auto particle = particles[i];
            auto p = KiriPoint2(particle.position * particleScale + offset, Vector3F(1.f, 0.f, 0.f));
            p.radius = particleScale * radius;
            points.emplace_back(p);
        }

        scene->AddRect(boundaryRect);
        scene->AddParticles(points);

        renderer->DrawCanvas();
        // renderer->SaveImages2File();
        cv::imshow("KIRI2D::SPH2D", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->clearCanvas();
        scene->clear();
    }
}

#include <kiri2d/sph/blue_noise_sph_solver.h>
void BlueNoiseSamplingVisual()
{
    auto load_polygon2d = [](std::vector<Vector2F> &points, size_t &num, const char *filePath)
    {
        std::ifstream file(filePath);
        file >> num;
        for (int i = 0; i < num; ++i)
        {
            Vector2F xy;
            file >> xy.x >> xy.y;
            points.emplace_back(xy);
        }

        file.close();
    };

    // scene renderer config
    float windowheight = 720.f;
    float windowwidth = 1280.f;

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    using namespace HDV;
    // 2d boundary polygon
    auto scaleSize = 600.f;

    // load 2d boundary file
    String boundaryFileName = "bunny";
    String filePath = String(RESOURCES_PATH) + "alpha_shapes/" + boundaryFileName + ".xy";
    std::vector<Vector2F> boundary2d_data;

    size_t bunnyNum;
    load_polygon2d(boundary2d_data, bunnyNum, filePath.c_str());

    KiriSDFPoly2D boundary_sdf;
    BoundingBox2F boundary_bbox;

    for (auto i = 0; i < boundary2d_data.size(); i++)
    {
        auto newPos = Vector2F(boundary2d_data[i].x, boundary2d_data[i].y);
        boundary_sdf.Append(newPos);
        boundary_bbox.merge(newPos);
    }

    // sdf sampling points
    std::vector<Vector2F> sdf_points;
    auto radius = 1.f / 140.f;
    auto density_radius = 0.95f * radius;
    auto lower = boundary_bbox.LowestPoint;
    auto higher = boundary_bbox.HighestPoint;
    auto wn = UInt(((higher - lower) / (density_radius * 2.f)).x);
    auto hn = UInt(((higher - lower) / (density_radius * 2.f)).y);
    for (auto i = 0; i <= wn; i++)
    {
        for (auto j = 0; j < hn; j++)
        {
            auto pos = lower + Vector2F(density_radius, density_radius) + Vector2F(i, j) * (density_radius * 2.f);

            if (boundary_sdf.FindRegion(pos) <= 0.f)
                sdf_points.emplace_back(pos);
        }
    }

    // std::cout << boundary_bbox.LowestPoint.x << "," << boundary_bbox.LowestPoint.y << ";" << boundary_bbox.HighestPoint.x << "," << boundary_bbox.HighestPoint.y << std::endl;

    // blue noise sampling
    // TODO worldsize
    const float timeStep = 0.00005f;
    auto worldSize = boundary_bbox.LowestPoint + boundary_bbox.HighestPoint;
    SPH::BlueNoiseSPHSolver blueNoiseSolver = SPH::BlueNoiseSPHSolver(worldSize, boundary_sdf);
    blueNoiseSolver.init(sdf_points, radius);

    // visualization
    Vector2F offset = (Vector2F(windowwidth, windowheight) - (boundary_bbox.width(), boundary_bbox.height()) * scaleSize) / 2.f;

    KiriSDFPoly2D boundary_vis;
    boundary_bbox.reset();
    for (auto i = 0; i < boundary2d_data.size(); i++)
    {
        auto newPos = Vector2F(boundary2d_data[i].x, boundary2d_data[i].y) * scaleSize + offset;
        boundary_vis.Append(newPos);
        boundary_bbox.merge(newPos);
    }

    std::vector<KiriCircle2> circles;
    for (auto i = 0; i < sdf_points.size(); i++)
        circles.emplace_back(KiriCircle2(sdf_points[i] * scaleSize + offset, Vector3F(100.f, 85.f, 134.f) / 255.f, radius * scaleSize));

    // draw boundary
    auto boundaryRect = KiriRect2(offset - Vector2F(radius) * 2.f * scaleSize, (worldSize + 4.f * Vector2F(radius)) * scaleSize);
    auto cnt = 0;
    while (1)
    {

        std::vector<KiriPoint2> points;
        blueNoiseSolver.update(timeStep);
        std::cout << cnt++ << std::endl;

        auto particles = blueNoiseSolver.GetParticles();
        for (auto i = 0; i < particles.size(); i++)
        {
            auto particle = particles[i];
            auto p = KiriPoint2(particle.position * scaleSize + offset, Vector3F(1.f, 0.f, 0.f));
            p.radius = scaleSize * radius;
            points.emplace_back(p);
        }

        scene->AddRect(boundaryRect);
        scene->AddParticles(points);

        // scene->AddCircles(circles);
        //  scene->AddObject(boundary_vis);

        renderer->DrawCanvas();
        renderer->SaveImages2File();
        cv::imshow("KIRI2D::Blue Noise Sampling", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->clearCanvas();
        scene->clear();
    }
}

void BlueNoiseSampling()
{
    auto load_polygon2d = [](std::vector<Vector2F> &points, size_t &num, const char *filePath)
    {
        std::ifstream file(filePath);
        file >> num;
        for (int i = 0; i < num; ++i)
        {
            Vector2F xy;
            file >> xy.x >> xy.y;
            points.emplace_back(xy);
        }

        file.close();
    };

    auto export_sampling_data = [](const String fileName, const std::vector<Vector2F> &center, const std::vector<float> &radius)
    {
        String filePath = String(EXPORT_PATH) + "csv/" + fileName;
        std::fstream file;
        file.open(filePath.c_str(), std::ios_base::out);
        file << "cx,cy,rad"
             << std::endl;
        for (int i = 0; i < center.size(); i++)
            file << center[i].x << "," << center[i].y << "," << radius[i] << std::endl;

        file.close();
    };

    // load 2d boundary file
    String boundaryFileName = "bunny";
    String filePath = String(RESOURCES_PATH) + "alpha_shapes/" + boundaryFileName + ".xy";
    std::vector<Vector2F> boundary2d_data;

    size_t bunnyNum;
    load_polygon2d(boundary2d_data, bunnyNum, filePath.c_str());

    KiriSDFPoly2D boundary_sdf;
    BoundingBox2F boundary_bbox;

    for (auto i = 0; i < boundary2d_data.size(); i++)
    {
        auto newPos = Vector2F(boundary2d_data[i].x, boundary2d_data[i].y);
        boundary_sdf.Append(newPos);
        boundary_bbox.merge(newPos);
    }

    // sdf sampling points
    std::vector<Vector2F> position;
    std::vector<float> rad;

    auto radius = 1.f / 140.f;
    auto density_radius = 0.95f * radius;
    auto lower = boundary_bbox.LowestPoint;
    auto higher = boundary_bbox.HighestPoint;
    auto wn = UInt(((higher - lower) / (density_radius * 2.f)).x);
    auto hn = UInt(((higher - lower) / (density_radius * 2.f)).y);
    for (auto i = 0; i <= wn; i++)
    {
        for (auto j = 0; j < hn; j++)
        {
            auto pos = lower + Vector2F(density_radius, density_radius) + Vector2F(i, j) * (density_radius * 2.f);

            if (boundary_sdf.FindRegion(pos) <= 0.f)
                position.emplace_back(pos);
        }
    }

    // std::cout << boundary_bbox.LowestPoint.x << "," << boundary_bbox.LowestPoint.y << ";" << boundary_bbox.HighestPoint.x << "," << boundary_bbox.HighestPoint.y << std::endl;

    // blue noise sampling
    // TODO worldsize
    const float timeStep = 0.00005f;
    auto worldSize = boundary_bbox.LowestPoint + boundary_bbox.HighestPoint;
    SPH::BlueNoiseSPHSolver blueNoiseSolver = SPH::BlueNoiseSPHSolver(worldSize, boundary_sdf);
    blueNoiseSolver.init(position, radius);

    for (auto i = 0; i < 1000; i++)
    {
        blueNoiseSolver.update(timeStep);
        std::cout << "step num=" << i << std::endl;
    }

    auto particles = blueNoiseSolver.GetParticles();
    position.clear();
    for (auto i = 0; i < particles.size(); i++)
    {
        auto particle = particles[i];
        position.emplace_back(particle.position);
        rad.emplace_back(radius);
    }

    export_sampling_data("jiang2015.csv", position, rad);
}

void main()
{
    KiriLog::init();

    // QuickHullVoronoi2d();

    // Sph2dExample();

    // BlueNoiseSampling();
    // BlueNoiseSamplingVisual();

    MSSampler2D();
    //     ExportParticleRadiusDist();
}