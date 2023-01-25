/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-10-16 14:32:34
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-10-16 14:35:41
 * @FilePath: \Kiri2D\demos\ms_sampler2d\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#include <kiri2d.h>
#include <tinycolormap.hpp>

using namespace KIRI2D;
using namespace HDV;

int main(int argc, char *argv[])
{
    // log system
    KiriLog::init();

    auto LoadPolygonFromFile = [](std::vector<Vector2F> &points, const char *file_path)
    {
        size_t length;
        std::ifstream file(file_path);
        file >> length;
        for (int i = 0; i < length; ++i)
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

    auto ExportRadius2CSVFile = [](const String fileName, const String idx, const Vector<double> &radius)
    {
        String voronoi_file = String(EXPORT_PATH) + "csv/" + fileName + "_samplers_" + idx + ".csv";
        std::fstream vfile;
        vfile.open(voronoi_file.c_str(), std::ios_base::out);
        vfile << "radius"
              << std::endl;
        for (int i = 0; i < radius.size(); i++)
            vfile << radius[i] << std::endl;

        vfile.close();
    };

    // scene renderer config
    float window_height = 4000.f;
    float window_width = 4000.f;

    auto offset = Vector2F(500.f);

    auto scene = std::make_shared<KiriScene2D<float>>((size_t)window_width, (size_t)window_height);
    auto renderer = std::make_shared<KiriRenderer2D<float>>(scene);

    auto ms_sampler = std::make_shared<Sampler::MultiSizedSampler2D>();

    auto scale_size = 1000.0;

    // define particle size distribution
    std::random_device seed;
    std::default_random_engine dengine(seed());
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    std::uniform_real_distribution<double> rdist(-1.0, 1.0);

    std::vector<double> radius_range;
    radius_range.push_back(20.0);
    radius_range.push_back(30.0);
    radius_range.push_back(80.0);
    radius_range.push_back(150.0);

    std::vector<double> radius_range_prob;
    radius_range_prob.push_back(0.7);
    radius_range_prob.push_back(0.2);
    radius_range_prob.push_back(0.1);

    ms_sampler->setRadiusDist(radius_range);
    ms_sampler->setRadiusDistProb(radius_range_prob);

    std::random_device engine;
    std::mt19937 gen(engine());
    std::piecewise_constant_distribution<double> pcdis{std::begin(radius_range), std::end(radius_range), std::begin(radius_range_prob)};

    Vec_Double ary1, ary2;
    for (size_t i = 0; i < radius_range.size(); i++)
    {
        ary1.emplace_back(radius_range[i]);
    }

    ary2.emplace_back(0.0);
    for (size_t i = 0; i < radius_range_prob.size(); i++)
    {
        ary2.emplace_back(radius_range_prob[i]);
    }

    auto total_sum = 0.0;
    for (size_t i = 0; i < radius_range_prob.size(); i++)
    {
        auto m = 0.5 * (ary2[i + 1] - ary2[i]) / (ary1[i + 1] - ary1[i]);
        auto b = (ary2[i] * ary1[i + 1] - ary1[i] * ary2[i + 1]) / (ary1[i + 1] - ary1[i]);
        total_sum += m * (ary1[i + 1] * ary1[i + 1] - ary1[i] * ary1[i]) + b * (ary1[i + 1] - ary1[i]);
    }

    // select boundary shape
    // you can choose {"bunny", "alligator", "beast", "cheburashka", "cow", "homer", "horse", "lucy", "nefertiti", "spot", "teapot", "woody", "xyzrgb_dragon"}
    auto boundary_filename = "bunny";
    auto boundary_polygon = std::make_shared<Voronoi::VoronoiPolygon2>();
    auto file_path = String(RESOURCES_PATH) + "alpha_shapes/" + boundary_filename + ".xy";
    std::vector<Vector2F> boundary_data;
    LoadPolygonFromFile(boundary_data, file_path.c_str());

    for (auto i = 0; i < boundary_data.size(); i++)
    {
        auto trans_pos = Vector2D(boundary_data[i].x, boundary_data[i].y) * scale_size * 3.0;
        boundary_polygon->add(trans_pos);
    }

    ms_sampler->setBoundary(boundary_polygon);

    auto total_area = boundary_polygon->area();
    auto total_num = total_area / (kiri_math_mini::pi<double>() * total_sum * total_sum);
    KIRI_LOG_DEBUG("avg_radius={0},total_area={1},total_num={2}", total_sum, total_area, total_num);

    std::vector<double> target_radius_array;
    auto init_particles_num = 250;
    for (size_t i = 0; i < init_particles_num; i++)
    {
        auto pos = boundary_polygon->rndInnerPoint();
        auto radius = pcdis(gen);
        target_radius_array.emplace_back(radius);
        ms_sampler->addSite(pos.x, pos.y, radius);
    }

    ExportRadius2CSVFile(boundary_filename, "target", target_radius_array);

    ms_sampler->setMaxiumNum(static_cast<int>(total_num * 2.5));

    ms_sampler->init();

    auto min_radius = std::numeric_limits<double>::max();
    auto max_radius = std::numeric_limits<double>::min();
    for (auto idx = 0; idx < 3000; idx++)
    {
        auto porosity = ms_sampler->compute();

        std::vector<KiriLine2<float>> precompute_lines;
        std::vector<Vector2F> precompute_points;

        auto sites = ms_sampler->sites();

        for (auto i = 0; i < sites.size(); i++)
        {
            auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
            if (site->isBoundaryVertex())
                continue;

            auto cell_polygon = site->polygon();
            if (cell_polygon)
            {
                for (auto j = 0; j < cell_polygon->positions().size(); j++)
                {
                    auto vert = cell_polygon->positions()[j];
                    auto vert1 = cell_polygon->positions()[(j + 1) % (cell_polygon->positions().size())];
                    auto line = KiriLine2(Vector2F(vert.x, vert.y) + offset, Vector2F(vert1.x, vert1.y) + offset);
                    line.thick = 1.f;
                    precompute_lines.emplace_back(line);
                }
            }

            precompute_points.emplace_back(Vector2F(site->x(), site->y()));
        }

        std::vector<KiriLine2<float>> lines;
        std::vector<KiriPoint2> points;
        std::vector<KiriCircle2> circles;
        for (auto i = 0; i < precompute_points.size(); i++)
        {
            points.emplace_back(KiriPoint2(precompute_points[i] + offset, Vector3F(1.f, 0.f, 0.f)));
        }

        for (auto i = 0; i < precompute_lines.size(); ++i)
        {
            lines.emplace_back(precompute_lines[i]);
        }

        auto mic = ms_sampler->computeMICBySSkel();

        Vec_Double predict_radius, real_radius;
        for (auto i = 0; i < mic.size(); i++)
        {
            auto mic_paint = KiriCircle2(Vector2F(mic[i].x, mic[i].y) + offset, Vector3F(1.f, 0.f, 0.f), mic[i].z);

            circles.emplace_back(mic_paint);

            min_radius = std::min(min_radius, mic[i].z);
            max_radius = std::max(max_radius, mic[i].z);

            predict_radius.emplace_back(mic[i].z);
            real_radius.emplace_back(mic[i].w);
        }

        // re-color
        for (auto i = 0; i < mic.size(); i++)
        {
            auto rad = (mic[i].z - min_radius) / (max_radius - min_radius);
            const tinycolormap::Color color = tinycolormap::GetColor(rad, tinycolormap::ColormapType::Plasma);
            circles[i].col = Vector3F(color.r(), color.g(), color.b());
        }

        scene->AddLines(lines);
        scene->AddParticles(points);
        scene->AddCircles(circles);

        renderer->DrawCanvas();

        if ((idx + 1) % 100 == 0)
            renderer->SaveImages2FileWithPrefix(boundary_filename);

        renderer->ClearCanvas();
        scene->Clear();

        auto cur_rmsp = ComputeRMSPE(predict_radius, real_radius);

        // export predict data
        // if ((idx + 1) % 100 == 0)
        //     ExportRadius2CSVFile(boundary_filename, std::to_string(idx), predict_radius);

        KIRI_LOG_DEBUG("name={0}, idx={1}, porosity={2},rmsp={3}", boundary_filename, idx, porosity, cur_rmsp);
    }

    return 0;
}
