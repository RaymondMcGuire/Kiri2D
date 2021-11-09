/***
 * @Author: Xu.WANG
 * @Date: 2021-02-21 18:37:46
 * @LastEditTime: 2021-10-20 00:18:46
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dExamples\src\main.cpp
 */

#include <kiri2d/voronoi/voro_treemap_nocaj12.h>
#include <kiri2d/renderer/renderer.h>
#include <kiri2d/sdf/sdf_poly_2d.h>
#include <kiri2d/treemap/treemap_layout.h> 
#include <kiri2d/voronoi/voro_poropti.h>
#include <kiri2d/voronoi/voro_poropti_treemap.h>
#include <root_directory.h>
#include <random>

#include <kiri2d/lib/tinycolormap.hpp>

using namespace KIRI;
using namespace KIRI2D;

float ComputeRMSE(const Vec_Float &predict, const Vec_Float &real)
{
    auto sum = 0.f;
    auto n = predict.size();
    for (size_t i = 0; i < n; i++)
        sum += std::powf(predict[i] - real[i], 2.f);

    return std::sqrt(sum / n);
}

float ComputeRMSPE(const Vec_Float &predict, const Vec_Float &real)
{
    auto sum = 0.f;
    auto n = predict.size();
    for (size_t i = 0; i < n; i++)
        sum += std::powf((predict[i] - real[i]) / real[i], 2.f);

    return std::sqrt(sum / n);
}

Vector2F Transform2Original(const Vector2F &v, float h)
{
    return Vector2F(v.x, h - v.y);
}

String readFileIntoString(const String &path)
{
    auto ss = std::ostringstream{};
    std::ifstream input_file(path);
    if (!input_file.is_open())
        exit(EXIT_FAILURE);

    ss << input_file.rdbuf();
    return ss.str();
}

Vector<Vector3F> LoadCSVFile2VoronoiSites(const String fileName)
{
    String filePath = String(EXPORT_PATH) + "csv/" + fileName;

    char delimiter = ',';

    auto file_contents = readFileIntoString(filePath);
    std::istringstream sstream(file_contents);
    Vector<String> row;
    String record;

    Vector<Vector3F> voro_sites;

    while (std::getline(sstream, record))
    {
        std::istringstream line(record);
        while (std::getline(line, record, delimiter))
            row.push_back(record);

        // KIRI_LOG_DEBUG("{0},{1},{2}", std::stof(row[0]), std::stof(row[1]), std::stof(row[2]));

        voro_sites.emplace_back(Vector3F(
            std::stof(row[0]),
            std::stof(row[1]),
            std::stof(row[2])));

        row.clear();
    }

    return voro_sites;
}

void ExportSamplingData2CSVFile(const String fileName, const Vector<Vector2F> &center, const Vector<float> &radius)
{
    String filePath = String(EXPORT_PATH) + "csv/" + fileName;
    std::fstream file;
    file.open(filePath.c_str(), std::ios_base::out);
    file << "cx,cy,rad"
         << std::endl;
    for (int i = 0; i < center.size(); i++)
        file << center[i].x << "," << center[i].y << "," << radius[i] << std::endl;

    file.close();
}

void ExportPoroityData2CSVFile(const String fileName, const String idx, const Vector<float> &error, const Vector<float> &poroity, const Vector<float> &radiusError, const Vector<Vector4F> &maxIC)
{
    String errorFile = String(EXPORT_PATH) + "csv/" + fileName + "_error_" + idx + ".csv";
    std::fstream efile;
    efile.open(errorFile.c_str(), std::ios_base::out);
    efile << "iter,error,radiusError,poroity"
          << std::endl;
    for (int i = 0; i < error.size(); i++)
        efile << i + 1 << "," << error[i] << "," << radiusError[i] << "," << poroity[i] << std::endl;

    efile.close();

    String radiusFile = String(EXPORT_PATH) + "csv/" + fileName + "_radius_" + idx + ".csv";
    std::fstream rfile;
    rfile.open(radiusFile.c_str(), std::ios_base::out);
    rfile << "curRadius,tarRadius"
          << std::endl;
    for (int i = 0; i < maxIC.size(); i++)
        rfile << maxIC[i].z << "," << maxIC[i].w << std::endl;
    rfile.close();
}

void ExportEvaluationData2CSVFile(const String fileName, const String idx, const Vec_Float &rmse, const Vec_Float &rmspe)
{
    String evalFile = String(EXPORT_PATH) + "csv/" + fileName + "_eval_" + idx + ".csv";
    std::fstream efile;
    efile.open(evalFile.c_str(), std::ios_base::out);
    efile << "iter,RMSE,RMSPE"
          << std::endl;
    for (int i = 0; i < rmse.size(); i++)
        efile << i + 1 << "," << rmse[i] << "," << rmspe[i] << std::endl;

    efile.close();
}

void ExportVoronoiData2CSVFile(const String fileName, const String idx, const Vector<Vector3F> &sites)
{
    String voronoiFile = String(EXPORT_PATH) + "csv/" + fileName + "_sites_" + idx + ".csv";
    std::fstream vfile;
    vfile.open(voronoiFile.c_str(), std::ios_base::out);
    vfile << "sitex,sitey,weight"
          << std::endl;
    for (int i = 0; i < sites.size(); i++)
        vfile << sites[i].x << "," << sites[i].y << "," << sites[i].z << std::endl;

    vfile.close();
}

void ExportSamplerData2CSVFile(const String fileName, const String idx, const Vector<KiriCircle2> &samplers)
{
    String voronoiFile = String(EXPORT_PATH) + "csv/" + fileName + "_samplers_" + idx + ".csv";
    std::fstream vfile;
    vfile.open(voronoiFile.c_str(), std::ios_base::out);
    vfile << "x,y,r"
          << std::endl;
    for (int i = 0; i < samplers.size(); i++)
        vfile << samplers[i].pos.x << "," << samplers[i].pos.y << "," << samplers[i].radius << std::endl;

    vfile.close();
}

void load_xy_file1(std::vector<Vector2F> &points, size_t &num, const char *filePath)
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
}

void GenRndTreemap()
{
    float height = 1080.f;
    float width = 1920.f;

    float aspect = height / width;
    float offset = height / 30.f;
    Vector2F offsetVec2 = Vector2F(offset, offset);

    String tempNodeName = "O";
    KiriRect2 topRect(offsetVec2, Vector2F(width, height) - offsetVec2 * 2.f);

    std::vector<float> radiusRange;
    radiusRange.push_back(0.2f / 2.f);
    radiusRange.push_back(0.2f);
    radiusRange.push_back(0.2f * 1.5f);

    std::vector<float> radiusRangeProb;
    radiusRangeProb.push_back(0.8f);
    radiusRangeProb.push_back(0.2f);

    std::random_device engine;
    std::mt19937 gen(engine());
    std::piecewise_constant_distribution<float> pcdis{std::begin(radiusRange), std::end(radiusRange), std::begin(radiusRangeProb)};

    std::vector<TreemapNode> nodes;
    std::vector<KiriCircle2> circles;

    size_t totalNum = 100;
    float totalValue = 0.f;
    for (size_t i = 0; i < totalNum; i++)
    {
        float radius = pcdis(gen);
        totalValue += radius;
        nodes.emplace_back(TreemapNode("A", -1, -1, radius, 0));
    }

    // TreemapNode topNode(tempNodeName, 35, 10, topRect);
    TreemapNode topNode(tempNodeName, 0, -1, totalValue, totalNum, topRect);

    TreemapLayoutPtr treemap2d = std::make_shared<TreemapLayout>(topNode, tempNodeName);
    treemap2d->AddTreeNodes(nodes);
    treemap2d->ConstructTreemapLayout();
    treemap2d->PrintTreemapLayout();

    auto voroTreeNodes = treemap2d->Convert2VoroTreeData();
    for (size_t i = 0; i < voroTreeNodes->GetTotalNodeNum(); i++)
    {
        KIRI_LOG_DEBUG("current node id={0}", i);
        auto array = voroTreeNodes->GetChildNodesById(i);
        for (size_t j = 0; j < array.size(); j++)
            KIRI_LOG_DEBUG("child: id={0}, pid={1}, weight={2}, depth={3}", array[j].id, array[j].pid, array[j].weight, array[j].depth);
    }
}

void VoronoiExample()
{
    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    // voronoi
    auto boundaryPoly = std::make_shared<KiriVoroCellPolygon2>();
    float width = 1000.f;
    float height = 1000.f;
    auto offsetVec2 = Vector2F((windowwidth - width) / 2.f, (windowheight - height) / 2.f);

    // boundary polygon
    KiriSDFPoly2D boundary;
    auto bp1 = Vector2F(0, 0);
    auto bp2 = Vector2F(width, 0);
    auto bp3 = Vector2F(width, height);
    auto bp4 = Vector2F(0, height);

    boundary.Append(bp1);
    boundary.Append(bp2);
    boundary.Append(bp3);
    boundary.Append(bp4);

    boundaryPoly->AddPolygonVertex2(bp1);
    boundaryPoly->AddPolygonVertex2(bp2);
    boundaryPoly->AddPolygonVertex2(bp3);
    boundaryPoly->AddPolygonVertex2(bp4);

    auto pd = std::make_shared<KiriPowerDiagram>();

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<float> dist(0.f, 1.f);

    auto cnt = 0, maxcnt = 100;
    while (cnt < maxcnt)
    {
        auto sitePos2 = Vector2F(dist(rndEngine) * width, dist(rndEngine) * height);
        if (boundary.FindRegion(sitePos2) < 0.f)
        {
            if (sitePos2.x < width / 2.f)
                pd->AddPowerSite(sitePos2, 0.f);
            else
                pd->AddPowerSite(sitePos2, dist(rndEngine) * 10000.f);
            cnt++;
        }
    }

    pd->SetBoundaryPolygon2(boundaryPoly);
    pd->ComputeDiagram();
    // pd->SetRelaxIterNumber(100);
    // pd->LloydRelaxation();

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    while (1)
    {
        Vector<KiriPoint2> points;
        Vector<KiriLine2> lines;

        auto sites = pd->GetVoroSites();
        for (size_t i = 0; i < sites.size(); i++)
        {
            // sites[i]->Print();
            auto pos = Transform2Original(Vector2F(sites[i]->GetValue().x, sites[i]->GetValue().y), height) + offsetVec2;

            if (sites[i]->GetValue().x < width / 2.f)
                points.emplace_back(KiriPoint2(pos, Vector3F(1.f, 0.f, 0.f)));
            else
                points.emplace_back(KiriPoint2(pos, Vector3F(1.f, 0.f, 1.f)));

            auto poly = sites[i]->GetCellPolygon();
            if (poly != NULL)
            {
                poly->ComputeVoroSitesList();
                auto list = poly->GetVoroSitesList();
                // list->PrintVertexList();

                auto node = list->GetHead();
                do
                {
                    auto start = Transform2Original(Vector2F(node->value), height) + offsetVec2;
                    node = node->next;
                    auto end = Transform2Original(Vector2F(node->value), height) + offsetVec2;
                    auto line = KiriLine2(start, end);
                    if (sites[i]->GetValue().x > width / 2.f)
                        line.col = Vector3F(134, 185, 253) / 255.f;

                    lines.emplace_back(line);
                } while (node != list->GetHead());
            }
        }

        scene->AddLines(lines);
        scene->AddParticles(points);

        renderer->DrawCanvas();
        renderer->SaveImages2File();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->ClearCanvas();
        scene->Clear();
    }
}

void VoronoiExample1()
{
    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    // voronoi
    auto boundaryPoly = std::make_shared<KiriVoroCellPolygon2>();
    float width = 1000.f;
    float height = 1000.f;
    auto offsetVec2 = Vector2F((windowwidth - width) / 2.f, (windowheight - height) / 2.f);

    // boundary polygon
    KiriSDFPoly2D boundary;
    auto bp1 = Vector2F(0, 0);
    auto bp2 = Vector2F(width, 0);
    auto bp3 = Vector2F(width, height);
    auto bp4 = Vector2F(0, height);

    boundary.Append(bp1);
    boundary.Append(bp2);
    boundary.Append(bp3);
    boundary.Append(bp4);

    boundaryPoly->AddPolygonVertex2(bp1);
    boundaryPoly->AddPolygonVertex2(bp2);
    boundaryPoly->AddPolygonVertex2(bp3);
    boundaryPoly->AddPolygonVertex2(bp4);

    auto pd = std::make_shared<KiriPowerDiagram>();

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<float> dist(0.f, 1.f);

    auto cnt = 0, maxcnt = 100;
    while (cnt < maxcnt)
    {
        auto sitePos2 = Vector2F(dist(rndEngine) * width, dist(rndEngine) * height);
        if (boundary.FindRegion(sitePos2) < 0.f)
        {
            pd->AddPowerSite(sitePos2, 0.f);
            cnt++;
        }
    }

    pd->SetBoundaryPolygon2(boundaryPoly);
    pd->ComputeDiagram();
    // pd->SetRelaxIterNumber(100);
    // pd->LloydRelaxation();

    auto del_tri = pd->ComputeDelaunayTriangulation();

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    while (1)
    {
        Vector<KiriPoint2> points;
        Vector<KiriLine2> lines;

        auto sites = pd->GetVoroSites();
        for (size_t i = 0; i < sites.size(); i++)
        {
            // sites[i]->Print();
            auto pos = Transform2Original(Vector2F(sites[i]->GetValue().x, sites[i]->GetValue().y), height) + offsetVec2;

            if (sites[i]->GetValue().x < width / 2.f)
                points.emplace_back(KiriPoint2(pos, Vector3F(1.f, 0.f, 0.f)));
            else
                points.emplace_back(KiriPoint2(pos, Vector3F(1.f, 0.f, 1.f)));

            auto poly = sites[i]->GetCellPolygon();
            if (poly != NULL)
            {
                poly->ComputeVoroSitesList();
                auto list = poly->GetVoroSitesList();
                // list->PrintVertexList();

                auto node = list->GetHead();
                do
                {
                    auto start = Transform2Original(Vector2F(node->value), height) + offsetVec2;
                    node = node->next;
                    auto end = Transform2Original(Vector2F(node->value), height) + offsetVec2;
                    auto line = KiriLine2(start, end);
                    lines.emplace_back(line);
                } while (node != list->GetHead());
            }
        }

        for (size_t i = 0; i < del_tri.size(); i++)
        {
            auto start = Transform2Original(Vector2F(del_tri[i].x, del_tri[i].y), height) + offsetVec2;
            auto end = Transform2Original(Vector2F(del_tri[i].z, del_tri[i].w), height) + offsetVec2;
            auto line = KiriLine2(start, end);
            line.col = Vector3F(0, 0, 0) / 255.f;
            line.thick = 2.f;
            lines.emplace_back(line);
        }

        scene->AddLines(lines);
        scene->AddParticles(points);

        renderer->DrawCanvas();
        renderer->SaveImages2File();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->ClearCanvas();
        scene->Clear();
    }
}

void VoronoiExample2()
{
    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    // voronoi
    auto boundaryPoly = std::make_shared<KiriVoroCellPolygon2>();
    float width = 1200.f;
    float height = 800.f;

    // boundary polygon
    KiriSDFPoly2D boundary;

    auto PI = 3.141592653f;
    auto numPoints = 8;
    auto radius = 500.f;
    auto offsetVec2 = Vector2F((windowwidth - width) / 2.f, (windowheight - height) / 2.f);

    for (auto j = 0; j < numPoints; j++)
    {
        auto angle = 2.0 * PI * (j * 1.f / numPoints);
        auto rotate = 2.0 * PI / numPoints / 2;
        auto y = std::sin(angle + rotate) * radius;
        auto x = std::cos(angle + rotate) * radius;
        auto pos = Vector2F(x, y) + Vector2F(width / 2, height / 2);

        boundary.Append(pos);
        boundaryPoly->AddPolygonVertex2(pos);
    }

    // auto bp1 = Vector2F(0, 0);
    // auto bp2 = Vector2F(width, 0);
    // auto bp3 = Vector2F(width, height);
    // auto bp4 = Vector2F(0, height);

    // auto bp1 = Vector2F(0, height);
    // auto bp2 = Vector2F(width, height);
    // auto bp3 = Vector2F(width, 0);
    // auto bp4 = Vector2F(0, 0);

    // boundary.Append(bp1);
    // boundary.Append(bp2);
    // boundary.Append(bp3);
    // boundary.Append(bp4);

    // boundaryPoly->AddPolygonVertex2(bp1);
    // boundaryPoly->AddPolygonVertex2(bp2);
    // boundaryPoly->AddPolygonVertex2(bp3);
    // boundaryPoly->AddPolygonVertex2(bp4);

    auto pd = std::make_shared<KiriPowerDiagram>();

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<float> dist(0.f, 1.f);

    auto cnt = 0, maxcnt = 50;
    while (cnt < maxcnt)
    {
        auto sitePos2 = Vector2F(dist(rndEngine) * width, dist(rndEngine) * height);
        if (boundary.FindRegion(sitePos2) < 0.f)
        {
            pd->AddVoroSite(sitePos2);
            cnt++;
        }
    }

    pd->SetBoundaryPolygon2(boundaryPoly);
    pd->ComputeDiagram();
    pd->SetRelaxIterNumber(100);
    pd->LloydRelaxation();

    // auto maxCir = pd->ComputeMaxInscribedCircle();
    // auto maxCir2 = KiriCircle2(Transform2Original(Vector2F(maxCir.x, maxCir.y), height) + offsetVec2, Vector3F(0.f, 0.f, 1.f), maxCir.z);
    // KIRI_LOG_DEBUG("MaxInscribedCircle, center=({0},{1}), radius={2}", maxCir.x, maxCir.y, maxCir.z);

    // auto porosity = pd->ComputeMinPorosity();
    // KIRI_LOG_DEBUG("Minium porosity = {0}", porosity);

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    Vector<KiriPoint2> points;
    Vector<KiriLine2> lines;
    Vector<KiriCircle2> circles;

    // debug shrink
    // boundaryPoly->ComputeStraightSkeleton(100.f);
    // //auto shrinks = boundaryPoly->GetShrinks();
    // auto skeletons = boundaryPoly->GetSkeletons();
    // auto mic = boundaryPoly->ComputeMICByStraightSkeleton();
    // auto micPos = Transform2Original(Vector2F(mic.x, mic.y), height) + offsetVec2;
    // circles.emplace_back(KiriCircle2(micPos, Vector3F(255.f, 0.f, 255.f), mic.z));

    // for (size_t i = 0; i < skeletons.size(); i++)
    // {
    //     auto start = Transform2Original(Vector2F(skeletons[i].x, skeletons[i].y), height) + offsetVec2;
    //     auto end = Transform2Original(Vector2F(skeletons[i].z, skeletons[i].w), height) + offsetVec2;
    //     auto line = KiriLine2(start, end);
    //     line.col = Vector3F(0.f, 0.f, 255.f);
    //     lines.emplace_back(line);
    // }

    // voronoi sites
    auto sites = pd->GetVoroSites();
    for (size_t i = 0; i < sites.size(); i++)
    {
        // sites[i]->Print();
        auto pos = Transform2Original(Vector2F(sites[i]->GetValue().x, sites[i]->GetValue().y), height) + offsetVec2;
        points.emplace_back(KiriPoint2(pos, Vector3F(1.f, 0.f, 0.f)));
        auto poly = sites[i]->GetCellPolygon();
        if (poly != NULL)
        {
            // poly->ComputeStraightSkeleton(1.f);
            poly->ComputeSSkel1998Convex();

            // auto mic = poly->ComputeMICByStraightSkeleton();
            // auto micPos = Transform2Original(Vector2F(mic.x, mic.y), height) + offsetVec2;
            // circles.emplace_back(KiriCircle2(micPos, Vector3F(0.f, 0.f, 100.f), mic.z));

            auto amic = poly->ComputeAllCByStraightSkeleton();
            for (size_t ic = 0; ic < amic.size(); ic++)
            {
                auto micPos = Transform2Original(Vector2F(amic[ic].x, amic[ic].y), height) + offsetVec2;

                circles.emplace_back(KiriCircle2(micPos, Vector3F(dist(rndEngine), dist(rndEngine), dist(rndEngine)), amic[ic].z, false));
            }

            auto skeletons = poly->GetSkeletons();
            for (size_t j = 0; j < skeletons.size(); j++)
            {
                auto start = Transform2Original(Vector2F(skeletons[j].x, skeletons[j].y), height) + offsetVec2;
                auto end = Transform2Original(Vector2F(skeletons[j].z, skeletons[j].w), height) + offsetVec2;
                auto line = KiriLine2(start, end);
                line.col = Vector3F(0.f, 0.f, 255.f);
                lines.emplace_back(line);
            }

            poly->ComputeVoroSitesList();
            auto list = poly->GetVoroSitesList();
            // list->PrintVertexList();

            auto node = list->GetHead();
            do
            {
                auto start = Transform2Original(Vector2F(node->value), height) + offsetVec2;
                node = node->next;
                auto end = Transform2Original(Vector2F(node->value), height) + offsetVec2;
                lines.emplace_back(KiriLine2(start, end));
            } while (node != list->GetHead());
        }
    }

    scene->AddLines(lines);
    scene->AddParticles(points);
    scene->AddCircles(circles);

    renderer->DrawCanvas();
    renderer->SaveImages2File();
    while (1)
    {
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
    }
}

void LloydRelaxationExample()
{
    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    // voronoi
    auto boundaryPoly = std::make_shared<KiriVoroCellPolygon2>();
    float width = 1000.f;
    float height = 1000.f;
    auto offsetVec2 = Vector2F((windowwidth - width) / 2.f, (windowheight - height) / 2.f);

    Vector<Vector2F> bunny2d;
    size_t bunnyNum;
    load_xy_file1(bunny2d, bunnyNum, "D:/project/Kiri2D/scripts/alphashape/test.xy");

    KiriSDFPoly2D boundary;

    for (size_t i = 0; i < bunny2d.size(); i++)
    {
        auto point = KiriPoint2(bunny2d[i] * 800.f + Vector2F(500.f, 100.f), Vector3F(1.f, 0.f, 0.f));
        boundary.Append(point.pos);
        boundaryPoly->AddPolygonVertex2(point.pos);
    }

    // auto PI = 3.141592653f;
    // auto numPoints = 8;
    // auto radius = 500.f;

    // for (auto j = 0; j < numPoints; j++)
    // {
    //     auto angle = 2.0 * PI * (j * 1.f / numPoints);
    //     auto rotate = 2.0 * PI / numPoints / 2;
    //     auto y = std::sin(angle + rotate) * radius;
    //     auto x = std::cos(angle + rotate) * radius;
    //     auto pos = Vector2F(x, y) + Vector2F(width / 2, height / 2);

    //     boundary.Append(pos);
    //     boundaryPoly->AddPolygonVertex2(pos);
    // }

    // auto bp1 = Vector2F(0, 0);
    // auto bp2 = Vector2F(width, 0);
    // auto bp3 = Vector2F(width, height);
    // auto bp3_1 = Vector2F(width / 2.f, height);
    // auto bp4 = Vector2F(0.f, height);

    // boundary.Append(bp1);
    // boundary.Append(bp2);
    // boundary.Append(bp3);
    // boundary.Append(bp4);

    // boundaryPoly->AddPolygonVertex2(bp1);
    // boundaryPoly->AddPolygonVertex2(bp2);
    // boundaryPoly->AddPolygonVertex2(bp3);
    // boundaryPoly->AddPolygonVertex2(bp4);

    auto pd = std::make_shared<KiriPowerDiagram>();

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<float> dist(0.f, 1.f);

    auto cnt = 0, maxcnt = 100;
    while (cnt < maxcnt)
    {
        auto sitePos2 = Vector2F(dist(rndEngine) * width, dist(rndEngine) * height);
        if (boundary.FindRegion(sitePos2) < 0.f)
        {
            auto radius = 10.f;
            // auto site = std::make_shared<KiriVoroSite>(sitePos2.x, sitePos2.y, MEpsilon<float>(), radius);
            pd->AddVoroSite(sitePos2);
            cnt++;
        }
    }

    pd->SetBoundaryPolygon2(boundaryPoly);
    pd->ComputeDiagram();
    // pd->SetRelaxIterNumber(100);
    // pd->LloydRelaxation();

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    while (1)
    {
        pd->LloydIterate();
        Vector<KiriPoint2> points;
        Vector<KiriLine2> lines;

        auto sites = pd->GetVoroSites();
        for (size_t i = 0; i < sites.size(); i++)
        {
            // sites[i]->Print();
            points.emplace_back(KiriPoint2(Vector2F(sites[i]->GetValue().x, sites[i]->GetValue().y) + offsetVec2, Vector3F(1.f, 0.f, 0.f)));
            auto poly = sites[i]->GetCellPolygon();
            if (poly != NULL)
            {
                poly->ComputeVoroSitesList();
                auto list = poly->GetVoroSitesList();
                // list->PrintVertexList();

                auto node = list->GetHead();
                do
                {
                    auto start = Vector2F(node->value) + offsetVec2;
                    node = node->next;
                    auto end = Vector2F(node->value) + offsetVec2;
                    lines.emplace_back(KiriLine2(start, end));
                } while (node != list->GetHead());
            }
        }

        scene->AddLines(lines);
        scene->AddParticles(points);

        renderer->DrawCanvas();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->ClearCanvas();
        scene->Clear();
    }
}

void NOCAJ12Example()
{
    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    // voronoi
    float width = 1000.f;
    float height = 1000.f;
    auto offsetVec2 = Vector2F((windowwidth - width) / 2.f, (windowheight - height) / 2.f);

    auto PI = 3.141592653f;
    auto numPoints = 8;
    auto radius = 500.f;

    Vector<Vector2F> boundary;
    for (auto j = 0; j < numPoints; j++)
    {
        auto angle = 2.0 * PI * (j * 1.f / numPoints);
        auto rotate = 2.0 * PI / numPoints / 2;
        auto y = std::sin(angle + rotate) * radius;
        auto x = std::cos(angle + rotate) * radius;
        auto pos = Vector2F(x, y) + Vector2F(width / 2, height / 2);

        boundary.emplace_back(pos);
    }

    auto nocaj12 = std::make_shared<KiriVoroTreeMapNocaj12>();
    nocaj12->SetRootBoundary2(boundary);
    nocaj12->GenExample(width, height);

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    while (1)
    {

        nocaj12->ComputeIterate();

        Vector<KiriPoint2> points;
        Vector<KiriLine2> lines;

        auto sites = nocaj12->GetLeafNodeSites();
        for (size_t i = 0; i < sites.size(); i++)
        {
            // sites[i]->Print();
            auto p = Transform2Original(Vector2F(sites[i]->GetValue().x, sites[i]->GetValue().y), height) + offsetVec2;
            points.emplace_back(KiriPoint2(p, Vector3F(1.f, 0.f, 0.f)));
            auto poly = sites[i]->GetCellPolygon();
            if (poly != NULL)
            {
                poly->ComputeVoroSitesList();
                auto list = poly->GetVoroSitesList();
                // list->PrintVertexList();

                auto node = list->GetHead();
                do
                {
                    auto start = Transform2Original(Vector2F(node->value), height) + offsetVec2;

                    node = node->next;
                    auto end = Transform2Original(Vector2F(node->value), height) + offsetVec2;

                    lines.emplace_back(KiriLine2(start, end));
                } while (node != list->GetHead());
            }
        }

        scene->AddLines(lines);
        scene->AddParticles(points);

        renderer->DrawCanvas();
        renderer->SaveImages2File();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->ClearCanvas();
        scene->Clear();
    }
}

void NOCAJ12Example1()
{
    // scene renderer config
    // float windowheight = 1080.f;
    // float windowwidth = 1920.f;

    // // voronoi
    // auto boundaryPoly = std::make_shared<KiriVoroCellPolygon2>();
    // float width = 1000.f;
    // float height = 1000.f;
    // auto offsetVec2 = Vector2F((windowwidth - width) / 2.f, (windowheight - height) / 2.f);

    // //!TODO  polygon intersection has bug
    // KiriSDFPoly2D boundary;
    // auto bp1 = Vector2F(0, 0);
    // auto bp2 = Vector2F(width, 0);
    // auto bp3 = Vector2F(width, height);
    // auto bp3_1 = Vector2F(width / 2.f, height);
    // auto bp4 = Vector2F(0.f, height);

    // boundary.Append(bp1);
    // boundary.Append(bp2);
    // boundary.Append(bp3);
    // boundary.Append(bp4);

    // boundaryPoly->AddPolygonVertex2(bp1);
    // boundaryPoly->AddPolygonVertex2(bp2);
    // boundaryPoly->AddPolygonVertex2(bp3);
    // boundaryPoly->AddPolygonVertex2(bp4);

    // auto nocaj12 = std::make_shared<KiriVoroTreemapNocaj12>();

    // // generate voronoi treemap
    // float aspect = height / width;
    // float offset = height / 30.f;

    // String tempNodeName = "O";
    // KiriRect2 topRect(offsetVec2, Vector2F(width, height) - offsetVec2 * 2.f);

    // std::vector<float> radiusRange;
    // radiusRange.push_back(0.2f / 2.f);
    // radiusRange.push_back(0.2f);
    // radiusRange.push_back(0.2f * 1.5f);

    // std::vector<float> radiusRangeProb;
    // radiusRangeProb.push_back(0.8f);
    // radiusRangeProb.push_back(0.2f);

    // std::random_device engine;
    // std::mt19937 gen(engine());
    // std::piecewise_constant_distribution<float> pcdis{std::begin(radiusRange), std::end(radiusRange), std::begin(radiusRangeProb)};

    // std::default_random_engine rndEngine(engine());
    // std::uniform_real_distribution<float> dist(0.f, 1.f);

    // std::vector<TreemapNode> nodes;
    // std::vector<KiriCircle2> circles;

    // size_t totalNum = 10;
    // float totalValue = 0.f;
    // for (size_t i = 0; i < totalNum; i++)
    // {
    //     float radius = pcdis(gen);
    //     totalValue += radius;
    //     nodes.emplace_back(TreemapNode("A", -1, -1, radius, 0));
    // }

    // TreemapNode topNode(tempNodeName, 0, -1, totalValue, totalNum, topRect);

    // TreemapLayoutPtr treemap2d = std::make_shared<TreemapLayout>(topNode, tempNodeName);
    // treemap2d->AddTreeNodes(nodes);
    // treemap2d->ConstructTreemapLayout();
    // treemap2d->PrintTreemapLayout();

    // auto depthid = 0;
    // auto voroTreeNodes = treemap2d->Convert2VoroTreeData();
    // auto depth0nodes = voroTreeNodes->GetLeafChildNodes();
    // auto child0weight = voroTreeNodes->GetLeafChildTotalWeight();

    // for (size_t i = 0; i < depth0nodes.size(); i++)
    // {
    //     auto site = std::make_shared<KiriVoroSite>(dist(rndEngine) * width, dist(rndEngine) * height);
    //     site->SetPercentage(depth0nodes[i].weight / child0weight);
    //     nocaj12->AddSite(site);
    // }

    // // auto site = nocaj12->GetSites();
    // // site[0]->SetPercentage(0.4f);
    // // site[1]->SetPercentage(0.4f);

    // nocaj12->SetBoundaryPolygon2(boundaryPoly);
    // nocaj12->Init();

    // // debug
    // // auto site1 = nocaj12->GetSites();
    // // for (size_t i = 0; i < site1.size(); i++)
    // //     site1[i]->PrintSite();

    // auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    // auto renderer = std::make_shared<KiriRenderer2D>(scene);

    // while (1)
    // {

    //     nocaj12->ComputeIterate();

    //     Vector<KiriPoint2> points;
    //     Vector<KiriLine2> lines;

    //     auto sites = nocaj12->GetSites();
    //     for (size_t i = 0; i < sites.size(); i++)
    //     {
    //         //sites[i]->Print();
    //         auto p = Transform2Original(Vector2F(sites[i]->GetValue().x, sites[i]->GetValue().y), height) + offsetVec2;
    //         points.emplace_back(KiriPoint2(p, Vector3F(1.f, 0.f, 0.f)));
    //         auto poly = sites[i]->GetCellPolygon();
    //         if (poly != NULL)
    //         {
    //             poly->ComputeVoroSitesList();
    //             auto list = poly->GetVoroSitesList();
    //             // list->PrintVertexList();

    //             auto node = list->GetHead();
    //             do
    //             {
    //                 auto start = Transform2Original(Vector2F(node->value), height) + offsetVec2;

    //                 node = node->next;
    //                 auto end = Transform2Original(Vector2F(node->value), height) + offsetVec2;

    //                 lines.emplace_back(KiriLine2(start, end));
    //             } while (node != list->GetHead());
    //         }
    //     }

    //     scene->AddLines(lines);
    //     scene->AddParticles(points);

    //     renderer->DrawCanvas();
    //     cv::imshow("KIRI2D", renderer->GetCanvas());
    //     cv::waitKey(5);
    //     renderer->ClearCanvas();
    //     scene->Clear();
    // }
}

void VoroTestExample()
{
    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    // voronoi
    auto boundaryPoly = std::make_shared<KiriVoroCellPolygon2>();
    float width = 1000.f;
    float height = 1000.f;
    auto offsetVec2 = Vector2F((windowwidth - width) / 2.f, (windowheight - height) / 2.f);

    KiriSDFPoly2D boundary;

    auto PI = 3.141592653f;
    auto numPoints = 8;
    auto radius = 500.f;

    for (auto j = 0; j < numPoints; j++)
    {
        auto angle = 2.0 * PI * (j * 1.f / numPoints);
        auto rotate = 2.0 * PI / numPoints / 2;
        auto y = std::sin(angle + rotate) * radius;
        auto x = std::cos(angle + rotate) * radius;
        auto pos = Vector2F(x, y) + Vector2F(width / 2, height / 2);

        boundary.Append(pos);
        boundaryPoly->AddPolygonVertex2(pos);
    }

    auto voroPorOptiCore = std::make_shared<KiriVoroPoroOptiCore>();

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<float> dist(0.f, 1.f);
    // std::uniform_real_distribution<float> rdist(-1.f, 1.f);

    // auto totalArea = boundaryPoly->GetPolygonArea();
    // auto avgRadius = std::sqrt(totalArea / maxcnt / KIRI_PI<float>());
    // KIRI_LOG_DEBUG("avg radius={0}", avgRadius);

    std::vector<float> radiusRange;
    radiusRange.push_back(20.f);
    radiusRange.push_back(50.f);
    radiusRange.push_back(150.f);

    std::vector<float> radiusRangeProb;
    radiusRangeProb.push_back(0.9f);
    radiusRangeProb.push_back(0.1f);

    std::random_device engine;
    std::mt19937 gen(engine());
    std::piecewise_constant_distribution<float> pcdis{std::begin(radiusRange), std::end(radiusRange), std::begin(radiusRangeProb)};

    auto cnt = 0, maxcnt = 10;
    while (cnt < maxcnt)
    {
        auto sitePos2 = Vector2F(dist(rndEngine) * width, dist(rndEngine) * height);
        if (boundary.FindRegion(sitePos2) < 0.f)
        {
            // auto radius = avgRadius / 2.f * rdist(rndEngine) + avgRadius;
            // KIRI_LOG_DEBUG("idx={0}, radius={1}", cnt, radius);
            auto radius = pcdis(gen);
            auto site = std::make_shared<KiriVoroSite>(sitePos2);
            site->SetRadius(20.f);
            voroPorOptiCore->AddSite(site);
            cnt++;
        }
    }
    voroPorOptiCore->SetBoundaryPolygon2(boundaryPoly);
    voroPorOptiCore->Init();

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    while (1)
    {
        voroPorOptiCore->ComputeIterate();
        Vector<KiriPoint2> points;
        Vector<KiriLine2> lines;

        auto sites = voroPorOptiCore->GetSites();
        for (size_t i = 0; i < sites.size(); i++)
        {
            // sites[i]->Print();
            points.emplace_back(KiriPoint2(Vector2F(sites[i]->GetValue().x, sites[i]->GetValue().y) + offsetVec2, Vector3F(1.f, 0.f, 0.f)));
            auto poly = sites[i]->GetCellPolygon();
            if (poly != NULL)
            {
                poly->ComputeVoroSitesList();
                auto list = poly->GetVoroSitesList();
                // list->PrintVertexList();

                auto node = list->GetHead();
                do
                {
                    auto start = Vector2F(node->value) + offsetVec2;
                    node = node->next;
                    auto end = Vector2F(node->value) + offsetVec2;
                    lines.emplace_back(KiriLine2(start, end));
                } while (node != list->GetHead());
            }
        }

        scene->AddLines(lines);
        scene->AddParticles(points);

        renderer->DrawCanvas();
        // renderer->SaveImages2File();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->ClearCanvas();
        scene->Clear();
    }
}

String UInt2Str4Digit(UInt Input)
{
    char output[5];
    snprintf(output, 5, "%04d", Input);
    return String(output);
};

void VoroPorosityOptimizeConvexExample()
{
    // scene renderer config
    float windowheight = 5000.f;
    float windowwidth = 5000.f;

    // voronoi
    float width = 3000.f;
    float height = 3000.f;
    auto offsetVec2 = Vector2F((windowwidth - width) / 2.f, (windowheight - height) / 2.f);

    auto PI = 3.141592653f;
    auto numPoints = 8;
    auto radius = 2000.f;

    Vector<Vector2F> boundary;
    for (auto j = 0; j < numPoints; j++)
    {
        auto angle = 2.0 * PI * (j * 1.f / numPoints);
        auto rotate = 2.0 * PI / numPoints / 2;
        auto y = std::sin(angle + rotate) * radius;
        auto x = std::cos(angle + rotate) * radius;
        auto pos = Vector2F(x, y) + Vector2F(width / 2, height / 2);

        boundary.emplace_back(pos);
    }

    auto opti = std::make_shared<KiriVoroPoroOpti>();
    opti->SetRootBoundary2(boundary);
    opti->GenExample(width, height);

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    Vector<float> errorArray, porosityArray, radiusErrorArray;
    Vector<Vector4F> lastMaxCircle;
    auto minRadius = Huge<float>();
    auto maxRadius = Tiny<float>();
    for (size_t i = 0; i < 3002; i++)
    {

        auto error = opti->ComputeIterate();

        if (i % 10 == 1)
        {
            // MIC calculated by vornoi
            // opti->ComputeChildIterate();

            Vector<KiriPoint2> points;
            Vector<KiriLine2> lines;
            Vector<KiriCircle2> circles;

            auto sites = opti->GetLeafNodeSites();
            for (size_t i = 0; i < sites.size(); i++)
            {
                // sites[i]->Print();
                auto p = Transform2Original(Vector2F(sites[i]->GetValue().x, sites[i]->GetValue().y), height) + offsetVec2;
                points.emplace_back(KiriPoint2(p, Vector3F(1.f, 0.f, 0.f)));
                auto poly = sites[i]->GetCellPolygon();
                if (poly != NULL)
                {
                    poly->ComputeVoroSitesList();
                    auto list = poly->GetVoroSitesList();
                    // list->PrintVertexList();

                    auto node = list->GetHead();
                    do
                    {
                        auto start = Transform2Original(Vector2F(node->value), height) + offsetVec2;

                        node = node->next;
                        auto end = Transform2Original(Vector2F(node->value), height) + offsetVec2;
                        auto line = KiriLine2(start, end);
                        line.thick = 5.f;
                        lines.emplace_back(line);
                    } while (node != list->GetHead());
                }
            }

            auto maxIC = opti->GetMICBySSkel();
            lastMaxCircle = maxIC;

            auto porosity = opti->GetMiniumPorosity();
            porosityArray.emplace_back(porosity);

            KIRI_LOG_DEBUG("iterate idx:{0}, porosity={1}, error={2}", i, porosity, error / maxIC.size());

            auto radiusError = 0.f;
            for (size_t i = 0; i < maxIC.size(); i++)
            {
                auto maxCir2 = KiriCircle2(Transform2Original(Vector2F(maxIC[i].x, maxIC[i].y), height) + offsetVec2, Vector3F(1.f, 0.f, 0.f), maxIC[i].z);
                circles.emplace_back(maxCir2);
                // KIRI_LOG_INFO("Site idx={0}, max radius={1}, target radius={2}", i, maxIC[i].z, maxIC[i].w);
                radiusError += std::abs(maxIC[i].z - maxIC[i].w);

                minRadius = std::min(minRadius, maxIC[i].z);
                maxRadius = std::max(maxRadius, maxIC[i].z);
            }

            errorArray.emplace_back(error / maxIC.size());
            radiusErrorArray.emplace_back(radiusError / maxIC.size());

            // re-color
            for (size_t i = 0; i < maxIC.size(); i++)
            {
                auto rad = (maxIC[i].z - minRadius) / (maxRadius - minRadius);
                const tinycolormap::Color color = tinycolormap::GetColor(rad, tinycolormap::ColormapType::Plasma);
                circles[i].col = Vector3F(color.r(), color.g(), color.b());
            }

            // auto skeletons = opti->GetCellSSkel();

            // for (size_t j = 0; j < skeletons.size(); j++)
            // {
            //     auto start = Transform2Original(Vector2F(skeletons[j].x, skeletons[j].y), height) + offsetVec2;
            //     auto end = Transform2Original(Vector2F(skeletons[j].z, skeletons[j].w), height) + offsetVec2;
            //     auto line = KiriLine2(start, end);
            //     line.col = Vector3F(0.f, 0.f, 100.f);

            //     lines.emplace_back(line);
            // }

            // auto shrinks = opti->GetCellShrinks();

            // for (size_t j = 0; j < shrinks.size(); j++)
            // {
            //     auto start = Transform2Original(Vector2F(shrinks[j].x, shrinks[j].y), height) + offsetVec2;
            //     auto end = Transform2Original(Vector2F(shrinks[j].z, shrinks[j].w), height) + offsetVec2;
            //     auto line = KiriLine2(start, end);
            //     line.col = Vector3F(0.f, 0.f, 100.f);
            //     lines.emplace_back(line);
            // }

            scene->AddParticles(points);
            scene->AddCircles(circles);
            scene->AddLines(lines);

            renderer->DrawCanvas();
            renderer->SaveImages2File();

            renderer->ClearCanvas();
            scene->Clear();
        }

        if (i % 100 == 1)
        {
            ExportPoroityData2CSVFile("convex", UInt2Str4Digit(i), errorArray, porosityArray, radiusErrorArray, lastMaxCircle);
        }
    }
}

void VoroPorosityTreemapOptiExample()
{
    float height = 1080.f;
    float width = 1920.f;

    float aspect = height / width;
    float offset = height / 30.f;
    Vector2F offsetVec2 = Vector2F(offset, offset);

    String tempNodeName = "O";
    KiriRect2 topRect(offsetVec2, Vector2F(width, height) - offsetVec2 * 2.f);

    std::vector<float> radiusRange;
    radiusRange.push_back(0.2f / 2.f);
    radiusRange.push_back(0.2f);
    radiusRange.push_back(0.2f * 1.5f);

    std::vector<float> radiusRangeProb;
    radiusRangeProb.push_back(0.8f);
    radiusRangeProb.push_back(0.2f);

    std::random_device engine;
    std::mt19937 gen(engine());
    std::piecewise_constant_distribution<float> pcdis{std::begin(radiusRange), std::end(radiusRange), std::begin(radiusRangeProb)};

    std::vector<TreemapNode> nodes;
    std::vector<KiriCircle2> circles;

    size_t totalNum = 200;
    float totalValue = 0.f;
    for (size_t i = 0; i < totalNum; i++)
    {
        float radius = pcdis(gen);
        totalValue += radius;
        nodes.emplace_back(TreemapNode("A", -1, -1, radius, 0));
    }

    // TreemapNode topNode(tempNodeName, 35, 10, topRect);
    TreemapNode topNode(tempNodeName, 0, -1, totalValue, totalNum, topRect);

    TreemapLayoutPtr treemap2d = std::make_shared<TreemapLayout>(topNode, tempNodeName);
    treemap2d->AddTreeNodes(nodes);
    treemap2d->ConstructTreemapLayout();
    // treemap2d->PrintTreemapLayout();

    // voronoi
    auto boundaryPoly = std::make_shared<KiriVoroCellPolygon2>();
    width = 1200.f;
    height = 800.f;

    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    // boundary polygon
    KiriSDFPoly2D boundary;

    auto PI = 3.141592653f;
    auto numPoints = 8;
    auto radius = 500.f;
    offsetVec2 = Vector2F((windowwidth - width) / 2.f, (windowheight - height) / 2.f);

    for (auto j = 0; j < numPoints; j++)
    {
        auto angle = 2.0 * PI * (j * 1.f / numPoints);
        auto rotate = 2.0 * PI / numPoints / 2;
        auto y = std::sin(angle + rotate) * radius;
        auto x = std::cos(angle + rotate) * radius;
        auto pos = Vector2F(x, y) + Vector2F(width / 2, height / 2);

        boundary.Append(pos);
        boundaryPoly->AddPolygonVertex2(pos);
    }

    auto voroTreeNodes = treemap2d->Convert2VoroTreeData();

    auto voroPorTreeMap = std::make_shared<KiriVoroPoroOptiTreeMap>(
        voroTreeNodes, boundaryPoly);
    voroPorTreeMap->InitTreeMapNodes();

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    while (1)
    {
        voroPorTreeMap->ComputeIterate();
        Vector<KiriPoint2> points;
        Vector<KiriLine2> lines;

        auto sites = voroPorTreeMap->GetNodeSitesByConstrain();
        // auto sites = voroPorTreeMap->GetLeafNodeSites();
        for (size_t i = 0; i < sites.size(); i++)
        {
            // sites[i]->Print();
            points.emplace_back(KiriPoint2(Vector2F(sites[i]->GetValue().x, sites[i]->GetValue().y) + offsetVec2, Vector3F(1.f, 0.f, 0.f)));
            auto poly = sites[i]->GetCellPolygon();
            if (poly != NULL)
            {
                poly->ComputeVoroSitesList();
                auto list = poly->GetVoroSitesList();
                // list->PrintVertexList();

                auto node = list->GetHead();
                do
                {
                    auto start = Vector2F(node->value) + offsetVec2;
                    node = node->next;
                    auto end = Vector2F(node->value) + offsetVec2;
                    lines.emplace_back(KiriLine2(start, end));
                } while (node != list->GetHead());
            }
        }

        scene->AddLines(lines);
        scene->AddParticles(points);

        renderer->DrawCanvas();
        renderer->SaveImages2File();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->ClearCanvas();
        scene->Clear();
    }
}

void UniParticleSampler()
{
    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    float width = 1000.f;
    float height = 1000.f;

    Vector<KiriPoint2> points;
    Vector<KiriLine2> lines;
    Vector<KiriCircle2> circles;

    KiriSDFPoly2D boundary;

    Vector<Vector2F> bunny2d;
    Vector<Vector2F> sbunny2d;
    size_t bunnyNum;
    // load_xy_file1(bunny2d, bunnyNum, "D:/project/Kiri2D/scripts/alphashape/test.xy");
    load_xy_file1(bunny2d, bunnyNum, "E:/PBCGLab/project/Kiri2D/scripts/alphashape/test.xy");

    for (size_t i = 0; i < bunny2d.size(); i++)
    {
        // auto newPos = bunny2d[i] * 1000.f + Vector2F(width / 2.f, height / 25.f);
        auto newPos = bunny2d[i];
        sbunny2d.emplace_back(newPos);
        boundary.Append(newPos);
    }

    BoundingBox2F bbox;
    for (size_t i = 0; i < bunnyNum; i++)
        bbox.merge(sbunny2d[i]);

    Vector<Vector2F> uniPoints;
    auto radius = 0.015f;
    auto lower = bbox.LowestPoint;
    auto higher = bbox.HighestPoint;
    auto wn = UInt(((higher - lower) / (radius * 2.f)).x);
    auto hn = UInt(((higher - lower) / (radius * 2.f)).y);
    for (size_t i = 0; i <= wn; i++)
    {

        auto lineh = KiriLine2(lower + Vector2F(0.f, i * 2.f * radius), lower + Vector2F(higher.x - lower.x, i * 2.f * radius));
        lineh.thick = 1.3f;
        // lineh.col = Vector3F(0.f, 0.f, 0.f);
        auto linev = KiriLine2(lower + Vector2F(i * 2.f * radius, 0.f), lower + Vector2F(i * 2.f * radius, higher.y - lower.y + radius));
        linev.thick = 1.3f;
        // linev.col = Vector3F(0.f, 0.f, 0.f);
        lines.emplace_back(lineh);
        lines.emplace_back(linev);
        for (size_t j = 0; j < hn; j++)
        {
            auto pos = lower + Vector2F(radius, radius) + Vector2F(i, j) * (radius * 2.f);
            uniPoints.emplace_back(pos);

            if (boundary.FindRegion(pos) < 0.f)
                circles.emplace_back(KiriCircle2(pos, Vector3F(100.f, 85.f, 134.f) / 255.f, radius));
        }
    }

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    // auto line = KiriLine2(start, end);
    // line.col = Vector3F(0.f, 0.f, 255.f);
    // lines.emplace_back(line);

    scene->AddLines(lines);
    scene->AddParticles(points);
    scene->AddCircles(circles);
    scene->AddObject(boundary);

    renderer->DrawCanvas();
    renderer->SaveImages2File();

    ExportSamplerData2CSVFile("uni_bunny", UInt2Str4Digit(0), circles);

    while (1)
    {
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
    }
}

void VoroPorosityOptimizeScaleExample()
{
    // scene renderer config
    float windowheight = 5000.f;
    float windowwidth = 5000.f;

    // voronoi
    float width = 3000.f;
    float height = 3000.f;
    // auto offsetVec2 = Vector2F((windowwidth - width) / 2.f, (windowheight - height) / 2.f);
    auto offsetVec2 = Vector2F(500.f, 500.f);

    // teapot 500 1500
    // cow 500 1000
    // woody 800 500
    // armadillo 800 400

    // iter
    auto maxIter = 4000;
    String boundaryFileName = "bunny";
    String filePath = String(RESOURCES_PATH) + "alpha_shapes/" + boundaryFileName + ".xy";

    Vector<Vector2F> bunny2d;
    size_t bunnyNum;
    load_xy_file1(bunny2d, bunnyNum, filePath.c_str());

    Vector<Vector2F> boundary;

    for (size_t i = 0; i < bunny2d.size(); i++)
    {
        auto newPos = bunny2d[i] * 4000.f + offsetVec2;
        boundary.emplace_back(newPos);
    }

    auto epsilon = 0.001f;
    auto opti = std::make_shared<KiriVoroPoroOpti>();
    opti->SetRootBoundary2(boundary);
    opti->GenExample(width, height);
    opti->SetMaxIterationNum(maxIter);

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    Vec_Float errorArray, porosityArray, radiusErrorArray;
    Vec_Float RMSEArray, RMSPEArray;
    Vector<Vector4F> lastMaxCircle;
    // auto minRadius = Huge<float>();
    // auto maxRadius = Tiny<float>();

    auto minRadius = 0.f;
    auto maxRadius = 300.f;
    for (size_t i = 0; i < maxIter; i++)
    {

        auto error = opti->ComputeIterate();
        auto cur_porosity = opti->GetMiniumPorosity();
        Vector<KiriPoint2> points;
        Vector<KiriLine2> lines;
        Vector<KiriCircle2> circles;
        // if (1)
        if ((i % 10 == 1) || (i == maxIter - 1))
        {
            auto sites = opti->GetLeafNodeSites();
            for (size_t i = 0; i < sites.size(); i++)
            {
                // sites[i]->Print();
                // auto p = Transform2Original(Vector2F(sites[i]->GetValue().x, sites[i]->GetValue().y) , height) + offsetVec2;
                auto p = Vector2F(sites[i]->GetValue().x, sites[i]->GetValue().y);
                points.emplace_back(KiriPoint2(p, Vector3F(1.f, 0.f, 0.f)));
                auto poly = sites[i]->GetCellPolygon();
                if (poly != NULL)
                {
                    poly->ComputeVoroSitesList();
                    auto list = poly->GetVoroSitesList();
                    // list->PrintVertexList();

                    auto node = list->GetHead();
                    do
                    {
                        // auto start = Transform2Original(Vector2F(node->value) * 10.f, height) + offsetVec2;
                        auto start = Vector2F(node->value);
                        node = node->next;
                        // auto end = Transform2Original(Vector2F(node->value) * 10.f, height) + offsetVec2;
                        auto end = Vector2F(node->value);
                        auto line = KiriLine2(start, end);
                        line.thick = 5.f;
                        lines.emplace_back(line);
                    } while (node != list->GetHead());
                }
            }

            auto sites_data = opti->GetVoronoiSitesData();
            ExportVoronoiData2CSVFile(boundaryFileName, UInt2Str4Digit(i), sites_data);

            auto maxIC = opti->GetMICBySSkel();
            lastMaxCircle = maxIC;

            auto porosity = opti->GetMiniumPorosity();
            porosityArray.emplace_back(porosity);

            auto radiusError = 0.f;
            auto radiusErrorPercent = 0.f;
            Vec_Float predictRadiusArray, realRadiusArray;
            for (size_t i = 0; i < maxIC.size(); i++)
            {
                // auto maxCir2 = KiriCircle2(Transform2Original(Vector2F(maxIC[i].x, maxIC[i].y) * 10.f, height) + offsetVec2, Vector3F(1.f, 0.f, 0.f), maxIC[i].z * 10.f);
                auto maxCir2 = KiriCircle2(Vector2F(maxIC[i].x, maxIC[i].y), Vector3F(1.f, 0.f, 0.f), maxIC[i].z);

                predictRadiusArray.emplace_back(maxIC[i].z);
                realRadiusArray.emplace_back(maxIC[i].w);

                circles.emplace_back(maxCir2);
                radiusError += std::abs(maxIC[i].z - maxIC[i].w);

                // minRadius = std::min(minRadius, maxIC[i].z);
                // maxRadius = std::max(maxRadius, maxIC[i].z);
            }

            errorArray.emplace_back(error / maxIC.size());
            radiusErrorArray.emplace_back(radiusError / maxIC.size());

            auto cur_rmse = ComputeRMSE(predictRadiusArray, realRadiusArray);
            auto cur_rmsp = ComputeRMSPE(predictRadiusArray, realRadiusArray);
            RMSEArray.emplace_back(cur_rmse);
            RMSPEArray.emplace_back(cur_rmsp);

            KIRI_LOG_DEBUG("iterate idx:{0}, porosity={1}, error={2}, rmse={3},rmsp={4}",
                           i, porosity, error / maxIC.size(),
                           cur_rmse, cur_rmsp);

            // re-color
            for (size_t i = 0; i < maxIC.size(); i++)
            {
                auto rad = (maxIC[i].z - minRadius) / (maxRadius - minRadius);
                const tinycolormap::Color color = tinycolormap::GetColor(rad, tinycolormap::ColormapType::Plasma);
                circles[i].col = Vector3F(color.r(), color.g(), color.b());
            }

            scene->AddParticles(points);
            scene->AddCircles(circles);
            scene->AddLines(lines);

            renderer->DrawCanvas();
            renderer->SaveImages2File();

            renderer->ClearCanvas();
            scene->Clear();
        }

        // if ((i % 1000 == 1) || (i == maxIter - 1))
        // {
        //     //ExportSamplerData2CSVFile("bunny", UInt2Str4Digit(i), circles);

        //     ExportSamplerData2CSVFile(boundaryFileName, UInt2Str4Digit(i), circles);
        //     ExportPoroityData2CSVFile(boundaryFileName, UInt2Str4Digit(i), errorArray, porosityArray, radiusErrorArray, lastMaxCircle);
        //     ExportEvaluationData2CSVFile(boundaryFileName, UInt2Str4Digit(i), RMSEArray, RMSPEArray);
        // }
    }
}

#include <kiri2d/straight_skeleton/sskel_slav.h>
void StraightSkeletonExample1()
{
    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    float width = 1000.f;
    float height = 1000.f;
    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    Vector2F offset = Vector2F(width, height) / 12.f;

    KiriSDFPoly2D boundary;
    Vec_Vec2F polygon;

    // polygon.emplace_back(Vector2F(100, 50));
    // polygon.emplace_back(Vector2F(150, 150));
    // polygon.emplace_back(Vector2F(50, 100));
    // polygon.emplace_back(Vector2F(50, 350));
    // polygon.emplace_back(Vector2F(350, 350));
    // polygon.emplace_back(Vector2F(350, 100));
    // polygon.emplace_back(Vector2F(250, 150));
    // polygon.emplace_back(Vector2F(300, 50));

    // polygon.emplace_back(Vector2F(100, 50));
    // polygon.emplace_back(Vector2F(150, 150));
    // polygon.emplace_back(Vector2F(50, 100));
    // polygon.emplace_back(Vector2F(50, 250));
    // polygon.emplace_back(Vector2F(150, 250));
    // polygon.emplace_back(Vector2F(50, 350));
    // polygon.emplace_back(Vector2F(350, 350));
    // polygon.emplace_back(Vector2F(350, 100));
    // polygon.emplace_back(Vector2F(250, 150));
    // polygon.emplace_back(Vector2F(300, 50));

    polygon.emplace_back(Vector2F(208, 131));
    polygon.emplace_back(Vector2F(213, 142));
    polygon.emplace_back(Vector2F(168, 141));
    polygon.emplace_back(Vector2F(260, 168));
    polygon.emplace_back(Vector2F(246, 149));
    polygon.emplace_back(Vector2F(277, 142));
    polygon.emplace_back(Vector2F(271, 163));
    polygon.emplace_back(Vector2F(302, 180));
    polygon.emplace_back(Vector2F(268, 173));
    polygon.emplace_back(Vector2F(305, 196));
    polygon.emplace_back(Vector2F(319, 225));
    polygon.emplace_back(Vector2F(367, 214));
    polygon.emplace_back(Vector2F(423, 169));
    polygon.emplace_back(Vector2F(471, 160));
    polygon.emplace_back(Vector2F(540, 208));
    polygon.emplace_back(Vector2F(588, 268));
    polygon.emplace_back(Vector2F(616, 270));
    polygon.emplace_back(Vector2F(644, 308));
    polygon.emplace_back(Vector2F(630, 446));
    polygon.emplace_back(Vector2F(647, 472));
    polygon.emplace_back(Vector2F(641, 459));
    polygon.emplace_back(Vector2F(656, 467));
    polygon.emplace_back(Vector2F(660, 450));
    polygon.emplace_back(Vector2F(646, 423));
    polygon.emplace_back(Vector2F(687, 447));
    polygon.emplace_back(Vector2F(666, 495));
    polygon.emplace_back(Vector2F(651, 495));
    polygon.emplace_back(Vector2F(711, 580));
    polygon.emplace_back(Vector2F(728, 584));
    polygon.emplace_back(Vector2F(714, 557));
    polygon.emplace_back(Vector2F(746, 560));
    polygon.emplace_back(Vector2F(735, 569));
    polygon.emplace_back(Vector2F(744, 617));
    polygon.emplace_back(Vector2F(769, 594));
    polygon.emplace_back(Vector2F(753, 624));
    polygon.emplace_back(Vector2F(771, 628));
    polygon.emplace_back(Vector2F(793, 700));
    polygon.emplace_back(Vector2F(842, 708));
    polygon.emplace_back(Vector2F(871, 759));
    polygon.emplace_back(Vector2F(902, 780));
    polygon.emplace_back(Vector2F(891, 788));
    polygon.emplace_back(Vector2F(871, 773));
    polygon.emplace_back(Vector2F(887, 799));
    polygon.emplace_back(Vector2F(947, 774));
    polygon.emplace_back(Vector2F(964, 782));
    polygon.emplace_back(Vector2F(978, 689));
    polygon.emplace_back(Vector2F(985, 678));
    polygon.emplace_back(Vector2F(990, 695));
    polygon.emplace_back(Vector2F(984, 555));
    polygon.emplace_back(Vector2F(868, 338));
    polygon.emplace_back(Vector2F(854, 294));
    polygon.emplace_back(Vector2F(869, 316));
    polygon.emplace_back(Vector2F(887, 314));
    polygon.emplace_back(Vector2F(892, 366));
    polygon.emplace_back(Vector2F(895, 322));
    polygon.emplace_back(Vector2F(805, 196));
    polygon.emplace_back(Vector2F(747, 61));
    polygon.emplace_back(Vector2F(759, 59));
    polygon.emplace_back(Vector2F(753, 43));
    polygon.emplace_back(Vector2F(691, 33));
    polygon.emplace_back(Vector2F(683, 98));
    polygon.emplace_back(Vector2F(661, 72));
    polygon.emplace_back(Vector2F(355, 83));
    polygon.emplace_back(Vector2F(333, 46));
    polygon.emplace_back(Vector2F(35, 70));
    polygon.emplace_back(Vector2F(70, 144));
    polygon.emplace_back(Vector2F(50, 165));
    polygon.emplace_back(Vector2F(77, 154));
    polygon.emplace_back(Vector2F(87, 125));
    polygon.emplace_back(Vector2F(99, 139));
    polygon.emplace_back(Vector2F(106, 118));
    polygon.emplace_back(Vector2F(122, 139));
    polygon.emplace_back(Vector2F(89, 152));
    polygon.emplace_back(Vector2F(169, 124));

    auto sskel_convex = std::make_shared<KIRI2D::SSKEL::SSkelSLAV>(polygon);

    Vector<KiriLine2> lines;
    auto skeletons = sskel_convex->GetSkeletons();
    for (size_t i = 0; i < skeletons.size(); i++)
    {
        auto [intersect, sinks] = skeletons[i];
        for (size_t j = 0; j < sinks.size(); j++)
        {
            auto line = KiriLine2(intersect + offset, sinks[j] + offset);
            line.thick = 1;
            lines.emplace_back(line);
        }
    }

    for (size_t i = 0; i < polygon.size(); i++)
    {
        boundary.Append(polygon[i] + offset);
    }

    scene->AddLines(lines);
    scene->AddObject(boundary);

    renderer->DrawCanvas();
    renderer->SaveImages2File();
    while (1)
    {
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
    }
}

#include <kiri2d/voronoi/voro_split_test.h>
void LoadVoronoiExample()
{
    // scene renderer config
    float windowheight = 5000.f;
    float windowwidth = 5000.f;

    // voronoi
    float width = 3000.f;
    float height = 3000.f;
    auto offsetVec2 = Vector2F(500.f, 500.f);

    String boundaryFileName = "bunny";
    String filePath = String(RESOURCES_PATH) + "alpha_shapes/" + boundaryFileName + ".xy";

    Vector<Vector2F> bunny2d;
    size_t bunnyNum;
    load_xy_file1(bunny2d, bunnyNum, filePath.c_str());

    Vector<Vector2F> boundary;
    auto boundaryPoly = std::make_shared<KiriVoroCellPolygon2>();

    for (size_t i = 0; i < bunny2d.size(); i++)
    {
        auto newPos = bunny2d[i] * 4000.f + offsetVec2;
        boundary.emplace_back(newPos);
        boundaryPoly->AddPolygonVertex2(newPos);
    }

    auto voro_data = LoadCSVFile2VoronoiSites("bunny_sites_3999.csv");

    auto spliter = std::make_shared<KiriVoroSplit>();

    for (size_t i = 0; i < voro_data.size(); i++)
        spliter->AddSite(std::make_shared<KiriVoroGroupSite>(voro_data[i]));

    spliter->SetBoundaryPolygon2(boundaryPoly);
    spliter->Init();

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    Vector<KiriPoint2> points;
    Vector<KiriLine2> lines;
    Vector<KiriCircle2> circles;

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<float> dist(0.f, 1.f);

    auto sites = spliter->GetVoroSites();
    for (size_t i = 0; i < sites.size(); i++)
    {
        auto site_i = std::dynamic_pointer_cast<KiriVoroGroupSite>(sites[i]);
        auto p = Vector2F(site_i->GetValue().x, site_i->GetValue().y);

        auto point = KiriPoint2(p, Vector3F(1.f, 1.f, 1.f));
        if (site_i->GetIsGroup())
            point.col = site_i->GetGroupColor();

        points.emplace_back(point);

        auto poly = site_i->GetCellPolygon();
        if (poly != NULL)
        {
            poly->ComputeVoroSitesList();
            auto list = poly->GetVoroSitesList();
            auto node = list->GetHead();
            do
            {
                auto start = Vector2F(node->value);

                node = node->next;
                auto end = Vector2F(node->value);
                auto line = KiriLine2(start, end);
                line.thick = 5.f;

                lines.emplace_back(line);
            } while (node != list->GetHead());

            if (poly->GetSkeletons().empty())
                poly->ComputeSSkel1998Convex();

            // auto mic = poly->ComputeMICByStraightSkeleton();

            auto mic = poly->ComputeMICByStraightSkeletonTest();
            auto gcolor = Vector3F(dist(rndEngine), dist(rndEngine), dist(rndEngine));
            // KIRI_LOG_DEBUG("g size={0}", mic.size());

            for (size_t idx = 0; idx < mic.size(); idx++)
            {
                auto maxCir2 = KiriCircle2(Vector2F(mic[idx].x, mic[idx].y), Vector3F(1.f, 0.f, 1.f), mic[idx].z);
                // maxCir2.fill = false;
                maxCir2.col = gcolor;
                circles.emplace_back(maxCir2);
            }

            // auto maxCir2 = KiriCircle2(Vector2F(mic.x, mic.y), Vector3F(1.f, 0.f, 1.f), mic.z);
            // if (site_i->GetIsGroup())
            //     maxCir2.col = site_i->GetGroupColor();

            // circles.emplace_back(maxCir2);
        }
    }

    // scene->AddParticles(points);
    scene->AddLines(lines);
    scene->AddCircles(circles);

    renderer->DrawCanvas();
    renderer->SaveImages2File();

    renderer->ClearCanvas();
    scene->Clear();
}

#include <kiri2d/sampling/poisson_disk_sampling.h>
void UniPoissonDiskSampler()
{
    UInt counter = 0;

    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    KiriSDFPoly2D boundary;

    Vector<Vector2F> bunny2d;
    Vector<Vector2F> sbunny2d;
    size_t bunnyNum;
    String boundaryFileName = "bunny";
    String filePath = String(RESOURCES_PATH) + "alpha_shapes/" + boundaryFileName + ".xy";
    load_xy_file1(bunny2d, bunnyNum, filePath.c_str());
    for (size_t i = 0; i < bunny2d.size(); i++)
    {
        // auto newPos = bunny2d[i] * 1000.f + Vector2F(width / 2.f, height / 25.f);
        auto newPos = bunny2d[i] * 1000.f;
        sbunny2d.emplace_back(newPos);
        boundary.Append(newPos);
    }

    BoundingBox2F bbox;
    for (size_t i = 0; i < bunnyNum; i++)
        bbox.merge(sbunny2d[i]);

    // Vector2F offset = Vector2F(windowwidth - bbox.width(), windowheight - bbox.height()) / 2.f;
    Vector2F offset = Vector2F(500.f, 0.f);
    boundary.SetOffset(offset);

    Vector<KiriPoint2> points;
    Vector<KiriLine2> lines;
    Vector<KiriCircle2> circles;

    // uni
    auto radius = 10.f;

    // multi
    auto max_radius = 150.f;
    std::vector<float> radius_range;
    radius_range.push_back(20.f);
    radius_range.push_back(30.f);
    radius_range.push_back(80.f);
    radius_range.push_back(max_radius);

    std::vector<float> radius_prob;
    radius_prob.push_back(0.5f);
    radius_prob.push_back(0.4f);
    radius_prob.push_back(0.1f);

    auto sampler = std::make_shared<KiriPoissonDiskSampling2D>();
    sampler->InitUniSampler(radius, Vector2F(bbox.width(), bbox.height()));
    // sampler->InitMultiRadiiSampler(20.f, max_radius, Vector2F(bbox.width(), bbox.height()));

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);
    while (1)
    {
        auto uni_points = sampler->UniSampling(radius, Vector2F(bbox.width(), bbox.height()));
        // auto mr_points = sampler->MultiRadiiSampling(radius_range, radius_prob, Vector2F(bbox.width(), bbox.height()), 3600);

        if (++counter % 10 == 0)
        {
            circles.clear();
            lines.clear();
            points.clear();

            for (auto i = 0; i < uni_points.size(); i++)
            {
                auto pos = uni_points[i];
                if (boundary.FindRegion(pos) < 0.f)
                    circles.emplace_back(KiriCircle2(pos + offset, Vector3F(100.f, 85.f, 134.f) / 255.f, radius / 2.f));
            }

            // for (auto i = 0; i < mr_points.size(); i++)
            // {
            //     auto pos = Vector2F(mr_points[i].x, mr_points[i].y);
            //     if (boundary.FindRegion(pos) < 0.f)
            //         circles.emplace_back(KiriCircle2(pos + offset, Vector3F(100.f, 85.f, 134.f) / 255.f, mr_points[i].z / 2.f));
            // }

            scene->AddLines(lines);
            scene->AddParticles(points);
            scene->AddCircles(circles);
            scene->AddObject(boundary);

            renderer->DrawCanvas();
            renderer->SaveImages2File();

            // KIRI_LOG_DEBUG("sampling iterate idx:{0}", counter);
            // ExportSamplerData2CSVFile(boundaryFileName, UInt2Str4Digit(counter), circles);

            cv::imshow("KIRI2D", renderer->GetCanvas());
            cv::waitKey(5);

            renderer->ClearCanvas();
            scene->Clear();
        }
    }
}

int main1()
{
    KIRI::KiriLog::Init();
    // VoronoiExample();
    //  VoronoiExample1();
    //   VoronoiExample2();

    // LloydRelaxationExample();

    // GenRndTreemap();

    // NOCAJ12Example();
    // NOCAJ12Example1();

    // VoroTestExample();

    // VoroPorosityTreemapOptiExample();

    // UniParticleSampler();

    // StraightSkeletonExample1();

    // VoroPorosityOptimizeConvexExample();
    // VoroPorosityOptimizeBunnyExample();

    // VoroPorosityOptimizeScaleExample();

    UniPoissonDiskSampler();

    // LoadVoronoiExample();

    return 0;
}
