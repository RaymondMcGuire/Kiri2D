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

bool InBound(Vector3F v, float size)
{
    if (v.x < -size || v.x > size)
        return false;
    if (v.y < -size || v.y > size)
        return false;
    if (v.z < -size || v.z > size)
        return false;

    return true;
}

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
    return Vector2F(v.x, v.y);
    // return Vector2F(v.x, h - v.y);
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

Vector<Vector3F> LoadCSVFile2VoronoiSitesV3(const String fileName)
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

Vector<Vector4F> LoadCSVFile2VoronoiSites(const String fileName)
{
    String filePath = String(EXPORT_PATH) + "csv/" + fileName;

    char delimiter = ',';

    auto file_contents = readFileIntoString(filePath);
    std::istringstream sstream(file_contents);
    Vector<String> row;
    String record;

    Vector<Vector4F> voro_sites;

    while (std::getline(sstream, record))
    {
        std::istringstream line(record);
        while (std::getline(line, record, delimiter))
            row.push_back(record);

        // KIRI_LOG_DEBUG("{0},{1},{2}", std::stof(row[0]), std::stof(row[1]), std::stof(row[2]));

        voro_sites.emplace_back(Vector4F(
            std::stof(row[0]),
            std::stof(row[1]),
            std::stof(row[2]),
            std::stof(row[3])));

        row.clear();
    }

    return voro_sites;
}

struct NSDataStruct
{
    Vector<Vector2F> pos;
    Vector<float> rad;
    Vector3F col;
};

void ExportNSData2CSVFile(const String fileName, const Vector<NSDataStruct> &ns)
{
    String filePath = String(EXPORT_PATH) + "csv/" + fileName;
    std::fstream file;
    file.open(filePath.c_str(), std::ios_base::out);
    for (int i = 0; i < ns.size(); i++)
    {
        String pos_data = "";
        String rad_data = "";
        String col_data = "";
        auto n = ns[i];
        for (size_t pidx = 0; pidx < n.pos.size(); pidx++)
        {
            if (pidx == n.pos.size() - 1)
            {
                pos_data += std::to_string(n.pos[pidx].x) + ":" + std::to_string(n.pos[pidx].y);
                rad_data += std::to_string(n.rad[pidx]);
            }

            else
            {
                pos_data += std::to_string(n.pos[pidx].x) + ":" + std::to_string(n.pos[pidx].y) + ";";
                rad_data += std::to_string(n.rad[pidx]) + ";";
            }
        }

        col_data += std::to_string(n.col.x) + ":" + std::to_string(n.col.y) + ":" + std::to_string(n.col.z);
        file << pos_data << "," << rad_data << "," << col_data << std::endl;
    }

    file.close();
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

void ExportVoronoiData2CSVFile(const String fileName, const String idx, const Vector<Vector4F> &sites)
{
    String voronoiFile = String(EXPORT_PATH) + "csv/" + fileName + "_sites_" + idx + ".csv";
    std::fstream vfile;
    vfile.open(voronoiFile.c_str(), std::ios_base::out);
    vfile << "sitex,sitey,weight,radius"
          << std::endl;
    for (int i = 0; i < sites.size(); i++)
        vfile << sites[i].x << "," << sites[i].y << "," << sites[i].z << "," << sites[i].w << std::endl;

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
    // auto bp1 = Vector2F(0, 0);
    // auto bp2 = Vector2F(width, 0);
    // auto bp3 = Vector2F(width, height);
    // auto bp4 = Vector2F(0, height);

    auto bp1 = Vector2F(-200, -200);
    auto bp2 = Vector2F(-200, 200);
    auto bp3 = Vector2F(200, 200);
    auto bp4 = Vector2F(200, -200);

    offsetVec2 = Vector2F(windowwidth, windowheight) / 2.f;

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

    // auto cnt = 0, maxcnt = 100;
    // while (cnt < maxcnt)
    // {
    //     auto sitePos2 = Vector2F(dist(rndEngine) * width, dist(rndEngine) * height);
    //     if (boundary.FindRegion(sitePos2) < 0.f)
    //     {
    //         pd->AddPowerSite(sitePos2, 0.f);
    //         cnt++;
    //     }
    // }

    pd->AddPowerSite(Vector2F(-128.66962f, 103.96347f), 10000.f);
    pd->AddPowerSite(Vector2F(-33.590614f, -164.7944f), 0.f);
    pd->AddPowerSite(Vector2F(71.96047f, -43.68138f), 0.f);
    pd->AddPowerSite(Vector2F(-41.80409f, -109.197426f), 0.f);
    pd->AddPowerSite(Vector2F(-138.16716f, -33.35321f), 0.f);
    pd->AddPowerSite(Vector2F(187.9643f, 128.24672f), 0.f);
    pd->AddPowerSite(Vector2F(138.98251f, 87.095856f), 0.f);
    pd->AddPowerSite(Vector2F(-65.154396f, -116.48123f), 0.f);
    pd->AddPowerSite(Vector2F(-81.71843f, 29.558516f), 0.f);
    pd->AddPowerSite(Vector2F(105.82991f, 65.12127f), 0.f);

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

        // for (size_t i = 0; i < del_tri.size(); i++)
        // {
        //     auto start = Transform2Original(Vector2F(del_tri[i].x, del_tri[i].y), height) + offsetVec2;
        //     auto end = Transform2Original(Vector2F(del_tri[i].z, del_tri[i].w), height) + offsetVec2;
        //     auto line = KiriLine2(start, end);
        //     line.col = Vector3F(0, 0, 0) / 255.f;
        //     line.thick = 2.f;
        //     lines.emplace_back(line);
        // }

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
            // auto site = std::make_shared<KiriVoroSite>(sitePos2.x, sitePos2.y, std::numeric_limits<float>::epsilon(), radius);
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

    auto voro_data = LoadCSVFile2VoronoiSitesV3("bunny_sites_3999.csv");

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
    Vector<NSDataStruct> ns_data;

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

            NSDataStruct ns;
            for (size_t idx = 0; idx < mic.size(); idx++)
            {
                if (mic[idx].z == 0.f)
                    continue;
                // auto maxCir2 = KiriCircle2(Vector2F(mic[idx].x, mic[idx].y), Vector3F(1.f, 0.f, 1.f), mic[idx].z);
                // // maxCir2.fill = false;
                // maxCir2.col = gcolor;
                // circles.emplace_back(maxCir2);

                ns.pos.emplace_back(Vector2F(mic[idx].x, mic[idx].y));
                ns.rad.emplace_back(mic[idx].z);
            }
            ns.col = gcolor;
            ns_data.emplace_back(ns);

            // auto maxCir2 = KiriCircle2(Vector2F(mic.x, mic.y), Vector3F(1.f, 0.f, 1.f), mic.z);
            // if (site_i->GetIsGroup())
            //     maxCir2.col = site_i->GetGroupColor();

            // circles.emplace_back(maxCir2);
        }
    }

    for (size_t i = 0; i < ns_data.size(); i++)
    {
        auto ns = ns_data[i];

        for (size_t j = 0; j < ns.pos.size(); j++)
        {
            auto maxCir2 = KiriCircle2(ns.pos[j], Vector3F(1.f, 0.f, 1.f), ns.rad[j]);
            // maxCir2.fill = false;
            maxCir2.col = ns.col;
            circles.emplace_back(maxCir2);
        }
    }

    // scene->AddParticles(points);
    scene->AddLines(lines);
    scene->AddCircles(circles);

    renderer->DrawCanvas();
    renderer->SaveImages2File();

    renderer->ClearCanvas();
    scene->Clear();

    ExportNSData2CSVFile("ns_data.csv", ns_data);
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

#ifdef KIRI_WINDOWS

#include <kiri_pbs_cuda/emitter/cuda_volume_emitter.cuh>

std::vector<std::string> split_str1(const std::string &s, char delim)
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

std::vector<NSPack> LoadCSVFile2NSPack1(const String fileName)
{
    String filePath = String(EXPORT_PATH) + "csv/" + fileName;

    char delimiter = ',';

    auto file_contents = readFileIntoString(filePath);
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
        auto pos_data = split_str1(row[0], ';');
        auto rad_data = split_str1(row[1], ';');
        auto col_data = split_str1(row[2], ':');

        for (size_t i = 0; i < pos_data.size(); i++)
        {
            auto pos_str = split_str1(pos_data[i], ':');
            ns_pack.AppendSubParticles(make_float2(std::stof(pos_str[0]), std::stof(pos_str[1])), std::stof(rad_data[i]));
        }
        ns_pack.SetColor(make_float3(std::stof(col_data[0]), std::stof(col_data[1]), std::stof(col_data[2])));
        ns_packs.emplace_back(ns_pack);

        row.clear();
    }

    return ns_packs;
}

void DebugNSParticles()
{

    // scene renderer config
    float windowheight = 5000.f;
    float windowwidth = 5000.f;

    auto emitter = std::make_shared<CudaVolumeEmitter>();

    DemNSBoxVolumeData data;
    // std::vector<NSPackPtr> pack_types;
    // pack_types.emplace_back(std::make_shared<NSPack>(MSM_L2, 10.f));
    // pack_types.emplace_back(std::make_shared<NSPack>(MSM_L3, 10.f));

    // emitter->BuildRndNSDemBoxVolume(data, make_float2(500.f, 200.f),
    //                                 make_float2(1500.f, 800.f), 10.f, 0.f,
    //                                 10000,
    //                                 pack_types);

    auto ns_packs = LoadCSVFile2NSPack1("ns_data.csv");
    emitter->BuildNsDemVolume(data, ns_packs);

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);
    std::vector<KiriCircle2> circles;

    circles.clear();

    for (size_t i = 0; i < data.sphere_data.size(); i++)
    {
        auto sp = data.sphere_data[i];
        circles.emplace_back(KiriCircle2(Vector2F(sp.center.x, sp.center.y), Vector3F(sp.color.x, sp.color.y, sp.color.z), sp.radius));
    }

    scene->AddCircles(circles);

    renderer->DrawCanvas();
    renderer->SaveImages2File();

    // KIRI_LOG_DEBUG("sampling iterate idx:{0}", counter);
    // ExportSamplerData2CSVFile(boundaryFileName, UInt2Str4Digit(counter), circles);

    // cv::imshow("KIRI2D", renderer->GetCanvas());
    // cv::waitKey(5);

    renderer->ClearCanvas();
    scene->Clear();
}

#endif

#include <kiri2d/voronoi/voro_ns_optimize.h>
void VoronoiNSOptimize()
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

    std::vector<Vector2F> bunny2d;
    size_t bunnyNum;
    load_xy_file1(bunny2d, bunnyNum, filePath.c_str());

    std::vector<Vector2F> boundary;
    auto boundaryPoly = std::make_shared<KiriVoroCellPolygon2>();

    for (size_t i = 0; i < bunny2d.size(); i++)
    {
        auto newPos = bunny2d[i] * 4000.f + offsetVec2;
        boundary.emplace_back(newPos);
        boundaryPoly->AddPolygonVertex2(newPos);
    }

    auto voro_data = LoadCSVFile2VoronoiSites("bunny_sites_3821.csv");
    auto ns_opti = std::make_shared<KiriVoroNSOptimize>();

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<float> dist(0.f, 1.f);

    auto avg_radius = 0.f;
    for (size_t i = 0; i < voro_data.size(); i++)
    {
        avg_radius += voro_data[i].w;
    }
    avg_radius /= voro_data.size();

    for (size_t i = 0; i < voro_data.size(); i++)
    {
        // auto group_site = std::make_shared<KiriVoroGroupSite>(voro_data[i]);
        auto group_site = std::make_shared<KiriVoroGroupSite>(Vector4F(voro_data[i].x, voro_data[i].y, voro_data[i].z, avg_radius));
        group_site->SetGroupId(i);
        group_site->SetGroupColor(Vector3F(dist(rndEngine), dist(rndEngine), dist(rndEngine)));
        ns_opti->AddSite(group_site);
    }

    ns_opti->SetBoundaryPolygon2(boundaryPoly);
    ns_opti->Init();

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    std::vector<KiriPoint2> points;

    std::vector<KiriLine2> lines, glines;
    std::vector<KiriCircle2> circles, gcircles;

    UInt iter_num = 0;
    while (1)
    {
        lines.clear();
        circles.clear();

        glines.clear();
        gcircles.clear();

        auto sites = ns_opti->GetVoroSites();
        // KIRI_LOG_DEBUG("current num={0}", sites.size());
        for (size_t i = 0; i < sites.size(); i++)
        {
            auto site_i = std::dynamic_pointer_cast<KiriVoroGroupSite>(sites[i]);
            auto p = Vector2F(site_i->GetValue().x, site_i->GetValue().y);

            auto point = KiriPoint2(p, Vector3F(1.f, 1.f, 1.f));
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

                    // if (sites[i]->GetIsFrozen())
                    //     glines.emplace_back(line);

                } while (node != list->GetHead());

                if (poly->GetSkeletons().empty())
                    poly->ComputeSSkel1998Convex();

                auto mic = poly->ComputeMICByStraightSkeletonTest();
                for (size_t idx = 0; idx < mic.size(); idx++)
                {
                    auto maxCir2 = KiriCircle2(Vector2F(mic[idx].x, mic[idx].y), Vector3F(1.f, 0.f, 1.f), mic[idx].z);
                    // maxCir2.fill = false;
                    maxCir2.col = site_i->GetGroupColor();
                    circles.emplace_back(maxCir2);

                    // if (sites[i]->GetIsFrozen())
                    //     gcircles.emplace_back(maxCir2);
                }
            }
        }

        auto union_polygons = ns_opti->GetFrozenPolygon();
        for (size_t i = 0; i < union_polygons.size(); i++)
        {
            auto poly_i = union_polygons[i];
            if (poly_i != nullptr)
            {
                poly_i->ComputeVoroSitesList();
                auto list = poly_i->GetVoroSitesList();
                auto node = list->GetHead();
                do
                {
                    auto start = Vector2F(node->value);

                    node = node->next;
                    auto end = Vector2F(node->value);
                    auto line = KiriLine2(start, end);
                    line.thick = 5.f;
                    glines.emplace_back(line);

                } while (node != list->GetHead());

                if (poly_i->GetSkeletons().empty())
                    poly_i->ComputeSSkel1998Convex();

                auto mic = poly_i->ComputeMICByStraightSkeletonTest();
                for (size_t idx = 0; idx < mic.size(); idx++)
                {
                    auto maxCir2 = KiriCircle2(Vector2F(mic[idx].x, mic[idx].y), Vector3F(1.f, 0.f, 1.f), mic[idx].z);
                    // maxCir2.fill = false;
                    maxCir2.col = poly_i->GetColor();
                    gcircles.emplace_back(maxCir2);
                }
            }
        }

        // scene->AddParticles(points);
        scene->AddLines(lines);
        scene->AddCircles(circles);

        renderer->DrawCanvas();
        renderer->SaveImages2File();

        renderer->ClearCanvas();
        scene->Clear();

        // debug g
        // scene->AddLines(glines);
        scene->AddCircles(gcircles);

        renderer->DrawCanvas();
        renderer->SaveImages2FileWithPrefix("group");

        renderer->ClearCanvas();
        scene->Clear();

        auto error = ns_opti->ComputeIterate();
        KIRI_LOG_DEBUG("iter={0}, error={1}", iter_num++, error);
    }
}

#include <kiri2d/bop12/booleanop.h>
// #include <kiri2d/poly/PolygonClipping.h>
void TestPolygonUnion()
{
    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    Vector2F offset(500.f);

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    cbop::Polygon subj, clip;

    cbop::Contour vert1, vert2;
    vert1.add(cbop::Point_2(10.0, 10.0));
    vert1.add(cbop::Point_2(10.0, 100.0));
    vert1.add(cbop::Point_2(100.0, 100.0));
    vert1.add(cbop::Point_2(100.0, 10.0));
    subj.push_back(vert1);

    vert2.add(cbop::Point_2(20.0, 50.0));
    vert2.add(cbop::Point_2(20.0, 150.0));
    vert2.add(cbop::Point_2(90.0, 150.0));
    vert2.add(cbop::Point_2(90.0, 50.0));
    clip.push_back(vert2);

    cbop::BooleanOpType op = cbop::INTERSECTION;

    cbop::Polygon result;
    cbop::compute(subj, clip, result, op);
    auto p = result.getContours()[0].getPoints();

    std::vector<KiriLine2> precompute_lines;

    for (size_t j = 0; j < p.size(); j++)
    {
        precompute_lines.emplace_back(KiriLine2(Vector2F(p[j].x(), p[j].y()) + offset, Vector2F(p[(j + 1) % p.size()].x(), p[(j + 1) % p.size()].y()) + offset));
    }

    while (1)
    {
        std::vector<KiriLine2> lines;

        // for (auto i = 0; i < vertices1.size(); ++i)
        // {
        //     lines.emplace_back(KiriLine2(Vector2F(vertices1[i].x_,vertices1[i].y_)+offset,Vector2F(vertices1[(i+1)%vertices1.size()].x_,vertices1[(i+1)%vertices1.size()].y_)+offset));
        //     lines.emplace_back(KiriLine2(Vector2F(vertices2[i].x_, vertices2[i].y_) + offset, Vector2F(vertices2[(i + 1) % vertices2.size()].x_, vertices2[(i + 1) % vertices2.size()].y_) + offset));
        // }

        for (auto i = 0; i < precompute_lines.size(); ++i)
        {
            lines.emplace_back(precompute_lines[i]);
        }

        scene->AddLines(lines);

        renderer->DrawCanvas();
        // renderer->SaveImages2File();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->ClearCanvas();
        scene->Clear();
    }
}
#include <kiri2d/hdv_toolkit/primitives/vertex2.h>
#include <kiri2d/hdv_toolkit/primitives/vertex3.h>
#include <kiri2d/hdv_toolkit/primitives/vertex4.h>
#include <kiri2d/hdv_toolkit/hull/convex_hull.h>
void QuickHullConvexHull2d()
{
    using namespace HDV;

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<float> dist(-1.f, 1.f);

    auto scale_size = 200.f;
    auto sampler_num = 1000;
    std::vector<Primitives::Vertex2Ptr> vet2;

    for (auto i = 0; i < sampler_num; i++)
    {
        auto x = dist(rndEngine) * scale_size;
        auto y = dist(rndEngine) * scale_size;

        auto v2 = std::make_shared<Primitives::Vertex2>(x, y, i);
        vet2.emplace_back(v2);

        // KIRI_LOG_DEBUG("vet2.emplace_back(std::make_shared<Primitives::Vertex2>({0}, {1}, {2}));", x, y, i);
    }

    auto cv2 = std::make_shared<Hull::ConvexHull2>();
    cv2->Generate(vet2);

    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    Vector2F offset = Vector2F(windowwidth, windowheight) / 2.f;

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    std::vector<KiriLine2> precompute_lines;

    auto simplexs = cv2->GetSortSimplexsList();
    for (size_t i = 0; i < simplexs.size(); i++)
    {
        // auto line = KiriLine2(Vector2F(res2[i]->Vertices[0]->X(), res2[i]->Vertices[0]->Y()) + offset, Vector2F(res2[i]->Vertices[1]->X(), res2[i]->Vertices[1]->Y()) + offset);
        auto line = KiriLine2(Vector2F(simplexs[i].x, simplexs[i].y) + offset, Vector2F(simplexs[i].z, simplexs[i].w) + offset);
        line.thick = 1.f;
        precompute_lines.emplace_back(line);
        // KIRI_LOG_DEBUG("from={0},{1}----to={2},{3}", simplexs[i].x, simplexs[i].y, simplexs[i].z, simplexs[i].w);
    }

    while (1)
    {
        cv2->Generate(vet2);

        std::vector<KiriLine2> lines;
        std::vector<KiriPoint2> points;

        for (size_t i = 0; i < vet2.size(); i++)
        {
            points.emplace_back(KiriPoint2(Vector2F(vet2[i]->X(), vet2[i]->Y()) + offset, Vector3F(1.f, 0.f, 0.f)));
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
        renderer->ClearCanvas();
        scene->Clear();
    }
}

#include <kiri2d/hdv_toolkit/delaunay/delaunay_triangulation2.h>
void QuickHullDelaunayTriangulation2d()
{
    using namespace HDV;

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<float> dist(-1.f, 1.f);

    auto scale_size = 200.f;
    auto sampler_num = 1000;
    std::vector<Primitives::Vertex2Ptr> vet2;

    // for (auto i = 0; i < sampler_num; i++)
    // {
    //     auto x = dist(rndEngine) * scale_size;
    //     auto y = dist(rndEngine) * scale_size;

    //     auto v2 = std::make_shared<Primitives::Vertex2>(x, y, i);
    //     vet2.emplace_back(v2);

    //     // KIRI_LOG_DEBUG("vet2.emplace_back(std::make_shared<Primitives::Vertex2>({0}, {1}, {2}));", x, y, i);
    // }

    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(-134.72092f, 133.11143f, 0));
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(-2.3260376f, -153.14186f, 1));
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(134.69754f, -133.12962f, 2));
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(-0.5413166f, -53.178318f, 3));
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(-134.27975f, 0.15768485f, 4));
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(134.6346f, 133.1898f, 5));
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(134.8886f, 0.08968643f, 6));
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(-137.44841f, -132.7984f, 7));
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(-0.06846472f, 52.867256f, 8));
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(-0.08238092f, 153.2265f, 9));

    // boundary
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(-400.f, -400.f, 10));
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(-400.f, 400.f, 11));
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(400.f, 400.f, 12));
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(400.f, -400.f, 13));

    auto dt2 = std::make_shared<Delaunay::DelaunayTriangulation2>();
    dt2->Generate(vet2);
    auto res2 = dt2->Cells;

    /*  for (size_t i = 0; i < vet2.size(); i++)
     {
         vet2[i]->ToString();
     }*/

    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    Vector2F offset = Vector2F(windowwidth, windowheight) / 2.f;

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    std::vector<KiriLine2> precompute_lines;

    for (size_t i = 0; i < res2.size(); i++)
    {
        auto simplex = res2[i]->mSimplex;
        auto line1 = KiriLine2(Vector2F(simplex->Vertices[0]->X(), simplex->Vertices[0]->Y()) + offset, Vector2F(simplex->Vertices[1]->X(), simplex->Vertices[1]->Y()) + offset);
        auto line2 = KiriLine2(Vector2F(simplex->Vertices[0]->X(), simplex->Vertices[0]->Y()) + offset, Vector2F(simplex->Vertices[2]->X(), simplex->Vertices[2]->Y()) + offset);
        auto line3 = KiriLine2(Vector2F(simplex->Vertices[1]->X(), simplex->Vertices[1]->Y()) + offset, Vector2F(simplex->Vertices[2]->X(), simplex->Vertices[2]->Y()) + offset);

        line1.thick = 1.f;
        line2.thick = 1.f;
        line3.thick = 1.f;
        precompute_lines.emplace_back(line1);
        precompute_lines.emplace_back(line2);
        precompute_lines.emplace_back(line3);
    }

    while (1)
    {
        std::vector<KiriLine2> lines;
        std::vector<KiriPoint2> points;

        for (size_t i = 0; i < vet2.size(); i++)
        {
            points.emplace_back(KiriPoint2(Vector2F(vet2[i]->X(), vet2[i]->Y()) + offset, Vector3F(1.f, 0.f, 0.f)));
        }

        for (auto i = 0; i < precompute_lines.size(); ++i)
        {
            lines.emplace_back(precompute_lines[i]);
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

#include <kiri2d/hdv_toolkit/voronoi/power_diagram.h>
void QuickHullVoronoi2d()
{
    using namespace HDV;

    auto pd2 = std::make_shared<Voronoi::PowerDiagram2D>();

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<float> dist(-1.f, 1.f);

    auto scale_size = 200.f;
    auto sampler_num = 100;

    for (auto i = 0; i < sampler_num; i++)
    {
        auto x = dist(rndEngine) * scale_size;
        auto y = dist(rndEngine) * scale_size;

        pd2->AddSite(std::make_shared<Voronoi::VoronoiSite2>(x, y, i));
    }

    // clip boundary
    auto BoundaryPolygon = std::make_shared<Voronoi::VoronoiCellPolygon<Primitives::Vertex2Ptr, Primitives::Vertex2>>();
    BoundaryPolygon->AddVert2(Vector2F(-scale_size, -scale_size));
    BoundaryPolygon->AddVert2(Vector2F(-scale_size, scale_size));
    BoundaryPolygon->AddVert2(Vector2F(scale_size, scale_size));
    BoundaryPolygon->AddVert2(Vector2F(scale_size, -scale_size));
    pd2->SetBoundaryPolygon(BoundaryPolygon);

    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    Vector2F offset = Vector2F(windowwidth, windowheight) / 2.f;

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    pd2->Compute();

    // auto linea = KiriLine2(Vector2F(-200.f, -200.f) + offset, Vector2F(-200.f, 200.f) + offset);
    // linea.thick = 1.f;
    // auto lineb = KiriLine2(Vector2F(-200.f, 200.f) + offset, Vector2F(200.f, 200.f) + offset);
    // lineb.thick = 1.f;
    // auto linec = KiriLine2(Vector2F(200.f, 200.f) + offset, Vector2F(200.f, -200.f) + offset);
    // linec.thick = 1.f;
    // auto lined = KiriLine2(Vector2F(200.f, -200.f) + offset, Vector2F(-200.f, -200.f) + offset);
    // lined.thick = 1.f;
    // precompute_lines.emplace_back(linea);
    // precompute_lines.emplace_back(lineb);
    // precompute_lines.emplace_back(linec);
    // precompute_lines.emplace_back(lined);

    // std::vector<KiriLine2> precompute_lines;
    // std::vector<Vector2F> precompute_points;

    // auto sites = pd2->GetSites();

    // for (size_t i = 0; i < sites.size(); i++)
    // {
    //     auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
    //     if (site->GetIsBoundaryVertex())
    //         continue;

    //     auto cellpolygon = site->CellPolygon;
    //     for (size_t j = 0; j < cellpolygon->Verts.size(); j++)
    //     {
    //         auto vert = cellpolygon->Verts[j];
    //         auto vert1 = cellpolygon->Verts[(j + 1) % (cellpolygon->Verts.size())];
    //         auto line = KiriLine2(Vector2F(vert.x, vert.y) + offset, Vector2F(vert1.x, vert1.y) + offset);
    //         line.thick = 1.f;
    //         precompute_lines.emplace_back(line);

    //         // KIRI_LOG_DEBUG("vert={0},{1}-----vert1={2},{3}", vert.x, vert.y, vert1.x, vert1.y);
    //     }
    //     // KIRI_LOG_DEBUG("site={0},size={1}", site->GetId(), cellpolygon->Verts.size());
    //     precompute_points.emplace_back(Vector2F(site->X(), site->Y()));

    //     // KIRI_LOG_DEBUG("pd2->AddSite(std::make_shared<Voronoi::VoronoiSite2>({0}f, {1}f, {2}));", site->X(), site->Y(), i);
    // }

    while (1)
    {
        // KIRI_LOG_DEBUG("-----------------new----------------------------------");

        std::vector<KiriLine2> precompute_lines;
        std::vector<Vector2F> precompute_points;

        pd2->LloydIteration();

        auto sites = pd2->GetSites();

        for (size_t i = 0; i < sites.size(); i++)
        {
            auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
            if (site->GetIsBoundaryVertex())
                continue;

            auto cellpolygon = site->CellPolygon;
            for (size_t j = 0; j < cellpolygon->Verts.size(); j++)
            {
                auto vert = cellpolygon->Verts[j];
                auto vert1 = cellpolygon->Verts[(j + 1) % (cellpolygon->Verts.size())];
                auto line = KiriLine2(Vector2F(vert.x, vert.y) + offset, Vector2F(vert1.x, vert1.y) + offset);
                line.thick = 1.f;
                precompute_lines.emplace_back(line);

                // KIRI_LOG_DEBUG("vert={0},{1}-----vert1={2},{3}", vert.x, vert.y, vert1.x, vert1.y);
            }
            // KIRI_LOG_DEBUG("site={0},size={1}", site->GetId(), cellpolygon->Verts.size());
            precompute_points.emplace_back(Vector2F(site->X(), site->Y()));

            // KIRI_LOG_DEBUG("pd2->AddSite(std::make_shared<Voronoi::VoronoiSite2>({0}f, {1}f, {2}));", site->X(), site->Y(), i);
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
        renderer->ClearCanvas();
        scene->Clear();
    }
}

static void ExportVoroFile(
    const std::vector<Vector3F> &position,
    const std::vector<Vector3F> &normal,
    const std::vector<int> &indices,
    const String fileName)
{
    String exportPath = String(EXPORT_PATH) + "voro/" + fileName + ".voro";

    std::fstream file;
    file.open(exportPath.c_str(), std::ios_base::out);

    file << position.size() << "  " << normal.size() << "  " << indices.size() << std::endl;

    for (auto i = 0; i < position.size(); i++)
        file << position[i].x << "  " << position[i].y << "  " << position[i].z << std::endl;

    for (auto i = 0; i < normal.size(); i++)
        file << normal[i].x << "  " << normal[i].y << "  " << normal[i].z << std::endl;

    for (auto i = 0; i < indices.size(); i++)
        file << indices[i] << std::endl;

    file.close();
}

#include <tiny_obj_loader.h>
#define TINYOBJLOADER_IMPLEMENTATION

bool WriteMat(const std::string &filename, const std::vector<tinyobj::material_t> &materials)
{
    FILE *fp = fopen(filename.c_str(), "w");
    if (!fp)
    {
        fprintf(stderr, "Failed to open file [ %s ] for write.\n", filename.c_str());
        return false;
    }

    for (size_t i = 0; i < materials.size(); i++)
    {

        tinyobj::material_t mat = materials[i];

        fprintf(fp, "newmtl %s\n", mat.name.c_str());
        fprintf(fp, "Ka %f %f %f\n", mat.ambient[0], mat.ambient[1], mat.ambient[2]);
        fprintf(fp, "Kd %f %f %f\n", mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
        fprintf(fp, "Ks %f %f %f\n", mat.specular[0], mat.specular[1], mat.specular[2]);
        fprintf(fp, "Kt %f %f %f\n", mat.transmittance[0], mat.specular[1], mat.specular[2]);
        fprintf(fp, "Ke %f %f %f\n", mat.emission[0], mat.emission[1], mat.emission[2]);
        fprintf(fp, "Ns %f\n", mat.shininess);
        fprintf(fp, "Ni %f\n", mat.ior);
        fprintf(fp, "illum %d\n", mat.illum);
        fprintf(fp, "\n");
        // @todo { texture }
    }

    fclose(fp);

    return true;
}

bool TinyObjWriter(const String &filename, const tinyobj::attrib_t &attributes, const std::vector<tinyobj::shape_t> &shapes, const std::vector<tinyobj::material_t> &materials, bool coordTransform = false)
{
    String exportPath = String(EXPORT_PATH) + "voro/" + filename + ".obj";

    FILE *fp = fopen(exportPath.c_str(), "w");
    if (!fp)
    {
        fprintf(stderr, "Failed to open file [ %s ] for write.\n", exportPath.c_str());
        return false;
    }

    std::string basename = filename;
    std::string material_filename = basename + ".mtl";

    int prev_material_id = -1;

    fprintf(fp, "mtllib %s\n\n", material_filename.c_str());

    // facevarying vtx
    for (size_t k = 0; k < attributes.vertices.size(); k += 3)
    {
        if (coordTransform)
        {
            fprintf(fp, "v %f %f %f\n",
                    attributes.vertices[k + 0],
                    attributes.vertices[k + 2],
                    -attributes.vertices[k + 1]);
        }
        else
        {
            fprintf(fp, "v %f %f %f\n",
                    attributes.vertices[k + 0],
                    attributes.vertices[k + 1],
                    attributes.vertices[k + 2]);
        }
    }

    fprintf(fp, "\n");

    // facevarying normal
    for (size_t k = 0; k < attributes.normals.size(); k += 3)
    {
        if (coordTransform)
        {
            fprintf(fp, "vn %f %f %f\n",
                    attributes.normals[k + 0],
                    attributes.normals[k + 2],
                    -attributes.normals[k + 1]);
        }
        else
        {
            fprintf(fp, "vn %f %f %f\n",
                    attributes.normals[k + 0],
                    attributes.normals[k + 1],
                    attributes.normals[k + 2]);
        }
    }

    fprintf(fp, "\n");

    // facevarying texcoord
    for (size_t k = 0; k < attributes.texcoords.size(); k += 2)
    {
        fprintf(fp, "vt %f %f\n",
                attributes.texcoords[k + 0],
                attributes.texcoords[k + 1]);
    }

    for (size_t i = 0; i < shapes.size(); i++)
    {
        fprintf(fp, "\n");

        if (shapes[i].name.empty())
        {
            fprintf(fp, "g Unknown\n");
        }
        else
        {
            fprintf(fp, "g %s\n", shapes[i].name.c_str());
        }

        bool has_vn = false;
        bool has_vt = false;
        // Assumes normals and textures are set shape-wise.
        if (shapes[i].mesh.indices.size() > 0)
        {
            has_vn = shapes[i].mesh.indices[0].normal_index != -1;
            has_vt = shapes[i].mesh.indices[0].texcoord_index != -1;
        }

        // face
        int face_index = 0;
        for (size_t k = 0; k < shapes[i].mesh.indices.size(); k += shapes[i].mesh.num_face_vertices[face_index++])
        {
            // Check Materials
            int material_id = shapes[i].mesh.material_ids[face_index];
            if (material_id != prev_material_id)
            {
                std::string material_name = materials[material_id].name;
                fprintf(fp, "usemtl %s\n", material_name.c_str());
                prev_material_id = material_id;
            }

            unsigned char v_per_f = shapes[i].mesh.num_face_vertices[face_index];
            // Imperformant, but if you want to have variable vertices per face, you need some kind of a dynamic loop.
            fprintf(fp, "f");
            for (int l = 0; l < v_per_f; l++)
            {
                const tinyobj::index_t &ref = shapes[i].mesh.indices[k + l];
                if (has_vn && has_vt)
                {
                    // v0/t0/vn0
                    fprintf(fp, " %d/%d/%d", ref.vertex_index + 1, ref.texcoord_index + 1, ref.normal_index + 1);
                    continue;
                }
                if (has_vn && !has_vt)
                {
                    // v0//vn0
                    fprintf(fp, " %d//%d", ref.vertex_index + 1, ref.normal_index + 1);
                    continue;
                }
                if (!has_vn && has_vt)
                {
                    // v0/vt0
                    fprintf(fp, " %d/%d", ref.vertex_index + 1, ref.texcoord_index + 1);
                    continue;
                }
                if (!has_vn && !has_vt)
                {
                    // v0 v1 v2
                    fprintf(fp, " %d", ref.vertex_index + 1);
                    continue;
                }
            }
            fprintf(fp, "\n");
        }
    }

    fclose(fp);

    //
    // Write material file
    //
    bool ret = WriteMat(material_filename, materials);

    return ret;
}

void QuickHullVoronoi3d()
{
    using namespace HDV;

    auto voro3 = std::make_shared<Voronoi::VoronoiMesh3>();

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<float> dist(-1.f, 1.f);

    auto scale_size = 1.f;
    auto sampler_num = 100;
    std::vector<Primitives::Vertex3Ptr> vet3;

    for (auto i = 0; i < sampler_num; i++)
    {
        auto x = dist(rndEngine) * scale_size;
        auto y = dist(rndEngine) * scale_size;
        auto z = dist(rndEngine) * scale_size;

        auto v3 = std::make_shared<HDV::Voronoi::VoronoiSite3>(x, y, z, i);
        vet3.emplace_back(v3);

        // KIRI_LOG_DEBUG("vet2.emplace_back(std::make_shared<Primitives::Vertex2>({0}, {1}, {2}));", x, y, i);
    }

    // boundary
    // auto v3b1 = std::make_shared<HDV::Voronoi::VoronoiSite3>(-scale_size * 2.f, -scale_size * 2.f, -scale_size * 2.f, sampler_num + 1);
    // auto v3b2 = std::make_shared<HDV::Voronoi::VoronoiSite3>(scale_size * 2.f, scale_size * 2.f, scale_size * 2.f, sampler_num + 2);

    // auto v3b3 = std::make_shared<HDV::Voronoi::VoronoiSite3>(-scale_size * 2.f, -scale_size * 2.f, scale_size * 2.f, sampler_num + 3);
    // auto v3b4 = std::make_shared<HDV::Voronoi::VoronoiSite3>(-scale_size * 2.f, scale_size * 2.f, scale_size * 2.f, sampler_num + 4);
    // auto v3b5 = std::make_shared<HDV::Voronoi::VoronoiSite3>(-scale_size * 2.f, scale_size * 2.f, -scale_size * 2.f, sampler_num + 5);

    // auto v3b6 = std::make_shared<HDV::Voronoi::VoronoiSite3>(scale_size * 2.f, -scale_size * 2.f, -scale_size * 2.f, sampler_num + 6);
    // auto v3b7 = std::make_shared<HDV::Voronoi::VoronoiSite3>(scale_size * 2.f, -scale_size * 2.f, scale_size * 2.f, sampler_num + 7);
    // auto v3b8 = std::make_shared<HDV::Voronoi::VoronoiSite3>(scale_size * 2.f, scale_size * 2.f, -scale_size * 2.f, sampler_num + 8);

    // v3b1->SetAsBoundaryVertex();
    // v3b2->SetAsBoundaryVertex();
    // v3b3->SetAsBoundaryVertex();
    // v3b4->SetAsBoundaryVertex();
    // v3b5->SetAsBoundaryVertex();
    // v3b6->SetAsBoundaryVertex();
    // v3b7->SetAsBoundaryVertex();
    // v3b8->SetAsBoundaryVertex();

    // vet3.emplace_back(v3b1);
    // vet3.emplace_back(v3b2);
    // vet3.emplace_back(v3b3);
    // vet3.emplace_back(v3b4);
    // vet3.emplace_back(v3b5);
    // vet3.emplace_back(v3b6);
    // vet3.emplace_back(v3b7);
    // vet3.emplace_back(v3b8);

    voro3->Generate(vet3);
    KIRI_LOG_DEBUG("resgion size={0}", voro3->Regions.size());

    // boundary bbox
    BoundingBox3F boundary_box;
    boundary_box.merge(Vector3F(-scale_size, -scale_size, -scale_size));
    boundary_box.merge(Vector3F(scale_size, scale_size, scale_size));

    boundary_box.merge(Vector3F(-scale_size, -scale_size, scale_size));
    boundary_box.merge(Vector3F(-scale_size, scale_size, scale_size));
    boundary_box.merge(Vector3F(-scale_size, scale_size, -scale_size));

    boundary_box.merge(Vector3F(scale_size, -scale_size, -scale_size));
    boundary_box.merge(Vector3F(scale_size, -scale_size, scale_size));
    boundary_box.merge(Vector3F(scale_size, scale_size, -scale_size));

    auto counter = 0;
    for (auto i = 0; i < voro3->Regions.size(); i++)
    {
        auto voronoi_site = std::dynamic_pointer_cast<HDV::Voronoi::VoronoiSite3>(voro3->Regions[i]->site);
        if (voronoi_site->GetIsBoundaryVertex())
            continue;

        if (boundary_box.contains(voronoi_site->Polygon->BBox.HighestPoint) && boundary_box.contains(voronoi_site->Polygon->BBox.LowestPoint))
            continue;

        // auto cells = voro3->Regions[i]->Cells;
        // auto draw = true;
        // for (size_t j = 0; j < cells.size(); j++)
        // {
        //     auto cc = cells[j]->CircumCenter;
        //     auto v3 = Vector3F(cc->mPosition[0], cc->mPosition[1], cc->mPosition[2]);
        //     if (!InBound(v3, scale_size))
        //     {
        //         draw = false;
        //         break;
        //     }
        // }

        // if (!draw)
        //     continue;

        std::vector<tinyobj::shape_t> obj_shapes;
        std::vector<tinyobj::material_t> obj_materials;
        tinyobj::attrib_t attrib;
        tinyobj::shape_t ch_shape;

        // convex hull edges
        auto pos = voronoi_site->Polygon->Positions;
        auto normal = voronoi_site->Polygon->Normals;
        auto indices = voronoi_site->Polygon->Indices;

        for (size_t j = 0; j < pos.size() / 3; j++)
        {

            auto idx1 = j * 3;
            auto idx2 = j * 3 + 1;
            auto idx3 = j * 3 + 2;

            attrib.vertices.emplace_back(pos[idx1].x);
            attrib.vertices.emplace_back(pos[idx1].y);
            attrib.vertices.emplace_back(pos[idx1].z);

            attrib.vertices.emplace_back(pos[idx2].x);
            attrib.vertices.emplace_back(pos[idx2].y);
            attrib.vertices.emplace_back(pos[idx2].z);

            attrib.vertices.emplace_back(pos[idx3].x);
            attrib.vertices.emplace_back(pos[idx3].y);
            attrib.vertices.emplace_back(pos[idx3].z);

            // attrib.normals.emplace_back(normal[idx1].x);
            // attrib.normals.emplace_back(normal[idx1].y);
            // attrib.normals.emplace_back(normal[idx1].z);

            // attrib.normals.emplace_back(normal[idx2].x);
            // attrib.normals.emplace_back(normal[idx2].y);
            // attrib.normals.emplace_back(normal[idx2].z);

            // attrib.normals.emplace_back(normal[idx3].x);
            // attrib.normals.emplace_back(normal[idx3].y);
            // attrib.normals.emplace_back(normal[idx3].z);

            tinyobj::index_t i1, i2, i3;
            i1.vertex_index = indices[idx1];
            i2.vertex_index = indices[idx2];
            i3.vertex_index = indices[idx3];

            // i1.normal_index = indices[idx1];
            // i2.normal_index = indices[idx2];
            // i3.normal_index = indices[idx3];

            i1.normal_index = -1;
            i2.normal_index = -1;
            i3.normal_index = -1;

            i1.texcoord_index = -1;
            i2.texcoord_index = -1;
            i3.texcoord_index = -1;

            ch_shape.mesh.indices.emplace_back(i1);
            ch_shape.mesh.indices.emplace_back(i2);
            ch_shape.mesh.indices.emplace_back(i3);

            ch_shape.mesh.num_face_vertices.emplace_back(3);
            ch_shape.mesh.material_ids.emplace_back(-1);
        }

        // write to file
        obj_shapes.emplace_back(ch_shape);
        TinyObjWriter(UInt2Str4Digit(counter++), attrib, obj_shapes, obj_materials);

        // ExportVoroFile(
        //     voronoi_site->Polygon->Positions,
        //     voronoi_site->Polygon->Normals,
        //     voronoi_site->Polygon->Indices,
        //     UInt2Str4Digit(counter++));
    }
}

int main()
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

    // UniPoissonDiskSampler();

    // LoadVoronoiExample();

    // DebugNSParticles();

    // KIRI_LOG_DEBUG("123");
    // VoronoiNSOptimize();

    // TestPolygonUnion();

    // QuickHullConvexHull2d();

    // QuickHullDelaunayTriangulation2d();

    // QuickHullVoronoi2d();
    //    VoronoiExample1();

    QuickHullVoronoi3d();

    return 0;
}
