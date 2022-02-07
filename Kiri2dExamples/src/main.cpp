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
    radiusRange.push_back(0.2f * 2.5);

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
    // radiusRange.push_back(0.2f * 2.5);

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
    radiusRange.push_back(0.2f * 2.5);

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
#include <kiri2d/hdv_toolkit/delaunay/delaunay_triangulation2.h>
#include <kiri2d/hdv_toolkit/delaunay/delaunay_triangulation3.h>
void QuickHullConvexHull3d()
{
    using namespace HDV;

    std::vector<Primitives::Vertex3Ptr> vet3;
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.38154536485671997, 0.1146271675825119, 0.6684877276420593));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.24616186320781708, 0.4564620852470398, 0.31010201573371887));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.35921716690063477, 0.48692381381988525, 0.336710661649704));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6343265175819397, 0.0794384554028511, 0.7002074718475342));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.8470532298088074, 0.45811811089515686, 0.40726399421691895));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.8116405606269836, 0.5894207954406738, 0.43800264596939087));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.4764929413795471, 0.34879007935523987, 0.5977563858032227));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.4468737542629242, 0.4204060435295105, 0.347401887178421));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.4420554041862488, 0.45802855491638184, 0.6625888347625732));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.7877325415611267, 0.4891592264175415, 0.342998743057251));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.3631060719490051, 0.1395331472158432, 0.4028662443161011));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.7630274891853333, 0.04730677604675293, 0.6007089018821716));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.8409253358840942, 0.4443286657333374, 0.3612082004547119));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.8785147666931152, 0.28778958320617676, 0.6280174255371094));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6030551791191101, 0.2018173336982727, 0.60934978723526));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.7312994003295898, 0.3841328024864197, 0.7008111476898193));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.5594123601913452, 0.23014654219150543, 0.7550745606422424));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.2169153392314911, 0.6580185890197754, 0.421272337436676));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.31979966163635254, 0.4698064923286438, 0.3549903631210327));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.22061549127101898, 0.48685160279273987, 0.3842077851295471));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.07072707265615463, 0.6613724231719971, 0.6572007536888123));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6411411166191101, 0.575503408908844, 0.6193653345108032));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.7741573452949524, 0.21821317076683044, 0.36062467098236084));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.7509543895721436, 0.37132030725479126, 0.522675633430481));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.7690234184265137, 0.2553803324699402, 0.7250300645828247));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.2977566719055176, 0.32577580213546753, 0.6474320292472839));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.7262822389602661, 0.3219805359840393, 0.35913342237472534));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.5279478430747986, 0.584001362323761, 0.5908053517341614));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.19639888405799866, 0.7609197497367859, 0.2988492548465729));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.20517075061798096, 0.054803285747766495, 0.39519381523132324));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.5520908832550049, 0.26270177960395813, 0.27787646651268005));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.4260469079017639, 0.3692712187767029, 0.41294586658477783));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6608512997627258, 0.5496494174003601, 0.44731688499450684));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6051326990127563, 0.15145641565322876, 0.6067054271697998));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.5419924259185791, 0.9405410289764404, 0.31930989027023315));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.5081912875175476, 0.9590598344802856, 0.26466289162635803));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.5079513192176819, 0.31332895159721375, 0.5777361392974854));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.17393185198307037, 0.755401611328125, 0.6720157265663147));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.8610098958015442, 0.24821564555168152, 0.43328291177749634));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.4345715343952179, 0.4681691527366638, 0.5155949592590332));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.5131782293319702, 0.46006253361701965, 0.35858088731765747));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6175758838653564, 0.2746777832508087, 0.26007571816444397));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6896041035652161, 0.5187002420425415, 0.4056354761123657));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.5481720566749573, 0.9835112690925598, 0.25742068886756897));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6547431945800781, 0.4671425223350525, 0.4660925269126892));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.38980695605278015, 0.1620403230190277, 0.5588512420654297));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6969920992851257, 0.08670969307422638, 0.3079794943332672));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.4928244650363922, 0.2831677496433258, 0.6367471218109131));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.7093964219093323, 0.1470286250114441, 0.31880077719688416));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.8482162356376648, 0.5279671549797058, 0.40371400117874146));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.4991582930088043, 0.19517174363136292, 0.42477959394454956));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.3727628290653229, 0.505084753036499, 0.5972374081611633));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.8029602766036987, 0.41443052887916565, 0.6251235604286194));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6300662755966187, 0.09997764229774475, 0.41645264625549316));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.7209357023239136, 0.050708968192338943, 0.31318268179893494));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.16147856414318085, 0.8286169767379761, 0.38890695571899414));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6654874682426453, 0.29885637760162354, 0.7438512444496155));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.07246582955121994, 0.6110273003578186, 0.4885712265968323));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.2686154842376709, 0.5899538993835449, 0.5954372882843018));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.14579883217811584, 0.6043238639831543, 0.6893882751464844));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.489347904920578, 0.33180323243141174, 0.1924036741256714));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.47071751952171326, 0.4080698490142822, 0.5519131422042847));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.8826482892036438, 0.37401124835014343, 0.5228121876716614));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.5837278962135315, 0.415609210729599, 0.3626022934913635));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.7871206402778625, 0.34567737579345703, 0.7289097309112549));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.8190171718597412, 0.16528236865997314, 0.5943465232849121));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.7671700119972229, 0.5507073402404785, 0.4056755304336548));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.8843605518341064, 0.2750672996044159, 0.5510888695716858));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.5052724480628967, 0.41969117522239685, 0.6983670592308044));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.25021272897720337, 0.31463423371315, 0.6785677671432495));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6953755021095276, 0.14149902760982513, 0.437629759311676));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.5068426132202148, 0.0951828733086586, 0.7099730968475342));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6820862293243408, 0.16675879061222076, 0.5543367266654968));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.21292303502559662, 0.21016167104244232, 0.47843223810195923));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6063047051429749, 0.0772307962179184, 0.7232301831245422));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.34390950202941895, 0.10781310498714447, 0.390392005443573));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.8020079731941223, 0.30070576071739197, 0.5059731006622314));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.12251769751310349, 0.38184821605682373, 0.356222927570343));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.19920434057712555, 0.43950775265693665, 0.5920177698135376));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.41542673110961914, 0.4361895024776459, 0.3829081952571869));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.7208181619644165, 0.31229427456855774, 0.7798858880996704));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.823697566986084, 0.3999618887901306, 0.531923770904541));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.23186556994915009, 0.401278018951416, 0.311093807220459));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.08388122171163559, 0.6041345000267029, 0.5608069896697998));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6046280860900879, 0.4452591836452484, 0.2808905839920044));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.633285641670227, 0.4264303743839264, 0.7469958662986755));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6978915929794312, 0.4412091076374054, 0.3944694995880127));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.4369846284389496, 0.19924719631671906, 0.6523071527481079));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6807717680931091, 0.556245744228363, 0.6691657304763794));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.4586567282676697, 0.3110058009624481, 0.6783725619316101));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.8028092384338379, 0.4317425489425659, 0.6201617121696472));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6059055328369141, 0.2091643363237381, 0.7335872650146484));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.27032196521759033, 0.4030919075012207, 0.34194907546043396));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.9019389152526855, 0.28607332706451416, 0.5791234374046326));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.16338388621807098, 0.48983773589134216, 0.4772653579711914));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.8833513259887695, 0.4419476389884949, 0.4198605418205261));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.21356308460235596, 0.8361571431159973, 0.3021114468574524));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.21050220727920532, 0.3016509413719177, 0.42408573627471924));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.15061642229557037, 0.4166421592235565, 0.3476000726222992));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(0.6098446846008301, 0.07590754330158234, 0.7174344658851624));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(-1.0, -0.9910658001899719, -0.7753245234489441));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(2.0, 1.9821316003799438, 1.5506490468978882));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(-1.0, -0.9910658001899719, 1.5506490468978882));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(-1.0, 1.9821316003799438, 1.5506490468978882));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(-1.0, 1.9821316003799438, -0.7753245234489441));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(2.0, -0.9910658001899719, -0.7753245234489441));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(2.0, -0.9910658001899719, 1.5506490468978882));
    // vet3.emplace_back(std::make_shared<Primitives::Vertex3>(2.0, 1.9821316003799438, -0.7753245234489441));

    auto dt3 = std::make_shared<Delaunay::DelaunayTriangulation3>();
    dt3->Generate(vet3);

    // auto cv3 = std::make_shared<Hull::ConvexHull3>();
    // cv3->Generate(vet3);

    // KIRI_LOG_DEBUG("size={0}, eps={1}", cv3->GetSimplexs().size(), std::numeric_limits<double>::epsilon());
}

void QuickHullConvexHull2d()
{
    using namespace HDV;

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    auto scale_size = 200.0;
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

void QuickHullDelaunayTriangulation2d()
{
    using namespace HDV;

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    auto scale_size = 200.0;
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

    // boundary
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(-400.0, -400.0, 10));
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(-400.0, 400.0, 11));
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(400.0, 400.0, 12));
    vet2.emplace_back(std::make_shared<Primitives::Vertex2>(400.0, -400.0, 13));

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

#include <kiri2d/hdv_toolkit/sampler/ms_sampler2.h>
#include <kiri2d/hdv_toolkit/sampler/ms_sampler3.h>
void QuickHullVoronoi2d()
{
    using namespace HDV;

    auto pd2 = std::make_shared<Voronoi::PowerDiagram2D>();

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    auto scale_size = 200.0;
    auto sampler_num = 100;

    for (auto i = 0; i < sampler_num; i++)
    {
        auto x = dist(rndEngine) * scale_size;
        auto y = dist(rndEngine) * scale_size;

        pd2->AddSite(std::make_shared<Voronoi::VoronoiSite2>(x, y, i));
    }

    // clip boundary
    auto BoundaryPolygon = std::make_shared<Voronoi::VoronoiCellPolygon<Primitives::Vertex2Ptr, Primitives::Vertex2>>();
    BoundaryPolygon->AddVert2(Vector2D(-scale_size, -scale_size));
    BoundaryPolygon->AddVert2(Vector2D(-scale_size, scale_size));
    BoundaryPolygon->AddVert2(Vector2D(scale_size, scale_size));
    BoundaryPolygon->AddVert2(Vector2D(scale_size, -scale_size));
    pd2->SetBoundaryPolygon(BoundaryPolygon);

    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    Vector2F offset = Vector2F(windowwidth, windowheight) / 2.f;

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    pd2->Compute();

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
            for (size_t j = 0; j < cellpolygon->Positions.size(); j++)
            {
                auto vert = cellpolygon->Positions[j];
                auto vert1 = cellpolygon->Positions[(j + 1) % (cellpolygon->Positions.size())];
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

void MSSampler2D()
{
    using namespace HDV;

    auto multiSizeSampler = std::make_shared<Sampler::MultiSizeSampler2D>();

    auto scale_size = 1000.0;

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    std::uniform_real_distribution<double> rdist(-1.0, 1.0);

    std::vector<double> radiusRange;
    radiusRange.push_back(10.0);
    radiusRange.push_back(20.0);
    radiusRange.push_back(50.0);
    radiusRange.push_back(100.0);

    std::vector<double> radiusRangeProb;
    radiusRangeProb.push_back(0.7);
    radiusRangeProb.push_back(0.2);
    radiusRangeProb.push_back(0.1);

    multiSizeSampler->SetRadiusDist(radiusRange);
    multiSizeSampler->SetRadiusDistProb(radiusRangeProb);

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
    auto BoundaryPolygon = std::make_shared<Voronoi::VoronoiCellPolygon<Primitives::Vertex2Ptr, Primitives::Vertex2>>();
    // BoundaryPolygon->AddVert2(Vector2D(-scale_size, -scale_size));
    // BoundaryPolygon->AddVert2(Vector2D(-scale_size, scale_size));
    // BoundaryPolygon->AddVert2(Vector2D(scale_size, scale_size));
    // BoundaryPolygon->AddVert2(Vector2D(scale_size, -scale_size));

    String boundaryFileName = "bunny";
    String filePath = String(RESOURCES_PATH) + "alpha_shapes/" + boundaryFileName + ".xy";
    std::vector<Vector2F> bunny2d;
    size_t bunnyNum;
    load_xy_file1(bunny2d, bunnyNum, filePath.c_str());

    for (size_t i = 0; i < bunny2d.size(); i++)
    {
        auto newPos = Vector2D(bunny2d[i].x, bunny2d[i].y) * scale_size * 3.0;
        BoundaryPolygon->AddVert2(newPos);
    }

    multiSizeSampler->SetBoundaryPolygon(BoundaryPolygon);

    auto total_area = BoundaryPolygon->GetArea();
    auto total_num = total_area / (kiri_math_mini::pi<double>() * total_sum * total_sum);

    KIRI_LOG_DEBUG("avg_radius={0},total_area={1},total_num={2}", total_sum, total_area, total_num);

    auto maxcnt = 100;
    for (size_t i = 0; i < maxcnt; i++)
    {
        auto pos = BoundaryPolygon->GetRndInnerPoint();
        auto radius = pcdis(gen);
        multiSizeSampler->AddSite(pos.x, pos.y, radius);
    }

    multiSizeSampler->SetMaxiumNum(static_cast<int>(total_num * 1.5));

    // scene renderer config
    float windowheight = 4000.f;
    float windowwidth = 4000.f;

    // Vector2F offset = Vector2F(windowwidth, windowheight) / 2.f;
    Vector2F offset = Vector2F(500.f);

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    multiSizeSampler->Init();

    std::vector<float> errorArray, porosityArray, radiusErrorArray;
    std::vector<Vector4D> lastMaxCircle;
    auto minRadius = std::numeric_limits<double>::max();
    auto maxRadius = std::numeric_limits<double>::min();

    for (size_t idx = 0; idx < 3000; idx++)
    {
        multiSizeSampler->Compute();
        // KIRI_LOG_DEBUG("-----------------new----------------------------------");

        std::vector<KiriLine2> precompute_lines;
        std::vector<Vector2F> precompute_points;

        auto sites = multiSizeSampler->GetSites();

        for (size_t i = 0; i < sites.size(); i++)
        {
            auto site = std::dynamic_pointer_cast<Voronoi::VoronoiSite2>(sites[i]);
            if (site->GetIsBoundaryVertex())
                continue;

            auto cellpolygon = site->CellPolygon;
            if (cellpolygon)
            {
                for (size_t j = 0; j < cellpolygon->Positions.size(); j++)
                {
                    auto vert = cellpolygon->Positions[j];
                    auto vert1 = cellpolygon->Positions[(j + 1) % (cellpolygon->Positions.size())];
                    auto line = KiriLine2(Vector2F(vert.x, vert.y) + offset, Vector2F(vert1.x, vert1.y) + offset);
                    line.thick = 1.f;
                    precompute_lines.emplace_back(line);

                    // KIRI_LOG_DEBUG("vert={0},{1}-----vert1={2},{3}", vert.x, vert.y, vert1.x, vert1.y);
                }
            }

            // KIRI_LOG_DEBUG("site={0},size={1}", site->GetId(), cellpolygon->Positions.size());
            precompute_points.emplace_back(Vector2F(site->X(), site->Y()));

            // KIRI_LOG_DEBUG("pd2->AddSite(std::make_shared<Voronoi::VoronoiSite2>({0}f, {1}f, {2}));", site->X(), site->Y(), i);
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

        auto maxIC = multiSizeSampler->GetMICBySSkel();
        lastMaxCircle = maxIC;

        for (size_t i = 0; i < maxIC.size(); i++)
        {
            // auto maxCir2 = KiriCircle2(Transform2Original(Vector2F(maxIC[i].x, maxIC[i].y) * 10.f, height) + offsetVec2, Vector3F(1.f, 0.f, 0.f), maxIC[i].z * 10.f);
            auto maxCir2 = KiriCircle2(Vector2F(maxIC[i].x, maxIC[i].y) + offset, Vector3F(1.f, 0.f, 0.f), maxIC[i].z);

            circles.emplace_back(maxCir2);

            minRadius = std::min(minRadius, maxIC[i].z);
            maxRadius = std::max(maxRadius, maxIC[i].z);
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

        if (idx % 10 == 0)
            renderer->SaveImages2File();
        // cv::imshow("KIRI2D", renderer->GetCanvas());
        // cv::waitKey(5);
        renderer->ClearCanvas();
        scene->Clear();
    }
}

static void ExportVoroFile(
    const std::vector<Vector3D> &position,
    const std::vector<Vector3D> &normal,
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

#include <kiri2d/model/model_tiny_obj_loader.h>
#include <cuda/cuda_helper.h>
#include <partio/Partio.h>
void ExportBgeoFileFromCPU(String Folder, String FileName, std::vector<Vector4D> data)
{
    String exportPath = String(EXPORT_PATH) + "bgeo/" + Folder + "/" + FileName + ".bgeo";

    Partio::ParticlesDataMutable *p = Partio::create();
    Partio::ParticleAttribute positionAttr = p->addAttribute("position", Partio::VECTOR, 3);
    Partio::ParticleAttribute pScaleAttr = p->addAttribute("pscale", Partio::FLOAT, 1);

    for (UInt i = 0; i < data.size(); i++)
    {
        Int particle = p->addParticle();
        float *pos = p->dataWrite<float>(positionAttr, particle);
        float *pscale = p->dataWrite<float>(pScaleAttr, particle);
        pos[0] = data[i].x;
        pos[1] = data[i].y;
        pos[2] = data[i].z;

        *pscale = data[i].w;
    }
    Partio::write(exportPath.c_str(), *p);

    p->release();
}

void QuickHullVoronoi3d()
{
    using namespace HDV;

    // ############################################################################# for boundary mesh
    auto boundaryModel = std::make_shared<KiriModelTinyObjLoader>("bunny", "models", ".obj");
    boundaryModel->Normalize();
    // boundaryModel->ScaleToBox(2.f);

    auto cellSize = 0.005f;
    auto bBMin = boundaryModel->GetAABBMin();
    auto bBMax = boundaryModel->GetAABBMax();
    auto lengths = bBMax - bBMin;
    auto maxLength = lengths.max();

    Vector3F newBBoxMin(bBMin), newBBoxMax(bBMax);
    KIRI_LOG_INFO("Model Face Vertices Size={0}, Face Normals Size={1}", boundaryModel->GetFaceVertices().size(), boundaryModel->GetFaceVertexNormals().size());
    KIRI_LOG_INFO("Model BBoxMin={0},{1},{2}, BBoxMax={3},{4},{5}", newBBoxMin.x, newBBoxMin.y, newBBoxMin.z, newBBoxMax.x, newBBoxMax.y, newBBoxMax.z);

    for (size_t i = 0; i < 3; i++)
    {
        if (maxLength == lengths[i])
            continue;

        auto delta = maxLength - lengths[i];
        newBBoxMin[i] = bBMin[i] - (delta / 2.f);
        newBBoxMax[i] = bBMax[i] + (delta / 2.f);
    }

    auto epsilon = (newBBoxMax - newBBoxMin) / 10001.f;
    newBBoxMax += epsilon;
    newBBoxMin -= epsilon;

    KIRI_LOG_INFO("Model BBoxMin={0},{1},{2}, BBoxMax={3},{4},{5}", newBBoxMin.x, newBBoxMin.y, newBBoxMin.z, newBBoxMax.x, newBBoxMax.y, newBBoxMax.z);

    // convert float array to vector3 array
    Vec_Vec3F faceVerticesList(boundaryModel->GetNFaceVertices());
    Vec_Vec3F faceNormalsList(boundaryModel->GetNFaceVertices());
    std::memcpy(faceVerticesList.data(), boundaryModel->GetFaceVertices().data(), boundaryModel->GetFaceVertices().size() * sizeof(float));
    std::memcpy(faceNormalsList.data(), boundaryModel->GetFaceVertexNormals().data(), boundaryModel->GetFaceVertexNormals().size() * sizeof(float));

    LevelSetShapeInfo mInfo(CudaAxisAlignedBox<float3>(KiriToCUDA(newBBoxMin), KiriToCUDA(newBBoxMax)), cellSize / 2.f, (uint)boundaryModel->GetNFaces());

    KIRI_LOG_INFO("Start Construct SDF!");
    auto cudaSampler = std::make_shared<CudaShapeSampler>(mInfo, KiriToCUDA(faceVerticesList), 13, 8);
    auto insidePoints = cudaSampler->GetInsidePoints(1000);
    KIRI_LOG_INFO("Generated Initial Points!");
    // ############################################################################# for boundary mesh

    auto multiSizeSampler3 = std::make_shared<Sampler::MultiSizeSampler3D>(cudaSampler);
    auto voro3 = std::make_shared<Voronoi::VoronoiMesh3>();

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    std::uniform_real_distribution<double> rdist(-1.0, 1.0);

    std::vector<double> radiusRange;
    radiusRange.push_back(0.01);
    radiusRange.push_back(0.03);
    radiusRange.push_back(0.08);
    radiusRange.push_back(0.15);

    std::vector<double> radiusRangeProb;
    radiusRangeProb.push_back(0.5);
    radiusRangeProb.push_back(0.4);
    radiusRangeProb.push_back(0.1);

    std::random_device engine;
    std::mt19937 gen(engine());
    std::piecewise_constant_distribution<double> pcdis{std::begin(radiusRange), std::end(radiusRange), std::begin(radiusRangeProb)};

    for (auto i = 0; i < insidePoints.size(); i++)
    {
        auto x = insidePoints[i].x;
        auto y = insidePoints[i].y;
        auto z = insidePoints[i].z;

        auto radius = pcdis(gen);

        multiSizeSampler3->AddSite(x, y, z, radius);

        // KIRI_LOG_DEBUG("multiSizeSampler3->AddSite(std::make_shared<Voronoi::VoronoiSite3>({0},{1},{2},{3});", x, y, z, i);
    }

    std::vector<csgjscpp::Polygon> boundaryPolygons;
    for (size_t j = 0; j < faceVerticesList.size() / 3; j++)
    {

        auto idx1 = j * 3;
        auto idx2 = j * 3 + 1;
        auto idx3 = j * 3 + 2;

        auto pos1 = faceVerticesList[idx1];
        auto pos2 = faceVerticesList[idx2];
        auto pos3 = faceVerticesList[idx3];

        auto norm1 = faceNormalsList[idx1];
        auto norm2 = faceNormalsList[idx2];
        auto norm3 = faceNormalsList[idx3];

        std::vector<csgjscpp::Vertex> csgVerts;

        csgVerts.push_back({csgjscpp::Vector(pos1.x, pos1.y, pos1.z), csgjscpp::Vector(norm1.x, norm1.y, norm1.z), csgjscpp::green});
        csgVerts.push_back({csgjscpp::Vector(pos2.x, pos2.y, pos2.z), csgjscpp::Vector(norm2.x, norm2.y, norm2.z), csgjscpp::green});
        csgVerts.push_back({csgjscpp::Vector(pos3.x, pos3.y, pos3.z), csgjscpp::Vector(norm3.x, norm3.y, norm3.z), csgjscpp::green});

        boundaryPolygons.push_back(csgjscpp::Polygon(csgVerts));
    }

    // // // auto cube = csgjscpp::csgmodel_cube({-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}, csgjscpp::green);
    // auto boundaryMesh = csgjscpp::modelfrompolygons(boundaryPolygons);

    // // // auto unionModel = csgjscpp::csgunion(cube, boundaryMesh);
    // csgjscpp::modeltoply(String(EXPORT_PATH) + "voro/test.ply", boundaryMesh);

    multiSizeSampler3->SetBoundaryPolygon(boundaryPolygons);

    KIRI_LOG_INFO("Start Init Voronoi Diagram 3D!");
    multiSizeSampler3->Init();
    KIRI_LOG_INFO("Finished Init Voronoi Diagram 3D!");

    auto data = multiSizeSampler3->GetSampledSpheres();
    KIRI_LOG_INFO("GetSampledSpheres size={0}!", data.size());
    ExportBgeoFileFromCPU("ms3", UInt2Str4Digit(0), data);

    // for (auto i = 0; i < 100; i++)
    // {
    //     multiSizeSampler3->Compute();
    //     // auto data = multiSizeSampler3->GetSampledSpheres();
    //     // KIRI_LOG_INFO("GetSampledSpheres size={0}!", data.size());
    //     // ExportBgeoFileFromCPU("ms3", UInt2Str4Digit(i), data);
    // }

    // KiriTimer timer;
    // float totalTime = 0.f;
    // int iterNumber = 120;
    // for (size_t i = 0; i < iterNumber; i++)
    // {
    //     float currentTime = 0.f;
    //     timer.Restart();
    //     multiSizeSampler3->LloydIteration();
    //     currentTime = timer.Elapsed();
    //     totalTime += currentTime;

    //     KIRI_LOG_DEBUG("Lloyd Iteration Idx={0}, Computation Time={1}", i, currentTime);
    // }
    // KIRI_LOG_DEBUG("Total Computation Time={0}", totalTime, totalTime / iterNumber);
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

    // QuickHullConvexHull3d();

    // QuickHullDelaunayTriangulation2d();

    // QuickHullVoronoi2d();
    //         VoronoiExample1();

    // QuickHullVoronoi3d();

    MSSampler2D();

    return 0;
}
