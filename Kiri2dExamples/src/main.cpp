/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-21 18:37:46
 * @LastEditTime: 2021-06-23 18:16:25
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dExamples\src\main.cpp
 */

#include <kiri2d/voronoi/voro_treemap_nocaj12.h>
#include <kiri2d/renderer/renderer.h>
#include <kiri2d/sdf/sdf_poly_2d.h>
#include <kiri2d/treemap/treemap_layout.h>
#include <kiri2d/voronoi/voro_poropti.h>
#include <kiri2d/voronoi/voro_poropti_treemap_core.h>
#include <root_directory.h>
#include <random>

using namespace KIRI;
using namespace KIRI2D;

Vector2F Transform2Original(const Vector2F &v, float h)
{
    return Vector2F(v.x, h - v.y);
}

void ExportPoroityData2CSVFile(const String fileName, const Vector<float> &error, const Vector<float> &poroity, const Vector<float> &radiusError, const Vector<Vector4F> &maxIC)
{
    String filePath = String(EXPORT_PATH) + "csv/" + fileName;
    std::fstream file;
    file.open(filePath.c_str(), std::ios_base::out);
    file << "iter,error"
         << std::endl;
    for (int i = 0; i < error.size(); i++)
        file << i + 1 << "," << error[i] << std::endl;

    file << "iter,poroity"
         << std::endl;
    for (int i = 0; i < poroity.size(); i++)
        file << i + 1 << "," << poroity[i] << std::endl;

    file << "iter,radiusError"
         << std::endl;
    for (int i = 0; i < radiusError.size(); i++)
        file << i + 1 << "," << radiusError[i] << std::endl;

    file << "curRadius,tarRadius"
         << std::endl;
    for (int i = 0; i < maxIC.size(); i++)
        file << maxIC[i].z << "," << maxIC[i].w << std::endl;

    file.close();
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

    size_t totalNum = 10;
    float totalValue = 0.f;
    for (size_t i = 0; i < totalNum; i++)
    {
        float radius = pcdis(gen);
        totalValue += radius;
        nodes.emplace_back(TreemapNode("A", -1, -1, radius, 0));
    }

    //TreemapNode topNode(tempNodeName, 35, 10, topRect);
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
            pd->AddVoroSite(sitePos2);
            //pd->AddPowerSite(sitePos2, dist(rndEngine) * 1000.f);
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
            //sites[i]->Print();
            auto pos = Transform2Original(Vector2F(sites[i]->GetValue().x, sites[i]->GetValue().y), height) + offsetVec2;
            points.emplace_back(KiriPoint2(pos, Vector3F(1.f, 0.f, 0.f)));
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

    //voronoi sites
    auto sites = pd->GetVoroSites();
    for (size_t i = 0; i < sites.size(); i++)
    {
        //sites[i]->Print();
        auto pos = Transform2Original(Vector2F(sites[i]->GetValue().x, sites[i]->GetValue().y), height) + offsetVec2;
        points.emplace_back(KiriPoint2(pos, Vector3F(1.f, 0.f, 0.f)));
        auto poly = sites[i]->GetCellPolygon();
        if (poly != NULL)
        {
            poly->ComputeStraightSkeleton(1.f);
            auto mic = poly->ComputeMICByStraightSkeleton();
            auto micPos = Transform2Original(Vector2F(mic.x, mic.y), height) + offsetVec2;
            circles.emplace_back(KiriCircle2(micPos, Vector3F(0.f, 0.f, 100.f), mic.z));
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
    for (size_t i = 0; i < 20; i++)
    {
        auto sitePos2 = Vector2F(dist(rndEngine) * width, dist(rndEngine) * height);
        if (boundary.FindRegion(sitePos2) < 0.f)
            pd->AddVoroSite(sitePos2);
        // pd->AddPowerSite(sitePos2, dist(rndEngine) * width * 100.f);
    }
    // auto cnt = 0, maxcnt = 100;
    // while (cnt < maxcnt)
    // {
    //     auto sitePos2 = Vector2F(dist(rndEngine) * width, dist(rndEngine) * height);
    //     if (boundary.FindRegion(sitePos2) < 0.f)
    //     {
    //         auto radius = 10.f;
    //         // auto site = std::make_shared<KiriVoroSite>(sitePos2.x, sitePos2.y, MEpsilon<float>(), radius);
    //         pd->AddVoroSite(sitePos2);
    //         cnt++;
    //     }
    // }

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
            //sites[i]->Print();
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
            //sites[i]->Print();
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
            //auto radius = avgRadius / 2.f * rdist(rndEngine) + avgRadius;
            //KIRI_LOG_DEBUG("idx={0}, radius={1}", cnt, radius);
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
            //sites[i]->Print();
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
        //renderer->SaveImages2File();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->ClearCanvas();
        scene->Clear();
    }
}

void VoroPorosityOptimizeExample()
{
    // scene renderer config
    float windowheight = 4000.f;
    float windowwidth = 4000.f;

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
    for (size_t i = 0; i < 2002; i++)
    {

        auto error = opti->ComputeIterate();
        errorArray.emplace_back(error);

        if (i % 10 == 1)
        {
            //MIC calculated by vornoi
            //opti->ComputeChildIterate();

            Vector<KiriPoint2> points;
            Vector<KiriLine2> lines;
            Vector<KiriCircle2> circles;

            auto sites = opti->GetLeafNodeSites();
            for (size_t i = 0; i < sites.size(); i++)
            {
                //sites[i]->Print();
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

            auto maxIC = opti->GetMICByStraightSkeleton(20.f);
            lastMaxCircle = maxIC;

            auto porosity = opti->ComputeMiniumPorosity(20.f);
            porosityArray.emplace_back(porosity);

            KIRI_LOG_DEBUG("iterate idx:{0}, porosity={1}, error={2}", i, porosity, error / maxIC.size());

            auto radiusError = 0.f;
            for (size_t i = 0; i < maxIC.size(); i++)
            {
                auto maxCir2 = KiriCircle2(Transform2Original(Vector2F(maxIC[i].x, maxIC[i].y), height) + offsetVec2, Vector3F(1.f, 0.f, 0.f), maxIC[i].z);
                circles.emplace_back(maxCir2);
                //KIRI_LOG_INFO("Site idx={0}, max radius={1}, target radius={2}", i, maxIC[i].z, maxIC[i].w);
                radiusError += std::abs(maxIC[i].z - maxIC[i].w);
            }
            radiusErrorArray.emplace_back(radiusError / maxIC.size());

            // auto skeletons = opti->GetCellSkeletons();

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
    }
    ExportPoroityData2CSVFile("test.csv", errorArray, porosityArray, radiusErrorArray, lastMaxCircle);

    // while (1)
    // {

    //     renderer->DrawCanvas();
    //     //renderer->SaveImages2File();
    //     cv::imshow("KIRI2D", renderer->GetCanvas());
    //     cv::waitKey(5);
    //     // renderer->ClearCanvas();
    //     // scene->Clear();
    // }
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

    size_t totalNum = 10;
    float totalValue = 0.f;
    for (size_t i = 0; i < totalNum; i++)
    {
        float radius = pcdis(gen);
        totalValue += radius;
        nodes.emplace_back(TreemapNode("A", -1, -1, radius, 0));
    }

    //TreemapNode topNode(tempNodeName, 35, 10, topRect);
    TreemapNode topNode(tempNodeName, 0, -1, totalValue, totalNum, topRect);

    TreemapLayoutPtr treemap2d = std::make_shared<TreemapLayout>(topNode, tempNodeName);
    treemap2d->AddTreeNodes(nodes);
    treemap2d->ConstructTreemapLayout();
    treemap2d->PrintTreemapLayout();

    auto voroTreeNodes = treemap2d->Convert2VoroTreeData();
    auto nodesResByDepth = voroTreeNodes->GetNodesByDepth(1);
    for (size_t i = 0; i < nodesResByDepth.size(); i++)
    {
        KIRI_LOG_DEBUG("child: id={0}, pid={1}, weight={2}, depth={3}", nodesResByDepth[i].id, nodesResByDepth[i].pid, nodesResByDepth[i].weight, nodesResByDepth[i].depth);
    }

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

    auto voroPorTreeMapOptiCore = std::make_shared<KiriVoroPoroOptiTreeMapCore>();

    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<float> dist(0.f, 1.f);

    auto cnt = 0;
    auto maxcnt = nodesResByDepth.size();
    while (cnt < maxcnt)
    {
        auto sitePos2 = Vector2F(dist(rndEngine) * width, dist(rndEngine) * height);
        if (boundary.FindRegion(sitePos2) < 0.f)
        {

            auto site = std::make_shared<KiriVoroSite>(sitePos2, nodesResByDepth[cnt].weight);
            voroPorTreeMapOptiCore->AddSite(site);
            cnt++;
        }
    }
    voroPorTreeMapOptiCore->SetBoundaryPolygon2(boundaryPoly);
    voroPorTreeMapOptiCore->Init();

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    while (1)
    {
        voroPorTreeMapOptiCore->ComputeIterate();
        Vector<KiriPoint2> points;
        Vector<KiriLine2> lines;

        auto sites = voroPorTreeMapOptiCore->GetSites();
        for (size_t i = 0; i < sites.size(); i++)
        {
            //sites[i]->Print();
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
        //renderer->SaveImages2File();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->ClearCanvas();
        scene->Clear();
    }
}

int main()
{
    KIRI::KiriLog::Init();
    //VoronoiExample();
    // VoronoiExample2();

    // LloydRelaxationExample();

    //GenRndTreemap();

    //NOCAJ12Example();
    //NOCAJ12Example1();

    //VoroTestExample();

    //VoroPorosityOptimizeExample();

    VoroPorosityTreemapOptiExample();

    // // scene renderer config
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
    // // boundaryPoly->AddPolygonVertex2(Vector2F(width, height));
    // //boundaryPoly->AddPolygonVertex2(Vector2F(0, height));

    // //auto pd = std::make_shared<KiriPowerDiagram>();
    // auto nocaj12 = std::make_shared<KiriVoroTreemapNocaj12>();

    // std::random_device seedGen;
    // std::default_random_engine rndEngine(seedGen());
    // std::uniform_real_distribution<float> dist(0.f, 1.f);
    // for (size_t i = 0; i < 100; i++)
    // {

    //     // auto site = std::make_shared<KiriVoroSite>(dist(rndEngine) * width, dist(rndEngine) * height);

    //     // if (i != 50 || i != 30)
    //     //     site->SetPercentage(dist(rndEngine));
    //     // else
    //     //     site->SetPercentage(5.f);
    //     // if (boundary.FindRegion(Vector2F(site->GetValue().x, site->GetValue().y)) < 0.f)
    //     //     nocaj12->AddSite(site);

    //     auto sitePos2 = Vector2F(dist(rndEngine) * width, dist(rndEngine) * height);
    //     if (boundary.FindRegion(sitePos2) < 0.f)
    //         pd->AddVoroSite(sitePos2);
    //     pd->AddPowerSite(sitePos2, dist(rndEngine) * width * 100.f);
    // }

    // nocaj12->SetBoundaryPolygon2(boundaryPoly);
    // nocaj12->Compute();

    // //pd->SetBoundaryPolygon2(boundaryPoly);
    // //pd->ComputeDiagram();
    // // pd->SetRelaxIterNumber(100);
    // // pd->LloydRelaxation();

    // Vector<KiriPoint2> points;
    // Vector<KiriLine2> lines;

    // auto sites = nocaj12->GetSites();
    // //auto sites = pd->GetVoroSites();
    // for (size_t i = 0; i < sites.size(); i++)
    // {
    //     //sites[i]->Print();
    //     points.emplace_back(KiriPoint2(Vector2F(sites[i]->GetValue().x, sites[i]->GetValue().y) + offsetVec2, Vector3F(1.f, 0.f, 0.f)));
    //     auto poly = sites[i]->GetCellPolygon();
    //     if (poly != NULL)
    //     {
    //         poly->ComputeVoroSitesList();
    //         auto list = poly->GetVoroSitesList();
    //         // list->PrintVertexList();

    //         auto node = list->GetHead();
    //         do
    //         {
    //             auto start = Vector2F(node->value) + offsetVec2;
    //             node = node->next;
    //             auto end = Vector2F(node->value) + offsetVec2;
    //             lines.emplace_back(KiriLine2(start, end));
    //         } while (node != list->GetHead());
    //     }
    // }

    // auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);

    // scene->AddLines(lines);
    // scene->AddParticles(points);
    // auto renderer = std::make_shared<KiriRenderer2D>(scene);

    // while (1)
    // {
    //     renderer->DrawCanvas();
    //     cv::imshow("KIRI2D", renderer->GetCanvas());
    //     cv::waitKey(5);
    //     renderer->ClearCanvas();
    // }

    return 0;
}
