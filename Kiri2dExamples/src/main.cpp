/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-21 18:37:46
 * @LastEditTime: 2021-05-26 12:02:20
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dExamples\src\main.cpp
 */

#include <kiri2d/geo/convex_hull3.h>
#include <kiri2d/voronoi/power_diagram.h>
#include <kiri2d/renderer/renderer.h>
#include <random>
using namespace KIRI;
using namespace KIRI2D;

int main()
{
    KIRI::KiriLog::Init();

    // scene renderer config
    float windowheight = 1080.f;
    float windowwidth = 1920.f;

    // voronoi
    auto boundaryPoly = std::make_shared<KiriVoroCellPolygon2>();
    float width = 1000.f;
    float height = 1000.f;
    auto offsetVec2 = Vector2F((windowwidth - width) / 2.f, (windowheight - height) / 2.f);

    boundaryPoly->AddPolygonVertex2(Vector2F(0, 0));
    boundaryPoly->AddPolygonVertex2(Vector2F(width, 0));
    boundaryPoly->AddPolygonVertex2(Vector2F(width, height));
    boundaryPoly->AddPolygonVertex2(Vector2F(0, height));

    auto pd = std::make_shared<KiriPowerDiagram>();
    std::random_device seedGen;
    std::default_random_engine rndEngine(seedGen());
    std::uniform_real_distribution<> dist(0.f, 1.f);
    for (size_t i = 0; i < 100; i++)
    {
        auto sitePos2 = Vector2F(dist(rndEngine) * width, dist(rndEngine) * height);
        pd->AddVoroSite(sitePos2);
    }

    pd->SetBoundaryPolygon2(boundaryPoly);
    pd->ComputeDiagram();

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

    auto scene = std::make_shared<KiriScene2D>((size_t)windowwidth, (size_t)windowheight);

    scene->AddLines(lines);
    scene->AddParticles(points);
    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    while (1)
    {
        renderer->DrawCanvas();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->ClearCanvas();
    }

    return 0;
}
