/***
 * @Author: Xu.WANG
 * @Date: 2021-02-21 18:37:46
 * @LastEditTime: 2021-06-25 01:13:50
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2dExamples\src\main_treemap.cpp
 */

#include <kiri2d/renderer/renderer.h>
#include <kiri2d/sdf/sdf_poly_2d.h>
#include <kiri2d/treemap/treemap_layout.h>

#include <random>
#include <list>

using namespace KIRI;
using namespace KIRI2D;

void load_xy_file(std::vector<Vector2F> &points, size_t &num, const char *filePath)
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

int main_treemaplayout()
{
    float height = 1080.f;
    float width = 1920.f;

    // boundary
    KiriSDFPoly2D boundary;
    std::vector<KiriPoint2> ppoints;
    std::vector<Vector2F> ptest;
    size_t testn;
    // load_xy_file(ptest, testn, "D:/project/Kiri/export/xy/test.xy");
    // load_xy_file(ptest, testn, "E:/PBCGLab/project/Kiri2D/scripts/alphashape/test.xy");
    load_xy_file(ptest, testn, "D:/project/Kiri2D/scripts/alphashape/test.xy");

    for (size_t i = 0; i < ptest.size(); i++)
    {
        auto point = KiriPoint2(ptest[i] * 800.f + Vector2F(500.f, 100.f), Vector3F(1.f, 0.f, 0.f));
        ppoints.emplace_back(point);
        boundary.Append(point.pos, Vector2F(0.f));
    }

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

    size_t totalNum = 10000;
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

    auto allrects = treemap2d->GetTreemapLayoutRect();
    for (size_t i = 0; i < allrects.size(); i++)
    {
        auto rect = allrects[i];
        float minw = std::min(rect.size.x, rect.size.y);
        float maxw = std::max(rect.size.x, rect.size.y);
        size_t np = static_cast<size_t>(std::roundf(maxw / minw));
        for (size_t j = 0; j < np; j++)
        {
            float rad = minw / 2.f;
            Vector2F p;
            if (rect.size.x >= rect.size.y)
                p = rect.original + Vector2F(rad + j * minw, rad);
            else
                p = rect.original + Vector2F(rad, rad + j * minw);

            if (boundary.FindRegion(p) < 0.f)
                circles.emplace_back(KiriCircle2(p, Vector3F(1.f, 0.f, 0.f), rad));
        }
    }

    auto scene = std::make_shared<KiriScene2D>((size_t)width, (size_t)height);

    // scene->AddParticles(ppoints);
    // scene->AddObject(boundary);

    // scene->AddLines(edges);
    // scene->AddParticles(points);

    // scene->AddRects(treemap2d->GetTreemapLayoutRect());
    scene->AddCircles(circles);

    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    while (1)
    {
        renderer->DrawCanvas();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->clearCanvas();
    }

    return 0;
}

#include <polyclipper2d/PolygonClipping.h>

int main111()
{
    float height = 1080.f;
    float width = 1920.f;

    // boundary
    KiriSDFPoly2D boundary;
    std::vector<KiriPoint2> ppoints;
    std::vector<Vector2F> ptest;
    size_t testn;

    load_xy_file(ptest, testn, "D:/project/Kiri2D/scripts/alphashape/test.xy");

    std::vector<PolyClip::Point2d> bunny2d;
    std::vector<PolyClip::Point2d> clip2d;
    clip2d.push_back(PolyClip::Point2d(0.f, 0.f));
    clip2d.push_back(PolyClip::Point2d(0.f, 0.3f));
    clip2d.push_back(PolyClip::Point2d(0.3f, 0.3f));
    clip2d.push_back(PolyClip::Point2d(0.3f, 0.f));

    for (size_t i = 0; i < ptest.size(); i++)
    {
        auto point = KiriPoint2(ptest[i] * 800.f + Vector2F(500.f, 100.f), Vector3F(1.f, 0.f, 0.f));
        ppoints.emplace_back(point);
        // boundary.Append(point.pos, Vector2F(0.f));
        bunny2d.push_back(PolyClip::Point2d(ptest[i].x, ptest[i].y));
    }

    PolyClip::Polygon polygon1(bunny2d);
    PolyClip::Polygon polygon2(clip2d);
    PolyClip::PloygonOpration::DetectIntersection(polygon1, polygon2);
    std::vector<std::vector<PolyClip::Point2d>> possible_result;
    if (PolyClip::PloygonOpration::Mark(polygon1, polygon2, possible_result, PolyClip::MarkIntersection))
    {
        std::vector<std::vector<PolyClip::Point2d>> results = PolyClip::PloygonOpration::ExtractIntersectionResults(polygon1);
        for (int i = 0; i < results.size(); ++i)
        {
            for (auto p : results[i])
                std::cout << "(" << p.x_ << ", " << p.y_ << ")"
                          << "---";
            std::cout << "\n";
            for (auto p : results[i])
                boundary.Append(Vector2F(p.x_, p.y_) * 800.f + Vector2F(500.f, 100.f), Vector2F(0.f));
        }
        std::cout << "intersection area is\n";
        std::cout << PolyClip::Utils::CalculatePolygonArea(results[0]) << "\n";
    }
    else
    {
        if (possible_result.size() == 0)
            std::cout << "No intersection\n";
        else
        {
            for (int i = 0; i < possible_result.size(); ++i)
            {
                for (auto p : possible_result[i])
                    std::cout << "(" << p.x_ << ", " << p.y_ << ")"
                              << "---";
                std::cout << "\n";
            }
        }
    }

    // render
    float aspect = height / width;
    float offset = height / 30.f;
    Vector2F offsetVec2 = Vector2F(offset, offset);
    auto scene = std::make_shared<KiriScene2D>((size_t)width, (size_t)height);

    scene->AddParticles(ppoints);
    scene->AddObject(boundary);

    // scene->AddLines(edges);

    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    while (1)
    {
        renderer->DrawCanvas();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->clearCanvas();
    }

    return 0;
}

Vector2F transform1(Vector2F a)
{
    return a * 800.f + Vector2F(500.f, 100.f);
}

#include <kiri2d/voronoi/voro_util.h>

int main41()
{
    KiriLog::init();

    float height = 1080.f;
    float width = 1920.f;

    // boundary
    KiriSDFPoly2D boundary;
    Vector<KiriLine2> lines;

    auto p1 = Vector2F(0.1f, 0.2f);
    auto p2 = Vector2F(0.2f, 0.1f);
    auto p3 = Vector2F(0.4f, 0.1f);
    auto p4 = Vector2F(0.4f, 0.5f);
    auto p5 = Vector2F(0.3f, 0.2f);
    auto p6 = Vector2F(0.25f, 0.4f);

    boundary.Append(transform1(p6), Vector2F(0.f));
    boundary.Append(transform1(p5), Vector2F(0.f));
    boundary.Append(transform1(p4), Vector2F(0.f));
    boundary.Append(transform1(p3), Vector2F(0.f));
    boundary.Append(transform1(p2), Vector2F(0.f));
    boundary.Append(transform1(p1), Vector2F(0.f));

    auto cw = KIRI::CheckClockwise2(p4, p5, p6) == 1 ? true : false;
    auto dir = DirectionBetween2Edges2(p4, p5, p6, cw);
    KIRI_LOG_DEBUG("check cw={0}, angle ={1}", cw, AngleBetween2Edges2(p4, p5, p6, cw));

    lines.emplace_back(KiriLine2(transform1(p5), transform1(p5 + dir)));

    // render
    float aspect = height / width;
    float offset = height / 30.f;
    Vector2F offsetVec2 = Vector2F(offset, offset);
    auto scene = std::make_shared<KiriScene2D>((size_t)width, (size_t)height);

    scene->AddObject(boundary);
    scene->AddLines(lines);

    auto renderer = std::make_shared<KiriRenderer2D>(scene);

    while (1)
    {
        renderer->DrawCanvas();
        cv::imshow("KIRI2D", renderer->GetCanvas());
        cv::waitKey(5);
        renderer->clearCanvas();
    }

    return 0;
}