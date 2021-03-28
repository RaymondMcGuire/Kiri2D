/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-21 18:37:46
 * @LastEditTime: 2021-03-29 03:32:35
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dExamples\src\main.cpp
 */

#include <kiri2d/renderer/renderer.h>
#include <kiri2d/sdf/sdf_poly_2d.h>
#include <kiri2d/treemap/treemap_layout.h>

#include <random>
#include <list>

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

int main()
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

    size_t totalNum = 10000;
    float totalValue = 0.f;
    for (size_t i = 0; i < totalNum; i++)
    {
        float radius = pcdis(gen);
        totalValue += radius;
        nodes.emplace_back(TreemapNode("A", radius, 0));
    }

    //TreemapNode topNode(tempNodeName, 35, 10, topRect);
    TreemapNode topNode(tempNodeName, totalValue, totalNum, topRect);

    TreemapLayoutPtr treemap2d = std::make_shared<TreemapLayout>(topNode, tempNodeName);
    treemap2d->AddTreeNodes(nodes);

    // treemap2d->AddTreeNode(TreemapNode("A", 4, 0));
    // treemap2d->AddTreeNode(TreemapNode("B", 3, 0));
    // treemap2d->AddTreeNode(TreemapNode("C", 1, 0));
    // treemap2d->AddTreeNode(TreemapNode("D", 10, 0));
    // treemap2d->AddTreeNode(TreemapNode("E", 2, 0));
    // treemap2d->AddTreeNode(TreemapNode("F", 3, 0));
    // treemap2d->AddTreeNode(TreemapNode("G", 6, 0));
    // treemap2d->AddTreeNode(TreemapNode("H", 1, 0));
    // treemap2d->AddTreeNode(TreemapNode("I", 3, 0));
    // treemap2d->AddTreeNode(TreemapNode("J", 2, 0));

    treemap2d->ConstructTreemapLayout();

    auto allrects = treemap2d->GetTreemapLayoutRect();
    for (size_t i = 0; i < allrects.size(); i++)
    {
        auto rect = allrects[i];
        auto p = rect.original + rect.size / 2.f;
        float rad = std::min(rect.size.x, rect.size.y) / 2.f;

        circles.emplace_back(KiriCircle2(p, Vector3F(1.f, 0.f, 0.f), rad));
    }

    auto scene = std::make_shared<KiriScene2D>((size_t)width, (size_t)height);

    // std::vector<KiriPoint2> ppoints;
    // std::vector<Vector2F> ptest;
    // size_t testn;
    // //load_xy_file(ptest, testn, "D:/project/Kiri/export/xy/test.xy");
    // load_xy_file(ptest, testn, "D:/project/Kiri2D/scripts/alphashape/test.xy");

    // KiriSDFPoly2D boundary;
    // boundary.Append(Vector2F(0.3f, 0.3f));
    // boundary.Append(Vector2F(0.3f, 4.7f));
    // boundary.Append(Vector2F(4.7f, 4.7f));
    // boundary.Append(Vector2F(4.7f, 0.3f));

    // std::vector<KiriLine2> edges;

    // edges.emplace_back(KiriLine2(Vector2F(0.511491, 0.151576) * width + offset, Vector2F(0.328024, 0.265777) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.328024, 0.265777) * width + offset, Vector2F(0.000000, 0.052614) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.000000, 0.052614) * width + offset, Vector2F(0.000000, 0.000000) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.000000, 0.000000) * width + offset, Vector2F(0.546750, 0.000000) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.546750, 0.000000) * width + offset, Vector2F(0.511491, 0.151576) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(1.000000, 0.418452) * width + offset, Vector2F(0.702465, 0.452691) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.702465, 0.452691) * width + offset, Vector2F(0.511491, 0.151576) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.511491, 0.151576) * width + offset, Vector2F(0.546750, 0.000000) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.546750, 0.000000) * width + offset, Vector2F(1.000000, 0.000000) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(1.000000, 0.000000) * width + offset, Vector2F(1.000000, 0.418452) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.328024, 0.265777) * width + offset, Vector2F(0.282745, 0.401936) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.282745, 0.401936) * width + offset, Vector2F(0.000000, 0.426054) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.000000, 0.426054) * width + offset, Vector2F(0.000000, 0.052614) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.000000, 0.052614) * width + offset, Vector2F(0.328024, 0.265777) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.702465, 0.452691) * width + offset, Vector2F(0.620256, 0.549363) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.620256, 0.549363) * width + offset, Vector2F(0.340386, 0.513157) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.340386, 0.513157) * width + offset, Vector2F(0.282745, 0.401936) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.282745, 0.401936) * width + offset, Vector2F(0.328024, 0.265777) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.328024, 0.265777) * width + offset, Vector2F(0.511491, 0.151576) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.511491, 0.151576) * width + offset, Vector2F(0.702465, 0.452691) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.340386, 0.513157) * width + offset, Vector2F(0.253343, 0.688532) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.253343, 0.688532) * width + offset, Vector2F(0.000000, 0.661637) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.000000, 0.661637) * width + offset, Vector2F(0.000000, 0.426054) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.000000, 0.426054) * width + offset, Vector2F(0.282745, 0.401936) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.282745, 0.401936) * width + offset, Vector2F(0.340386, 0.513157) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.613220, 0.679874) * width + offset, Vector2F(0.478311, 0.804271) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.478311, 0.804271) * width + offset, Vector2F(0.282452, 0.741229) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.282452, 0.741229) * width + offset, Vector2F(0.253343, 0.688532) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.253343, 0.688532) * width + offset, Vector2F(0.340386, 0.513157) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.340386, 0.513157) * width + offset, Vector2F(0.620256, 0.549363) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.620256, 0.549363) * width + offset, Vector2F(0.613220, 0.679874) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.929682, 1.000000) * width + offset, Vector2F(0.613220, 0.679874) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.613220, 0.679874) * width + offset, Vector2F(0.620256, 0.549363) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.620256, 0.549363) * width + offset, Vector2F(0.702465, 0.452691) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.702465, 0.452691) * width + offset, Vector2F(1.000000, 0.418452) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(1.000000, 0.418452) * width + offset, Vector2F(1.000000, 1.000000) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(1.000000, 1.000000) * width + offset, Vector2F(0.929682, 1.000000) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.282451, 0.741229) * width + offset, Vector2F(0.198055, 1.000000) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.198055, 1.000000) * width + offset, Vector2F(0.000000, 1.000000) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.000000, 1.000000) * width + offset, Vector2F(0.000000, 0.661637) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.000000, 0.661637) * width + offset, Vector2F(0.253343, 0.688532) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.253343, 0.688532) * width + offset, Vector2F(0.282452, 0.741229) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.489985, 1.000000) * width + offset, Vector2F(0.478311, 0.804271) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.478311, 0.804271) * width + offset, Vector2F(0.613220, 0.679874) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.613220, 0.679874) * width + offset, Vector2F(0.929682, 1.000000) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.929682, 1.000000) * width + offset, Vector2F(0.489985, 1.000000) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.478311, 0.804271) * width + offset, Vector2F(0.489985, 1.000000) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.489985, 1.000000) * width + offset, Vector2F(0.198055, 1.000000) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.198055, 1.000000) * width + offset, Vector2F(0.282451, 0.741229) * width + offset));
    // edges.emplace_back(KiriLine2(Vector2F(0.282452, 0.741229) * width + offset, Vector2F(0.478311, 0.804271) * width + offset));

    // std::vector<KiriPoint2> points;
    // points.emplace_back(KiriPoint2(Vector2F(0.149758, 0.286679) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    // points.emplace_back(KiriPoint2(Vector2F(0.749196, 0.204762) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    // points.emplace_back(KiriPoint2(Vector2F(0.458271, 0.389274) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    // points.emplace_back(KiriPoint2(Vector2F(0.350847, 0.886708) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    // points.emplace_back(KiriPoint2(Vector2F(0.614671, 0.870973) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    // points.emplace_back(KiriPoint2(Vector2F(0.141437, 0.818411) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    // points.emplace_back(KiriPoint2(Vector2F(0.274716, 0.094388) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    // points.emplace_back(KiriPoint2(Vector2F(0.804290, 0.683524) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    // points.emplace_back(KiriPoint2(Vector2F(0.422865, 0.662960) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    // points.emplace_back(KiriPoint2(Vector2F(0.171200, 0.538052) * width + offset, Vector3F(1.f, 0.f, 0.f)));

    //auto scene = std::make_shared<KiriScene2D>(700, 700);

    // // scene->AddLines(edges);
    // // scene->AddParticles(points);

    // for (size_t i = 0; i < ptest.size(); i++)
    // {
    //     if (ptest[i].x < 0.f || ptest[i].y < 0.f)
    //         continue;
    //     ppoints.emplace_back(KiriPoint2(ptest[i] * width + offset, Vector3F(1.f, 0.f, 0.f)));
    // }
    // scene->AddParticles(ppoints);

    scene->AddRects(treemap2d->GetTreemapLayoutRect());
    scene->AddCircles(circles);

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
