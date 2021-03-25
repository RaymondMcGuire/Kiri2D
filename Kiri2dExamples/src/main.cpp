/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-21 18:37:46
 * @LastEditTime: 2021-03-26 05:35:03
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dExamples\src\main.cpp
 */

#include <kiri2d/renderer/renderer.h>
#include <kiri2d/sdf/sdf_poly_2d.h>

#include <treehh/tree.hh>
#include <algorithm>
#include <string>
#include <iostream>

using namespace KIRI2D;
using namespace std;

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

struct mapweight
{
    string name;
    float value;
    size_t child_num;

    mapweight() : name("NoName"), value(0.f), child_num(0) {}

    mapweight(string n, float v, size_t cn)
        : name(n), value(v), child_num(cn) {}
};

void tree_map_layout(tree<mapweight> &map, tree<mapweight>::iterator &node)
{
    float partA = 0.f, partB = (*node).value;
    size_t num = 0, child_num = (*node).child_num;

    auto sibs = map.begin(node);
    auto sibe = map.end(node);

    while (sibs != sibe)
    {
        ++num;
        partA += (*sibs).value;
        partB -= (*sibs).value;

        if (partA >= partB || num == child_num - 1)
        {
            break;
        }
        ++sibs;
    }

    tree<mapweight>::iterator right = map.wrap(map.next_sibling(sibs), map.end(node), mapweight("O", partB, child_num - num));
    tree<mapweight>::iterator left = map.wrap(map.begin(node), map.next_sibling(sibs), mapweight("O", partA, num));

    if (num > 1)
        tree_map_layout(map, left);

    if ((*node).child_num - num > 1)
        tree_map_layout(map, right);

    return;
}

int main()
{

    tree<mapweight> treemap;
    tree<mapweight>::iterator top_tree, data_tree;

    top_tree = treemap.begin();
    data_tree = treemap.insert(top_tree, mapweight("O", 35, 10));

    treemap.append_child(data_tree, mapweight("A", 4, 0));
    treemap.append_child(data_tree, mapweight("B", 3, 0));
    treemap.append_child(data_tree, mapweight("C", 1, 0));
    treemap.append_child(data_tree, mapweight("D", 10, 0));
    treemap.append_child(data_tree, mapweight("E", 2, 0));
    treemap.append_child(data_tree, mapweight("F", 3, 0));
    treemap.append_child(data_tree, mapweight("G", 6, 0));
    treemap.append_child(data_tree, mapweight("H", 1, 0));
    treemap.append_child(data_tree, mapweight("I", 3, 0));
    treemap.append_child(data_tree, mapweight("J", 2, 0));

    tree_map_layout(treemap, data_tree);

    float height = 400.f;
    float width = 600.f;
    float view_width = 5.f;
    float offset = 0.1f;
    float aspect = height / width;
    Vector2F camera_scale = Vector2F(view_width, aspect * view_width);

    bool vertical_segment = true;

    vector<KiriRect2> rects;

    tree<mapweight>::iterator sibss = treemap.begin(data_tree);
    tree<mapweight>::iterator sibee = treemap.end(data_tree);

    size_t depth, last_depth = 0;
    KiriRect2 rect(Vector2F(0.f), Vector2F(0.f));
    KiriRect2 lrect(Vector2F(0.f), Vector2F(0.f));
    float maxSize = 35.f;
    float cur_val, last_val = 0.f;
    while (sibss != sibee)
    {
        for (int k = 0; k < treemap.depth(sibss) - 1; ++k)
            cout << " ";

        cout << "depth=" << treemap.depth(sibss) << ";" << (*sibss).name << ":" << (*sibss).value << "," << (*sibss).child_num << endl;

        depth = treemap.depth(sibss);
        rect.lowest.x = 0.f;
        rect.lowest.y = 0.f;

        rect.highest.x = (*sibss).value / maxSize;
        rect.highest.y = 1.f;

        cur_val = (*sibss).value;
        if (depth == last_depth + 1 && (*sibss).name != "O")
        {
            rect.lowest.x = lrect.lowest.x;
            rect.lowest.y = depth % 2 == 0 ? lrect.lowest.y + (last_val - cur_val) / last_val : lrect.lowest.y;

            rect.highest.x = depth % 2 == 1 ? lrect.highest.x + cur_val / last_val : lrect.highest.x;
            rect.highest.y = lrect.highest.y;
        }
        else if (depth != last_depth + 1 && (*sibss).name == "O")
        {
            rect.lowest.x = lrect.highest.x;
            rect.lowest.y = lrect.lowest.y;

            rect.highest.x = lrect.highest.x + cur_val / last_val : lrect.highest.x;
            rect.highest.y = lrect.highest.y;
        }

        if ((*sibss).name != "O")
        {
            rects.emplace_back(KiriRect2(rect.lowest, rect.highest));
        }

        lrect = rect;
        last_depth = depth;

        ++sibss;
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