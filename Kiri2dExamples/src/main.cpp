/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-21 18:37:46
 * @LastEditTime: 2021-03-26 16:55:58
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
    KiriRect2 rect;

    mapweight() : name("NoName"), value(0.f), child_num(0) {}

    mapweight(string n, float v, size_t cn)
        : name(n), value(v), child_num(cn) {}

    mapweight(string n, float v, size_t cn, KiriRect2 r)
        : name(n), value(v), child_num(cn), rect(r) {}
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

    KiriRect2 cur_rect_left, cur_rect_right;
    if ((*node).rect.size.x >= (*node).rect.size.y)
    {
        cur_rect_left.original.x = (*node).rect.original.x;
        cur_rect_left.original.y = (*node).rect.original.y;

        cur_rect_left.size.x = partA / (*node).value * (*node).rect.size.x;
        cur_rect_left.size.y = (*node).rect.size.y;

        cur_rect_right.original.x = (*node).rect.original.x + partA / (*node).value * (*node).rect.size.x;
        cur_rect_right.original.y = (*node).rect.original.y;

        cur_rect_right.size.x = partB / (*node).value * (*node).rect.size.x;
        cur_rect_right.size.y = (*node).rect.size.y;
    }
    else
    {
        cur_rect_left.original.x = (*node).rect.original.x;
        cur_rect_left.original.y = (*node).rect.original.y + partB / (*node).value * (*node).rect.size.y;

        cur_rect_left.size.x = (*node).rect.size.x;
        cur_rect_left.size.y = partA / (*node).value * (*node).rect.size.y;

        cur_rect_right.original.x = (*node).rect.original.x;
        cur_rect_right.original.y = (*node).rect.original.y;

        cur_rect_right.size.x = (*node).rect.size.x;
        cur_rect_right.size.y = partB / (*node).value * (*node).rect.size.y;
    }

    if ((*node).child_num - num > 1)
    {
        tree<mapweight>::iterator right = map.wrap(map.next_sibling(sibs), map.end(node), mapweight("O", partB, child_num - num, cur_rect_right));
        tree_map_layout(map, right);
    }
    else
    {
        (*map.next_sibling(sibs)).rect = cur_rect_right;
    }

    if (num > 1)
    {
        tree<mapweight>::iterator left = map.wrap(map.begin(node), map.next_sibling(sibs), mapweight("O", partA, num, cur_rect_left));
        tree_map_layout(map, left);
    }
    else
    {
        (*sibs).rect = cur_rect_left;
    }

    return;
}

int main()
{
    float height = 400.f;
    float width = 600.f;
    Vector2F offset = (width, height) / 100.f;

    tree<mapweight> treemap;
    tree<mapweight>::iterator top_tree, data_tree;

    top_tree = treemap.begin();

    KiriRect2 top_rect(offset, Vector2F(width, height) - 2.f * offset);
    data_tree = treemap.insert(top_tree, mapweight("O", 35, 10, top_rect));

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

    vector<KiriRect2> rects;

    tree<mapweight>::iterator sibss = treemap.begin(data_tree);
    tree<mapweight>::iterator sibee = treemap.end(data_tree);

    while (sibss != sibee)
    {
        for (int k = 0; k < treemap.depth(sibss) - 1; ++k)
            cout << " ";

        cout << "depth=" << treemap.depth(sibss) << ";" << (*sibss).name << ":" << (*sibss).value << "," << (*sibss).child_num << endl;

        if ((*sibss).name != "O")
        {
            //cout << (*sibss).rect.size.x << "," << (*sibss).rect.size.y << endl;
            rects.emplace_back((*sibss).rect);
        }

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

    scene->AddRects(rects);

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
