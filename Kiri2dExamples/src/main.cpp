/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-21 18:37:46
 * @LastEditTime: 2021-05-19 03:36:05
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dExamples\src\main.cpp
 */

#include <kiri2d/linked_list/doubly_linked_list.h>
#include <kiri2d/geo/convex_hull3.h>

using namespace KIRI2D;

int main()
{
    KIRI::KiriLog::Init();

    auto v = KiriDoublyLinkedList<int>();
    v.Push(15);
    v.Push(17);
    v.Push(12);
    v.Push(14);
    v.Push(21);

    v.Print();
    v.PrintReverse();

    v.Remove([](int x)
             { return x % 2 == 1; });

    v.Print();

    v.RemoveAll();
    v.Print();

    Vector<int> intList;
    intList.emplace_back(3);
    intList.emplace_back(5);
    intList.emplace_back(6);
    intList.emplace_back(1);
    intList.emplace_back(4);
    intList.emplace_back(6);
    intList.emplace_back(9);

    String printStr = "[ ";
    for (size_t i = 0; i < intList.size(); i++)
    {
        printStr += std::to_string(intList[i]) + " ";
    }
    printStr += "]";
    KIRI_LOG_DEBUG("int list = {0}", printStr);

    auto lastElem = intList.back();
    intList.pop_back();
    intList[3] = lastElem;

    printStr = "[ ";
    for (size_t i = 0; i < intList.size(); i++)
    {
        printStr += std::to_string(intList[i]) + " ";
    }
    printStr += "]";
    KIRI_LOG_DEBUG("int list = {0}", printStr);

    // // face
    // auto v1 = KiriVertex3(Vector3F(0.f));
    // auto v2 = KiriVertex3(Vector3F(10.f, 0.f, 0.f));
    // auto v3 = KiriVertex3(Vector3F(5.f, 5.f, 0.f));
    // auto f = std::make_shared<KiriFace3>(v1, v2, v3);
    // f->PrintFaceInfo();

    // auto e1 = f->GetEdgesByIdx(0);
    // e1->PrintEdgeInfo();
    // e1->GetNextEdge()->PrintEdgeInfo();
    // e1->GetPrevEdge()->PrintEdgeInfo();

    // convex hull 3d
    auto ch3 = std::make_shared<KiriConvexHull3>();
    ch3->AddVertex(Vector3F(1.f));
    ch3->AddVertex(Vector3F(2.f));
    ch3->AddVertex(Vector3F(3.f));
    ch3->AddVertex(Vector3F(1.f, 2.f, 3.f));
    ch3->AddVertex(Vector3F(3.f, 2.f, 5.f));

    ch3->PrintVertexInfo();

    ch3->ComputeConvexHull();

    ch3->PrintVertexInfo();
    ch3->PrintCurFacetsInfo();
    ch3->PrintConflictGraphInfo();
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
