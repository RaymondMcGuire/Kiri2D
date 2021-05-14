/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-21 18:37:46
 * @LastEditTime: 2021-05-14 15:53:01
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dExamples\src\main.cpp
 */

#include <kiri2d/linked_list/doubly_linked_list.h>

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
