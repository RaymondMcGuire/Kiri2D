/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-21 18:37:46
 * @LastEditTime: 2021-02-23 01:08:41
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dExamples\src\main.cpp
 */

#include <kiri2d/renderer/renderer.h>
#include <kiri2d/sdf/sdf_poly_2d.h>
using namespace KIRI2D;

int main()
{
    KiriSDFPoly2D boundary;
    boundary.Append(Vector2F(0.3f, 0.3f));
    boundary.Append(Vector2F(0.3f, 4.7f));
    boundary.Append(Vector2F(4.7f, 4.7f));
    boundary.Append(Vector2F(4.7f, 0.3f));

    auto scene = std::make_shared<KiriScene2D>(700, 700);
    scene->AddObject(boundary);
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