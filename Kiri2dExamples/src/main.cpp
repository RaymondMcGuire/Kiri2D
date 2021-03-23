/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-21 18:37:46
 * @LastEditTime: 2021-03-24 02:44:20
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2dExamples\src\main.cpp
 */

#include <kiri2d/renderer/renderer.h>
#include <kiri2d/sdf/sdf_poly_2d.h>
using namespace KIRI2D;

int main()
{
    // KiriSDFPoly2D boundary;
    // boundary.Append(Vector2F(0.3f, 0.3f));
    // boundary.Append(Vector2F(0.3f, 4.7f));
    // boundary.Append(Vector2F(4.7f, 4.7f));
    // boundary.Append(Vector2F(4.7f, 0.3f));

    std::vector<KiriLine2> edges;
    float width = 4.8f;
    float offset = 0.1f;

    edges.emplace_back(KiriLine2(Vector2F(0.511491, 0.151576) * width + offset, Vector2F(0.328024, 0.265777) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.328024, 0.265777) * width + offset, Vector2F(0.000000, 0.052614) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.000000, 0.052614) * width + offset, Vector2F(0.000000, 0.000000) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.000000, 0.000000) * width + offset, Vector2F(0.546750, 0.000000) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.546750, 0.000000) * width + offset, Vector2F(0.511491, 0.151576) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(1.000000, 0.418452) * width + offset, Vector2F(0.702465, 0.452691) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.702465, 0.452691) * width + offset, Vector2F(0.511491, 0.151576) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.511491, 0.151576) * width + offset, Vector2F(0.546750, 0.000000) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.546750, 0.000000) * width + offset, Vector2F(1.000000, 0.000000) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(1.000000, 0.000000) * width + offset, Vector2F(1.000000, 0.418452) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.328024, 0.265777) * width + offset, Vector2F(0.282745, 0.401936) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.282745, 0.401936) * width + offset, Vector2F(0.000000, 0.426054) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.000000, 0.426054) * width + offset, Vector2F(0.000000, 0.052614) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.000000, 0.052614) * width + offset, Vector2F(0.328024, 0.265777) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.702465, 0.452691) * width + offset, Vector2F(0.620256, 0.549363) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.620256, 0.549363) * width + offset, Vector2F(0.340386, 0.513157) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.340386, 0.513157) * width + offset, Vector2F(0.282745, 0.401936) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.282745, 0.401936) * width + offset, Vector2F(0.328024, 0.265777) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.328024, 0.265777) * width + offset, Vector2F(0.511491, 0.151576) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.511491, 0.151576) * width + offset, Vector2F(0.702465, 0.452691) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.340386, 0.513157) * width + offset, Vector2F(0.253343, 0.688532) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.253343, 0.688532) * width + offset, Vector2F(0.000000, 0.661637) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.000000, 0.661637) * width + offset, Vector2F(0.000000, 0.426054) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.000000, 0.426054) * width + offset, Vector2F(0.282745, 0.401936) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.282745, 0.401936) * width + offset, Vector2F(0.340386, 0.513157) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.613220, 0.679874) * width + offset, Vector2F(0.478311, 0.804271) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.478311, 0.804271) * width + offset, Vector2F(0.282452, 0.741229) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.282452, 0.741229) * width + offset, Vector2F(0.253343, 0.688532) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.253343, 0.688532) * width + offset, Vector2F(0.340386, 0.513157) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.340386, 0.513157) * width + offset, Vector2F(0.620256, 0.549363) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.620256, 0.549363) * width + offset, Vector2F(0.613220, 0.679874) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.929682, 1.000000) * width + offset, Vector2F(0.613220, 0.679874) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.613220, 0.679874) * width + offset, Vector2F(0.620256, 0.549363) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.620256, 0.549363) * width + offset, Vector2F(0.702465, 0.452691) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.702465, 0.452691) * width + offset, Vector2F(1.000000, 0.418452) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(1.000000, 0.418452) * width + offset, Vector2F(1.000000, 1.000000) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(1.000000, 1.000000) * width + offset, Vector2F(0.929682, 1.000000) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.282451, 0.741229) * width + offset, Vector2F(0.198055, 1.000000) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.198055, 1.000000) * width + offset, Vector2F(0.000000, 1.000000) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.000000, 1.000000) * width + offset, Vector2F(0.000000, 0.661637) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.000000, 0.661637) * width + offset, Vector2F(0.253343, 0.688532) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.253343, 0.688532) * width + offset, Vector2F(0.282452, 0.741229) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.489985, 1.000000) * width + offset, Vector2F(0.478311, 0.804271) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.478311, 0.804271) * width + offset, Vector2F(0.613220, 0.679874) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.613220, 0.679874) * width + offset, Vector2F(0.929682, 1.000000) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.929682, 1.000000) * width + offset, Vector2F(0.489985, 1.000000) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.478311, 0.804271) * width + offset, Vector2F(0.489985, 1.000000) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.489985, 1.000000) * width + offset, Vector2F(0.198055, 1.000000) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.198055, 1.000000) * width + offset, Vector2F(0.282451, 0.741229) * width + offset));
    edges.emplace_back(KiriLine2(Vector2F(0.282452, 0.741229) * width + offset, Vector2F(0.478311, 0.804271) * width + offset));

    std::vector<KiriPoint2> points;
    points.emplace_back(KiriPoint2(Vector2F(0.149758, 0.286679) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    points.emplace_back(KiriPoint2(Vector2F(0.749196, 0.204762) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    points.emplace_back(KiriPoint2(Vector2F(0.458271, 0.389274) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    points.emplace_back(KiriPoint2(Vector2F(0.350847, 0.886708) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    points.emplace_back(KiriPoint2(Vector2F(0.614671, 0.870973) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    points.emplace_back(KiriPoint2(Vector2F(0.141437, 0.818411) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    points.emplace_back(KiriPoint2(Vector2F(0.274716, 0.094388) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    points.emplace_back(KiriPoint2(Vector2F(0.804290, 0.683524) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    points.emplace_back(KiriPoint2(Vector2F(0.422865, 0.662960) * width + offset, Vector3F(1.f, 0.f, 0.f)));
    points.emplace_back(KiriPoint2(Vector2F(0.171200, 0.538052) * width + offset, Vector3F(1.f, 0.f, 0.f)));

    auto scene = std::make_shared<KiriScene2D>(700, 700);
    scene->AddLines(edges);
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