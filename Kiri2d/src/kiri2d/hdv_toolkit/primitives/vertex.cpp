/***
 * @Author: Xu.WANG
 * @Date: 2021-12-01 18:54:58
 * @LastEditTime: 2021-12-01 18:54:59
 * @LastEditors: Xu.WANG
 * @Description:
 */

#include <kiri2d/hdv_toolkit/primitives/vertex.h>

namespace HDV::Primitives
{
    float Vertex::Magnitude()
    {
        return std::sqrtf(SqrMagnitude());
    }

    float Vertex::SqrMagnitude()
    {

        auto sum = 0.f;
        auto dim = GetDimension();

        for (auto i = 0; i < dim; i++)
            sum += mPosition[i] * mPosition[i];

        return sum;
    }

    float Vertex::Distance(Vertex v)
    {
        return std::sqrtf(SqrDistance(v));
    }

    float Vertex::SqrDistance(Vertex v)
    {
        auto dim = std::min(GetDimension(), v.GetDimension());
        auto sum = 0.f;

        for (auto i = 0; i < dim; i++)
        {
            float x = mPosition[i] - v.GetPosition()[i];
            sum += x * x;
        }

        return sum;
    }

    void Vertex::ToString()
    {
        auto dim = GetDimension();
        std::string data = "";
        for (auto i = 0; i < dim; i++)
        {
            data += std::to_string(mPosition[i]);
            if (i != dim - 1)
                data += ",";
        }

        KIRI_LOG_DEBUG("[Vertex: Dimension={0},  Data={1}]", dim, data);
    }
}