/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 15:22:11
 * @LastEditTime: 2021-02-22 18:15:43
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\sdf\sdf2d.cpp
 */

#include <kiri2d/sdf/sdf2d.h>
#include <kiri2d/sdf/sdf_func.h>

namespace KIRI2D
{
    void KiriSDF2D::ReCalcBBox(Vector2F p)
    {
        mBBoxMin[0] = std::min(p[0], mBBoxMin[0]);
        mBBoxMin[1] = std::min(p[1], mBBoxMin[1]);

        mBBoxMax[0] = std::max(p[0], mBBoxMax[0]);
        mBBoxMax[1] = std::max(p[1], mBBoxMax[1]);
    }

    void KiriSDF2D::Append(Vector2F p, Vector2F v)
    {
        mPoints.emplace_back(p);
        mVelocities.emplace_back(v);

        this->ReCalcBBox(p);
    }

    const Int KiriSDF2D::FindRegion(Vector2F p)
    {
        Int res = 1;
        Int len = mPoints.size();
        for (size_t i = 0, j = len - 1; i < len; j = i++)
        {
            if ((mPoints[i].y > p.y) != (mPoints[j].y > p.y) &&
                (p.x < (mPoints[j].x - mPoints[i].x) * (p.y - mPoints[i].y) / (mPoints[j].y - mPoints[i].y) + mPoints[i].x))
            {
                res *= -1;
            }
        }

        return res;
    }

    const SDF2DInfo KiriSDF2D::CalcClosestDistance(Vector2F p)
    {
        auto minDis = Huge<float>();
        int startIdx = mPoints.size() - 1, endIdx = 0;

        // find the closest distance
        for (size_t i = startIdx, j = endIdx; j < mPoints.size(); i = j++)
        {
            auto dis = P2LS2D(p, mPoints[i], mPoints[j]);
            if (minDis > dis)
            {
                minDis = dis;
                startIdx = i;
                endIdx = j;
            }
        }

        Int sign = this->FindRegion(p);
        return SDF2DInfo(sign * minDis, startIdx, endIdx);
    }
}