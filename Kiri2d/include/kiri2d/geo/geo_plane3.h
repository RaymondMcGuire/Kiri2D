/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-05-21 02:35:06
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri\KiriCore\include\kiri2d\geo\geo_plane3.h
 */

#ifndef _KIRI_GEO_PLANE3_H_
#define _KIRI_GEO_PLANE3_H_

#pragma once

#include <kiri_pch.h>

namespace KIRI
{
    // geo plane3 equation: ax + bx +cx +d=0
    class KiriGeoPlane3
    {
    public:
        explicit KiriGeoPlane3(const Vector3F &v1, const Vector3F &v2, const Vector3F &v3)
        {
            mCoefA = v1.y * (v2.z - v3.z) + v2.y * (v3.z - v1.z) + v3.y * (v1.z - v2.z);
            mCoefB = v1.z * (v2.x - v3.x) + v2.z * (v3.x - v1.x) + v3.z * (v1.x - v2.x);
            mCoefC = v1.x * (v2.y - v3.y) + v2.x * (v3.y - v1.y) + v3.x * (v1.y - v2.y);
            mCoefD = -1.f * (v1.x * (v2.y * v3.z - v3.y * v2.z) + v2.x * (v3.y * v1.z - v1.y * v3.z) + v3.x * (v1.y * v2.z - v2.y * v1.z));
        };

        ~KiriGeoPlane3() noexcept {};

        Vector2F GetDualPointMappedToPlane() const
        {
            auto nPlane = GetNormlizedZPlane();
            return Vector2F(nPlane.x / 2.f, nPlane.y / 2.f);
        }

        Vector3F GetNormlizedZPlane() const
        {
            return -1.f * Vector3F(mCoefA / mCoefC, mCoefB / mCoefC, mCoefA / mCoefD);
        }

    private:
        float mCoefA, mCoefB, mCoefC, mCoefD;
    };

    typedef SharedPtr<KiriGeoPlane3> KiriGeoPlane3Ptr;
}
#endif