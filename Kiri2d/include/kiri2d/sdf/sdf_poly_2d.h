/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 18:16:11
 * @LastEditTime: 2021-02-22 18:18:57
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\sdf\sdf_poly_2d.h
 */
#ifndef _KIRI2D_SDF_POLY2D_H_
#define _KIRI2D_SDF_POLY2D_H_

#pragma once

#include <kiri2d/sdf/sdf2d.h>

namespace KIRI2D
{
    class KiriSDFPoly2D : public KiriSDF2D
    {
    public:
        KiriSDFPoly2D(
            String name = "sdf_poly2d",
            Vector3F color = Vector3F(0.f),
            bool moveable = false)
            : KiriSDF2D(
                  name,
                  color,
                  moveable)
        {
        }

        ~KiriSDFPoly2D() noexcept {}
    };
    typedef SharedPtr<KiriSDF2D> KiriSDF2DPtr;
}
#endif