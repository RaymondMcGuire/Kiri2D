/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2023-01-11 14:46:17
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2023-01-25 23:08:38
 * @FilePath: \Kiri2D\core\include\kiri2d\data\shape_struct.h
 * @Description:
 * @Copyright (c) 2023 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _KIRI2D_SHAPE_STRUCT_H_
#define _KIRI2D_SHAPE_STRUCT_H_

#pragma once

#include <kiri_pch.h>

namespace KIRI2D
{
    template <class RealType>
    struct KiriPoint2
    {
        VectorX<2, RealType> pos;
        VectorX<3, RealType> col;
        RealType radius;

        KiriPoint2(
            VectorX<2, RealType> _pos,
            VectorX<3, RealType> _col,
            RealType _radius = static_cast<RealType>(1.0))
            : pos(_pos),
              col(_col),
              radius(_radius) {}
    };

    template <class RealType>
    struct KiriCircle2
    {
        VectorX<2, RealType> pos;
        VectorX<3, RealType> col;
        RealType radius;
        bool fill;

        KiriCircle2(
            VectorX<2, RealType> _pos,
            VectorX<3, RealType> _col,
            RealType _radius,
            bool _fill = true)
            : pos(_pos),
              col(_col),
              radius(_radius),
              fill(_fill) {}
    };

    template <class RealType>
    struct KiriLine2
    {
        VectorX<2, RealType> start;
        VectorX<2, RealType> end;
        VectorX<3, RealType> col = VectorX<3, RealType>(253, 185, 134) / static_cast<RealType>(255.0);
        RealType thick;

        KiriLine2(
            VectorX<2, RealType> _start,
            VectorX<2, RealType> _end,
            RealType _thick = 1.f)
            : start(_start),
              end(_end),
              thick(_thick) {}
    };

    template <class RealType>
    struct KiriRect2
    {
        VectorX<2, RealType> original;
        VectorX<2, RealType> size;

        KiriRect2()
            : original(VectorX<2, RealType>(0)), size(VectorX<2, RealType>(0)) {}

        KiriRect2(
            VectorX<2, RealType> _original,
            VectorX<2, RealType> _size)
            : original(_original),
              size(_size) {}
    };

} // namespace KIRI

#endif /* _KIRI2D_SHAPE_STRUCT_H_ */