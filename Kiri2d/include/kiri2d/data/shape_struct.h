/***
 * @Author: Xu.WANG
 * @Date: 2021-08-23 18:20:43
 * @LastEditTime: 2021-12-12 17:15:08
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _KIRI2D_SHAPE_STRUCT_H_
#define _KIRI2D_SHAPE_STRUCT_H_

#pragma once

#include <kiri_pch.h>

namespace KIRI2D
{
    struct KiriPoint2
    {
        Vector2F pos;
        Vector3F col;
        float radius = 1.f;

        KiriPoint2(
            Vector2F _pos,
            Vector3F _col)
            : pos(_pos),
              col(_col) {}
    };

    struct KiriCircle2
    {
        Vector2F pos;
        Vector3F col;
        float radius;
        bool fill;

        KiriCircle2(
            Vector2F _pos,
            Vector3F _col,
            float _radius,
            bool _fill = true)
            : pos(_pos),
              col(_col),
              radius(_radius),
              fill(_fill) {}
    };

    struct KiriLine2
    {
        Vector2F start;
        Vector2F end;
        Vector3F col = Vector3F(253, 185, 134) / 255.f;
        float thick = 5.f;

        KiriLine2(
            Vector2F _start,
            Vector2F _end)
            : start(_start),
              end(_end) {}
    };

    struct KiriRect2
    {
        Vector2F original;
        Vector2F size;

        KiriRect2()
            : original(Vector2F(0.f)), size(Vector2F(0.f)) {}

        KiriRect2(
            Vector2F _original,
            Vector2F _size)
            : original(_original),
              size(_size) {}
    };

} // namespace KIRI

#endif /* _KIRI2D_SHAPE_STRUCT_H_ */