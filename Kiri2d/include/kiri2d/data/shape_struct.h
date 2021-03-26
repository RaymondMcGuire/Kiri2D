/*** 
 * @Author: Xu.WANG
 * @Date: 2021-03-27 01:49:01
 * @LastEditTime: 2021-03-27 01:50:20
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\data\shape_struct.h
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

        KiriPoint2(
            Vector2F _pos,
            Vector3F _col)
            : pos(_pos),
              col(_col) {}
    };

    struct KiriLine2
    {
        Vector2F start;
        Vector2F end;

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