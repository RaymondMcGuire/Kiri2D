/*** 
 * @Author: Xu.WANG
 * @Date: 2021-03-27 01:49:01
 * @LastEditTime: 2021-05-18 16:45:15
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\data\vertex3.h
 */

#ifndef _KIRI_VERTEX3_H_
#define _KIRI_VERTEX3_H_

#pragma once

#include <kiri_pch.h>

namespace KIRI2D
{
    class KiriVertex3
    {
    public:
        explicit KiriVertex3::KiriVertex3() : KiriVertex3(Vector3F(0.f)) {}

        explicit KiriVertex3::KiriVertex3(Vector3F v)
        {
            mIdx = -1;
            mValue = v;
        }

        ~KiriVertex3() noexcept {}

        void SetIdx(UInt idx) { mIdx = idx; }
        constexpr UInt GetIdx() { return mIdx; }

        void SetValue(const Vector3F &v) { mValue = v; }
        const Vector3F &GetValue() { return mValue; }

        const bool IsEqual(KiriVertex3 vert);
        bool LinearDependent(const Vector3F &v);

    private:
        Vector3F mValue;
        UInt mIdx;
    };
} // namespace KIRI

#endif /* _KIRI_VERTEX3_H_ */