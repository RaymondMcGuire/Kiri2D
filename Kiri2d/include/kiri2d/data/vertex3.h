/*** 
 * @Author: Xu.WANG
 * @Date: 2021-03-27 01:49:01
 * @LastEditTime: 2021-05-18 01:21:36
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

        const bool IsEqual(KiriVertex3 vert)
        {
            // KIRI_LOG_DEBUG("vertex equal:{0},{1},{2} / {3},{4},{5} / {6},{7}", mValue.x, mValue.y, mValue.z, vert.GetValue().x, vert.GetValue().y, vert.GetValue().z, mIdx, vert.GetIdx());
            return (mValue.x == vert.GetValue().x && mValue.y == vert.GetValue().y && mValue.z == vert.GetValue().z && mIdx == vert.GetIdx());
        }

        bool LinearDependent(const Vector3F &v)
        {
            //KIRI_LOG_DEBUG("check linearly dependent, a={0},{1},{2}, b={3},{4},{5}", mValue.x, mValue.y, mValue.z, v.x, v.y, v.z);
            auto epsilon = MEpsilon<float>();
            if (mValue.x == 0 && v.x == 0)
            {
                if (mValue.y == 0 && v.y == 0)
                {
                    if (mValue.z == 0 && v.z == 0)
                        return true;

                    if (mValue.z == 0 || v.z == 0)
                        return false;

                    return true;
                }

                if (mValue.y == 0 || v.y == 0)
                    return false;

                if (mValue.z / mValue.y >= v.z / v.y - epsilon && mValue.z / mValue.y <= v.z / v.y + epsilon)
                    return true;
                else
                    return false;
            }

            if (mValue.x == 0 || v.x == 0)
                return false;

            if (mValue.y / mValue.x <= v.y / v.x + epsilon &&
                mValue.y / mValue.x >= v.y / v.x - epsilon &&
                mValue.z / mValue.x >= v.y / v.x - epsilon &&
                mValue.z / mValue.x <= v.z / v.x + epsilon)
                return true;
            else
                return false;
        }

    private:
        Vector3F mValue;
        UInt mIdx;
    };
} // namespace KIRI

#endif /* _KIRI_VERTEX3_H_ */