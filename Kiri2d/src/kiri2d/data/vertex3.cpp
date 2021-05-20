/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 18:33:21
 * @LastEditTime: 2021-05-18 16:45:37
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\data\vertex3.cpp
 */

#include <kiri2d/data/vertex3.h>

namespace KIRI2D
{
    const bool KiriVertex3::IsEqual(KiriVertex3 vert)
    {
        // KIRI_LOG_DEBUG("vertex equal:{0},{1},{2} / {3},{4},{5} / {6},{7}", mValue.x, mValue.y, mValue.z, vert.GetValue().x, vert.GetValue().y, vert.GetValue().z, mIdx, vert.GetIdx());
        return (mValue.x == vert.GetValue().x && mValue.y == vert.GetValue().y && mValue.z == vert.GetValue().z && mIdx == vert.GetIdx());
    }

    bool KiriVertex3::LinearDependent(const Vector3F &v)
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
}