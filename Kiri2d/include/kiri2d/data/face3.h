/*** 
 * @Author: Xu.WANG
 * @Date: 2021-03-27 01:49:01
 * @LastEditTime: 2021-05-18 20:41:27
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\data\face3.h
 */

#ifndef _KIRI_FACE3_H_
#define _KIRI_FACE3_H_

#pragma once

#include <kiri2d/data/edge3.h>

namespace KIRI2D
{
    class KiriFace3
    {
    public:
        explicit KiriFace3::KiriFace3() : KiriFace3(KiriVertex3(), KiriVertex3(), KiriVertex3()) {}

        explicit KiriFace3::KiriFace3(KiriVertex3 a, KiriVertex3 b, KiriVertex3 c, KiriVertex3 orient)
            : KiriFace3(a, b, c)
        {
            OrientFace(orient);
        }

        explicit KiriFace3::KiriFace3(KiriVertex3 a, KiriVertex3 b, KiriVertex3 c)
        {
            mIdx = -1;
            mVisible = false;

            mVertices[0] = a;
            mVertices[1] = b;
            mVertices[2] = c;

            mNormal = (-((b.GetValue() - a.GetValue()).cross(c.GetValue() - b.GetValue()))).normalized();
        }

        ~KiriFace3() noexcept {}

        inline constexpr UInt GetIdx() { return mIdx; }
        inline constexpr bool GetVisible() { return mVisible; }
        inline constexpr Int GetEdgeCount() const { return 3; }
        inline const Vector3F &GetNormal() const { return mNormal; }
        inline const KiriEdge3Ptr &GetEdgesByIdx(Int idx) const { return mEdges[idx]; }
        inline KiriVertex3 GetVertexByIdx(Int idx) const { return mVertices[idx]; }

        void SetIdx(UInt idx)
        {
            mIdx = idx;
            CreateEdges();
        }

        void SetVisible(bool vis)
        {
            mVisible = vis;
        }

        inline const bool CheckConflict(const Vector3F &v)
        {
            return (mNormal.dot(v) > (mNormal.dot(mVertices[0].GetValue()) + MEpsilon<float>()));
        }

        const KiriEdge3Ptr &GetEdge(KiriVertex3 a, KiriVertex3 b);

        void OrientFace(KiriVertex3 orient);
        void PrintFaceInfo();

        void LinkFace(KiriFace3 f, KiriVertex3 a, KiriVertex3 b);

    private:
        UInt mIdx;
        bool mVisible;
        KiriVertex3 mVertices[3];
        Vector3F mNormal;
        KiriEdge3Ptr mEdges[3];

        /*** 
         * @description: Check the relation between point v and face
         * @param {Vector3F} v
         * @return {true=blow false=above}
         */
        inline const bool CheckFaceDir(const Vector3F &v)
        {
            return (mNormal.dot(v) < mNormal.dot(mVertices[0].GetValue()));
        }

        void ReverseFaceDir();
        void CreateEdges();
    };
    typedef SharedPtr<KiriFace3> KiriFace3Ptr;
} // namespace KIRI

#endif /* _KIRI_FACE3_H_ */