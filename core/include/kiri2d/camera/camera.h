/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-02-23 00:18:39
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2023-01-25 23:29:16
 * @FilePath: \Kiri2D\core\include\kiri2d\camera\camera.h
 * @Description:
 * @Copyright (c) 2023 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _KIRI2D_CAMERA_H_
#define _KIRI2D_CAMERA_H_

#pragma once

#include <kiri_pch.h>

namespace KIRI2D
{

    template <class RealType>
    struct Camera2DProperty
    {
        VectorX<2, RealType> LookAt;
        VectorX<2, RealType> WindowCellSize;
        VectorX<2, RealType> WindowCenter;
        RealType ViewDistance;

        Camera2DProperty(
            VectorX<2, RealType> lookAt,
            VectorX<2, RealType> windowCellSize,
            VectorX<2, RealType> windowCenter,
            RealType viewDistance = 1.f)
            : LookAt(lookAt),
              WindowCellSize(windowCellSize),
              WindowCenter(windowCenter),
              ViewDistance(viewDistance)
        {
        }
    };

    template <class RealType>
    class KiriCamera2D
    {
    public:
        KiriCamera2D(const Camera2DProperty<RealType> &cameraProperty)
        {
            mRotateMatrix = Matrix2x2<RealType>::identity();
            mCameraData.LookAt = cameraProperty.LookAt;
            mCameraData.WindowCellSize = cameraProperty.WindowCellSize;
            mCameraData.WindowCenter = cameraProperty.WindowCenter;
            mCameraData.ViewDistance = cameraProperty.ViewDistance;
            mCameraData.CameraDistance = 1.f;
        }

        inline const RealType viewScale()
        {
            return mCameraData.CameraDistance / mCameraData.ViewDistance;
        }

        inline const VectorX<2, RealType> project(VectorX<2, RealType> pixel)
        {
            auto rel_position = mRotateMatrix * (pixel - mCameraData.LookAt) * this->viewScale();

            return rel_position / mCameraData.WindowCellSize + mCameraData.WindowCenter;
        }

        ~KiriCamera2D() {}

    private:
        struct Camera2DData
        {
            VectorX<2, RealType> LookAt;
            VectorX<2, RealType> WindowCellSize;
            VectorX<2, RealType> WindowCenter;

            RealType ViewDistance;
            RealType CameraDistance;
        };

        Camera2DData mCameraData;
        Matrix2x2<RealType> mRotateMatrix;
    };
    typedef SharedPtr<KiriCamera2D<float>> KiriCamera2DFPtr;
    typedef SharedPtr<KiriCamera2D<double>> KiriCamera2DDPtr;
}
#endif