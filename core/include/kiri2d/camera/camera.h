/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2021-02-23 00:18:39
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-10-16 13:58:26
 * @FilePath: \Kiri2D\core\include\kiri2d\camera\camera.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _KIRI2D_CAMERA_H_
#define _KIRI2D_CAMERA_H_

#pragma once

#include <kiri_pch.h>

namespace KIRI2D
{

    struct Camera2DProperty
    {
        Vector2F LookAt;
        Vector2F WindowCellSize;
        Vector2F WindowCenter;
        float ViewDistance;

        Camera2DProperty(
            Vector2F lookAt,
            Vector2F windowCellSize,
            Vector2F windowCenter,
            float viewDistance = 1.f)
            : LookAt(lookAt),
              WindowCellSize(windowCellSize),
              WindowCenter(windowCenter),
              ViewDistance(viewDistance)
        {
        }
    };

    class KiriCamera2D
    {
    public:
        KiriCamera2D(const Camera2DProperty &cameraProperty)
        {
            mRotateMatrix = Matrix2x2F::identity();
            mCameraData.LookAt = cameraProperty.LookAt;
            mCameraData.WindowCellSize = cameraProperty.WindowCellSize;
            mCameraData.WindowCenter = cameraProperty.WindowCenter;
            mCameraData.ViewDistance = cameraProperty.ViewDistance;
            mCameraData.CameraDistance = 1.f;
        }

        inline const float viewScale()
        {
            return mCameraData.CameraDistance / mCameraData.ViewDistance;
        }

        inline const Vector2F project(Vector2F pixel)
        {
            auto rel_position = mRotateMatrix * (pixel - mCameraData.LookAt) * this->viewScale();

            return rel_position / mCameraData.WindowCellSize + mCameraData.WindowCenter;
        }

        ~KiriCamera2D() {}

    private:
        struct Camera2DData
        {
            Vector2F LookAt;
            Vector2F WindowCellSize;
            Vector2F WindowCenter;

            float ViewDistance;
            float CameraDistance;
        };

        Camera2DData mCameraData;
        Matrix2x2F mRotateMatrix;
    };
    typedef SharedPtr<KiriCamera2D> KiriCamera2DPtr;
}
#endif