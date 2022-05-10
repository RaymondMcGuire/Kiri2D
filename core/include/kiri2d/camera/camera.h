/***
 * @Author: Xu.WANG
 * @Date: 2021-02-22 13:41:44
 * @LastEditTime: 2021-02-22 14:44:00
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\camera\camera.h
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

        // Constructor
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

        inline const float ViewScale()
        {
            return mCameraData.CameraDistance / mCameraData.ViewDistance;
        }

        inline const Vector2F Project(Vector2F pixel)
        {
            auto relPosition = mRotateMatrix * (pixel - mCameraData.LookAt) * this->ViewScale();

            return relPosition / mCameraData.WindowCellSize + mCameraData.WindowCenter;
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