/***
 * @Author: Xu.WANG
 * @Date: 2020-10-19 18:18:39
 * @LastEditTime: 2021-03-01 12:18:38
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri\KiriCore\include\kiri_core\model\model_tiny_obj_loader.h
 * @Reference:https://ttnghia.github.io ; Banana
 */

#ifndef _KIRI_MODEL_TINY_OBJ_LOADER_H_
#define _KIRI_MODEL_TINY_OBJ_LOADER_H_
#pragma once
#include <kiri_pch.h>

class KiriModelTinyObjLoader
{
public:
    KiriModelTinyObjLoader() { ClearData(); }
    KiriModelTinyObjLoader(const String &name, const String &folder, const String &ext);

    bool Load(const String &filePath);
    void ScaleToBox(float BoxScale);
    void Normalize();

    auto GetMeshCenter() const
    {
        KIRI_ASSERT(mMeshReady);
        return float(0.5) * (mAABBMin + mAABBMax);
    }
    auto GetNTriangles() const { return mNumTriangles; }
    const auto &GetAABBMin() const
    {
        KIRI_ASSERT(mMeshReady);
        return mAABBMin;
    }
    const auto &GetAABBMax() const
    {
        KIRI_ASSERT(mMeshReady);
        return mAABBMax;
    }

    const auto &GetVertices() const
    {
        KIRI_ASSERT(mMeshReady);
        return mVertices;
    }
    const auto &GetNormals() const
    {
        KIRI_ASSERT(mMeshReady);
        return mNormals;
    }
    const auto &GetTexCoord2D() const
    {
        KIRI_ASSERT(mMeshReady);
        return mTexCoord2D;
    }

    const auto &GetFaces() const
    {
        KIRI_ASSERT(mMeshReady);
        return mFaces;
    }
    const auto &GetFaceVertices() const
    {
        KIRI_ASSERT(mMeshReady);
        return mFaceVertices;
    }
    const auto &GetFaceVertexNormals() const
    {
        KIRI_ASSERT(mMeshReady);
        return mFaceVertexNormals;
    }
    const auto &GetFaceVertexColors() const
    {
        KIRI_ASSERT(mMeshReady);
        return mFaceVertexColors;
    }
    const auto &GetFaceVTexCoord2D() const
    {
        KIRI_ASSERT(mMeshReady);
        return mFaceVertexTexCoord2D;
    }

    auto GetNFaces() const
    {
        KIRI_ASSERT(mMeshReady);
        return (mFaces.size() / 3);
    }
    auto GetNVertices() const
    {
        KIRI_ASSERT(mMeshReady);
        return (mVertices.size() / 3);
    }
    auto GetNFaceVertices() const
    {
        KIRI_ASSERT(mMeshReady);
        return (mFaceVertices.size() / 3);
    }

private:
    void ComputeFaceVertexData();
    void ClearData();

    String mName;
    String mExtension;
    String mFolder;

    bool mMeshReady = false;
    UInt mNumTriangles = 0;

    Vector3F mAABBMin;
    Vector3F mAABBMax;

    // TODO change type to uint
    Vec_Float mFaces;
    Vec_Float mVertices;
    Vec_Float mNormals;
    Vec_Float mTexCoord2D;
    Vec_Float mFaceVertices;
    Vec_Float mFaceVertexNormals;
    Vec_Float mFaceVertexColors;
    Vec_Float mFaceVertexTexCoord2D;
};

typedef std::shared_ptr<KiriModelTinyObjLoader> KiriModelTinyObjLoaderPtr;

#endif
