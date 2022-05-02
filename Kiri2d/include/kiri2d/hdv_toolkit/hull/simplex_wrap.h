/***
 * @Author: Xu.WANG
 * @Date: 2021-12-23 17:57:21
 * @LastEditTime: 2022-01-02 20:24:33
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _HDV_SIMPLEX_WRAP_H_
#define _HDV_SIMPLEX_WRAP_H_

#pragma once

#include <kiri_pch.h>
namespace HDV::Hull
{
    class SimplexWrap
    {
    public:
        explicit SimplexWrap() : Next{nullptr}, Prev{} {}
        explicit SimplexWrap(
            int dimension,
            std::vector<int> &beyondList)
            : Next{nullptr}, Prev{}
        {
            Normals.assign(dimension, 0.0);

            Vertices.assign(dimension, -1);
            AdjacentFaces.assign(dimension, std::make_shared<SimplexWrap>());
            BeyondList = beyondList;
        }
        virtual ~SimplexWrap()
        {
        }

        void Clear()
        {
            Next = nullptr;
            Prev.reset();
            FurthestVertex = -1;
            BeyondList.clear();
            VerticesBeyond.clear();
            Vertices.clear();
            AdjacentFaces.clear();
        }

        std::shared_ptr<SimplexWrap> Next;
        std::weak_ptr<SimplexWrap> Prev;

        double Offset = 0.0;
        bool IsNormalFlipped = false;

        bool GetInList() const { return mInList; }
        void SetInList(bool il) { mInList = il; }

        int GetTag() const { return mTag; }
        void SetTag(int tag) { mTag = tag; }

        std::vector<double> Normals;

        std::vector<int> Vertices;
        int FurthestVertex;

        std::vector<int> BeyondList;
        std::vector<int> VerticesBeyond;
        std::vector<std::shared_ptr<SimplexWrap>> AdjacentFaces;

        void ToString()
        {
            std::string vert_data = "";
            for (size_t i = 0; i < Vertices.size(); i++)
            {
                vert_data += std::to_string(Vertices[i]) + "->";
            }

            KIRI_LOG_DEBUG("Tag = {0}; InList = {1}; Verts={2}; Normal={3},{4},{5}; Offset={6}; IsNormalFlipped={7}, FurthestVertex={8}",
                           mTag,
                           mInList,
                           vert_data,
                           Normals[0], Normals[1], Normals[2], Offset, IsNormalFlipped,
                           FurthestVertex);
        }

    private:
        int mTag = 0;
        bool mInList = false;
    };

} // namespace HDV::Hull

#endif /* _HDV_SIMPLEX_WRAP_H_ */