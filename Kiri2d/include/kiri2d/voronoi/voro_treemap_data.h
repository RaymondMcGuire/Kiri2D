/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-28 10:09:23
 * @LastEditTime: 2021-06-24 15:42:59
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\voronoi\voro_treemap_data.h
 */

#ifndef _KIRI_VORO_TREEMAP_DATA_H_
#define _KIRI_VORO_TREEMAP_DATA_H_

#pragma once

#include <kiri_pch.h>

namespace KIRI
{

    struct VoroTreeMapNode
    {
        UInt depth;
        UInt id;
        UInt pid;
        float weight;
        UInt child_num;
    };

    class KiriVoroTreeMapData
    {
    public:
        KiriVoroTreeMapData() {}

        ~KiriVoroTreeMapData() noexcept {}

        void Clear() { mData.clear(); }
        void AddNode(VoroTreeMapNode node)
        {
            mData[node.id] = node;
        }

        UInt GetTotalNodeNum() const { return mData.size(); }

        UInt GetMaxDepth()
        {
            auto max = 0;
            for (const auto &[key, value] : mData)
            {
                if (value.depth > max)
                    max = value.depth;
            }

            return max;
        }

        VoroTreeMapNode GetTreeMapNodeById(UInt idx) { return mData[idx]; }

        float GetChildTotalWeightById(UInt idx)
        {
            auto w = 0.f;
            for (const auto &[key, value] : mData)
                if (value.pid == idx)
                    w += value.weight;

            return w;
        }

        Vector<VoroTreeMapNode> GetChildNodesById(UInt idx)
        {
            Vector<VoroTreeMapNode> child;
            for (const auto &[key, value] : mData)
                if (value.pid == idx)
                    child.emplace_back(value);

            return child;
        }

        Vector<VoroTreeMapNode> GetLeafChildNodes()
        {
            Vector<VoroTreeMapNode> child;
            for (const auto &[key, value] : mData)
                if (value.child_num == 0)
                    child.emplace_back(value);

            return child;
        }

        float GetLeafChildTotalWeight()
        {
            auto w = 0.f;
            for (const auto &[key, value] : mData)
                if (value.child_num == 0)
                    w += value.weight;
            return w;
        }

        Vector<VoroTreeMapNode> GetNodesByDepth(UInt depth)
        {
            Vector<VoroTreeMapNode> nodes;
            for (const auto &[key, value] : mData)
                if (value.depth == depth)
                    nodes.emplace_back(value);

            return nodes;
        }

        VoroTreeMapNode GetRootNode()
        {
            for (const auto &[key, value] : mData)
                if (value.depth == 0)
                    return value;
            return VoroTreeMapNode();
        }

    private:
        UnSortedMap<UInt, VoroTreeMapNode> mData;
    };

    typedef SharedPtr<KiriVoroTreeMapData> KiriVoroTreeMapDataPtr;
}
#endif