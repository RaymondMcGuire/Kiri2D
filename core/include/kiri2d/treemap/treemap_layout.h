/***
 * @Author: Xu.WANG
 * @Date: 2021-03-27 01:42:49
 * @LastEditTime: 2021-06-01 20:11:05
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\treemap\treemap_layout.h
 */

#ifndef _KIRI2D_TREEMAP_LAYOUT_H_
#define _KIRI2D_TREEMAP_LAYOUT_H_

#pragma once

#include <treehh/tree.hh>
#include <kiri2d/data/shape_struct.h>
#include <kiri2d/voronoi/voro_treemap_data.h>

using namespace KIRI2D;

namespace KIRI
{

    struct TreemapNode
    {
        String name;
        UInt id;
        UInt pid;
        float value;
        size_t child_num;
        KiriRect2 rect;

        TreemapNode() : name("TreemapNode"), id(-1), pid(-1), value(0.f), child_num(0) {}

        TreemapNode(String _name, UInt _id, UInt _pid, float _value, size_t _child_num)
            : name(name), id(_id), pid(_pid), value(_value), child_num(_child_num) {}

        TreemapNode(String _name, UInt _id, UInt _pid, float _value, size_t _child_num, KiriRect2 _rect)
            : name(_name), id(_id), pid(_pid), value(_value), child_num(_child_num), rect(_rect) {}
    };

    class TreemapLayout
    {
    public:
        explicit TreemapLayout(
            const TreemapNode topNode,
            const String &tempNodeName = "O")
            : mTempNodeName(tempNodeName)
        {
            mTopTree = mTreemap.begin();
            mDataTree = mTreemap.insert(mTopTree, topNode);
            mVoroTreeMapData = std::make_shared<KiriVoroTreeMapData>();
        }

        ~TreemapLayout() {}

        const KiriVoroTreeMapDataPtr &Convert2VoroTreeData();

        void ConstructTreemapLayout();

        void AddTreeNode(const TreemapNode &node);
        void AddTreeNodes(const std::vector<TreemapNode> &nodes);

        std::vector<KiriRect2> GetTreemapLayoutRect();
        void PrintTreemapLayout();

    private:
        String mTempNodeName;
        UInt mCnt = 1;
        tree<TreemapNode> mTreemap;
        tree<TreemapNode>::iterator mTopTree, mDataTree;
        KiriVoroTreeMapDataPtr mVoroTreeMapData;
        void TreemapRecur(tree<TreemapNode> &map, tree<TreemapNode>::iterator &node);
        void GetSplitRect(KiriRect2 parentRect, KiriRect2 &left, KiriRect2 &right, const float valueA, const float valueB, const float total);
    };

    typedef SharedPtr<TreemapLayout> TreemapLayoutPtr;
} // namespace KIRI

#endif /* _KIRI2D_TREEMAP_LAYOUT_H_ */
