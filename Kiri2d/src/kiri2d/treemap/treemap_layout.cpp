/*** 
 * @Author: Xu.WANG
 * @Date: 2021-03-27 01:56:27
 * @LastEditTime: 2021-03-27 03:04:43
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\treemap\treemap_layout.cpp
 */

#include <kiri2d/treemap/treemap_layout.h>

namespace KIRI2D
{
    void TreemapLayout::ConstructTreemapLayout()
    {
        TreemapRecur(mTreemap, mDataTree);
    }

    void TreemapLayout::AddTreeNode(const TreemapNode &node)
    {
        mTreemap.append_child(mDataTree, node);
    }

    void TreemapLayout::AddTreeNodes(const std::vector<TreemapNode> &nodes)
    {
        for (size_t i = 0; i < nodes.size(); i++)
        {
            mTreemap.append_child(mDataTree, nodes[i]);
        }
    }

    void TreemapLayout::PrintTreemapLayout()
    {
        tree<TreemapNode>::iterator sibe = mTreemap.begin(mDataTree);
        tree<TreemapNode>::iterator end = mTreemap.end(mDataTree);

        while (sibe != end)
        {
            for (int k = 0; k < mTreemap.depth(sibe) - 1; ++k)
                std::cout << " ";

            KIRI_LOG_INFO("Depth={0},Name={1}:Value={2},ChildNum={3}",
                          mTreemap.depth(sibe), (*sibe).name, (*sibe).value, (*sibe).child_num);

            ++sibe;
        }
    }

    std::vector<KiriRect2> TreemapLayout::GetTreemapLayoutRect()
    {
        std::vector<KiriRect2> rects;
        tree<TreemapNode>::iterator sibe = mTreemap.begin(mDataTree);
        tree<TreemapNode>::iterator end = mTreemap.end(mDataTree);

        while (sibe != end)
        {
            if ((*sibe).name != mTempNodeName)
                rects.emplace_back((*sibe).rect);

            ++sibe;
        }

        return rects;
    }

    void TreemapLayout::GetSplitRect(KiriRect2 parentRect, KiriRect2 &left, KiriRect2 &right, const float valueA, const float valueB, const float total)
    {
        if (parentRect.size.x >= parentRect.size.y)
        {
            left.original = Vector2F(parentRect.original);
            left.size = Vector2F(valueA / total * parentRect.size.x, parentRect.size.y);

            right.original = Vector2F(parentRect.original.x + valueA / total * parentRect.size.x, parentRect.original.y);
            right.size = Vector2F(valueB / total * parentRect.size.x, parentRect.size.y);
        }
        else
        {
            left.original = Vector2F(parentRect.original.x, parentRect.original.y + valueB / total * parentRect.size.y);
            left.size = Vector2F(parentRect.size.x, valueA / total * parentRect.size.y);

            right.original = Vector2F(parentRect.original);
            right.size = Vector2F(parentRect.size.x, valueB / total * parentRect.size.y);
        }
    }

    void TreemapLayout::TreemapRecur(tree<TreemapNode> &map, tree<TreemapNode>::iterator &node)
    {
        float partA = 0.f, partB = (*node).value;
        size_t num = 0, childNum = (*node).child_num;

        auto sibs = map.begin(node);
        auto sibe = map.end(node);

        // find split point
        while (sibs != sibe)
        {
            ++num;
            partA += (*sibs).value;
            partB -= (*sibs).value;

            if (partA >= partB || num == childNum - 1)
                break;

            ++sibs;
        }

        if (partB < 0.f || partA < 0.f || partB > (*node).value || partA > (*node).value)
        {
            printf("node value=%.3f, partA=%.3f,partB=%.3f,num=%zd,child num=%zd,sibs value=%.3f \n", (*node).value, partA, partB, num, childNum, (*sibs).value);
        }

        KiriRect2 leftRect, rightRect;
        GetSplitRect((*node).rect, leftRect, rightRect, partA, partB, (*node).value);

        // right parts
        if ((*node).child_num - num > 1)
        {
            tree<TreemapNode>::iterator right = map.wrap(map.next_sibling(sibs), map.end(node), TreemapNode(mTempNodeName, partB, childNum - num, rightRect));
            TreemapRecur(map, right);
        }
        else
            (*map.next_sibling(sibs)).rect = rightRect;

        // left parts
        if (num > 1)
        {
            tree<TreemapNode>::iterator left = map.wrap(map.begin(node), map.next_sibling(sibs), TreemapNode(mTempNodeName, partA, num, leftRect));
            TreemapRecur(map, left);
        }
        else
            (*sibs).rect = leftRect;

        return;
    }
}