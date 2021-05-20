/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-05-18 21:13:09
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\geo\vertex_conflict_list.h
 */

#ifndef _KIRI_VERTEX_CONFLICT_LISTS_H_
#define _KIRI_VERTEX_CONFLICT_LISTS_H_

#pragma once

#include <kiri2d/linked_list/doubly_linked_list.h>
#include <kiri2d/data/face3.h>

namespace KIRI2D
{

    class KiriVertexConflictLists : public KiriDoublyLinkedList<KiriFace3>
    {
    public:
        KiriVertexConflictLists() : KiriDoublyLinkedList<KiriFace3>(){};
        ~KiriVertexConflictLists(){};

        void BuildVisibleList(Vector<KiriFace3> &visList)
        {
            for (auto x = first; x; x = x->next)
            {
                x->value.SetVisible(true);
                visList.emplace_back(x->value);
            }
        }

        void PrintVertexConflictList()
        {
            KIRI_LOG_DEBUG("----------Vertex CONFLICT LIST----------");
            KIRI_LOG_DEBUG("vertex conflict list number={0}", this->Size());
            for (auto x = first; x; x = x->next)
                KIRI_LOG_DEBUG("face id={0}, visible={1}", x->value.GetIdx(), x->value.GetVisible());

            KIRI_LOG_DEBUG("--------------------------------------");
        }
    };

    typedef SharedPtr<KiriVertexConflictLists> KiriVertexConflictListsPtr;
}
#endif