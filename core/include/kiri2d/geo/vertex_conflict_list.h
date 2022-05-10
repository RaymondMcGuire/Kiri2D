/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-05-24 10:42:43
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri\KiriCore\include\kiri2d\geo\vertex_conflict_list.h
 */

#ifndef _KIRI_VERTEX_CONFLICT_LISTS_H_
#define _KIRI_VERTEX_CONFLICT_LISTS_H_

#pragma once

#include <kiri2d/linked_list/doubly_linked_list.h>
#include <kiri2d/data/face3.h>

namespace KIRI
{

    class KiriVertexConflictLists : public KiriDoublyLinkedList<KiriFace3Ptr>
    {
    public:
        KiriVertexConflictLists() : KiriDoublyLinkedList<KiriFace3Ptr>(){};
        ~KiriVertexConflictLists(){};

        void BuildVisibleList(Vector<KiriFace3Ptr> &visList)
        {
            for (auto x = mFirst; x; x = x->next)
            {
                x->value->SetVisible(true);
                visList.emplace_back(x->value);
            }
        }

        void ResetVisible()
        {
            for (auto x = mFirst; x; x = x->next)
                x->value->SetVisible(false);
        }

        void PrintVertexConflictList()
        {
            KIRI_LOG_DEBUG("----------Vertex CONFLICT LIST----------");
            KIRI_LOG_DEBUG("vertex conflict list number={0}", this->Size());
            for (auto x = mFirst; x; x = x->next)
                KIRI_LOG_DEBUG("face id={0}, visible={1}", x->value->GetIdx(), x->value->GetVisible());

            KIRI_LOG_DEBUG("--------------------------------------");
        }
    };

    typedef SharedPtr<KiriVertexConflictLists> KiriVertexConflictListsPtr;
}
#endif