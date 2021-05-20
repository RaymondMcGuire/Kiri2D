/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-05-18 16:25:41
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\geo\face_conflict_list.h
 */

#ifndef _KIRI_FACE_CONFLICT_LISTS_H_
#define _KIRI_FACE_CONFLICT_LISTS_H_

#pragma once

#include <kiri2d/linked_list/doubly_linked_list.h>
#include <kiri2d/data/vertex3.h>

namespace KIRI2D
{

    class KiriFaceConflictLists : public KiriDoublyLinkedList<KiriVertex3>
    {
    public:
        KiriFaceConflictLists() : KiriDoublyLinkedList<KiriVertex3>(){};
        ~KiriFaceConflictLists(){};

        void PrintFaceConflictList()
        {
            KIRI_LOG_DEBUG("----------FACE CONFLICT LIST----------");
            KIRI_LOG_DEBUG("face conflict list number={0}", this->Size());
            for (auto x = first; x; x = x->next)
                KIRI_LOG_DEBUG("vertex id={0}, value=({1},{2},{3})", x->value.GetIdx(), x->value.GetValue().x, x->value.GetValue().y, x->value.GetValue().z);

            KIRI_LOG_DEBUG("--------------------------------------");
        }
    };

    typedef SharedPtr<KiriFaceConflictLists> KiriFaceConflictListsPtr;
}
#endif