/*** 
 * @Author: Xu.WANG
 * @Date: 2021-07-22 10:58:21
 * @LastEditTime: 2021-07-22 21:19:48
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\straight_skeleton\sskel_lav.h
 */

#ifndef _KIRI2D_SSKEL_LAV_H_
#define _KIRI2D_SSKEL_LAV_H_

#pragma once

#include <kiri2d/linked_list/circular_doubly_linked_list.h>
#include <kiri2d/straight_skeleton/sskel_event.h>

namespace KIRI2D::SSKEL
{
    class SSkelLAV : public KIRI::KiriCircularDoublyLinkedList<SSkelVertexPtr>
    {
    public:
        explicit SSkelLAV(Vec_Vec2F poly)
            : KIRI::KiriCircularDoublyLinkedList<SSkelVertexPtr>()
        {
            auto size = poly.size();
            for (size_t i = size; i < size + size; i++)
            {
                auto prev = poly[(i - 1) % size];
                auto curr = poly[i % size];
                auto next = poly[(i + 1) % size];
                this->Push(std::make_shared<SSkelVertex>(prev, curr, next));
            }
        }

        SSkelLAV() = default;
        SSkelLAV(const SSkelLAV &) = delete;
        SSkelLAV &operator=(const SSkelLAV &) = delete;

        ~SSkelLAV() {}

        void PrintSSkelLAV();
        void GenEvents();

    private:
        void Push(const SSkelVertexPtr &x)
        {

            if (mHead == NULL)
            {
                mHead = SharedPtr<Node>(new Node(x));
                mHead->next = mHead->prev = mHead;
                mCounter++;
            }
            else
            {

                auto new_node = SharedPtr<Node>(new Node(x));
                mHead->prev->next = new_node;
                new_node->prev = mHead->prev;
                new_node->next = mHead;
                mHead->prev = new_node;
                mCounter++;
            }
        }
    };
    typedef SharedPtr<SSkelLAV> SSkelLAVPtr;
}

#endif