/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-10-16 15:12:44
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-10-16 15:22:41
 * @FilePath: \Kiri2D\core\include\kiri2d\geo\vector2_list.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _KIRI_VECTOR2_LIST_H_
#define _KIRI_VECTOR2_LIST_H_

#pragma once

#include <kiri2d/linked_list/circular_doubly_linked_list.h>

namespace KIRI
{

    class KiriVector2List : public KiriCircularDoublyLinkedList<Vector2F>
    {
    public:
        KiriVector2List() : KiriCircularDoublyLinkedList<Vector2F>(){};
        ~KiriVector2List(){};

        void push(const Vector2F &x)
        {

            if (mHead == NULL)
            {
                mHead = SharedPtr<Node>(new Node(x));
                mHead->next = mHead->prev = mHead;
                mCounter++;
            }
            else
            {
                if (!checkVertex2Identity(x))
                {
                    auto new_node = SharedPtr<Node>(new Node(x));
                    mHead->prev->next = new_node;
                    new_node->prev = mHead->prev;
                    new_node->next = mHead;
                    mHead->prev = new_node;
                    mCounter++;
                }
            }
        }

        void clone(const SharedPtr<KiriVector2List> &list)
        {
            if (mHead != NULL)
            {
                auto x = mHead;
                do
                {
                    list->push(Vector2F(x->value.x, x->value.y));
                    x = x->next;
                } while (x != mHead);
            }
        }

        void reverseVertexList()
        {
            auto list = std::make_shared<KiriVector2List>();
            clone(list);
            removeAll();
            if (list->head() != NULL)
            {
                auto x = list->head();
                do
                {
                    push(Vector2F(x->value.x, x->value.y));
                    x = x->prev;
                } while (x != list->head());
            }
        }

        void printVertexList()
        {
            KIRI_LOG_DEBUG("----------Vertex LIST----------");
            KIRI_LOG_DEBUG("vector2d list number={0}", this->Size());

            if (mHead != NULL)
            {
                auto x = mHead;
                do
                {
                    KIRI_LOG_DEBUG("vector2d value=({0},{1})", x->value.x, x->value.y);
                    x = x->next;
                } while (x != mHead);
            }

            KIRI_LOG_DEBUG("--------------------------------------");
        }

    private:
        bool checkVertex2Identity(const Vector2F &v1)
        {
            auto epsilon = 0.1f;
            auto v2 = mHead->prev->value;
            auto v3 = mHead->value;
            if (std::abs(v1.x - v2.x) < epsilon && std::abs(v1.y - v2.y) < epsilon)
                return true;
            if (std::abs(v1.x - v3.x) < epsilon && std::abs(v1.y - v3.y) < epsilon)
                return true;
            return false;
        }
    };

    typedef SharedPtr<KiriVector2List> KiriVector2ListPtr;
}
#endif