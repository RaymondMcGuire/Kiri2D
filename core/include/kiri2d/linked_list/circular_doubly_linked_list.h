/***
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-07-22 15:36:50
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\linked_list\circular_doubly_linked_list.h
 */

#ifndef _KIRI_CIRCULAR_DOUBLY_LINKED_LIST_H_
#define _KIRI_CIRCULAR_DOUBLY_LINKED_LIST_H_

#pragma once

#include <kiri_pch.h>

namespace KIRI
{
    template <typename T>
    class KiriCircularDoublyLinkedList
    {
    protected:
        struct Node
        {
            T value;
            SharedPtr<Node> next;
            SharedPtr<Node> prev;

            Node(const T x)
            {
                this->value = x;
                this->next = NULL;
                this->prev = NULL;
            }
            ~Node() {}
        };
        SharedPtr<Node> mHead;
        UInt mCounter = 0;

    public:
        KiriCircularDoublyLinkedList()
        {
            mHead = NULL;
        }

        virtual ~KiriCircularDoublyLinkedList()
        {
            auto x = mHead;
            while (x)
            {
                auto next = x->next;
                x->prev = x->next = NULL;
                x = next;
            }
        }

        void Push2Head(const T x)
        {
            mCounter++;
            if (mHead == NULL)
            {
                mHead = SharedPtr<Node>(new Node(x));
                mHead->next = mHead->prev = mHead;
            }
            else
            {
                auto new_node = SharedPtr<Node>(new Node(x));
                auto last = mHead->prev;
                new_node->next = mHead;
                new_node->prev = last;
                last->next = new_node;
                mHead->prev = new_node;
                mHead = new_node;
            }
        }

        void Push2Tail(const T x)
        {
            mCounter++;
            if (mHead == NULL)
            {
                mHead = SharedPtr<Node>(new Node(x));
                mHead->next = mHead->prev = mHead;
            }
            else
            {
                auto new_node = SharedPtr<Node>(new Node(x));
                mHead->prev->next = new_node;
                new_node->prev = mHead->prev;
                new_node->next = mHead;
                mHead->prev = new_node;
            }
        }

        void removeAll()
        {
            if (mHead != NULL)
            {
                auto current = mHead->next;
                while (current != mHead)
                {
                    auto temp = current->next;
                    current->prev = current->next = NULL;
                    current = temp;
                }
                mHead = NULL;
            }
            mCounter = 0;
        }

        constexpr UInt Size() const { return mCounter; }

        void print()
        {
            String printStr = "";
            if (mHead != NULL)
            {
                auto x = mHead;
                do
                {
                    printStr += std::to_string(x->value) + " ";
                    x = x->next;
                } while (x != mHead);
            }

            KIRI_LOG_DEBUG("circular doubly linked list = [ {0}]", printStr);
        }

        void PrintReverse()
        {

            String printStr = "";

            if (mHead != NULL)
            {
                auto x = mHead->prev;
                do
                {
                    printStr += std::to_string(x->value) + " ";
                    x = x->prev;
                } while (x != mHead->prev);
            }

            KIRI_LOG_DEBUG("circular doubly linked list reverse= [ {0}]", printStr);
        }

        const SharedPtr<Node> &head() const { return mHead; }
    };
}
#endif