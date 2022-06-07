/***
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-05-24 10:42:14
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri\KiriCore\include\kiri2d\linked_list\doubly_linked_list.h
 */

#ifndef _KIRI_DOUBLY_LINKED_LIST_H_
#define _KIRI_DOUBLY_LINKED_LIST_H_

#pragma once

#include <kiri_pch.h>

namespace KIRI
{
    template <typename T>
    class KiriDoublyLinkedList
    {
    public:
        KiriDoublyLinkedList()
        {
            mFirst = mLast = NULL;
        }

        virtual ~KiriDoublyLinkedList()
        {
            auto x = mFirst;
            while (x)
            {
                auto next = x->next;
                x->prev = x->next = NULL;
                x = next;
            }
        }

        void push(const T x)
        {
            mCounter++;
            if (mFirst == NULL)
            {
                mFirst = SharedPtr<Node>(new Node(x));
                mLast = mFirst;
            }
            else
            {
                auto new_node = SharedPtr<Node>(new Node(x));
                mFirst->prev = new_node;
                new_node->next = mFirst;
                mFirst = new_node;
            }
        }

        void Push2Tail(const T x)
        {
            mCounter++;
            if (mFirst == NULL)
            {
                mFirst = SharedPtr<Node>(new Node(x));
                mLast = mFirst;
            }
            else
            {
                auto new_node = SharedPtr<Node>(new Node(x));
                mLast->next = new_node;
                new_node->prev = mLast;
                mLast = new_node;
            }
        }

        void removeAll()
        {
            while (mFirst != NULL)
            {
                auto next = mFirst->next;
                if (next)
                    next->prev = NULL;
                mFirst = next;
            }
            mCounter = 0;
        }

        /***
         * @description: integer example for remove:  l.remove([](int x) {return x%2 == 1;});
         * @param {function<bool(T)>} predicate
         * @return {*}
         */
        void remove(std::function<bool(T)> predicate)
        {
            while (mFirst != NULL && predicate(mFirst->value))
            {
                mCounter--;
                auto next = mFirst->next;
                if (next)
                    next->prev = NULL;
                mFirst = next;
            }
            auto x = mFirst;
            while (x)
            {
                if (predicate(x->value))
                {
                    mCounter--;
                    auto prev = x->prev;
                    auto next = x->next;
                    prev->next = next;

                    if (next)
                        next->prev = prev;

                    if (x == mLast)
                        mLast = prev;
                }

                x = x->next;
            }
        }

        constexpr UInt Size() const { return mCounter; }

        void print()
        {
            // static_assert(
            //     IsSame_Int<T>::value ||
            //         IsSame_SizeT<T>::value ||
            //         IsSame_Float<T>::value ||
            //         IsSame_Double<T>::value ||
            //         IsSame_Bool<T>::value,
            //     "data type is not correct");

            String printStr = "";
            for (auto x = mFirst; x; x = x->next)
                printStr += std::to_string(x->value) + " ";

            KIRI_LOG_DEBUG("doubly linked list = [ {0}]", printStr);
        }

        void PrintReverse()
        {
            // static_assert(
            //     IsSame_Int<T>::value ||
            //         IsSame_SizeT<T>::value ||
            //         IsSame_Float<T>::value ||
            //         IsSame_Double<T>::value ||
            //         IsSame_Bool<T>::value,
            //     "data type is not correct");

            String printStr = "";
            for (auto x = mLast; x; x = x->prev)
                printStr += std::to_string(x->value) + " ";

            KIRI_LOG_DEBUG("doubly linked list reverse= [ {0}]", printStr);
        }

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
            ~Node()
            {
                // KIRI_LOG_DEBUG("destruct node value: {0}", this->value);
            }
        };
        SharedPtr<Node> mFirst;
        SharedPtr<Node> mLast;
        UInt mCounter = 0;
    };
}
#endif