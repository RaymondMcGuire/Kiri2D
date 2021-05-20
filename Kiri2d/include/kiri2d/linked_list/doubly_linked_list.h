/*** 
 * @Author: Xu.WANG
 * @Date: 2021-05-14 14:43:27
 * @LastEditTime: 2021-05-19 03:17:09
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri2d\linked_list\doubly_linked_list.h
 */

#ifndef _KIRI2D_DOUBLY_LINKED_LIST_H_
#define _KIRI2D_DOUBLY_LINKED_LIST_H_

#pragma once

#include <kiri_pch.h>

namespace KIRI2D
{
    template <typename T>
    class KiriDoublyLinkedList
    {
    public:
        KiriDoublyLinkedList()
        {
            first = last = NULL;
        }

        ~KiriDoublyLinkedList()
        {
            auto x = first;
            while (x)
            {
                auto next = x->next;
                x->prev = x->next = NULL;
                x = next;
            }
        }

        void Push(const T x)
        {
            mCounter++;
            if (first == NULL)
            {
                first = SharedPtr<Node>(new Node(x));
                last = first;
            }
            else
            {
                auto new_node = SharedPtr<Node>(new Node(x));
                first->prev = new_node;
                new_node->next = first;
                first = new_node;
            }
        }

        void Push2Tail(const T x)
        {
            mCounter++;
            if (first == NULL)
            {
                first = SharedPtr<Node>(new Node(x));
                last = first;
            }
            else
            {
                auto new_node = SharedPtr<Node>(new Node(x));
                last->next = new_node;
                new_node->prev = last;
                last = new_node;
            }
        }

        void RemoveAll()
        {
            while (first != NULL)
            {
                auto next = first->next;
                if (next)
                    next->prev = NULL;
                first = next;
            }
        }

        /*** 
         * @description: integer example for remove:  l.Remove([](int x) {return x%2 == 1;});
         * @param {function<bool(T)>} predicate
         * @return {*}
         */
        void Remove(std::function<bool(T)> predicate)
        {
            while (predicate(first->value))
            {
                auto next = first->next;
                if (next)
                    next->prev = NULL;
                first = next;
            }

            auto x = first;
            while (x)
            {
                if (predicate(x->value))
                {
                    auto prev = x->prev;
                    auto next = x->next;
                    prev->next = next;

                    if (next)
                        next->prev = prev;

                    if (x == last)
                        last = prev;
                }

                x = x->next;
            }
        }

        constexpr UInt Size() { return mCounter; }

        void Print()
        {
            static_assert(
                IsSame_Int<T>::value ||
                    IsSame_SizeT<T>::value ||
                    IsSame_Float<T>::value ||
                    IsSame_Double<T>::value ||
                    IsSame_Bool<T>::value,
                "data type is not correct");

            String printStr = "";
            for (auto x = first; x; x = x->next)
                printStr += std::to_string(x->value) + " ";

            KIRI_LOG_DEBUG("doubly linked list = [ {0}]", printStr);
        }

        void PrintReverse()
        {
            static_assert(
                IsSame_Int<T>::value ||
                    IsSame_SizeT<T>::value ||
                    IsSame_Float<T>::value ||
                    IsSame_Double<T>::value ||
                    IsSame_Bool<T>::value,
                "data type is not correct");

            String printStr = "";
            for (auto x = last; x; x = x->prev)
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
                //KIRI_LOG_DEBUG("destruct node value: {0}", this->value);
            }
        };
        SharedPtr<Node> first;
        SharedPtr<Node> last;

    private:
        UInt mCounter = 0;
    };
}
#endif