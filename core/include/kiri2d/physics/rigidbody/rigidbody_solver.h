/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-12-17 19:23:24
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-12-17 19:25:55
 * @FilePath: \Kiri2D\core\include\kiri2d\physics\rigidbody\rigidbody_solver.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _RIGIDBODY_SOLVER_H_
#define _RIGIDBODY_SOLVER_H_

#pragma once

#include <kiri_pch.h>

namespace PHY::RIGIDBODY
{

    class RigidbodySolver
    {
    public:
        explicit RigidbodySolver()
        {
        }

        virtual ~RigidbodySolver()
        {
        }
    };

} // namespace PHY::RIGIDBODY

#endif /* _RIGIDBODY_SOLVER_H_ */