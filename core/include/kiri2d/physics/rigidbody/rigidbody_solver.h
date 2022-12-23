/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-12-23 13:08:40
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-12-23 14:06:11
 * @FilePath: \Kiri2D\core\include\kiri2d\physics\rigidbody\rigidbody_solver.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _RIGIDBODY_SOLVER_H_
#define _RIGIDBODY_SOLVER_H_

#pragma once

#include <kiri2d/physics/rigidbody/collision_handler.h>

namespace PHY::RIGIDBODY {

class RigidBodySolver {
public:
  explicit RigidBodySolver() {}

  virtual ~RigidBodySolver() {}

private:
};

} // namespace PHY::RIGIDBODY

#endif /* _RIGIDBODY_SOLVER_H_ */