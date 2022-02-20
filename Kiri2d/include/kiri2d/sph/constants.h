/***
 * @Author: Xu.WANG
 * @Date: 2022-02-19 12:49:37
 * @LastEditTime: 2022-02-20 14:04:06
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _KIRI2D_SPH2D_CONSTANTS_H_
#define _KIRI2D_SPH2D_CONSTANTS_H_

#pragma once

#include <kiri_pch.h>

namespace KIRI2D::SPH::Constants
{
    const float REST_DENSITY = 1000;

    const float STIFFNESS = 10000;
    const float VISCOCITY = 12000;
    const float TENSION = 10000.0f;

    const float GRAVITY = -12000.f;

    const float COHESION = 20.f;

} // namespace KIRI2D::SPH

#endif /* _KIRI2D_SPH2D_CONSTANTS_H_ */
