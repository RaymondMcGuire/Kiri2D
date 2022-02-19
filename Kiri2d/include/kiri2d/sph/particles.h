/***
 * @Author: Xu.WANG
 * @Date: 2022-02-19 12:44:32
 * @LastEditTime: 2022-02-19 12:44:32
 * @LastEditors: Xu.WANG
 * @Description:
 */
#ifndef _KIRI2D_SPH2D_PARTICLES_H_
#define _KIRI2D_SPH2D_PARTICLES_H_

#pragma once

#include <kiri2d/sph/constants.h>

namespace KIRI2D::SPH
{
    class Particles
    {
    public:
        Particles()
        {
        }

        Particles(Vector2F pos, float m)
        {
            position = pos;
            velocity = Vector2F(0.f);
            force = Vector2F(0.f);

            mass = m;
            density = 0;
            pressure = 0;

            color = 0;
            normal = Vector2F(0.f);
        }

        float getVelocityLength2() const
        {
            return velocity.x * velocity.x + velocity.y * velocity.y;
        }

        float getForceLength2() const
        {
            return force.x * force.x + force.y * force.y;
        }

        float getNormalLength2() const
        {
            return normal.x * normal.x + normal.y * normal.y;
        }

        Vector2F position;
        Vector2F velocity;
        Vector2F force;

        float mass;
        float density;
        float pressure;

        float color;
        Vector2F normal;
    };

} // namespace KIRI2D::SPH

#endif /* _KIRI2D_SPH2D_PARTICLES_H_ */
