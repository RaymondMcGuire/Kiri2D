/***
 * @Author: Xu.WANG
 * @Date: 2022-02-19 13:00:13
 * @LastEditTime: 2022-02-19 13:01:15
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _KIRI2D_SPH2D_SOLVER_H_
#define _KIRI2D_SPH2D_SOLVER_H_

#pragma once

#include <kiri2d/sph/grid.h>

namespace KIRI2D::SPH
{
    using namespace Constants;
    class SPHSolver
    {
    public:
        SPHSolver()
        {
            worldSize = Vector2F(2.f, 1.f);
            particles = std::vector<Particles>();
        }

        SPHSolver(Vector2F ws)
        {
            worldSize = ws;
            particles = std::vector<Particles>();
        }

        void initWithBoxVolume(Vector2F volume, float radius)
        {
            auto center = worldSize / 2.f;
            auto halfVolume = volume / 2.f;
            auto space = radius * 2.f;

            auto configSpace = std::sqrtf(2.f) * space;
            auto initPos = center - halfVolume * configSpace;
            initPos.y = radius;

            particleVolume = space * space;
            particleMass = particleVolume * REST_DENSITY;
            kernelRadius = 2.f * space;

            grid = Grid(Vector2F(worldSize.x, worldSize.y), kernelRadius);

            for (auto i = 0; i < volume.x; i++)
            {
                for (auto j = 0; j < volume.y; j++)
                {
                    Vector2F pos = initPos + Vector2F(i, j) * configSpace;

                    Particles p = Particles(pos, particleMass);
                    particles.push_back(p);
                }
            }
            grid.updateStructure(particles);
            numberParticles = volume.x * volume.y;
        }

        void reset()
        {
            for (int i = 0; i < numberParticles; i++)
            {
                particles[i].force = Vector2F(0.0f, 0.0f);
            }
        }

        void update(float dt)
        {

            findNeighborhoods();
            calculateDensity();
            calculatePressure();
            calculateForceDensity();
            integrationStep(dt);
            collisionHandling();

            grid.updateStructure(particles);

            reset();
        }

        void repulsionForce(Vector2F position)
        {
            for (int i = 0; i < numberParticles; i++)
            {
                Vector2F x = particles[i].position - position;

                float dist2 = x.x * x.x + x.y * x.y;

                if (dist2 < kernelRadius * 3)
                {
                    particles[i].force += x * 800000.0f * particles[i].density;
                }
            }
        }

        void attractionForce(Vector2F position)
        {
            for (int i = 0; i < numberParticles; i++)
            {
                Vector2F x = position - particles[i].position;

                float dist2 = x.x * x.x + x.y * x.y;

                if (dist2 < kernelRadius * 3)
                {
                    particles[i].force += x * 800000.0f * particles[i].density;
                }
            }
        }

        const std::vector<Particles> GetParticles() const { return particles; }

    private:
        int numberParticles;
        std::vector<Particles> particles;
        std::vector<std::vector<int>> neighborhoods;
        Grid grid;

        float particleVolume;
        float particleMass;
        float kernelRadius;
        Vector2F worldSize;

        // Poly6 Kernel
        float kernel(Vector2F x, float h)
        {
            float r2 = x.x * x.x + x.y * x.y;
            float h2 = h * h;

            if (r2 < 0 || r2 > h2)
                return 0.0f;

            return 315.0f / (64.0f * kiri_math_mini::pi<float>() * pow(h, 9)) * pow(h2 - r2, 3);
        }

        // Gradient of Spiky Kernel
        Vector2F gradKernel(Vector2F x, float h)
        {
            float r = sqrt(x.x * x.x + x.y * x.y);
            if (r == 0.0f)
                return Vector2F(0.0f, 0.0f);

            float t1 = -45.0f / (kiri_math_mini::pi<float>() * pow(h, 6));
            Vector2F t2 = x / r;
            float t3 = pow(h - r, 2);

            return t1 * t2 * t3;
        }

        // Laplacian of Viscosity Kernel
        float laplaceKernel(Vector2F x, float h)
        {
            float r = sqrt(x.x * x.x + x.y * x.y);
            return 45.0f / (kiri_math_mini::pi<float>() * pow(h, 6)) * (h - r);
        }

        void findNeighborhoods()
        {
            neighborhoods = std::vector<std::vector<int>>();
            float maxDist2 = kernelRadius * kernelRadius;

            for each (const Particles &p in particles)
            {
                std::vector<int> neighbors = std::vector<int>();
                std::vector<Cell> neighboringCells = grid.getNeighboringCells(p.position);

                for each (const Cell &cell in neighboringCells)
                {
                    for each (int index in cell)
                    {
                        Vector2F x = p.position - particles[index].position;
                        float dist2 = x.x * x.x + x.y * x.y;
                        if (dist2 <= maxDist2)
                        {
                            neighbors.push_back(index);
                        }
                    }
                }
                // std::cout << "neighbor size=" << neighbors.size() << std::endl;
                neighborhoods.push_back(neighbors);
            }
        }

        void calculateDensity()
        {
            for (int i = 0; i < numberParticles; i++)
            {
                std::vector<int> neighbors = neighborhoods[i];
                float densitySum = 0.0f;

                for (int n = 0; n < neighbors.size(); n++)
                {
                    int j = neighbors[n];

                    Vector2F x = particles[i].position - particles[j].position;
                    densitySum += particles[j].mass * kernel(x, kernelRadius);
                }

                particles[i].density = densitySum;
            }
        }

        void calculatePressure()
        {
            for (int i = 0; i < numberParticles; i++)
            {
                particles[i].pressure = std::max(STIFFNESS * (particles[i].density - REST_DENSITY), 0.0f);
            }
        }

        void calculateForceDensity()
        {
            for (int i = 0; i < numberParticles; i++)
            {
                Vector2F fPressure = Vector2F(0.0f, 0.0f);
                Vector2F fViscosity = Vector2F(0.0f, 0.0f);
                Vector2F fGravity = Vector2F(0.0f, 0.0f);

                std::vector<int> neighbors = neighborhoods[i];

                // particles[i].color = 0;

                for (int n = 0; n < neighbors.size(); n++)
                {
                    int j = neighbors[n];
                    Vector2F x = particles[i].position - particles[j].position;

                    // Pressure force density
                    fPressure += particles[j].mass * (particles[i].pressure + particles[j].pressure) / (2.0f * particles[j].density) * gradKernel(x, kernelRadius);

                    // Viscosity force density
                    fViscosity += particles[j].mass * (particles[j].velocity - particles[i].velocity) / particles[j].density * laplaceKernel(x, kernelRadius);

                    // Color field
                    // particles[i].color += particles[j].mass / particles[j].density * kernel(x, kernelRadius);
                }

                // Gravitational force density
                fGravity = particles[i].density * Vector2F(0, GRAVITY);

                fPressure *= -1.0f;
                fViscosity *= VISCOCITY;

                // particles[i].force += fPressure + fViscosity + fGravity + fSurface;
                particles[i].force += fPressure + fViscosity + fGravity;
            }
        }

        void integrationStep(float dt)
        {
            for (int i = 0; i < numberParticles; i++)
            {
                particles[i].velocity += dt * particles[i].force / particles[i].density;
                particles[i].position += dt * particles[i].velocity;
            }
        }

        void collisionHandling()
        {
            for (int i = 0; i < numberParticles; i++)
            {
                if (particles[i].position.x < 0.0f)
                {
                    particles[i].position.x = 0.0f;
                    particles[i].velocity.x = -0.5f * particles[i].velocity.x;
                }
                else if (particles[i].position.x > worldSize.x)
                {
                    particles[i].position.x = worldSize.x;
                    particles[i].velocity.x = -0.5f * particles[i].velocity.x;
                }

                if (particles[i].position.y < 0.0f)
                {
                    particles[i].position.y = 0.0f;
                    particles[i].velocity.y = -0.5f * particles[i].velocity.y;
                }
                else if (particles[i].position.y > worldSize.y)
                {
                    particles[i].position.y = worldSize.y;
                    particles[i].velocity.y = -0.5f * particles[i].velocity.y;
                }
            }
        }
    };

} // namespace KIRI2D::SPH

#endif /* _KIRI2D_SPH2D_SOLVER_H_ */
