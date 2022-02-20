/***
 * @Author: Xu.WANG
 * @Date: 2022-02-19 13:00:13
 * @LastEditTime: 2022-02-19 13:01:15
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _KIRI2D_BLUE_NOISE_SPH2D_SOLVER_H_
#define _KIRI2D_BLUE_NOISE_SPH2D_SOLVER_H_

#pragma once

#include <kiri2d/sph/grid.h>
#include <kiri2d/sdf/sdf_poly_2d.h>

namespace KIRI2D::SPH
{
    using namespace Constants;
    class BlueNoiseSPHSolver
    {
    public:
        BlueNoiseSPHSolver()
        {
            worldSize = Vector2F(2.f, 1.f);
            particles = std::vector<Particles>();
        }

        BlueNoiseSPHSolver(Vector2F ws, KIRI2D::KiriSDFPoly2D sdf)
        {
            worldSize = ws;
            particles = std::vector<Particles>();
            boundarySDF = sdf;
        }

        void init(std::vector<Vector2F> data, float radius)
        {
            auto space = radius * 2.f;
            particleVolume = space * space;
            particleMass = particleVolume * REST_DENSITY;
            kernelRadius = 2.f * space;

            grid = Grid(Vector2F(worldSize.x, worldSize.y), kernelRadius);

            for (auto i = 0; i < data.size(); i++)
            {
                Particles p = Particles(data[i], particleMass);
                particles.push_back(p);
            }

            grid.updateStructure(particles);
            numberParticles = data.size();

            std::cout << "init particles number = " << numberParticles << std::endl;
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
                particles[i].force = Vector2F(0.0f);
                particles[i].normal = Vector2F(0.0f);
            }
        }

        void update(float dt)
        {

            findNeighborhoods();
            calculateDensity();
            calculateNormals();
            calculatePressure();
            calculateCohesion();
            integrationStep(dt);

            // collisionHandling();

            grid.updateStructure(particles);

            reset();
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

        KIRI2D::KiriSDFPoly2D boundarySDF;

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

        float cohesionKernel(Vector2F x, float h)
        {
            float r2 = x.x * x.x + x.y * x.y;
            float h2 = h * h;

            if (r2 > h2)
                return 0.f;

            float res = 0.f;
            float r = sqrt(r2);
            float r3 = r2 * r;
            float m_k = 32.f / (kiri_math_mini::pi<float>() * pow(h, 9));
            float m_c = pow(h, 6) / 64.f;
            if (r > 0.5f * h)
                res = m_k * pow(h - r, 3) * r3;
            else
                res = m_k * 2.f * pow(h - r, 3) * r3 - m_c;

            return res;
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
                        if (dist2 <= maxDist2 && dist2 != 0.f)
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

            // std::cout << "density=" << particles[1000].density << std::endl;
        }

        void calculateNormals()
        {

            for (int i = 0; i < numberParticles; i++)
            {
                std::vector<int> neighbors = neighborhoods[i];

                for (int n = 0; n < neighbors.size(); n++)
                {
                    int j = neighbors[n];
                    Vector2F x = particles[i].position - particles[j].position;

                    particles[i].normal -= particles[j].mass / particles[j].density * gradKernel(x, kernelRadius);
                }
                particles[i].normal.normalize();
            }

            // std::cout << "normal=" << particles[1000].normal.x << "," << particles[1000].normal.y << std::endl;
        }

        void calculatePressure()
        {
            for (int i = 0; i < numberParticles; i++)
            {
                particles[i].pressure = std::max(STIFFNESS * (particles[i].density - REST_DENSITY), 0.0f);
            }

            for (int i = 0; i < numberParticles; i++)
            {
                std::vector<int> neighbors = neighborhoods[i];
                auto xi = particles[i].position;
                for (int n = 0; n < neighbors.size(); n++)
                {
                    int j = neighbors[n];
                    Vector2F x = xi - particles[j].position;

                    // NOTE mass i or mass j
                    particles[i].force -= particles[i].mass * (particles[i].pressure / (particles[i].density * particles[i].density) + particles[j].pressure / (particles[j].density * particles[j].density)) * gradKernel(x, kernelRadius);
                }

                auto dist = boundarySDF.FindRegion(xi);

                // remove normal acceleration
                if (dist >= 0.f)
                    particles[i].force -= particles[i].force.dot(particles[i].normal) * particles[i].normal;
            }

            // std::cout << "pressure force=" << particles[1000].force.x << "," << particles[1000].force.y << std::endl;
        }

        void calculateCohesion()
        {
            for (int i = 0; i < numberParticles; i++)
            {
                std::vector<int> neighbors = neighborhoods[i];
                for (int n = 0; n < neighbors.size(); n++)
                {
                    int j = neighbors[n];
                    Vector2F xixj = particles[i].position - particles[j].position;
                    xixj.normalize();

                    float kij = 2.f * REST_DENSITY / (particles[i].density + particles[j].density);

                    // std::cout << "xi=" << particles[i].position.x << "," << particles[i].position.y << "xj=" << particles[j].position.x << "," << particles[j].position.y << std::endl;
                    // std::cout << "xixj=" << xixj.x << "," << xixj.y << "; kij= " << kij << std::endl;

                    particles[i].force -= COHESION * particles[i].mass * kij * cohesionKernel(particles[i].position - particles[j].position, kernelRadius) * xixj;
                }
            }

            // std::cout << "cohesion force=" << particles[1000].force.x << "," << particles[1000].force.y << std::endl;
        }

        void integrationStep(float dt)
        {
            for (int i = 0; i < numberParticles; i++)
            {
                particles[i].velocity += dt * particles[i].force;
                particles[i].velocity *= 0.9f;

                auto dist = boundarySDF.FindRegion(particles[i].position);
                if (dist >= 0.f)
                    particles[i].velocity -= particles[i].velocity.dot(particles[i].normal) * particles[i].normal;
                else
                    particles[i].position += dt * particles[i].velocity;
            }

            // std::cout << "velocity=" << particles[1000].velocity.x << "," << particles[1000].velocity.y << std::endl;
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

#endif /* _KIRI2D_BLUE_NOISE_SPH2D_SOLVER_H_ */
