/***
 * @Author: Jayden Zhang
 * @Date: 2020-09-27 02:54:00
 * @LastEditTime: 2021-09-13 14:00:59
 * @LastEditors: Xu.WANG
 * @Description:
 * @FilePath: \Kiri2D\Kiri2d\include\kiri_utils.h
 */

#ifndef _KIRI_PBS_UTILS_H_
#define _KIRI_PBS_UTILS_H_

#pragma once

#include <filesystem>
#include <sys/types.h>
#include <sys/stat.h>

#include <root_directory.h>
#include <partio/Partio.h>

#include <kiri_pbs_cuda/kiri_pbs_pch.cuh>

namespace KIRI2D
{

    class KiriPbsUtils
    {

    public:
        static std::string Num2DigitStr(uint number)
        {
            char output[5];
            snprintf(output, 5, "%04d", number);
            return std::string(output);
        };

        static void Write2BGEO(
            std::string folderPath,
            std::string fileName,
            float2 *positions,
            float3 *colors,
            float *radius,
            size_t *labels,
            uint particlesNum)
        {
            std::string export_file = folderPath + "/" + fileName + ".bgeo";

            try
            {
                struct stat info;

                if (stat(folderPath.c_str(), &info) != 0)
                {
                    std::error_code ec;
                    bool success = std::filesystem::create_directories(folderPath, ec);
                    if (!success)
                    {
                        std::cout << ec.message() << std::endl;
                    }
                }

                Partio::ParticlesDataMutable *p = Partio::create();
                Partio::ParticleAttribute position_attr = p->addAttribute("position", Partio::VECTOR, 3);
                Partio::ParticleAttribute color_attr = p->addAttribute("Cd", Partio::FLOAT, 3);
                Partio::ParticleAttribute pscale_attr = p->addAttribute("pscale", Partio::FLOAT, 1);
                Partio::ParticleAttribute label_attr = p->addAttribute("label", Partio::INT, 1);

                // transfer GPU data to CPU
                size_t float_bytes = particlesNum * sizeof(float);
                size_t float2_bytes = particlesNum * sizeof(float2);
                size_t float3_bytes = particlesNum * sizeof(float3);
                size_t uint_bytes = particlesNum * sizeof(size_t);

                float2 *cpu_positions = (float2 *)malloc(float3_bytes);
                float3 *cpu_colors = (float3 *)malloc(float3_bytes);
                float *cpu_radius = (float *)malloc(float_bytes);
                size_t *cpu_labels = (size_t *)malloc(uint_bytes);

                cudaMemcpy(cpu_positions, positions, float2_bytes, cudaMemcpyDeviceToHost);
                cudaMemcpy(cpu_colors, colors, float3_bytes, cudaMemcpyDeviceToHost);
                cudaMemcpy(cpu_radius, radius, float_bytes, cudaMemcpyDeviceToHost);
                cudaMemcpy(cpu_labels, labels, uint_bytes, cudaMemcpyDeviceToHost);

                for (uint i = 0; i < particlesNum; i++)
                {
                    int particle = p->addParticle();
                    float *pos = p->dataWrite<float>(position_attr, particle);
                    float *col = p->dataWrite<float>(color_attr, particle);
                    float *pscale = p->dataWrite<float>(pscale_attr, particle);
                    int *label = p->dataWrite<int>(label_attr, particle);

                    pos[0] = cpu_positions[i].x;
                    pos[1] = cpu_positions[i].y;
                    pos[2] = 0.f;
                    col[0] = cpu_colors[i].x;
                    col[1] = cpu_colors[i].y;
                    col[2] = cpu_colors[i].z;

                    // TODO
                    *pscale = cpu_radius[i];

                    *label = cpu_labels[i];
                }

                Partio::write(export_file.c_str(), *p);

                p->release();

                free(cpu_positions);
                free(cpu_colors);
                free(cpu_labels);
                free(cpu_radius);

                KIRI_LOG_INFO("Successfully saved bgeo file:{0}", export_file);
            }
            catch (std::exception &e)
            {
                std::cout << e.what() << std::endl;
            }
        };

        static void Write2BGEO(
            std::string folderPath,
            std::string fileName,
            float2 *positions,
            float3 *colors,
            float radius,
            size_t labels,
            uint particlesNum)
        {
            std::string export_file = folderPath + "/" + fileName + ".bgeo";

            try
            {

                struct stat info;

                if (stat(folderPath.c_str(), &info) != 0)
                {
                    std::error_code ec;
                    bool success = std::filesystem::create_directories(folderPath, ec);
                    if (!success)
                    {
                        std::cout << ec.message() << std::endl;
                    }
                }

                Partio::ParticlesDataMutable *p = Partio::create();
                Partio::ParticleAttribute position_attr = p->addAttribute("position", Partio::VECTOR, 3);
                Partio::ParticleAttribute color_attr = p->addAttribute("Cd", Partio::FLOAT, 3);
                Partio::ParticleAttribute pscale_attr = p->addAttribute("pscale", Partio::FLOAT, 1);
                Partio::ParticleAttribute label_attr = p->addAttribute("label", Partio::INT, 1);

                // transfer GPU data to CPU
                size_t float_bytes = particlesNum * sizeof(float);
                size_t float2_bytes = particlesNum * sizeof(float2);
                size_t float3_bytes = particlesNum * sizeof(float3);
                size_t uint_bytes = particlesNum * sizeof(size_t);

                float2 *cpu_positions = (float2 *)malloc(float3_bytes);
                float3 *cpu_colors = (float3 *)malloc(float3_bytes);

                cudaMemcpy(cpu_positions, positions, float2_bytes, cudaMemcpyDeviceToHost);
                cudaMemcpy(cpu_colors, colors, float3_bytes, cudaMemcpyDeviceToHost);

                for (uint i = 0; i < particlesNum; i++)
                {
                    int particle = p->addParticle();
                    float *pos = p->dataWrite<float>(position_attr, particle);
                    float *col = p->dataWrite<float>(color_attr, particle);
                    float *pscale = p->dataWrite<float>(pscale_attr, particle);
                    int *label = p->dataWrite<int>(label_attr, particle);

                    pos[0] = cpu_positions[i].x;
                    pos[1] = cpu_positions[i].y;
                    pos[2] = 0.f;
                    col[0] = cpu_colors[i].x;
                    col[1] = cpu_colors[i].y;
                    col[2] = cpu_colors[i].z;

                    // TODO
                    *pscale = radius;

                    *label = labels;
                }

                Partio::write(export_file.c_str(), *p);

                p->release();

                free(cpu_positions);
                free(cpu_colors);

                KIRI_LOG_INFO("Successfully saved bgeo file:{0}", export_file);
            }
            catch (std::exception &e)
            {
                std::cout << e.what() << std::endl;
            }
        };

        static void Write2BGEO(
            std::string folderPath,
            std::string fileName,
            float2 *positions,
            float3 colors,
            float radius,
            size_t labels,
            uint particlesNum)
        {
            std::string export_file = folderPath + "/" + fileName + ".bgeo";

            try
            {

                struct stat info;

                if (stat(folderPath.c_str(), &info) != 0)
                {
                    std::error_code ec;
                    bool success = std::filesystem::create_directories(folderPath, ec);
                    if (!success)
                    {
                        std::cout << ec.message() << std::endl;
                    }
                }

                Partio::ParticlesDataMutable *p = Partio::create();
                Partio::ParticleAttribute position_attr = p->addAttribute("position", Partio::VECTOR, 3);
                Partio::ParticleAttribute color_attr = p->addAttribute("Cd", Partio::FLOAT, 3);
                Partio::ParticleAttribute pscale_attr = p->addAttribute("pscale", Partio::FLOAT, 1);
                Partio::ParticleAttribute label_attr = p->addAttribute("label", Partio::INT, 1);

                // transfer GPU data to CPU
                size_t float_bytes = particlesNum * sizeof(float);
                size_t float2_bytes = particlesNum * sizeof(float2);
                size_t float3_bytes = particlesNum * sizeof(float3);
                size_t uint_bytes = particlesNum * sizeof(size_t);

                float2 *cpu_positions = (float2 *)malloc(float2_bytes);
                cudaMemcpy(cpu_positions, positions, float2_bytes, cudaMemcpyDeviceToHost);

                for (uint i = 0; i < particlesNum; i++)
                {
                    int particle = p->addParticle();
                    float *pos = p->dataWrite<float>(position_attr, particle);
                    float *col = p->dataWrite<float>(color_attr, particle);
                    float *pscale = p->dataWrite<float>(pscale_attr, particle);
                    int *label = p->dataWrite<int>(label_attr, particle);

                    pos[0] = cpu_positions[i].x;
                    pos[1] = cpu_positions[i].y;
                    pos[2] = 0.f;
                    col[0] = colors.x;
                    col[1] = colors.y;
                    col[2] = colors.z;

                    // TODO
                    *pscale = radius;

                    *label = labels;
                }

                Partio::write(export_file.c_str(), *p);

                p->release();

                free(cpu_positions);

                KIRI_LOG_INFO("Successfully saved bgeo file:{0}", export_file);
            }
            catch (std::exception &e)
            {
                std::cout << e.what() << std::endl;
            }
        };
    };

} // namespace KIRI2D
#endif