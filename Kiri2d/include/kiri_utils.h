/*** 
 * @Author: Jayden Zhang
 * @Date: 2020-09-27 02:54:00
 * @LastEditTime: 2021-09-03 17:35:25
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\include\kiri_utils.h
 */

#ifndef _KIRI_UTILS_H_
#define _KIRI_UTILS_H_

#pragma once

#include <root_directory.h>
#include <partio/Partio.h>

#include <kiri_pch.h>
#include <filesystem>
#include <sys/types.h>
#include <sys/stat.h>

#include <kiri_pbs_cuda2d_define.h>

namespace KIRI
{
    String UInt2Str4Digit(
        UInt input)
    {
        char output[5];
        snprintf(output, 5, "%04d", input);
        return String(output);
    };

    void ExportBgeoFileCUDA(
        String folder_path,
        String file_name,
        float2 *positions,
        float3 *colors,
        float *radius,
        size_t *labels,
        UInt particles_num)
    {
        String export_file = folder_path + "/" + file_name + ".bgeo";

        try
        {

            struct stat info;

            if (stat(folder_path.c_str(), &info) != 0)
            {
                std::error_code ec;
                bool success = std::filesystem::create_directories(folder_path, ec);
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
            size_t fbytes = particles_num * sizeof(float);
            size_t f2bytes = particles_num * sizeof(float2);
            size_t f3bytes = particles_num * sizeof(float3);
            size_t uintbytes = particles_num * sizeof(size_t);

            float2 *cpu_positions = (float2 *)malloc(f3bytes);
            float3 *cpu_colors = (float3 *)malloc(f3bytes);
            float *cpu_radius = (float *)malloc(fbytes);
            size_t *cpu_labels = (size_t *)malloc(uintbytes);

            cudaMemcpy(cpu_positions, positions, f2bytes, cudaMemcpyDeviceToHost);
            cudaMemcpy(cpu_colors, colors, f3bytes, cudaMemcpyDeviceToHost);
            cudaMemcpy(cpu_radius, radius, fbytes, cudaMemcpyDeviceToHost);
            cudaMemcpy(cpu_labels, labels, uintbytes, cudaMemcpyDeviceToHost);

            for (UInt i = 0; i < particles_num; i++)
            {
                Int particle = p->addParticle();
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
    }

    void ExportBgeoFileCUDA(
        String folder_path,
        String file_name,
        float2 *positions,
        float3 *colors,
        float radius,
        size_t labels,
        UInt particles_num)
    {
        String export_file = folder_path + "/" + file_name + ".bgeo";

        try
        {

            struct stat info;

            if (stat(folder_path.c_str(), &info) != 0)
            {
                std::error_code ec;
                bool success = std::filesystem::create_directories(folder_path, ec);
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
            size_t fbytes = particles_num * sizeof(float);
            size_t f2bytes = particles_num * sizeof(float2);
            size_t f3bytes = particles_num * sizeof(float3);
            size_t uintbytes = particles_num * sizeof(size_t);

            float2 *cpu_positions = (float2 *)malloc(f3bytes);
            float3 *cpu_colors = (float3 *)malloc(f3bytes);

            cudaMemcpy(cpu_positions, positions, f2bytes, cudaMemcpyDeviceToHost);
            cudaMemcpy(cpu_colors, colors, f3bytes, cudaMemcpyDeviceToHost);

            for (UInt i = 0; i < particles_num; i++)
            {
                Int particle = p->addParticle();
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
    }

    void ExportBgeoFileCUDA(
        String folder_path,
        String file_name,
        float2 *positions,
        float3 colors,
        float radius,
        size_t labels,
        UInt particles_num)
    {
        String export_file = folder_path + "/" + file_name + ".bgeo";

        try
        {

            struct stat info;

            if (stat(folder_path.c_str(), &info) != 0)
            {
                std::error_code ec;
                bool success = std::filesystem::create_directories(folder_path, ec);
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
            size_t fbytes = particles_num * sizeof(float);
            size_t f2bytes = particles_num * sizeof(float2);
            size_t f3bytes = particles_num * sizeof(float3);
            size_t uintbytes = particles_num * sizeof(size_t);

            float2 *cpu_positions = (float2 *)malloc(f3bytes);
            cudaMemcpy(cpu_positions, positions, f2bytes, cudaMemcpyDeviceToHost);

            for (UInt i = 0; i < particles_num; i++)
            {
                Int particle = p->addParticle();
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
    }

} // namespace KIRI
#endif