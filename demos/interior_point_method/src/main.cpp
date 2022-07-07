/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-06-25 01:39:47
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-07-06 12:35:40
 * @FilePath: \Kiri2D\demos\interior_point_method\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#include <kiri2d.h>
using namespace KIRI2D;

#include <ipm.h>
#include <gridding.h>

#include <root_directory.h>
#include <partio/Partio.h>
std::vector<Vector4D> ReadBgeoFileForCPU(String Folder, String Name, Vector3D Offset = Vector3D(0.0), bool FlipYZ = false)
{
    std::vector<Vector4D> pos_array;
    String root_folder = "bgeo";
    String extension = ".bgeo";
    String file_path = String(RESOURCES_PATH) + root_folder + "/" + Folder + "/" + Name + extension;
    std::cout << file_path << std::endl;
    Partio::ParticlesDataMutable *data = Partio::read(file_path.c_str());

    Partio::ParticleAttribute pos_attr;
    Partio::ParticleAttribute pscale_attr;
    if (!data->attributeInfo("position", pos_attr) || (pos_attr.type != Partio::FLOAT && pos_attr.type != Partio::VECTOR) || pos_attr.count != 3)
    {
        KIRI_LOG_ERROR("Failed to Get Proper Position Attribute");
    }

    bool pscaleLoaded = data->attributeInfo("pscale", pscale_attr);

    double max_y = 0.0;
    for (auto i = 0; i < data->numParticles(); i++)
    {
        const float *pos = data->data<float>(pos_attr, i);
        if (pscaleLoaded)
        {
            const float *pscale = data->data<float>(pscale_attr, i);
            if (i == 0)
            {
                KIRI_LOG_INFO("pscale={0}", *pscale);
            }

            if (FlipYZ)
            {
                pos_array.emplace_back(Vector4D(pos[0] + Offset.x, pos[2] + Offset.z, pos[1] + Offset.y, *pscale));
            }
            else
            {
                pos_array.emplace_back(Vector4D(pos[0] + Offset.x, pos[1] + Offset.y, pos[2] + Offset.z, *pscale));
                if (pos[1] > max_y)
                    max_y = pos[1];
            }
        }
        else
        {
            if (FlipYZ)
            {
                pos_array.emplace_back(Vector4D(pos[0] + Offset.x, pos[2] + Offset.z, pos[1] + Offset.y, 0.01f));
            }
            else
            {
                pos_array.emplace_back(Vector4D(pos[0] + Offset.x, pos[1] + Offset.y, pos[2] + Offset.z, 0.01f));
            }
        }
    }

    // printf("max_Y=%.3f , offset=(%.3f,%.3f,%.3f) \n", max_y, Offset.x, Offset.y, Offset.z);

    data->release();

    return pos_array;
}

void ExportBgeoFileFromCPU(String Folder, String FileName, std::vector<Vector4D> Positions)
{
    String exportPath = String(EXPORT_PATH) + "bgeo/" + Folder + "/" + FileName + ".bgeo";

    Partio::ParticlesDataMutable *p = Partio::create();
    Partio::ParticleAttribute positionAttr = p->addAttribute("position", Partio::VECTOR, 3);
    Partio::ParticleAttribute pScaleAttr = p->addAttribute("pscale", Partio::FLOAT, 1);

    for (UInt i = 0; i < Positions.size(); i++)
    {
        Int particle = p->addParticle();
        float *pos = p->dataWrite<float>(positionAttr, particle);
        float *pscale = p->dataWrite<float>(pScaleAttr, particle);
        pos[0] = Positions[i].x;
        pos[1] = Positions[i].y;
        pos[2] = Positions[i].z;

        // TODO
        *pscale = Positions[i].w;
    }
    Partio::write(exportPath.c_str(), *p);

    p->release();
}

int main(int argc, char *argv[])
{
    // log system
    KiriLog::init();

    auto bgeo_data = ReadBgeoFileForCPU("box", "box_normal");
    auto data_size = bgeo_data.size();
    KIRI_LOG_DEBUG("data size={0}", data_size);

    int n = data_size / 10;

    std::vector<double> data;
    std::vector<Vector3D> data_pos;
    std::vector<double> data_radius;
    std::vector<Vector4D> positions;

    for (auto i = 0; i < n; i++)
    {
        data_pos.emplace_back(Vector3D(bgeo_data[i].x, bgeo_data[i].y, bgeo_data[i].z) * 100.0);
        data_radius.emplace_back(bgeo_data[i].w * 100.0);
        // KIRI_LOG_DEBUG("radius={0}", bgeo_data[i].w * 100.0);
    }

    for (auto j = 0; j < n; j++)
    {
        data.emplace_back(Random::get(-1.0, 1.0));
    }

    int inequ_num = 2 * n + n * (n - 1) / 2;

    auto ipm = std::make_shared<OPTIMIZE::IPM::InteriorPointMethod>(data, inequ_num, data_radius, data_pos);
    auto results = ipm->solution();

    for (auto j = 0; j < n; j++)
    {
        if (data_radius[j] / 100.0 * double(results[j]) >= 1e-4)
            positions.emplace_back(Vector4D(data_pos[j].x / 100.0, data_pos[j].y / 100.0, data_pos[j].z / 100.0, data_radius[j] / 100.0 * double(results[j])));
        // KIRI_LOG_DEBUG("radius={0};{1}", data_radiusi[j] / 100.0 * double(results[j]), double(results[j]));
    }

    // KIRI_LOG_DEBUG("FIRST STAGE!!!!!!!");
    //  first stage
    //  auto gridding0 = std::make_shared<OPTIMIZE::IPM::Gradding>(data_pos, data_radius, 6, 6, 6);
    //  auto max_hash_id0 = gridding0->maxGridHash();

    // for (auto i = 0; i < max_hash_id0; i++)
    // {
    //     auto [data_i, data_i_idx] = gridding0->getDataByGridHash(i);
    //     auto datai_size = data_i.size();
    //     KIRI_LOG_DEBUG("data pos size={0}", datai_size);

    //     std::vector<Vector3D> data_posi;
    //     std::vector<double> data_radiusi;
    //     for (auto j = 0; j < data_i.size(); j++)
    //     {
    //         data_posi.emplace_back(Vector3D(data_i[j].x, data_i[j].y, data_i[j].z));
    //         data_radiusi.emplace_back(data_i[j].w);
    //     }

    //     if (datai_size > 1)
    //     {

    //         data.clear();

    //         for (auto j = 0; j < datai_size; j++)
    //         {
    //             data.emplace_back(Random::get(-1.0, 1.0));
    //         }

    //         int n = datai_size;
    //         int inequ_num = 2 * n + n * (n - 1) / 2;

    //         auto ipm = std::make_shared<OPTIMIZE::IPM::InteriorPointMethod>(data, inequ_num, data_radiusi, data_posi);
    //         auto results = ipm->solution();

    //         for (auto j = 0; j < datai_size; j++)
    //         {
    //             data_radius[data_i_idx[j]] = data_radiusi[j] * double(results[j]);
    //         }
    //     }
    // }

    // KIRI_LOG_DEBUG("SECOND STAGE!!!!!!!");
    // // second stage
    // auto gridding = std::make_shared<OPTIMIZE::IPM::Gradding>(data_pos, data_radius, 3, 3, 3);
    // auto max_hash_id = gridding->maxGridHash();

    // for (auto i = 0; i < max_hash_id; i++)
    // {
    //     auto [data_i, data_i_idx] = gridding->getDataByGridHash(i);
    //     auto datai_size = data_i.size();
    //     KIRI_LOG_DEBUG("data pos size={0}", datai_size);

    //     std::vector<Vector3D> data_posi;
    //     std::vector<double> data_radiusi;
    //     for (auto j = 0; j < data_i.size(); j++)
    //     {
    //         data_posi.emplace_back(Vector3D(data_i[j].x, data_i[j].y, data_i[j].z));
    //         data_radiusi.emplace_back(data_i[j].w);
    //     }

    //     if (datai_size > 1)
    //     {

    //         data.clear();

    //         for (auto j = 0; j < datai_size; j++)
    //         {
    //             data.emplace_back(Random::get(-1.0, 1.0));
    //         }

    //         int n = datai_size;
    //         int inequ_num = 2 * n + n * (n - 1) / 2;

    //         auto ipm = std::make_shared<OPTIMIZE::IPM::InteriorPointMethod>(data, inequ_num, data_radiusi, data_posi);
    //         auto results = ipm->solution();

    //         for (auto j = 0; j < datai_size; j++)
    //         {
    //             if (data_radiusi[j] / 100.0 * double(results[j]) >= 1e-4)
    //                 positions.emplace_back(Vector4D(data_posi[j].x / 100.0, data_posi[j].y / 100.0, data_posi[j].z / 100.0, data_radiusi[j] / 100.0 * double(results[j])));
    //             // KIRI_LOG_DEBUG("radius={0};{1}", data_radiusi[j] / 100.0 * double(results[j]), double(results[j]));
    //         }
    //         break;
    //     }
    //     else
    //     {
    //         for (auto j = 0; j < datai_size; j++)
    //             positions.emplace_back(Vector4D(data_posi[j].x / 100.0, data_posi[j].y / 100.0, data_posi[j].z / 100.0, data_radiusi[j] / 100.0));
    //     }
    // }
    ExportBgeoFileFromCPU("box", "box_opti", positions);
    return 0;
}
