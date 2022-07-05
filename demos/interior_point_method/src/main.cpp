/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-06-25 01:39:47
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-07-05 09:34:25
 * @FilePath: \Kiri2D\demos\interior_point_method\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-06-25 01:39:47
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-06-29 09:17:25
 * @FilePath: \Kiri2D\demos\interior_point_method\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#include <kiri2d.h>
using namespace KIRI2D;

#include <ipm.h>

#include <root_directory.h>
#include <partio/Partio.h>
Array1Vec4F ReadBgeoFileForCPU(String Folder, String Name, Vector3F Offset = Vector3F(0.f), bool FlipYZ = false)
{
    Array1Vec4F pos_array;
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

    float max_y = 0.f;
    for (Int i = 0; i < data->numParticles(); i++)
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
                pos_array.append(Vector4F(pos[0] + Offset.x, pos[2] + Offset.z, pos[1] + Offset.y, *pscale));
            }
            else
            {
                pos_array.append(Vector4F(pos[0] + Offset.x, pos[1] + Offset.y, pos[2] + Offset.z, *pscale));
                if (pos[1] > max_y)
                    max_y = pos[1];
            }
        }
        else
        {
            if (FlipYZ)
            {
                pos_array.append(Vector4F(pos[0] + Offset.x, pos[2] + Offset.z, pos[1] + Offset.y, 0.01f));
            }
            else
            {
                pos_array.append(Vector4F(pos[0] + Offset.x, pos[1] + Offset.y, pos[2] + Offset.z, 0.01f));
            }
        }
    }

    // printf("max_Y=%.3f , offset=(%.3f,%.3f,%.3f) \n", max_y, Offset.x, Offset.y, Offset.z);

    data->release();

    return pos_array;
}

void ExportBgeoFileFromCPU(String Folder, String FileName, Array1Vec4F Positions)
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

    auto bgeo_data = ReadBgeoFileForCPU("box", "box_small");
    auto data_size = bgeo_data.size();
    KIRI_LOG_DEBUG("data size={0}", data_size);

    std::vector<double> data;
    std::vector<Vector3F> data_pos;
    std::vector<Vector3F> data_pos1;
    std::vector<double> data_radius;

    for (auto i = 0; i < data_size; i++)
    {
        data_pos.emplace_back(Vector3F(bgeo_data[i].x, bgeo_data[i].y, bgeo_data[i].z) * 100.f);
        data_pos1.emplace_back(Vector3F(bgeo_data[i].x, bgeo_data[i].y, bgeo_data[i].z));
        data_radius.emplace_back(bgeo_data[i].w * 100.f);
    }

    KIRI_LOG_DEBUG("dist1={0}; dist2={1}", (data_pos[0] - data_pos[1]).length(), (data_pos1[0] - data_pos1[1]).length());

    // data_pos.emplace_back(Vector3F(0.25, 0.2, 0.25));
    // data_pos.emplace_back(Vector3F(0.25, 0.4, 0.25));

    // data_radius.emplace_back(0.2);
    // data_radius.emplace_back(0.2);

    Array1Vec4F positions;

    for (auto i = 0; i < 1; i++)
    {
        data.clear();

        for (auto j = 0; j < data_size; j++)
        {
            data.emplace_back(Random::get(-1.0, 1.0));
        }

        KIRI_LOG_INFO("Attempt num={0}", i + 1);

        for (auto j = 0; j < data_size; j++)
        {
            KIRI_LOG_INFO("Init Data[{0}]={1}", j, data[j]);
        }

        int n = data_size;
        int inequ_num = 2 * n + n * (n - 1) / 2;

        auto ipm = std::make_shared<OPTIMIZE::IPM::InteriorPointMethod>(data, inequ_num, data_radius, data_pos);
        auto results = ipm->solution();
        for (auto j = 0; j < data_size; j++)
        {
            // positions.append(Vector4F(bgeo_data[j].x, bgeo_data[j].y, bgeo_data[j].z, bgeo_data[j].w));
            positions.append(Vector4F(bgeo_data[j].x, bgeo_data[j].y, bgeo_data[j].z, bgeo_data[j].w * double(results[j])));
            //  KIRI_LOG_INFO("RES Data[{0}]={1};{2};{3}", j, bgeo_data[j].w, results[j], bgeo_data[j].w * results[j]);
        }
        ExportBgeoFileFromCPU("box", "box_opti", positions);
    }
    return 0;
}
