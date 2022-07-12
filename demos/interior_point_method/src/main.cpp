/*** 
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-07-12 11:08:48
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-07-12 17:43:27
 * @FilePath: \Kiri2D\demos\interior_point_method\src\main.cpp
 * @Description: 
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved. 
 */
#include <kiri2d.h>
using namespace KIRI2D;

#include <ipm.h>
#include <sph_grid.h>

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

    auto bgeo_data = ReadBgeoFileForCPU("box", "box_small");
    auto data_size = bgeo_data.size();
    KIRI_LOG_DEBUG("data size={0}", data_size);

    int n = data_size;
    double scale = 100.0;

    
    std::vector<particle> data_particles;
    std::vector<Vector3D> data_pos;

    std::vector<Vector4D> positions;

     BoundingBox3D bounding_box;
     auto max_radius = 0.0;
    for (auto i = 0; i < n; i++)
    {
        particle p;
        p.pos = Vector3D(bgeo_data[i].x, bgeo_data[i].y, bgeo_data[i].z) * scale;
        p.radius = bgeo_data[i].w * scale;
        p.optimize = false;
        data_particles.emplace_back(p);
        data_pos.emplace_back(p.pos);

        // std::cout << "radius=" << p.radius << std::endl;


        max_radius = std::max(p.radius,max_radius);
        bounding_box.merge( p.pos);
    }

    auto searcher = std::make_shared<OPTIMIZE::IPM::Grid>(bounding_box.HighestPoint , bounding_box.LowestPoint,max_radius);
searcher->updateStructure(data_pos);
 auto neighborhoods = std::vector<std::vector<int>>();
            float maxDist2 = max_radius * max_radius;

            for (int i = 0; i < data_pos.size(); i++)
            {
                std::vector<int> neighbors = std::vector<int>();
                std::vector<OPTIMIZE::IPM::Cell> neighboringCells = searcher->getNeighboringCells(data_pos[i]);

                for each (const OPTIMIZE::IPM::Cell &cell in neighboringCells)
                {
                    for each (int index in cell)
                    {
                    
                            neighbors.push_back(index);
                       
                    }
                }
                 std::cout << "neighbor size=" << neighbors.size() << std::endl;
                neighborhoods.push_back(neighbors);
            }


            for (int i = 0; i < neighborhoods.size(); i++)
            {
                
    
                std::vector<int> neighbors = neighborhoods[i];
                n = neighbors.size();
                std::vector<double> data;
                 std::vector<particle> tmp_particles;

                 bool flag = true;

                for (auto j = 0; j < n; j++)
                    {
                         tmp_particles.emplace_back(data_particles[neighbors[j]]);
                         
                         if(data_particles[neighbors[j]].optimize == false)
                         {
                            flag=false;
                         }
                            
                         KIRI_LOG_DEBUG("tmp particle id={0}; optimized={1}",neighbors[j],data_particles[neighbors[j]].optimize);
                    }

                    if(flag)
                        continue;

                for (auto j = 0; j < n; j++)
                    {
                        data.emplace_back(Random::get(-1.0, 1.0));
                    }

                    int inequ_num = 2 * n + n * (n - 1) / 2;

                    auto ipm = std::make_shared<OPTIMIZE::IPM::InteriorPointMethod>(data, inequ_num, tmp_particles);
                    auto results = ipm->solution();

                    for (auto j = 0; j < n; j++)
                    {
                         data_particles[neighbors[j]].optimize = true;
                          KIRI_LOG_DEBUG("new particle id={0}; radius={1}, scale={2}",neighbors[j],data_particles[neighbors[j]].radius,double(results[j]));
                         data_particles[neighbors[j]].radius =data_particles[neighbors[j]].radius * double(results[j]);
                         
                    }

            }

             for (auto j = 0; j < data_particles.size(); j++)
                    {
                        // if (data_radius[j] / scale * double(results[j]) >= 1e-4)
                        positions.emplace_back(Vector4D(data_particles[j].pos.x / scale, data_particles[j].pos.y / scale, data_particles[j].pos.z / scale, data_particles[j].radius / scale ));
                        KIRI_LOG_DEBUG("radius={0};", data_particles[j].radius / scale);
                    }

    // for (auto j = 0; j < n; j++)
    // {
    //     data.emplace_back(Random::get(-1.0, 1.0));
    // }

    // int inequ_num = 2 * n + n * (n - 1) / 2;

    // auto ipm = std::make_shared<OPTIMIZE::IPM::InteriorPointMethod>(data, inequ_num, data_radius, data_pos);
    // auto results = ipm->solution();

    // for (auto j = 0; j < n; j++)
    // {
    //     // if (data_radius[j] / scale * double(results[j]) >= 1e-4)
    //     positions.emplace_back(Vector4D(data_pos[j].x / scale, data_pos[j].y / scale, data_pos[j].z / scale, data_radius[j] / scale * double(results[j])));
    //     KIRI_LOG_DEBUG("radius={0};{1}", data_radius[j] / scale * double(results[j]), double(results[j]));
    // }

   ExportBgeoFileFromCPU("box", "box_opti", positions);
    return 0;
}
