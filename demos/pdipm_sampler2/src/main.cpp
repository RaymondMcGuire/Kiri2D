/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-08-23 13:13:41
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-08-23 14:10:26
 * @FilePath: \Kiri2D\demos\pdipm_sampler2\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#include <gridding.h>
#include <kiri2d.h>
#include <offset_gridding.h>
#include <partio/Partio.h>
#include <primal_dual_ipm.h>
#include <root_directory.h>
#include <sph_grid.h>

#include <Discregrid/All>
#include <Eigen/Dense>

using namespace KIRI2D;

std::vector<Vector4D> ReadBgeoFileForCPU(String Folder, String Name,
                                         Vector3D Offset = Vector3D(0.0),
                                         bool FlipYZ = false)
{
  std::vector<Vector4D> pos_array;
  String root_folder = "bgeo";
  String extension = ".bgeo";
  String file_path = String(RESOURCES_PATH) + root_folder + "/" + Folder + "/" +
                     Name + extension;
  std::cout << file_path << std::endl;
  Partio::ParticlesDataMutable *data = Partio::read(file_path.c_str());

  Partio::ParticleAttribute pos_attr;
  Partio::ParticleAttribute pscale_attr;
  if (!data->attributeInfo("position", pos_attr) ||
      (pos_attr.type != Partio::FLOAT && pos_attr.type != Partio::VECTOR) ||
      pos_attr.count != 3)
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
        pos_array.emplace_back(Vector4D(pos[0] + Offset.x, pos[2] + Offset.z,
                                        pos[1] + Offset.y, *pscale));
      }
      else
      {
        pos_array.emplace_back(Vector4D(pos[0] + Offset.x, pos[1] + Offset.y,
                                        pos[2] + Offset.z, *pscale));
        if (pos[1] > max_y)
          max_y = pos[1];
      }
    }
    else
    {
      if (FlipYZ)
      {
        pos_array.emplace_back(Vector4D(pos[0] + Offset.x, pos[2] + Offset.z,
                                        pos[1] + Offset.y, 0.01f));
      }
      else
      {
        pos_array.emplace_back(Vector4D(pos[0] + Offset.x, pos[1] + Offset.y,
                                        pos[2] + Offset.z, 0.01f));
      }
    }
  }

  // printf("max_Y=%.3f , offset=(%.3f,%.3f,%.3f) \n", max_y, Offset.x,
  // Offset.y, Offset.z);

  data->release();

  return pos_array;
}

void ExportBgeoFileFromCPU(String Folder, String FileName,
                           std::vector<Vector4D> Positions)
{
  String exportPath =
      String(EXPORT_PATH) + "bgeo/" + Folder + "/" + FileName + ".bgeo";

  Partio::ParticlesDataMutable *p = Partio::create();
  Partio::ParticleAttribute positionAttr =
      p->addAttribute("position", Partio::VECTOR, 3);
  Partio::ParticleAttribute pScaleAttr =
      p->addAttribute("pscale", Partio::FLOAT, 1);

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

void computeVolume(const std::vector<particle> &particles)
{
  auto volume = 0.0;
  for (auto i = 0; i < particles.size(); i++)
  {
    volume += 4 / 3 * KIRI_PI<double>() * pow(particles[i].radius, 3);
  }
  KIRI_LOG_INFO("volume={0}", volume);
}

int main(int argc, char *argv[])
{
  // log system
  KiriLog::init();

  auto bgeo_data = ReadBgeoFileForCPU("box", "box_80");
  auto data_size = bgeo_data.size();
  KIRI_LOG_DEBUG("data size={0}; mkl max threads={1}", data_size,
                 mkl_get_max_threads());
  mkl_set_num_threads(mkl_get_max_threads());
  int n = data_size;
  double scale = 100.0;

  auto cdf_file_path =
      String(RESOURCES_PATH) + "cdf" + "/" + "box" + "/" + "box_80" + ".cdf";
  auto sdf = std::unique_ptr<Discregrid::CubicLagrangeDiscreteGrid>(
      new Discregrid::CubicLagrangeDiscreteGrid(cdf_file_path));

  std::vector<OPTIMIZE::IPM::particle> data_particles;
  std::vector<Vector4D> positions;

  BoundingBox3D bounding_box;
  auto max_radius = 0.0;
  for (auto i = 0; i < n; i++)
  {
    OPTIMIZE::IPM::particle p;
    p.pos = Vector3D(bgeo_data[i].x, bgeo_data[i].y, bgeo_data[i].z) * scale;
    p.radius = bgeo_data[i].w * scale;
    p.new_radius = bgeo_data[i].w * scale;

    auto dist = sdf->interpolate(
        0u, Eigen::Vector3d(bgeo_data[i].x, bgeo_data[i].y, bgeo_data[i].z));

    p.max_radius = abs(dist) * scale;

    data_particles.emplace_back(p);
    max_radius = std::max(p.radius, max_radius);
    bounding_box.merge(p.pos);
  }

  //---------------------- sph kernel search
  auto sph_searcher = std::make_shared<OPTIMIZE::IPM::Grid>(
      bounding_box.HighestPoint, bounding_box.LowestPoint, max_radius * 1.5);
  sph_searcher->updateStructure(data_particles);
  std::vector<std::vector<int>> neighborhoods;
  for (int i = 0; i < data_size; i++)
  {
    auto particle = data_particles[i];
    auto neighbors = std::vector<int>();
    std::vector<OPTIMIZE::IPM::Cell> neighboringCells =
        sph_searcher->getNeighboringCells(particle.pos);

    for each (const OPTIMIZE::IPM::Cell &cell in neighboringCells)
    {
      for each (int index in cell)
      {
        if (i != index)
          neighbors.push_back(index);
      }
    }
    neighborhoods.emplace_back(neighbors);
  }

  for (auto iter = 0; iter < 20; iter++)
  {

    // offset_gridding
    // auto offset_gridding = std::make_shared<OPTIMIZE::IPM::OffsetGridding>(
    //     data_particles, 3, 3, 3);

    auto offset_gridding = std::make_shared<OPTIMIZE::IPM::Gridding>(
        data_particles, 3, 3, 3);

    auto offset_grid_size = offset_gridding->maxGridHash();

    // auto flag = (iter + 1) % 2;
    for (auto i = 0; i < offset_grid_size; i++)
    {
      auto boundary_constrains_num = 0;
      auto [grid_particles, particles_index] =
          offset_gridding->getDataByGridHash(i);
      n = grid_particles.size();
      KIRI_LOG_DEBUG("grid data size={0}", n);

      if (n < 1)
      {
        continue;
      }

      // boundary constrains
      for (auto idx = 0; idx < particles_index.size(); idx++)
      {
        auto p_index = particles_index[idx];
        auto real_neighbors = std::vector<int>();
        for (auto j = 0; j < neighborhoods[p_index].size(); j++)
        {
          if (std::find(particles_index.begin(), particles_index.end(),
                        neighborhoods[p_index][j]) == particles_index.end())
            real_neighbors.emplace_back(neighborhoods[p_index][j]);
        }
        grid_particles[idx].neighbors = real_neighbors;
        boundary_constrains_num += real_neighbors.size();
      }

      VectorXreal results;
      int signal = -2;
      while (signal == -2)
      {
        // pmipm
        std::vector<double> data;
        for (auto j = 0; j < n; j++)
        {
          data.emplace_back(Random::get(0.0, 1.0));
        }

        int equ_num = 0;
        int inequ_num =
            2 * (n - equ_num) + n * (n - 1) / 2 + boundary_constrains_num;

        auto ipm = std::make_shared<OPTIMIZE::IPM::PrimalDualIPM>(
            data, grid_particles, data_particles, equ_num, inequ_num);

        signal = ipm->signal();
        results = ipm->solution();
      }

      for (auto j = 0; j < n; j++)
      {
        data_particles[particles_index[j]].new_radius = double(results[j]);
        data_particles[particles_index[j]].radius = double(results[j]);
        //  grid_particles[j].new_radius = double(results[j]);
      }

      offset_gridding->updateData(data_particles);
    }

    // for (auto j = 0; j < data_size; j++)
    // {
    //   data_particles[j].radius = data_particles[j].new_radius;
    // }

    computeVolume(data_particles);

    // export all
    positions.clear();
    for (auto j = 0; j < data_size; j++)
    {
      positions.emplace_back(Vector4D(
          data_particles[j].pos.x / scale, data_particles[j].pos.y / scale,
          data_particles[j].pos.z / scale, data_particles[j].radius / scale));
    }

    ExportBgeoFileFromCPU("box", "box_opti_iter_" + std::to_string(iter),
                          positions);
  }

  return 0;
}
