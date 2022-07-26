/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-07-21 11:00:14
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-07-25 12:34:42
 * @FilePath: \Kiri2D\demos\pdipm_sampler\src\main.cpp
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#include <gridding.h>
#include <kiri2d.h>
#include <partio/Partio.h>
#include <primal_dual_ipm.h>
#include <root_directory.h>
#include <sph_grid.h>

// #include <voronoi/voronoi_polygon.h>

// #include <Discregrid/All>
#include <Eigen/Dense>

using namespace KIRI2D;

std::vector<Vector4D> ReadBgeoFileForCPU(String Folder, String Name,
                                         Vector3D Offset = Vector3D(0.0),
                                         bool FlipYZ = false) {
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
      pos_attr.count != 3) {
    KIRI_LOG_ERROR("Failed to Get Proper Position Attribute");
  }

  bool pscaleLoaded = data->attributeInfo("pscale", pscale_attr);

  double max_y = 0.0;
  for (auto i = 0; i < data->numParticles(); i++) {
    const float *pos = data->data<float>(pos_attr, i);
    if (pscaleLoaded) {
      const float *pscale = data->data<float>(pscale_attr, i);
      if (i == 0) {
        KIRI_LOG_INFO("pscale={0}", *pscale);
      }

      if (FlipYZ) {
        pos_array.emplace_back(Vector4D(pos[0] + Offset.x, pos[2] + Offset.z,
                                        pos[1] + Offset.y, *pscale));
      } else {
        pos_array.emplace_back(Vector4D(pos[0] + Offset.x, pos[1] + Offset.y,
                                        pos[2] + Offset.z, *pscale));
        if (pos[1] > max_y)
          max_y = pos[1];
      }
    } else {
      if (FlipYZ) {
        pos_array.emplace_back(Vector4D(pos[0] + Offset.x, pos[2] + Offset.z,
                                        pos[1] + Offset.y, 0.01f));
      } else {
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
                           std::vector<Vector4D> Positions) {
  String exportPath =
      String(EXPORT_PATH) + "bgeo/" + Folder + "/" + FileName + ".bgeo";

  Partio::ParticlesDataMutable *p = Partio::create();
  Partio::ParticleAttribute positionAttr =
      p->addAttribute("position", Partio::VECTOR, 3);
  Partio::ParticleAttribute pScaleAttr =
      p->addAttribute("pscale", Partio::FLOAT, 1);

  for (UInt i = 0; i < Positions.size(); i++) {
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

int main(int argc, char *argv[]) {
  // log system
  KiriLog::init();

  auto bgeo_data = ReadBgeoFileForCPU("box", "box_180");
  auto data_size = bgeo_data.size();
  KIRI_LOG_DEBUG("data size={0}", data_size);

  int n = data_size;
  double scale = 100.0;

  // auto cdf_file_path =
  //     String(RESOURCES_PATH) + "cdf" + "/" + "box" + "/" + "box" + ".cdf";
  // auto sdf = std::unique_ptr<Discregrid::CubicLagrangeDiscreteGrid>(
  //     new Discregrid::CubicLagrangeDiscreteGrid(cdf_file_path));

  std::vector<OPTIMIZE::IPM::particle> data_particles;
  std::vector<Vector3D> data_pos;
  std::vector<Vector4D> positions;

  BoundingBox3D bounding_box;
  auto max_radius = 0.0;
  for (auto i = 0; i < n; i++) {
    OPTIMIZE::IPM::particle p;
    p.pos = Vector3D(bgeo_data[i].x, bgeo_data[i].y, bgeo_data[i].z) * scale;
    p.radius = bgeo_data[i].w * scale;
    p.optimize = false;

    // auto dist = sdf->interpolate(
    //     0u, Eigen::Vector3d(bgeo_data[i].x, bgeo_data[i].y, bgeo_data[i].z));
    // // KIRI_LOG_DEBUG("dist={0}", dist);
    // p.max_radius = abs(dist) * scale;

    data_particles.emplace_back(p);
    data_pos.emplace_back(p.pos);

    // std::cout << "radius=" << p.radius << std::endl;

    max_radius = std::max(p.radius, max_radius);
    bounding_box.merge(p.pos);
  }

  bool flag = true;
  auto optimized_number = 0;
  std::vector<double> data;

  for (auto j = 0; j < n; j++) {
    data.emplace_back(Random::get(0.0, 1.0));
  }

  // search neighbor particles
  auto searcher = std::make_shared<OPTIMIZE::IPM::Grid>(
      bounding_box.HighestPoint, bounding_box.LowestPoint, max_radius * 2.f);
  searcher->updateStructure(data_pos);
  // auto neighborhoods = std::vector<std::vector<int>>();
  float maxDist2 = max_radius * max_radius;

  auto dist_constrains_num = 0;
  std::unordered_map<std::string, int> pair_hash_table;
  for (int i = 0; i < data_pos.size(); i++) {
    std::vector<int> neighbors = std::vector<int>();
    std::vector<OPTIMIZE::IPM::Cell> neighboringCells =
        searcher->getNeighboringCells(data_pos[i]);

    for each (const OPTIMIZE::IPM::Cell &cell in neighboringCells) {
      for each (int index in cell) {
        if (index != i) {
          auto key_str = std::to_string(i) + "_" + std::to_string(index);
          if (pair_hash_table.find(key_str) == pair_hash_table.end()) {
            neighbors.emplace_back(index);

            auto reverse_key_str =
                std::to_string(index) + "_" + std::to_string(i);
            pair_hash_table.emplace(reverse_key_str, 0);
          }
        }
      }
    }
    // std::cout << "neighbor size=" << neighbors.size() << std::endl;
    data_particles[i].neighbors = neighbors;
    dist_constrains_num += neighbors.size();
    // neighborhoods.push_back(neighbors);
  }

  int equ_num = optimized_number;
  // int inequ_num = 2 * (n - optimized_number) + n * (n - 1) / 2;
  int inequ_num = 2 * (n - optimized_number) + dist_constrains_num;

  KIRI_LOG_DEBUG("equ number={0}; inequ number={1}", equ_num, inequ_num);

  double start_timer = clock();

  auto ipm = std::make_shared<OPTIMIZE::IPM::PrimalDualIPM>(
      data, data_particles, data_particles, equ_num, inequ_num);
  auto results = ipm->solution();

  double end_timer = clock();
  double thisTime = (double)(end_timer - start_timer) / CLOCKS_PER_SEC;

  KIRI_LOG_DEBUG("running time={0}", thisTime);

  for (auto j = 0; j < data_particles.size(); j++) {
    positions.emplace_back(Vector4D(
        data_particles[j].pos.x / scale, data_particles[j].pos.y / scale,
        data_particles[j].pos.z / scale,
        data_particles[j].radius * double(results[j]) / scale));
  }
  ExportBgeoFileFromCPU("box", "box_opti", positions);

  // -----------
  // auto gridding = std::make_shared<OPTIMIZE::IPM::Gradding>(data_particles,
  // 3, 3, 3); auto grid_size = gridding->maxGridHash(); for (auto i = 13; i <
  // 14; i++)
  // {
  //   auto [grid_particles, particles_index] = gridding->getDataByGridHash(i);
  //   KIRI_LOG_DEBUG("grid data size={0}", grid_particles.size());

  //   n = grid_particles.size();
  //   std::vector<double> data;

  //   bool flag = true;
  //   auto optimized_number = 0;

  //   for (auto j = 0; j < n; j++)
  //   {
  //     data.emplace_back(Random::get(0.0, 1.0));
  //   }

  //   int equ_num = optimized_number;
  //   int inequ_num = 2 * (n - optimized_number) + n * (n - 1) / 2;

  //   auto ipm = std::make_shared<OPTIMIZE::IPM::PrimalDualIPM>(
  //       data, grid_particles, equ_num, inequ_num);
  //   auto results = ipm->solution();

  //   for (auto j = 0; j < n; j++)
  //   {
  //     data_particles[particles_index[j]].optimize = true;
  //     data_particles[particles_index[j]].radius =
  //         data_particles[particles_index[j]].radius * double(results[j]);
  //   }

  //   for (auto j = 0; j < grid_particles.size(); j++)
  //   {
  //     positions.emplace_back(Vector4D(
  //         data_particles[particles_index[j]].pos.x / scale,
  //         data_particles[particles_index[j]].pos.y / scale,
  //         data_particles[particles_index[j]].pos.z / scale,
  //         data_particles[particles_index[j]].radius / scale));
  //   }
  // }

  //----------
  // auto searcher = std::make_shared<OPTIMIZE::IPM::Grid>(
  //     bounding_box.HighestPoint, bounding_box.LowestPoint, max_radius * 2.0);
  // searcher->updateStructure(data_pos);
  // auto neighborhoods = std::vector<std::vector<int>>();
  // float maxDist2 = max_radius * max_radius;

  // for (int i = 0; i < data_pos.size(); i++) {
  //   std::vector<int> neighbors = std::vector<int>();
  //   std::vector<OPTIMIZE::IPM::Cell> neighboringCells =
  //       searcher->getNeighboringCells(data_pos[i]);

  //   for each (const OPTIMIZE::IPM::Cell &cell in neighboringCells) {
  //     for each (int index in cell) {
  //       // if (index != i)
  //       neighbors.push_back(index);
  //     }
  //   }
  //   std::cout << "neighbor size=" << neighbors.size() << std::endl;
  //   neighborhoods.push_back(neighbors);
  // }

  // // for (int i = 0; i < neighborhoods.size(); i++) {
  // for (int i = 0; i < 1; i++) {
  //   std::vector<int> neighbors = neighborhoods[i];
  //   n = neighbors.size();
  //   std::vector<double> data;
  //   std::vector<particle> tmp_particles;

  //   bool flag = true;
  //   auto optimized_number = 0;
  //   auto dist_constrains_num = 0;

  //   // neighbor particles
  //   for (auto j = 0; j < n; j++) {
  //     auto neighbor_particle = data_particles[neighbors[j]];
  //     if (neighbor_particle.optimize) {
  //       optimized_number++;
  //     }

  //     // std::vector<int> neighbors_neighbors_need_optimize;
  //     // std::vector<int> neighbors_neighbors_need_constrain;
  //     // std::vector<OPTIMIZE::IPM::Cell> neighboringCells =
  //     //     searcher->getNeighboringCells(neighbor_particle.pos);

  //     // for each (const OPTIMIZE::IPM::Cell &cell in neighboringCells) {
  //     //   for each (int index in cell) {
  //     //     // KIRI_LOG_DEBUG("neighbor index={0};", index);
  //     //     if (index == neighbors[j])
  //     //       continue;

  //     //     auto itr = std::find(neighbors.begin(), neighbors.end(), index);
  //     //     if (itr != neighbors.end())
  //     //       neighbors_neighbors_need_optimize.push_back(
  //     //           std::distance(neighbors.begin(), itr));
  //     //     else
  //     //       neighbors_neighbors_need_constrain.push_back(index);
  //     //   }
  //     // }
  //     // neighbor_particle.need_optimize = neighbors_neighbors_need_optimize;
  //     // neighbor_particle.need_constrain =
  //     neighbors_neighbors_need_constrain;
  //     // dist_constrains_num += neighbor_particle.need_optimize.size();

  //     tmp_particles.emplace_back(neighbor_particle);
  //   }

  //   for (auto j = 0; j < n; j++) {
  //     data.emplace_back(Random::get(0.0, 1.0));
  //   }

  //   int equ_num = optimized_number;
  //   int inequ_num = 2 * (n - optimized_number) + n * (n - 1) / 2;
  //   // int inequ_num = 2 * (n - optimized_number) + dist_constrains_num;

  //   auto ipm = std::make_shared<OPTIMIZE::IPM::PrimalDualIPM>(
  //       data, tmp_particles, data_particles, equ_num, inequ_num);
  //   auto results = ipm->solution();

  //   for (auto j = 0; j < n; j++) {
  //     data_particles[neighbors[j]].optimize = true;
  //     data_particles[neighbors[j]].radius =
  //         data_particles[neighbors[j]].radius * double(results[j]);

  //     // positions.emplace_back(
  //     //     Vector4D(data_particles[neighbors[j]].pos.x / scale,
  //     //              data_particles[neighbors[j]].pos.y / scale,
  //     //              data_particles[neighbors[j]].pos.z / scale,
  //     //              data_particles[neighbors[j]].radius / scale));
  //   }
  // }

  // auto volume = 0.0;

  // for (auto j = 0; j < data_particles.size(); j++) {
  //   // if (data_particles[j].radius / scale >= 1e-4) {
  //   positions.emplace_back(Vector4D(
  //       data_particles[j].pos.x / scale, data_particles[j].pos.y / scale,
  //       data_particles[j].pos.z / scale, data_particles[j].radius / scale));
  //   volume += 4 / 3 * KIRI_PI<double>() * data_particles[j].radius *
  //             data_particles[j].radius * data_particles[j].radius;
  //   // KIRI_LOG_DEBUG("radius={0};", data_particles[j].radius / scale);
  //   //}
  // }
  // KIRI_LOG_DEBUG("volume={0};", volume);

  // ExportBgeoFileFromCPU("box", "box_opti", positions);
  return 0;
}
