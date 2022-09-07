/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2022-07-21 11:00:14
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2022-09-02 16:39:52
 * @FilePath: \Kiri2D\demos\pdipm_sampler\include\sph_grid.h
 * @Description:
 * @Copyright (c) 2022 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */

#ifndef _SPH_GRID_H
#define _SPH_GRID_H

#pragma once
#include <data.h>

namespace OPTIMIZE::IPM {
typedef std::vector<int> Cell;

class Grid {
public:
  Grid() {}

  Grid(Vector3D high, Vector3D low, double radius) {
    kernelRadius = radius;
    auto world_size = high - low;
    mLowestPoint = low;

    numberCells.x = world_size.x / kernelRadius + 1;
    numberCells.y = world_size.y / kernelRadius + 1;
    numberCells.z = world_size.z / kernelRadius + 1;
  }

  std::vector<Cell> getNeighboringCells(Vector3D position) {

    auto resultCells = std::vector<Cell>();
    auto rel_pos = position - mLowestPoint;
    int xCell = rel_pos.x / kernelRadius;
    int yCell = rel_pos.y / kernelRadius;
    int zCell = rel_pos.z / kernelRadius;

    resultCells.emplace_back(cells[xCell][yCell][zCell]);
    if (xCell > 0)
      resultCells.emplace_back(cells[xCell - 1][yCell][zCell]);
    if (xCell < numberCells.x - 1)
      resultCells.emplace_back(cells[xCell + 1][yCell][zCell]);

    if (yCell > 0)
      resultCells.emplace_back(cells[xCell][yCell - 1][zCell]);
    if (yCell < numberCells.y - 1)
      resultCells.emplace_back(cells[xCell][yCell + 1][zCell]);

    if (zCell > 0)
      resultCells.emplace_back(cells[xCell][yCell][zCell - 1]);
    if (zCell < numberCells.z - 1)
      resultCells.emplace_back(cells[xCell][yCell][zCell + 1]);

    if (xCell > 0 && yCell > 0)
      resultCells.emplace_back(cells[xCell - 1][yCell - 1][zCell]);
    if (xCell > 0 && yCell < numberCells.y - 1)
      resultCells.emplace_back(cells[xCell - 1][yCell + 1][zCell]);

    if (xCell > 0 && zCell > 0)
      resultCells.emplace_back(cells[xCell - 1][yCell][zCell - 1]);
    if (xCell > 0 && zCell < numberCells.z - 1)
      resultCells.emplace_back(cells[xCell - 1][yCell][zCell + 1]);

    if (yCell > 0 && zCell > 0)
      resultCells.emplace_back(cells[xCell][yCell - 1][zCell - 1]);
    if (yCell > 0 && zCell < numberCells.z - 1)
      resultCells.emplace_back(cells[xCell][yCell - 1][zCell + 1]);

    if (xCell < numberCells.x - 1 && yCell > 0)
      resultCells.emplace_back(cells[xCell + 1][yCell - 1][zCell]);
    if (xCell < numberCells.x - 1 && yCell < numberCells.y - 1)
      resultCells.emplace_back(cells[xCell + 1][yCell + 1][zCell]);

    if (xCell < numberCells.x - 1 && zCell > 0)
      resultCells.emplace_back(cells[xCell + 1][yCell][zCell - 1]);
    if (xCell < numberCells.x - 1 && zCell < numberCells.z - 1)
      resultCells.emplace_back(cells[xCell + 1][yCell][zCell + 1]);

    if (yCell < numberCells.y - 1 && zCell > 0)
      resultCells.emplace_back(cells[xCell][yCell + 1][zCell - 1]);
    if (yCell < numberCells.y - 1 && zCell < numberCells.z - 1)
      resultCells.emplace_back(cells[xCell][yCell + 1][zCell + 1]);

    if (xCell < numberCells.x - 1 && yCell < numberCells.y - 1 &&
        zCell < numberCells.z - 1)
      resultCells.emplace_back(cells[xCell + 1][yCell + 1][zCell + 1]);
    if (xCell < numberCells.x - 1 && yCell < numberCells.y - 1 && zCell > 0)
      resultCells.emplace_back(cells[xCell + 1][yCell + 1][zCell - 1]);

    if (xCell < numberCells.x - 1 && yCell > 0 && zCell < numberCells.z - 1)
      resultCells.emplace_back(cells[xCell + 1][yCell - 1][zCell + 1]);
    if (xCell < numberCells.x - 1 && yCell > 0 && zCell > 0)
      resultCells.emplace_back(cells[xCell + 1][yCell - 1][zCell - 1]);

    if (xCell > 0 && yCell < numberCells.y - 1 && zCell < numberCells.z - 1)
      resultCells.emplace_back(cells[xCell - 1][yCell + 1][zCell + 1]);
    if (xCell > 0 && yCell < numberCells.y - 1 && zCell > 0)
      resultCells.emplace_back(cells[xCell - 1][yCell + 1][zCell - 1]);

    if (xCell > 0 && yCell > 0 && zCell < numberCells.z - 1)
      resultCells.emplace_back(cells[xCell - 1][yCell - 1][zCell + 1]);
    if (xCell > 0 && yCell > 0 && zCell > 0)
      resultCells.emplace_back(cells[xCell - 1][yCell - 1][zCell - 1]);

    return resultCells;
  }

  void updateStructure(std::vector<particle> &data) {
    cells = std::vector<std::vector<std::vector<Cell>>>(
        numberCells.x,
        std::vector<std::vector<Cell>>(
            numberCells.y, std::vector<Cell>(numberCells.z, Cell())));

    for (int i = 0; i < data.size(); i++) {
      auto rel_pos = data[i].pos - mLowestPoint;
      int xCell = rel_pos.x / kernelRadius;
      int yCell = rel_pos.y / kernelRadius;
      int zCell = rel_pos.z / kernelRadius;
      cells[xCell][yCell][zCell].emplace_back(i);
    }
  }

private:
  Size3 numberCells;

  double kernelRadius;

  Vector3D mLowestPoint;

  std::vector<std::vector<std::vector<Cell>>> cells;
};
typedef std::shared_ptr<Grid> GridPtr;
} // namespace OPTIMIZE::IPM

#endif /* _SPH_GRID_H */
