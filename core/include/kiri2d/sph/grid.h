/***
 * @Author: Xu.WANG
 * @Date: 2022-02-19 12:52:52
 * @LastEditTime: 2022-02-19 12:53:25
 * @LastEditors: Xu.WANG
 * @Description:
 */

#ifndef _KIRI2D_SPH2D_GRID_H_
#define _KIRI2D_SPH2D_GRID_H_

#pragma once

#include <kiri2d/sph/particles.h>

namespace KIRI2D::SPH
{
    typedef std::vector<int> Cell;

    class Grid
    {
    public:
        Grid()
        {
        }

        Grid(Vector2F worldSize, float radius)
        {
            kernelRadius = radius;

            numberCellsX = worldSize.x / kernelRadius + 1;
            numberCellsY = worldSize.y / kernelRadius + 1;
        }

        std::vector<Cell> getNeighboringCells(Vector2F position)
        {

            std::vector<Cell> resultCells = std::vector<Cell>();

            int xCell = position.x / kernelRadius;
            int yCell = position.y / kernelRadius;

            resultCells.push_back(cells[xCell][yCell]);
            if (xCell > 0)
                resultCells.push_back(cells[xCell - 1][yCell]);
            if (xCell < numberCellsX - 1)
                resultCells.push_back(cells[xCell + 1][yCell]);

            if (yCell > 0)
                resultCells.push_back(cells[xCell][yCell - 1]);
            if (yCell < numberCellsY - 1)
                resultCells.push_back(cells[xCell][yCell + 1]);

            if (xCell > 0 && yCell > 0)
                resultCells.push_back(cells[xCell - 1][yCell - 1]);
            if (xCell > 0 && yCell < numberCellsY - 1)
                resultCells.push_back(cells[xCell - 1][yCell + 1]);
            if (xCell < numberCellsX - 1 && yCell > 0)
                resultCells.push_back(cells[xCell + 1][yCell - 1]);
            if (xCell < numberCellsX - 1 && yCell < numberCellsY - 1)
                resultCells.push_back(cells[xCell + 1][yCell + 1]);

            return resultCells;
        }

        void updateStructure(std::vector<Particles> &particles)
        {
            cells = std::vector<std::vector<Cell>>(numberCellsX, std::vector<Cell>(numberCellsY, Cell()));

            for (int i = 0; i < particles.size(); i++)
            {
                int xCell = particles[i].position.x / kernelRadius;
                int yCell = particles[i].position.y / kernelRadius;
                cells[xCell][yCell].push_back(i);
            }
        }

    private:
        int numberCellsX;
        int numberCellsY;

        float kernelRadius;

        std::vector<std::vector<Cell>> cells;
    };

} // namespace KIRI2D::SPH

#endif /* _KIRI2D_SPH2D_GRID_H_ */
