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

int main(int argc, char *argv[])
{
    // log system
    KiriLog::init();

    std::vector<double> data;

    std::vector<Vector2F> data_pos;
    std::vector<double> data_radius;

    data_pos.emplace_back(Vector2F(0.25, 0.25));
    data_pos.emplace_back(Vector2F(0.25, 0.5));
    data_pos.emplace_back(Vector2F(0.25, 0.75));
    data_pos.emplace_back(Vector2F(0.5, 0.25));
    data_pos.emplace_back(Vector2F(0.5, 0.5));
    data_pos.emplace_back(Vector2F(0.5, 0.75));
    data_pos.emplace_back(Vector2F(0.75, 0.25));
    data_pos.emplace_back(Vector2F(0.75, 0.5));
    data_pos.emplace_back(Vector2F(0.75, 0.75));

    data_radius.emplace_back(0.22331851);
    data_radius.emplace_back(0.24589801);
    data_radius.emplace_back(0.22490009);
    data_radius.emplace_back(0.21019611);
    data_radius.emplace_back(0.22617922);

    data_radius.emplace_back(0.22282152);
    data_radius.emplace_back(0.2403757);
    data_radius.emplace_back(0.24426692);
    data_radius.emplace_back(0.2110915);

    for (auto i = 0; i < 1; i++)
    {
        data.clear();
        data.emplace_back(Random::get(-1.0, 1.0));
        data.emplace_back(Random::get(-1.0, 1.0));
        data.emplace_back(Random::get(-1.0, 1.0));
        data.emplace_back(Random::get(-1.0, 1.0));
        data.emplace_back(Random::get(-1.0, 1.0));
        data.emplace_back(Random::get(-1.0, 1.0));
        data.emplace_back(Random::get(-1.0, 1.0));
        data.emplace_back(Random::get(-1.0, 1.0));
        data.emplace_back(Random::get(-1.0, 1.0));

        KIRI_LOG_INFO("Attempt num={0}", i + 1);

        for (auto j = 0; j < data.size(); j++)
        {
            KIRI_LOG_INFO("Init Data[{0}]={1}", j, data[j]);
        }

        int n = data.size();
        int inequ_num = 2 * n + n * (n - 1) / 2;

        auto ipm = std::make_shared<OPTIMIZE::IPM::InteriorPointMethod>(data, inequ_num, data_radius, data_pos);
    }
    return 0;
}
