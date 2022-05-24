/***
 * @Author: Xu.WANG
 * @Date: 2022-05-10 17:22:25
 * @LastEditTime: 2022-05-10 17:22:26
 * @LastEditors: Xu.WANG
 * @Description:
 */
#include <uni_dem_example.h>

using namespace KIRI2D;

void main(int argc, char *argv[])
{
    KiriLog::init();

    auto app = std::make_shared<UniDEM2DExample>();

    while (CUDA_DEM_APP_PARAMS.run)
        app->update();

    return;
}