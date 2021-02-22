/*** 
 * @Author: Xu.WANG
 * @Date: 2021-02-22 18:24:22
 * @LastEditTime: 2021-02-22 18:25:04
 * @LastEditors: Xu.WANG
 * @Description: 
 * @FilePath: \Kiri2D\Kiri2d\src\kiri2d\scene.cpp
 */

#include <kiri2d/scene.h>

namespace KIRI2D
{
    void KiriScene2D::AddObject(KiriSDF2D object)
    {
        mSDFObjects.emplace_back(object);
    }
}