#ifndef _DATA_H_
#define _DATA_H_

#pragma once

#include <kiri2d.h>
using namespace KIRI2D;

#include <tuple>

namespace OPTIMIZE::IPM
{
    struct particle
    {
        Vector3D pos;
        double radius;
        double max_radius;
        double min_dist;
        bool optimize;
    };
}

#endif