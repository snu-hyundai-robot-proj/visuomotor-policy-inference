#pragma once

#include <vector>
#include <Eigen/Dense>

#include "utils/common_math.h"

namespace workspace_limit
{
    bool selfCollision(const Eigen::Vector2d x, const double r);    
    bool x_limit(const double x, const double x_max, const double x_min);
    bool y_limit(const double y, const double y_max, const double y_min);
    bool z_limit(const double z, const double z_max, const double z_min);
}
