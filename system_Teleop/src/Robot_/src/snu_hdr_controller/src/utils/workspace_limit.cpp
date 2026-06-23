#include "utils/workspace_limit.h"

namespace workspace_limit
{
    bool selfCollision(const Eigen::Vector2d x, const double r)
    {
        double dist;
        dist = x.norm();

        if(dist <= r){
            std::cerr << "[WARNING] Self collision risk detected!"<<std::endl;
            return true;
        }
        else{
            return false;
        }
    }

    bool x_limit(const double x, const double x_max, const double x_min)
    {
        if(x > x_min && x < x_max){
            return false;
        }
        else
        {
            std::cerr << "[WARNING] X-axis position exceeded workspace boundary." << std::endl;
            return true;
        }
    }

    bool y_limit(const double y, const double y_max, const double y_min)
    {
        if(y > y_min && y < y_max){
            return false;
        }
        else
        {
            std::cerr << "[WARNING] Y-axis position exceeded workspace boundary." << std::endl;
            return true;
        }
    }


    bool z_limit(const double z, const double z_max, const double z_min)
    {
        if(z > z_min && z < z_max){
            return false;
        }
        else
        {
            std::cerr << "[WARNING] Z-axis position exceeded workspace boundary." << std::endl;
            return true;
        }
    }


}
