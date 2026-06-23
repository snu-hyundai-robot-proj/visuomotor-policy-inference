#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "utils/common_math.h"

class AdmittanceController
{
public:
    struct Param
    {
        Eigen::Vector6d mass = Eigen::Vector6d::Ones();      // should be positive
        Eigen::Vector6d stiff = Eigen::Vector6d::Ones();     // shoud be positive
        Eigen::Vector6d zeta = Eigen::Vector6d::Ones();      // damping ratio
        Eigen::Vector6d adm_axis = Eigen::Vector6d::Ones();  // select specific axes to implement admittance control, should be binary
        Eigen::Matrix3d R_ref = Eigen::Matrix3d::Identity(); // reference frame for the final output
        double dt = 0.001; // 1khz
    };

    AdmittanceController() {};

    void initialize(const Param &p);
    void setParam(const Param &p);
    void compute(const Eigen::Ref<const Eigen::VectorXd> &x,
                 const Eigen::Ref<const Eigen::VectorXd> &x_dot,
                 const Eigen::Ref<const Eigen::VectorXd> &f_ext);

    Eigen::Vector6d getAdmittance(const Eigen::Ref<const Eigen::MatrixXd> &R_ref = Eigen::Matrix3d::Identity());
    Eigen::Vector6d saturateTaskVelocity(Eigen::Vector6d& twist, double v_max, double w_max);

    // if |x| >= a then, x = x, else x = 0
    Eigen::VectorXd noiseGate(const Eigen::Ref<const Eigen::VectorXd> &x,
                              const Eigen::Ref<const Eigen::VectorXd> &thres);

    double getDT();

    int dof = 6;

private:
    void defineGains();

    Param p_;
    Eigen::Vector6d inv_mass_;
    Eigen::Vector6d d_;
    Eigen::Vector6d axis_mask_;

    Eigen::Vector6d x_ddot_adm_, x_dot_adm_, x_adm_;

    std::ofstream debug_amit{"/home/dyros/ros2_ws/src/snu_hdr_controller/log/admittance_x_dot.txt"};
};