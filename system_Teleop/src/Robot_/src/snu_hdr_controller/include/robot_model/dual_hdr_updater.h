#pragma once

#include <fstream>
#include <Eigen/Dense>
#include <chrono>
#include <map>

#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joint-collection.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/aba.hpp>

#include "utils/common_math.h"

class DualHdrUpdater{
    public:
        static constexpr int n_joints_ = 6;
        static constexpr int n_arms_ = 2;
        static constexpr int n_dof_ = n_joints_*n_arms_;
        const std::vector<std::string> arm_names_ = {"left", "right"};

        DualHdrUpdater(){};    

        void initialize(const std::string urdf_path);

        void updateModel(const Eigen::Ref<const Eigen::VectorXd> &q,
                         const Eigen::Ref<const Eigen::VectorXd> &qd,
                         const Eigen::Ref<const Eigen::VectorXd> &ft); // update robot dynimics base on the current observation

        void updateKinematics(const Eigen::Ref<const Eigen::VectorXd> &q);
        // void updateDynamics();

        Eigen::Isometry3d getForwardKinematics(const int ee_index, const Eigen::Ref<const Eigen::VectorXd> &q);
        Eigen::VectorXd getInverseKinematics(const Eigen::Ref<const Eigen::VectorXd> &q_init,
                                             const Eigen::Isometry3d &x_goal,
                                             const int ee_index,
                                             const int max_iter = 1000,
                                             const double eps = 1e-4,
                                             const double dt = 0.005);

        void setInitialValues();

        void setPosition(const Eigen::Ref<const Eigen::VectorXd> &q,
                         std::string arm_name);

        Eigen::VectorXd getPosition();

        bool idle_controlled_{true};

        // Eigen::Matrix<double, 7, 7> M_; // joint mass_matrix_;
        // Eigen::Matrix<double, 7, 7> M_inv_; // inverse of joint mass_matrix_;
        // Eigen::Matrix<double, 6, 6> A_; // lambda_matrix_;
        // Eigen::Matrix<double, 7, 1> NLE_; // non-linear effect (corriolis + gravity)
        // Eigen::Matrix<double, 7, 7> C_; // coriolis_;
        // Eigen::Matrix<double, 7, 1> G_; // gravity_;

        std::unordered_map<std::string, Eigen::Matrix<double, n_joints_, 1>> q_init_;
        std::unordered_map<std::string, Eigen::Matrix<double, n_joints_, 1>> q_, q_cmd_;
        std::unordered_map<std::string, Eigen::Matrix<double, n_joints_, 1>> qd_;

        std::unordered_map<std::string, Eigen::Matrix<double, 6, 1>> f_ext_, f_ext_ee_;

        std::unordered_map<std::string, Eigen::Matrix<double, 6, n_joints_>> J_;
        std::unordered_map<std::string, Eigen::Matrix<double, 6, n_joints_>> J_pinv_;

        std::unordered_map<std::string, Eigen::Isometry3d> initial_transform_;
        std::unordered_map<std::string, Eigen::Isometry3d> transform_;

        std::unordered_map<std::string, Eigen::Isometry3d> x_ee_;
        std::unordered_map<std::string, Eigen::Matrix<double, 6, 1>> v_ee_;

        std::unordered_map<std::string, Eigen::Matrix<double, 6, 1>> xd_;
               
    private:

        pinocchio::Model model_;
        pinocchio::Data data_;

        std::string left_ee_frame_= "dg5f_flange_link";
        std::string right_ee_frame_= "rh56_flange_link";        
        
        std::vector<int> ee_ids_;        

};

    