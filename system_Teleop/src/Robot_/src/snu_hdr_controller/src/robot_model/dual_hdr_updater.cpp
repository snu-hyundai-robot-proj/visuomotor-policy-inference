#include "robot_model/dual_hdr_updater.h"
#include <Eigen/Geometry>
#include <fstream>


void DualHdrUpdater::initialize(const std::string urdf_path)
{   
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);    

    // std::cout << "로봇의 전체 프레임 목록:" << std::endl;
    // for (size_t i = 0; i < model_.frames.size(); ++i) {
    //     std::cout << "Frame " << i << ": " << model_.frames[i].name << std::endl;
    // }

    ee_ids_ = {
        model_.getFrameId(left_ee_frame_),
        model_.getFrameId(right_ee_frame_)
    };

}

void DualHdrUpdater::updateModel(const Eigen::Ref<const Eigen::VectorXd> &q,
                                  const Eigen::Ref<const Eigen::VectorXd> &qd,
                                  const Eigen::Ref<const Eigen::VectorXd> &ft)
{
    Eigen::VectorXd q_total = q;
    Eigen::VectorXd qd_total = qd;

    // segment per each arm
    for (size_t i = 0; i < arm_names_.size(); i++)
    {
        q_[arm_names_[i]] = q.segment(i * n_joints_, n_joints_);
        qd_[arm_names_[i]] = qd.segment(i * n_joints_, n_joints_);
        f_ext_[arm_names_[i]] = ft.segment(i * 6, 6);

        Eigen::Matrix3d R_be;
        R_be = initial_transform_[arm_names_[i]].linear();
        f_ext_ee_[arm_names_[i]].head<3>() = R_be.transpose()*f_ext_[arm_names_[i]].head<3>();
        f_ext_ee_[arm_names_[i]].tail<3>() = R_be.transpose()*f_ext_[arm_names_[i]].tail<3>();
    }

    // Forward kinematics
    pinocchio::forwardKinematics(model_, data_, q_total, qd_total);
    pinocchio::updateFramePlacements(model_, data_);

    updateKinematics(q_total);

    // std::cout<<"J_left: \n"<<J_[arm_names_[0]]<<"\n";
    // std::cout<<"J_right: \n"<<J_[arm_names_[1]]<<"\n";

    // compute core parameters
    // pinocchio::crba(model_, data_, q_total, pinocchio::Convention::WORLD); // compute Jabian & FK with Convention::WORLD

    // pinocchio::computeCoriolisMatrix(model_, data_, q_, qd_); // obtain corioli matrix for MOB
    // pinocchio::computeGeneralizedGravity(model_, data_, q);   // obtain grative vector
    // updateDynamics();
}

void DualHdrUpdater::updateKinematics(const Eigen::Ref<const Eigen::VectorXd> &q)
{
    for (size_t i = 0; i < arm_names_.size(); i++)
    {
        Eigen::MatrixXd J_temp(6, n_dof_), J;
        const int offset = i * n_joints_;

        J_temp.setZero();

        pinocchio::computeFrameJacobian(model_, data_, q, ee_ids_[i], pinocchio::LOCAL_WORLD_ALIGNED, J_temp);
        J_[arm_names_[i]] = J_temp.middleCols(offset, n_joints_);

        J = J_[arm_names_[i]];

        double lambda = 1e-4;

        J_pinv_[arm_names_[i]] = J.transpose()*(J * J.transpose() + lambda*Eigen::MatrixXd::Identity(6,6)).inverse();

        transform_[arm_names_[i]] = data_.oMf[ee_ids_[i]].toHomogeneousMatrix();
        xd_[arm_names_[i]] = J_[arm_names_[i]]*qd_[arm_names_[i]];

        x_ee_[arm_names_[i]] = initial_transform_[arm_names_[i]].inverse()*transform_[arm_names_[i]];

        v_ee_[arm_names_[i]].head<3>() = initial_transform_[arm_names_[i]].linear().transpose()*xd_[arm_names_[i]].head<3>();
        v_ee_[arm_names_[i]].tail<3>() = initial_transform_[arm_names_[i]].linear().transpose()*xd_[arm_names_[i]].tail<3>();
    }
}


// void DualHdrUpdater::updateDynamics()
// {
//     M_ = data_.M;
//     M_.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>(); // make full symetric matrix

//     M_inv_ =  M_.inverse();

//     C_ = data_.C;
//     G_ = data_.g;

//     // NLE_ = data_.nle; // C*q_dot + G
//     NLE_ = C_*qd_ + G_;

//     A_ = (J_*M_inv_*J_.transpose()).inverse(); // mass matrix in the task space, A = (J*M^-1*J^T)^-1
//     J_bar_ = M_inv_*J_.transpose()*A_; // dynamically consistant inverse of jacobian        
//     N_ = I_ - J_.transpose()*J_bar_.transpose(); // Null-space Projection
    
//     // tau_ext_ = mob_.run(M_, C_, G_, tau_measured_ + NLE_, qd_); // origin
//     // f_ext_ = J_bar_.transpose()*tau_ext_;

//     // f_ee_ext_.head<3>() = transform_.linear().transpose()*f_ext_.head<3>();
//     // f_ee_ext_.tail<3>() = transform_.linear().transpose()*f_ext_.tail<3>();    

// }

Eigen::Isometry3d DualHdrUpdater::getForwardKinematics(const int ee_index, const Eigen::Ref<const Eigen::VectorXd> &q)
{
    pinocchio::Data data_tmp;
    Eigen::Isometry3d x;

    data_tmp = pinocchio::Data(model_);
    pinocchio::forwardKinematics(model_, data_tmp, q);
    pinocchio::updateFramePlacements(model_, data_tmp);

    x = data_tmp.oMf[ee_ids_[ee_index]].toHomogeneousMatrix();

    return x;
}

Eigen::VectorXd DualHdrUpdater::getInverseKinematics(const Eigen::Ref<const Eigen::VectorXd> &q_init,
                                                     const Eigen::Isometry3d &x_goal,
                                                     const int ee_index,
                                                     const int max_iter,
                                                     const double eps,
                                                     const double dt)
{
    int iter = 0;
    bool success = false;
    double damp = 1e-12;
    double k = 100.0;
    Eigen::Vector6d e;
    Eigen::VectorXd v(model_.nv);
    Eigen::MatrixXd J;
    Eigen::VectorXd q = q_init;
    Eigen::Isometry3d x;

    J.resize(6, model_.nv);
    J.setZero();

    while(true)
    {
        pinocchio::framesForwardKinematics(model_, data_, q);
        pinocchio::computeFrameJacobian(model_, data_, q, ee_ids_[ee_index], pinocchio::WORLD, J);

        x.matrix() = data_.oMf[ee_ids_[ee_index]].toHomogeneousMatrix();

        e.head<3>() = x_goal.translation() - x.translation();
        e.tail<3>() = -common_math::getPhi(x.linear(), x_goal.linear());

        Eigen::Matrix6d JJt;
        JJt = J * J.transpose();
        JJt += damp*Eigen::Matrix6d::Identity();
        v = J.transpose() * JJt.ldlt().solve(k*e);

        q = q + v*dt;

        if (e.norm() < eps)
        {
            success = true;
            break;
        }
        if (iter>= max_iter)
        {
            success = false;
            break;
        }

        iter ++;
    }

    if (success)
    {
        // std::cout << "CLIK has been solved!" << std::endl;
        return q;
    }
    else
    {
        // std::cout << "Fail to solve CLIK"<<std::endl;
        return q_init;
    }
}

void DualHdrUpdater::setInitialValues()
{
    for (size_t i = 0; i < arm_names_.size(); i++)
    {
        initial_transform_[arm_names_[i]] = transform_[arm_names_[i]];
        q_init_[arm_names_[i]] = q_[arm_names_[i]];
        qd_[arm_names_[i]].setZero();
        f_ext_[arm_names_[i]].setZero();
        f_ext_ee_[arm_names_[i]].setZero();
    }
}

// void DualHdrUpdater::setTimeStamp(const double t)
// {
//     t_stamp_ = t;
// }

void DualHdrUpdater::setPosition(const Eigen::Ref<const Eigen::VectorXd> &q, std::string arm_name)
{
    q_cmd_[arm_name] = q;
}

Eigen::VectorXd DualHdrUpdater::getPosition()
{    
    Eigen::VectorXd cmd;
    cmd.resize(n_dof_);

    for (size_t i = 0; i < arm_names_.size(); i++)
    {
        const int offset = i * n_joints_;
        cmd.segment(offset, n_joints_) = q_cmd_[arm_names_[i]];
    }

    return cmd;   
    
}