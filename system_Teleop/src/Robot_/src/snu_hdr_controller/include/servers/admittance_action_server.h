#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>


#include "action_server_base.h"
#include "utils/common_math.h"
#include "utils/admittance.h"

#include "robot_model/dual_hdr_updater.h"

#include "snu_hdr_msgs/action/admittance.hpp"

template <typename ActionT>
class AdmittanceActionServer : public ActionServerBase<ActionT>
{
public:
    using Base = ActionServerBase<ActionT>;
    using GoalHandle = typename Base::GoalHandle; 
    using Goal = typename Base::Goal;
    using Result = typename Base::Result;

    AdmittanceActionServer(const std::string &name, const rclcpp::Node::SharedPtr &node);
    
    bool compute(DualHdrUpdater &mu) override;
    bool timerReset() override;

protected:
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Goal> goal) override;
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle) override;

    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) override;

private:
    std::shared_ptr<const Goal> goal_{nullptr};

    std::map<std::string, AdmittanceController> admittance_;

    std::map<std::string, bool> active_arms_;
    std::map<std::string, Eigen::Vector6d> q_init_;
    std::map<std::string, Eigen::Vector6d> q_admit_;
    std::map<std::string, Eigen::Isometry3d> x_ee_d_;

    rclcpp::Time time_;

    std::ofstream debug_ft{"/home/dyros/ros2_ws/src/snu_hdr_controller/log/f_ext.txt"};
    std::ofstream debug_q{"/home/dyros/ros2_ws/src/snu_hdr_controller/log/q.txt"};
    std::ofstream debug_qd{"/home/dyros/ros2_ws/src/snu_hdr_controller/log/qd.txt"};

};