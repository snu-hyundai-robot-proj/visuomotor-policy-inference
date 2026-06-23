#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose.hpp>

#include "action_server_base.h"
#include "snu_hdr_msgs/action/ee_move.hpp"

#include "utils/common_math.h"
#include "robot_model/dual_hdr_updater.h"

template <typename ActionT>
class EEMoveActionServer : public ActionServerBase<ActionT>
{
public:
    using Base = ActionServerBase<ActionT>;
    using GoalHandle = typename Base::GoalHandle; 
    using Goal = typename Base::Goal;
    using Result = typename Base::Result;

    EEMoveActionServer(const std::string &name, const rclcpp::Node::SharedPtr &node);
    
    bool compute(DualHdrUpdater &mu) override;    
    bool timerReset() override;

protected:
    // ActionServerBase 인터페이스
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Goal> goal) override;
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle) override;

    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) override;

private:
    std::shared_ptr<const Goal> goal_{nullptr};

    std::map<std::string, bool> active_arms_;
    std::map<std::string, Eigen::Isometry3d> x_ee_init_, x_ee_d_;
    double duration_;


};