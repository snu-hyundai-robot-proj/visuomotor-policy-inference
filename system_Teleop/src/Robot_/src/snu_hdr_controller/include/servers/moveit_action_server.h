#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>

#include "action_server_base.h"
#include "utils/common_math.h"
#include "robot_model/dual_hdr_updater.h"

template <typename ActionT>
class MoveitActionServer : public ActionServerBase<ActionT>
{
public:
    using Base = ActionServerBase<ActionT>;
    using GoalHandle = typename Base::GoalHandle; 
    using Goal = typename Base::Goal;
    using Result = typename Base::Result;

    MoveitActionServer(const std::string &name, const rclcpp::Node::SharedPtr &node);
    
    bool compute(DualHdrUpdater &mu) override;
    bool timerReset() override;

protected:
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Goal> goal) override;
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle) override;

    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle) override;

private:
    std::shared_ptr<const Goal> goal_{nullptr};

    void parseTrajectory(const moveit_msgs::msg::RobotTrajectory &traj);

    Eigen::VectorXd q_;
    Eigen::VectorXd qd_;

    std::vector<Eigen::VectorXd> q_traj_;
    int traj_index_;
    std::vector<double> time_from_start_;

    rclcpp::Time start_time_;    
    rclcpp::Time traj_start_time_;        

    std::ofstream debug_q_traj{"debug_q_traj.txt"};

};