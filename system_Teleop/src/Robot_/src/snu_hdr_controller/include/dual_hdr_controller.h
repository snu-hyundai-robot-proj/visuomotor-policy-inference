#pragma once

#include <thread>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include "robot_model/dual_hdr_updater.h"
#include "utils/moveit_interface.h"
#include "utils/common_math.h"
#include "utils/workspace_limit.h"
#include "utils/ft_sensor.h"

#include "servers/action_server_base.h"
#include "servers/idle_action_server.h"
#include "servers/common_action_interface.h"
#include "servers/joint_move_action_server.h"
#include "servers/moveit_action_server.h"
#include "servers/admittance_action_server.h"

#include <fstream>

using namespace common_math;
using namespace workspace_limit;

namespace dual_hdr_controller
{

  class DualHdrController
  {
  public:
    DualHdrController(const std::string urdf_path, const rclcpp::Node::SharedPtr &node);

    void initialize(const std::string urdf_path);
    void registerServers(const rclcpp::Node::SharedPtr &node);

    // observe the current values
    bool update(const Eigen::VectorXd &q,
                const Eigen::VectorXd &qd);

    void validate();
    Eigen::VectorXd write();

    bool getTimerReset();

    // **FT sensor node getter**
    rclcpp::Node::SharedPtr getFtNode() const { return ft_node_; }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Node::SharedPtr ft_node_;

    std::vector<std::shared_ptr<CommonActionInterface>> action_servers_;
    std::shared_ptr<IdleControlServer> idle_control_server_;
    std::shared_ptr<MoveitActionServer<moveit_msgs::action::ExecuteTrajectory>> moveit_action_server_;
    std::shared_ptr<JointMoveActionServer<snu_hdr_msgs::action::JointMove>> joint_move_action_server_;
    std::shared_ptr<AdmittanceActionServer<snu_hdr_msgs::action::Admittance>> admittance_action_server_;

    std::string urdf_path_;
    DualHdrUpdater model_;

    std::shared_ptr<MoveItInterface> moveit_interface_;

    std::shared_ptr<FTSensorReader> ft_sensor_;

    bool timer_reset_{false};
  };
}