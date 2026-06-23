#pragma once

#include <rclcpp/rclcpp.hpp>

#include "utils/common_math.h"
#include "robot_model/dual_hdr_updater.h"

#include <Eigen/Dense>
#include <map>

class IdleControlServer 
{
public:
    IdleControlServer(const std::string& name, const rclcpp::Node::SharedPtr& node);

    void compute(DualHdrUpdater& model);
    bool timerReset(DualHdrUpdater &model);

private:
  rclcpp::Node::SharedPtr node_;
  std::string name_;
};