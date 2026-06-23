#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

class DualHdrUpdater;  // forward declaration

class CommonActionInterface
{
public:
  virtual ~CommonActionInterface() = default;
  virtual bool compute(DualHdrUpdater & model) = 0;
  virtual bool timerReset() = 0;

};