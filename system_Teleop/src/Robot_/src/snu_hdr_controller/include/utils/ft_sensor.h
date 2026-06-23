#pragma once

#include <vector>
#include <string>
#include <memory>
#include <mutex>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class FTSensorReader
{
public:
    FTSensorReader(const rclcpp::Node::SharedPtr &node, const std::string &topic);

    // 최신 FT 벡터 가져오기 (12차원)
    Eigen::VectorXd getFT() const;

private:
    void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    mutable std::mutex mutex_;
    Eigen::VectorXd ft_;
    Eigen::Matrix3d R_bs_;
};