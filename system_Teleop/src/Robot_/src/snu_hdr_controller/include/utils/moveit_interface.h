#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>


class MoveItInterface
{
public:
    MoveItInterface(const rclcpp::Node::SharedPtr& node,
                    const std::string& topic_name = "/joint_states");

    void publishJointState(const Eigen::VectorXd& q);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    std::vector<std::string> joint_names_;
};