#include "utils/moveit_interface.h"

MoveItInterface::MoveItInterface(const rclcpp::Node::SharedPtr& node, const std::string& topic_name)
: node_(node)
{
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();

    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(topic_name, qos);

    joint_names_ = {
        // left arm
        "dg5f_j1","dg5f_j2","dg5f_j3",
        "dg5f_j4","dg5f_j5","dg5f_j6",

        // right arm
        "rh56_j1","rh56_j2","rh56_j3",
        "rh56_j4","rh56_j5","rh56_j6"
    };

    RCLCPP_INFO(node_->get_logger(),"MoveItInterface publishing to %s",topic_name.c_str());
}

void MoveItInterface::publishJointState(const Eigen::VectorXd& q)
{
    if (q.size() != static_cast<int>(joint_names_.size()))
    {
        RCLCPP_ERROR(node_->get_logger(), "Joint size mismatch: expected %ld, got %ld",
                     joint_names_.size(), q.size());
        return;
    }

    sensor_msgs::msg::JointState msg;
    // msg.header.stamp = node_->now();
    msg.header.stamp = node_->get_clock()->now();
    msg.name = joint_names_;

    msg.position.resize(q.size());
    for (size_t i = 0; i < q.size(); ++i)
        msg.position[i] = q[i];

    joint_state_pub_->publish(msg);
}