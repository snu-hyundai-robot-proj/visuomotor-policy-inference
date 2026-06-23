#include "utils/ft_sensor.h"

FTSensorReader::FTSensorReader(const rclcpp::Node::SharedPtr &node, const std::string &topic)
    : node_(node), ft_(Eigen::VectorXd::Zero(12)) // 12차원 초기화
{
    sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        topic, rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            this->callback(msg);
        });

    // rotation matrix of sensor frame w.r.t robot base
    R_bs_ << 0, -1,  0,
             1,  0,  0,
             0,  0, -1;
}

Eigen::VectorXd FTSensorReader::getFT() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::VectorXd ft_base(ft_.size());  // 12차원
    constexpr int dof = 6;                // 각 센서 DOF

    // 센서가 2개라 가정
    for (int i = 0; i < ft_.size() / dof; ++i)
    {
        // i번째 센서 벡터
        Eigen::VectorXd ft_sensor = ft_.segment(i * dof, dof);

        // 변환 적용
        Eigen::VectorXd ft_base_sensor(dof);
        ft_base_sensor.head<3>() = R_bs_ * ft_sensor.head<3>();
        ft_base_sensor.tail<3>() = R_bs_ * ft_sensor.tail<3>();

        // 결과를 ft_base에 넣기
        ft_base.segment(i * dof, dof) = ft_base_sensor;
    }

    return ft_base;
}

void FTSensorReader::callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->data.size() != 12)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Received FT vector size != 12 (%zu)", msg->data.size());
        return;
    }

    for (size_t i = 0; i < 12; ++i)
    {
        ft_(i) = msg->data[i];
    }
}