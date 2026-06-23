#include <memory>
#include <string>
#include <vector>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "net_ft_driver/interfaces/net_ft_interface.hpp"

std::atomic<bool> flag_(false);

// bias reset callback
void setBias(const std_msgs::msg::Bool &msg)
{
    flag_.store(msg.data);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dual_ft_reader");

    // Parameters for 2 sensors
    node->declare_parameter<std::string>("sensor_type1", "ati_axia");
    node->declare_parameter<std::string>("ip1", "192.168.4.21");
    node->declare_parameter<int>("sampling_rate1", 500);

    node->declare_parameter<std::string>("sensor_type2", "ati_axia");
    node->declare_parameter<std::string>("ip2", "192.168.4.22");
    node->declare_parameter<int>("sampling_rate2", 500);

    node->declare_parameter<int>("internal_filter", 4);
    node->declare_parameter<std::string>("topic", "/ft_combined");

    const auto sensor_type1 = node->get_parameter("sensor_type1").as_string();
    const auto ip1          = node->get_parameter("ip1").as_string();
    const auto sampling_rate1 = node->get_parameter("sampling_rate1").as_int();

    const auto sensor_type2 = node->get_parameter("sensor_type2").as_string();
    const auto ip2          = node->get_parameter("ip2").as_string();
    const auto sampling_rate2 = node->get_parameter("sampling_rate2").as_int();

    const auto internal_filter = node->get_parameter("internal_filter").as_int();
    const auto topic = node->get_parameter("topic").as_string();
    const auto zero_topic = std::string("/set_zero") + topic;

    // Publisher for combined 12-d vector
    auto pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        topic, rclcpp::SensorDataQoS());

    auto sub = node->create_subscription<std_msgs::msg::Bool>(
        zero_topic, 10, setBias);

    RCLCPP_INFO(node->get_logger(),
        "Config sensors: %s @ %s, %s @ %s, internal_filter=%d",
        sensor_type1.c_str(), ip1.c_str(),
        sensor_type2.c_str(), ip2.c_str(),
        internal_filter);

    // Create driver instances
    auto driver1 = net_ft_driver::NetFTInterface::create(sensor_type1, ip1);
    auto driver2 = net_ft_driver::NetFTInterface::create(sensor_type2, ip2);

    if (!driver1 || !driver2) {
        RCLCPP_ERROR(node->get_logger(), "Failed to create one or both drivers");
        return 1;
    }

    driver1->set_sampling_rate(sampling_rate1);
    driver2->set_sampling_rate(sampling_rate2);
    driver1->set_internal_filter(internal_filter);
    driver2->set_internal_filter(internal_filter);

    driver1->set_bias();
    driver2->set_bias();

    if (!driver1->start_streaming() || !driver2->start_streaming()) {
        RCLCPP_ERROR(node->get_logger(), "start_streaming() failed on one or both sensors");
        return 1;
    }

    rclcpp::on_shutdown([&](){
        RCLCPP_INFO(node->get_logger(),"Stopping FT streaming");
        driver1->stop_streaming();
        driver2->stop_streaming();
    });

    // Timer period based on sampling_rate1 (assuming both same)
    auto period = std::chrono::microseconds(1000000 / sampling_rate1);

    auto timer = node->create_wall_timer(
        period,
        [&, node, pub]() {
            auto data1 = driver1->receive_data();
            auto data2 = driver2->receive_data();

            if (!data1 || !data2) return;

            if (flag_.load()) {
                driver1->set_bias();
                driver2->set_bias();
                flag_.store(false);
            }

            const auto& v1 = data1->ft_values; // 6-d
            const auto& v2 = data2->ft_values; // 6-d

            std_msgs::msg::Float64MultiArray msg;
            msg.data.resize(12);  // 6 + 6
            for (size_t i = 0; i < 6; ++i) {
                msg.data[i] = v1[i];
                msg.data[i + 6] = v2[i];
            }

            pub->publish(msg);
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}