#include <memory>
#include <string>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

#include "net_ft_driver/interfaces/net_ft_interface.hpp"

std::atomic<bool> flag_(false);

void setBias(const std_msgs::msg::Bool &msg)
{
  flag_.store(msg.data);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("net_ft_reader");

  node->declare_parameter<std::string>("sensor_type", "ati_axia");
  node->declare_parameter<std::string>("ip", "192.168.4.21");
  node->declare_parameter<int>("sampling_rate", 500);
  node->declare_parameter<int>("internal_filter", 4);
  node->declare_parameter<std::string>("topic", "/ft");

  const auto sensor_type   = node->get_parameter("sensor_type").as_string();
  const auto ip            = node->get_parameter("ip").as_string();
  const auto sampling_rate = node->get_parameter("sampling_rate").as_int();
  const auto internal_filter = node->get_parameter("internal_filter").as_int();
  const auto topic = node->get_parameter("topic").as_string();

  const auto zero_topic = std::string("/set_zero") + topic;

  auto pub = node->create_publisher<geometry_msgs::msg::WrenchStamped>(
      topic, rclcpp::SensorDataQoS());

  auto sub = node->create_subscription<std_msgs::msg::Bool>(
      zero_topic, 10, setBias);

  RCLCPP_INFO(node->get_logger(),
              "Config: sensor_type=%s ip=%s sampling_rate=%d internal_filter=%d",
              sensor_type.c_str(), ip.c_str(), sampling_rate, internal_filter);

  auto driver = net_ft_driver::NetFTInterface::create(sensor_type, ip);
  if (!driver) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create driver");
    return 1;
  }

  driver->set_sampling_rate(sampling_rate);
  driver->set_internal_filter(internal_filter);

  driver->set_bias();

  if (!driver->start_streaming()) {
    RCLCPP_ERROR(node->get_logger(), "start_streaming() failed");
    return 1;
  }

  /* shutdown handler */
  rclcpp::on_shutdown([&](){
      RCLCPP_INFO(node->get_logger(),"Stopping FT streaming");
      driver->stop_streaming();
  });

  /* timer period (sampling_rate 기반) */
  auto period = std::chrono::microseconds(1000000 / sampling_rate);

  auto timer = node->create_wall_timer(
      period,
      [&, node, pub]() {

        auto data = driver->receive_data();
        if (!data) return;

        if (flag_.load()) {
          driver->set_bias();
          flag_.store(false);
        }

        const auto& v = data->ft_values;

        geometry_msgs::msg::WrenchStamped msg;
        msg.header.stamp = node->now();
        msg.header.frame_id = "ft_sensor";

        msg.wrench.force.x  = v[0];
        msg.wrench.force.y  = v[1];
        msg.wrench.force.z  = v[2];
        msg.wrench.torque.x = v[3];
        msg.wrench.torque.y = v[4];
        msg.wrench.torque.z = v[5];

        pub->publish(msg);
      });

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}