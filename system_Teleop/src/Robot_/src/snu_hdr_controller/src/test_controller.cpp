#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "dual_hdr_controller.h"

int main(int argc, char** argv)
{
    // ROS2 초기화
    rclcpp::init(argc, argv);

    // 테스트용 노드 생성
    auto node = std::make_shared<rclcpp::Node>("dual_hdr_test_node");

    // ⚠️ 본인의 URDF 경로로 변경하세요
    std::string urdf_path = "/home/dyros/ros2_ws/src/snu_hdr_description/urdf/snu_hdr_wo.urdf";

    try
    {
        dual_hdr_controller::DualHdrController controller(urdf_path, node);

        std::cout << "Controller successfully created." << std::endl;
    }
    catch(const std::exception& e)
    {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}