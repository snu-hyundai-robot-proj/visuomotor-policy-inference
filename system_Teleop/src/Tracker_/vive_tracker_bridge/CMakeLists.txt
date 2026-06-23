cmake_minimum_required(VERSION 3.8)
project(vive_tracker_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(system_interface REQUIRED)

add_executable(tracker_bridge_node
  src/vive_tracker_bridge_node.cpp
)

add_library(${PROJECT_NAME}_headers INTERFACE)

target_include_directories(${PROJECT_NAME}_headers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(tracker_bridge_node 
rclcpp 
std_msgs 
geometry_msgs 
tf2_ros
system_interface
)

target_link_libraries(tracker_bridge_node
  ${PROJECT_NAME}_headers
)

install(TARGETS tracker_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_headers
  EXPORT ${PROJECT_NAME}Targets
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
