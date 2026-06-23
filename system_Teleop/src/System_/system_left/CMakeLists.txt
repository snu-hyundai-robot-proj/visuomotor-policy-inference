cmake_minimum_required(VERSION 3.8)
project(system_left)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(system_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(system_left src/system_left.cpp
)

ament_target_dependencies(system_left
  rclcpp
  system_interface
  sensor_msgs
  geometry_msgs
  control_msgs
)

install(TARGETS
  system_left
  DESTINATION lib/${PROJECT_NAME}
)

# add_executable(system_recorder
#   src/system_recorder.cpp
# )
# ament_target_dependencies(system_recorder
#   rclcpp
#   system_interface
# )
# target_link_libraries(system_recorder
#   ${OpenCV_LIBS}
# )

# install(TARGETS
#   system_left
#   system_recorder
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
