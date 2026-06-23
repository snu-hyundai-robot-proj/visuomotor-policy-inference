cmake_minimum_required(VERSION 3.8)
project(system_right)


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
find_package(OpenCV REQUIRED)

add_executable(system_right src/system_right.cpp
)

ament_target_dependencies(system_right
  rclcpp
  system_interface
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  system_right
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
#   system_right
#   system_recorder
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
