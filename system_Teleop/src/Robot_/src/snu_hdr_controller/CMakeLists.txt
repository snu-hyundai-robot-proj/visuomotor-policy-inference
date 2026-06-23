cmake_minimum_required(VERSION 3.8)
project(snu_hdr_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF)
  add_compile_options(-O3 -march=native -mtune=native -ffast-math -DNDEBUG)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)
find_package(snu_hdr_msgs REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${eigenpy_INCLUDE_DIRS}    
)

add_library(${PROJECT_NAME} SHARED
    src/robot_model/dual_hdr_updater.cpp           
    src/utils/common_math.cpp    
    src/utils/moveit_interface.cpp     
    src/utils/workspace_limit.cpp     
    src/utils/admittance.cpp
    src/utils/ft_sensor.cpp
    
    src/servers/idle_action_server.cpp
    src/servers/joint_move_action_server.cpp
    src/servers/moveit_action_server.cpp
    src/servers/admittance_action_server.cpp

    src/dual_hdr_controller.cpp
    
)

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wno-unused-variable
  -Wno-array-bounds
  -Wno-stringop-overread
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  rclcpp_action 
  sensor_msgs 
  geometry_msgs
  trajectory_msgs
  moveit_msgs
  snu_hdr_msgs
  Eigen3
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${eigenpy_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} 
    Eigen3::Eigen
    rclcpp::rclcpp            
)


pybind11_add_module(${PROJECT_NAME}_py
    src/utils/bindings.cpp
)

target_link_libraries(${PROJECT_NAME}_py
    PRIVATE
    ${PROJECT_NAME}
    Eigen3::Eigen
    pinocchio::pinocchio

)


# Install the include headers
install(DIRECTORY include/ DESTINATION include/)

# Install the wrapper library into the Python site-packages directory
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
)

install(TARGETS ${PROJECT_NAME}_py
    LIBRARY DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/shc_python
)

install(
  DIRECTORY shc_python/
  DESTINATION lib/python3.12/site-packages/shc_python
)

# Export package dependencies for ROS 2
# ament_export_dependencies(ament_cmake Eigen3 Boost eigenpy ament_cmake_python)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()