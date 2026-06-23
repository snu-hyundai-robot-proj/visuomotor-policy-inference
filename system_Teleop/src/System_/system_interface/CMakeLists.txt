cmake_minimum_required(VERSION 3.8)
project(system_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# file(GLOB MSG_FILES "msg/*.msg")
# file(GLOB SRV_FILES "msg/*.msg")
# file(GLOB MSG_FILES "msg/*.msg")

set(MSG_FILES 
"msg/PoseData.msg" 
"msg/FrameAlignedState.msg"
"msg/UiCommand.msg"
"msg/StartRecording.msg" 
"msg/InspireSensor.msg")

set(SRV_FILES 
"srv/RobotCalibration.srv" 
"srv/StartStopRecording.srv"
"srv/DetectObject.srv")
# message(STATUS "MSG_FILES=${MSG_FILES}")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
)

# message(STATUS "SRV_FILES=${SRV_FILES}")
# message(STATUS "ACTION_FILES=${ACTION_FILES}")

ament_export_dependencies(rosidl_default_runtime)

ament_package()