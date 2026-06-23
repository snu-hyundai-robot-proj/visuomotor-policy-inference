cmake_minimum_required(VERSION 3.8)
project(hdr_client_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(WITH_ASAN "Compile with AddressSanitizer" OFF)

find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

if(DEFINED AMENT_PACKAGE_NAME)
  find_package(ament_cmake REQUIRED)
endif()

file(GLOB_RECURSE HDR_CLIENT_DRIVER_SRC
  src/hdr_client_driver.cpp
  src/request_lib.cpp
  src/socket_manager.cpp

  src/functions/version.cpp
  src/functions/project.cpp
  src/functions/control.cpp
  src/functions/robot.cpp
  src/functions/io_plc.cpp
  src/functions/etc.cpp
  src/functions/file_manager.cpp
  src/functions/task.cpp
  src/functions/console.cpp
  src/functions/hardware_interface.cpp
)

add_library(hdr_client_driver SHARED ${HDR_CLIENT_DRIVER_SRC})
add_library(${PROJECT_NAME}::hdr_client_driver ALIAS hdr_client_driver)

target_link_libraries(hdr_client_driver PUBLIC 
  ${CURL_LIBRARIES}
)

target_include_directories(hdr_client_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(hdr_client_driver PUBLIC cxx_std_17)
target_compile_options(hdr_client_driver PRIVATE -Wall -Wextra -Wno-unused-parameter)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")

if(WITH_ASAN)
  target_compile_options(hdr_client_driver PUBLIC -fsanitize=address)
  target_link_options(hdr_client_driver PUBLIC -fsanitize=address)
endif()

add_subdirectory(examples)

include(GNUInstallDirs)

install(TARGETS hdr_client_driver EXPORT hdr_client_driverTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT hdr_client_driverTargets
  NAMESPACE hdr_client_driver::
  DESTINATION lib/cmake/hdr_client_driver
)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/hdr_client_driverConfig.cmake
"include(\"\${CMAKE_CURRENT_LIST_DIR}/hdr_client_driverTargets.cmake\")")

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/hdr_client_driverConfig.cmake
  DESTINATION lib/cmake/hdr_client_driver
)

if(DEFINED AMENT_PACKAGE_NAME)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME} "")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/share/ament_index/resource_index/packages/${PROJECT_NAME}
    DESTINATION share/ament_index/resource_index/packages)

  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv "prepend-non-duplicate;AMENT_PREFIX_PATH;")
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ros_package_path.dsv "prepend-non-duplicate;ROS_PACKAGE_PATH;")

  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ament_prefix_path.dsv
    ${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/hook/ros_package_path.dsv
    DESTINATION share/${PROJECT_NAME}/hook
  )

  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    DESTINATION share/${PROJECT_NAME}
  )
endif()
