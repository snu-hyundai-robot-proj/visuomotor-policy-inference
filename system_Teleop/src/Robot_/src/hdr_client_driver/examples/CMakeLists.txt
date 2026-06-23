find_package(Threads REQUIRED)

add_executable(example_get_robot_pos get_robot_pos.cpp)
target_link_libraries(example_get_robot_pos
  PRIVATE hdr_client_driver::hdr_client_driver
  PRIVATE Threads::Threads
)

add_executable(example_set_robot_pos set_robot_pos.cpp)
target_link_libraries(example_set_robot_pos
  PRIVATE hdr_client_driver::hdr_client_driver
  PRIVATE Threads::Threads
)

add_executable(example_get_versions get_versions.cpp)
target_link_libraries(example_get_versions
  PRIVATE hdr_client_driver::hdr_client_driver
  PRIVATE Threads::Threads
)

add_executable(example_get_robot_log get_robot_log.cpp)
target_link_libraries(example_get_robot_log
  PRIVATE hdr_client_driver::hdr_client_driver
  PRIVATE Threads::Threads
)

target_include_directories(example_get_robot_pos PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_include_directories(example_set_robot_pos PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_include_directories(example_get_versions PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_include_directories(example_get_robot_log PRIVATE ${CMAKE_SOURCE_DIR}/include)