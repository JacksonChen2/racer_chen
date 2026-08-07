cmake_minimum_required(VERSION 3.8)
project(racer_3d_sionna_comm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(racer_3d_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(ZLIB REQUIRED)

add_library(racer_3d_sionna_core
  src/allocation.cpp
  src/hgrid.cpp
  src/planning.cpp
  src/safety.cpp
  src/scenario.cpp
  src/serialization.cpp
  src/voxel_map.cpp
)
target_include_directories(racer_3d_sionna_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(racer_3d_sionna_core
  Eigen3::Eigen JsonCpp::JsonCpp ZLIB::ZLIB)
ament_target_dependencies(racer_3d_sionna_core sensor_msgs std_msgs)

add_executable(racer_3d_sionna_agent src/agent_node.cpp)
target_link_libraries(racer_3d_sionna_agent
  racer_3d_sionna_core JsonCpp::JsonCpp)
ament_target_dependencies(racer_3d_sionna_agent
  geometry_msgs nav_msgs racer_3d_interfaces rclcpp sensor_msgs std_msgs)

add_executable(racer_3d_communication_emulator
  src/communication_emulator_node.cpp)
ament_target_dependencies(racer_3d_communication_emulator
  racer_3d_interfaces rclcpp)

add_executable(racer_3d_sionna_mock_sim src/mock_simulator_node.cpp)
target_link_libraries(racer_3d_sionna_mock_sim
  racer_3d_sionna_core JsonCpp::JsonCpp)
ament_target_dependencies(racer_3d_sionna_mock_sim
  geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs)

add_executable(racer_3d_sionna_monitor src/monitor_node.cpp)
target_link_libraries(racer_3d_sionna_monitor
  racer_3d_sionna_core JsonCpp::JsonCpp)
ament_target_dependencies(racer_3d_sionna_monitor
  nav_msgs racer_3d_interfaces rclcpp std_msgs)

install(TARGETS
  racer_3d_sionna_core
  racer_3d_sionna_agent
  racer_3d_communication_emulator
  racer_3d_sionna_mock_sim
  racer_3d_sionna_monitor
  EXPORT export_racer_3d_sionna_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch isaac_sim scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets")
  install(DIRECTORY assets DESTINATION share/${PROJECT_NAME})
endif()
install(FILES README.md requirements-sionna.txt
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/sionna_channel_node
  scripts/sionna_channel_node.py
  scripts/generate_hybrid_radio_cache.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_voxel_map test/test_voxel_map.cpp)
  target_link_libraries(test_voxel_map racer_3d_sionna_core)
  ament_add_gtest(test_planning test/test_planning.cpp)
  target_link_libraries(test_planning racer_3d_sionna_core)
  ament_add_gtest(test_allocation_safety test/test_allocation_safety.cpp)
  target_link_libraries(test_allocation_safety racer_3d_sionna_core)
  ament_add_gtest(test_communication_serialization
    test/test_communication_serialization.cpp)
  target_link_libraries(test_communication_serialization
    racer_3d_sionna_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(racer_3d_sionna_core)
ament_export_dependencies(Eigen3 racer_3d_interfaces)
ament_package()
