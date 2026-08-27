cmake_minimum_required(VERSION 3.16)
project(c2_explorer_isaac LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(c2_explorer_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(position_command_adapter src/position_command_adapter.cpp)
ament_target_dependencies(position_command_adapter
  geometry_msgs nav_msgs c2_explorer_msgs rclcpp std_msgs)

add_executable(exploration_trigger src/exploration_trigger.cpp)
ament_target_dependencies(exploration_trigger
  geometry_msgs nav_msgs rclcpp sensor_msgs)

add_executable(c2_communication_proxy src/c2_communication_proxy.cpp)
ament_target_dependencies(c2_communication_proxy
  c2_explorer_msgs rclcpp)

install(TARGETS position_command_adapter exploration_trigger
  c2_communication_proxy
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/monitor_completion.py
  scripts/c2_sionna_channel.py
  scripts/run_c2_warehouse.sh
  scripts/run_c2_warehouse_visualization.sh
  DESTINATION share/${PROJECT_NAME}/scripts)
install(PROGRAMS
  scripts/c2_sionna_channel.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES
  isaac_sim/c2_explorer_isaac.py
  isaac_sim/crazyflie_cpp_bridge.py
  isaac_sim/pointcloud_cpp_bridge.py
  isaac_sim/racer_so3_cpp_bridge.py
  isaac_sim/safety_cpp_bridge.py
  isaac_sim/scenario_cpp_bridge.py
  DESTINATION share/${PROJECT_NAME}/isaac_sim)

ament_package()
