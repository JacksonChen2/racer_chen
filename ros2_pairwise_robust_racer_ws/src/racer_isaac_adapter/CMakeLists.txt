cmake_minimum_required(VERSION 3.16)
project(racer_isaac_adapter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(racer_fidelity_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(position_command_adapter src/position_command_adapter.cpp)
ament_target_dependencies(position_command_adapter
  geometry_msgs nav_msgs racer_fidelity_msgs rclcpp std_msgs)

add_executable(exploration_trigger src/exploration_trigger.cpp)
ament_target_dependencies(exploration_trigger
  geometry_msgs nav_msgs rclcpp sensor_msgs)

install(TARGETS position_command_adapter exploration_trigger
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config scripts isaac_sim
  DESTINATION share/${PROJECT_NAME})

ament_package()
