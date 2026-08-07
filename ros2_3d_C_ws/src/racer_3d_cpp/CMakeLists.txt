cmake_minimum_required(VERSION 3.8)
project(racer_3d_cpp)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(ZLIB REQUIRED)

add_library(racer_3d_core
  src/allocation.cpp
  src/hgrid.cpp
  src/planning.cpp
  src/safety.cpp
  src/scenario.cpp
  src/serialization.cpp
  src/voxel_map.cpp
)
target_include_directories(racer_3d_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(racer_3d_core Eigen3::Eigen JsonCpp::JsonCpp ZLIB::ZLIB)
ament_target_dependencies(racer_3d_core sensor_msgs std_msgs)

add_executable(racer_3d_cpp_agent src/agent_node.cpp)
target_link_libraries(racer_3d_cpp_agent racer_3d_core JsonCpp::JsonCpp)
ament_target_dependencies(racer_3d_cpp_agent
  geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs)

add_executable(racer_3d_cpp_mock_sim src/mock_simulator_node.cpp)
target_link_libraries(racer_3d_cpp_mock_sim racer_3d_core JsonCpp::JsonCpp)
ament_target_dependencies(racer_3d_cpp_mock_sim
  geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs)

add_executable(racer_3d_cpp_monitor src/monitor_node.cpp)
target_link_libraries(racer_3d_cpp_monitor racer_3d_core JsonCpp::JsonCpp)
ament_target_dependencies(racer_3d_cpp_monitor nav_msgs rclcpp std_msgs)

install(TARGETS racer_3d_core racer_3d_cpp_agent racer_3d_cpp_mock_sim
  racer_3d_cpp_monitor
  EXPORT export_racer_3d_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config launch isaac_sim scripts test_results
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_voxel_map test/test_voxel_map.cpp)
  target_link_libraries(test_voxel_map racer_3d_core)
  ament_add_gtest(test_planning test/test_planning.cpp)
  target_link_libraries(test_planning racer_3d_core)
  ament_add_gtest(test_allocation_safety test/test_allocation_safety.cpp)
  target_link_libraries(test_allocation_safety racer_3d_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(racer_3d_core)
ament_export_dependencies(Eigen3)
ament_package()
