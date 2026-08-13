cmake_minimum_required(VERSION 3.8)
project(racer_sionna_comm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(racer_fidelity_msgs REQUIRED)
find_package(racer_sionna_interfaces REQUIRED)

add_library(racer_sionna_link_model src/link_model.cpp)
target_include_directories(racer_sionna_link_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(racer_sionna_communication_proxy
  src/communication_proxy_node.cpp)
target_link_libraries(racer_sionna_communication_proxy racer_sionna_link_model)
target_include_directories(racer_sionna_communication_proxy PRIVATE include)
ament_target_dependencies(racer_sionna_communication_proxy
  rclcpp racer_fidelity_msgs racer_sionna_interfaces)

install(TARGETS
  racer_sionna_link_model
  racer_sionna_communication_proxy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config assets DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/sionna_channel_node.py
  scripts/convert_usd_to_sionna.py
  scripts/generate_hybrid_radio_cache.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(racer_sionna_link_model_test test/test_link_model.cpp)
  target_link_libraries(racer_sionna_link_model_test racer_sionna_link_model)
  target_include_directories(racer_sionna_link_model_test PRIVATE include)
  ament_add_pytest_test(racer_sionna_ideal_proxy_test
    test/test_ideal_proxy.py TIMEOUT 40)
endif()

ament_export_include_directories(include)
ament_export_libraries(racer_sionna_link_model)
ament_package()
