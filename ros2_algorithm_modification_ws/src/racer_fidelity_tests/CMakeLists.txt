cmake_minimum_required(VERSION 3.8)
project(racer_fidelity_tests)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)

get_filename_component(REPOSITORY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(BASELINE_ROOT
  "${REPOSITORY_ROOT}/original_racer_ros1_ws/src/RACER/swarm_exploration")
set(PORT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../racer_original_core/upstream")
set(COMPAT_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/../racer_original_core/include")

add_executable(baseline_bspline_fixture
  test/bspline_fixture.cpp
  "${BASELINE_ROOT}/bspline/src/non_uniform_bspline.cpp")
target_include_directories(baseline_bspline_fixture PRIVATE
  "${BASELINE_ROOT}/bspline/include" "${COMPAT_INCLUDE}" ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(baseline_bspline_fixture rclcpp)

add_executable(ported_bspline_fixture
  test/bspline_fixture.cpp
  "${PORT_ROOT}/bspline/src/non_uniform_bspline.cpp")
target_include_directories(ported_bspline_fixture PRIVATE
  "${PORT_ROOT}/bspline/include" "${COMPAT_INCLUDE}" ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(ported_bspline_fixture rclcpp)

add_executable(degenerate_hover_fixture
  test/degenerate_hover_fixture.cpp
  "${PORT_ROOT}/poly_traj/src/polynomial_traj.cpp")
target_include_directories(degenerate_hover_fixture PRIVATE
  "${PORT_ROOT}/poly_traj/include" ${EIGEN3_INCLUDE_DIRS})

file(GLOB BASELINE_LKH_SOURCES "${BASELINE_ROOT}/utils/lkh_mtsp_solver/src/*.c")
file(GLOB PORTED_LKH_SOURCES "${PORT_ROOT}/utils/lkh_mtsp_solver/src/*.c")
add_executable(baseline_lkh_fixture
  "${BASELINE_ROOT}/utils/lkh_mtsp_solver/LKHmain.c" ${BASELINE_LKH_SOURCES})
target_include_directories(baseline_lkh_fixture PRIVATE
  "${BASELINE_ROOT}/utils/lkh_mtsp_solver/include")
target_compile_options(baseline_lkh_fixture PRIVATE -fcommon)
target_link_libraries(baseline_lkh_fixture m)
add_executable(ported_lkh_fixture
  "${PORT_ROOT}/utils/lkh_mtsp_solver/LKHmain.c" ${PORTED_LKH_SOURCES})
target_include_directories(ported_lkh_fixture PRIVATE
  "${PORT_ROOT}/utils/lkh_mtsp_solver/include")
target_compile_options(ported_lkh_fixture PRIVATE -fcommon)
target_link_libraries(ported_lkh_fixture m)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME racer_source_fidelity
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/source_fidelity.py"
      "${BASELINE_ROOT}" "${PORT_ROOT}")
  add_test(NAME racer_bspline_same_input
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/compare_outputs.py"
      "$<TARGET_FILE:baseline_bspline_fixture>" "$<TARGET_FILE:ported_bspline_fixture>")
  add_test(NAME racer_lkh_same_input
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/compare_lkh.py"
      "$<TARGET_FILE:baseline_lkh_fixture>" "$<TARGET_FILE:ported_lkh_fixture>")
  add_test(NAME racer_safety_layer_isolation
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/safety_isolation.py"
      "${CMAKE_CURRENT_SOURCE_DIR}/../racer_isaac_adapter/isaac_sim")
  add_test(NAME racer_wire_schema_fidelity
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/wire_schema_fidelity.py"
      "${REPOSITORY_ROOT}/original_racer_ros1_ws/src/RACER"
      "${CMAKE_CURRENT_SOURCE_DIR}/../racer_fidelity_msgs")
  add_test(NAME racer_callback_serialization
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/callback_serialization.py"
      "${COMPAT_INCLUDE}/ros/ros.h")
  add_test(NAME racer_degenerate_hover_regression
    COMMAND "$<TARGET_FILE:degenerate_hover_fixture>")
  add_test(NAME racer_trigger_readiness
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/trigger_readiness.py"
      "${CMAKE_CURRENT_SOURCE_DIR}/../racer_isaac_adapter/src/exploration_trigger.cpp")
  add_test(NAME racer_isaac_graph_readiness
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/isaac_graph_readiness.py"
      "${CMAKE_CURRENT_SOURCE_DIR}/../racer_isaac_adapter/isaac_sim/original_racer_isaac.py")
  add_test(NAME racer_completion_monitor
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/completion_monitor.py"
      "${CMAKE_CURRENT_SOURCE_DIR}/../racer_isaac_adapter/scripts/monitor_completion.py")
endif()

install(TARGETS baseline_bspline_fixture ported_bspline_fixture
  degenerate_hover_fixture baseline_lkh_fixture ported_lkh_fixture
  RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_package()
