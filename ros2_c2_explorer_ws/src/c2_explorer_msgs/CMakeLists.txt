cmake_minimum_required(VERSION 3.8)
project(c2_explorer_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bspline.msg"
  "msg/ChunkData.msg"
  "msg/ChunkStamps.msg"
  "msg/CommStatistics.msg"
  "msg/DroneState.msg"
  "msg/GridTour.msg"
  "msg/HGrid.msg"
  "msg/Heartbeat.msg"
  "msg/IdxList.msg"
  "msg/LinkQuality.msg"
  "msg/LinkQualityArray.msg"
  "msg/MeetingOpt.msg"
  "msg/MeetingOptResponse.msg"
  "msg/PositionCommand.msg"
  "msg/Rendezvous.msg"
  "srv/SolveMTSP.srv"
  "srv/SolveTSP.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
