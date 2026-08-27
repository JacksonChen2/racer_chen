#pragma once
#include <geometry_msgs/Pose.h>
#include <c2_explorer_msgs/msg/position_command.hpp>
namespace quadrotor_msgs {
using PositionCommand = c2_explorer_msgs::msg::PositionCommand;
using PositionCommandConstPtr = PositionCommand::ConstSharedPtr;
}
