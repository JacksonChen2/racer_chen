#pragma once
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/msg/pose_array.hpp>
namespace geometry_msgs {
using PoseArray = msg::PoseArray;
using PoseArrayConstPtr = PoseArray::ConstSharedPtr;
}
