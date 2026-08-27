#include <ros/ros.h>

namespace ros::detail {

std::mutex &globalMutex() {
  static std::mutex value;
  return value;
}

std::shared_ptr<rclcpp::Node> &globalNode() {
  static std::shared_ptr<rclcpp::Node> value;
  return value;
}

std::string &requestedNodeName() {
  static std::string value{"c2_explorer"};
  return value;
}

std::atomic<unsigned long> &serviceClientIndex() {
  static std::atomic<unsigned long> value{0};
  return value;
}

}  // namespace ros::detail
