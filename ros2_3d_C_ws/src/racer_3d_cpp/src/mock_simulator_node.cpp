#include "racer_3d_cpp/scenario.hpp"
#include "racer_3d_cpp/safety.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <json/json.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace racer_3d_cpp {
namespace {

sensor_msgs::msg::PointCloud2 makeCloud(
    const rclcpp::Time &stamp, const std::vector<Point3> &points,
    const std::vector<bool> &hits) {
  sensor_msgs::msg::PointCloud2 message;
  message.header.stamp = stamp;
  message.header.frame_id = "map";
  message.height = 1U;
  message.width = static_cast<std::uint32_t>(points.size());
  message.is_bigendian = false;
  message.is_dense = true;
  message.point_step = 16U;
  message.row_step = message.point_step * message.width;
  const std::array<std::string, 4> names{"x", "y", "z", "intensity"};
  for (std::size_t index = 0; index < names.size(); ++index) {
    sensor_msgs::msg::PointField field;
    field.name = names[index];
    field.offset = static_cast<std::uint32_t>(4U * index);
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1U;
    message.fields.push_back(field);
  }
  message.data.resize(message.row_step);
  for (std::size_t index = 0; index < points.size(); ++index) {
    float values[4]{static_cast<float>(points[index].x()),
                    static_cast<float>(points[index].y()),
                    static_cast<float>(points[index].z()),
                    hits[index] ? 1.0F : 0.0F};
    std::memcpy(message.data.data() + index * message.point_step, values,
                sizeof(values));
  }
  return message;
}

std::string compactJson(const Json::Value &value) {
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "";
  return Json::writeString(builder, value);
}

}  // namespace

class Racer3DCppMockSimulator final : public rclcpp::Node {
 public:
  Racer3DCppMockSimulator() : Node("racer_3d_cpp_mock_sim") {
    scenario_name_ = declare_parameter<std::string>(
        "scenario_name", "acceptance_15x9x2");
    drone_count_ = declare_parameter<int>("drone_count", 3);
    lidar_range_ = declare_parameter<double>("lidar_range", 7.0);
    max_speed_ = declare_parameter<double>("max_speed", 0.35);
    max_acceleration_ = declare_parameter<double>("max_acceleration", 1.4);
    scenario_ = getScenario(scenario_name_);
    if (scenario_.starts.size() < static_cast<std::size_t>(drone_count_)) {
      throw std::runtime_error("scenario has fewer starts than drone_count");
    }
    positions_.assign(scenario_.starts.begin(),
                      scenario_.starts.begin() + drone_count_);
    velocities_.assign(drone_count_, Point3::Zero());
    commands_.assign(drone_count_, Point3::Zero());
    yaws_.assign(drone_count_, 0.0);
    yaw_commands_.assign(drone_count_, 0.0);
    path_lengths_.assign(drone_count_, 0.0);
    contact_active_.assign(drone_count_, false);
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    for (int id = 0; id < drone_count_; ++id) {
      const std::string ns = "/drone_" + std::to_string(id);
      odom_pubs_.push_back(create_publisher<nav_msgs::msg::Odometry>(
          ns + "/odom", qos));
      cloud_pubs_.push_back(create_publisher<sensor_msgs::msg::PointCloud2>(
          ns + "/points", qos));
      cmd_subs_.push_back(create_subscription<geometry_msgs::msg::Twist>(
          ns + "/cmd_vel_3d", qos,
          [this, id](geometry_msgs::msg::Twist::ConstSharedPtr message) {
            commands_[id] = Point3(message->linear.x, message->linear.y,
                                   message->linear.z);
            yaw_commands_[id] = message->angular.z;
          }));
    }
    metrics_pub_ = create_publisher<std_msgs::msg::String>(
        "/racer_3d/sim_metrics", qos);
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50), [this]() { step(); });
    RCLCPP_INFO(get_logger(), "C++ 3-D analytic mock backend ready");
  }

 private:
  void step() {
    constexpr double dt = 0.05;
    elapsed_ += dt;
    for (int id = 0; id < drone_count_; ++id) {
      Point3 command = limitNorm(commands_[id], max_speed_);
      Point3 delta = command - velocities_[id];
      if (delta.norm() > max_acceleration_ * dt) {
        delta *= max_acceleration_ * dt / delta.norm();
      }
      Point3 velocity = velocities_[id] + delta;
      Point3 proposed = positions_[id] + velocity * dt;
      const double clearance =
          obstacleClearance(proposed, scenario_.obstacles);
      const bool contact = clearance <= DRONE_RADIUS;
      if (contact && !contact_active_[id]) ++collision_events_;
      contact_active_[id] = contact;
      if (contact) {
        ++safety_interventions_;
        velocity.setZero();
        proposed = positions_[id];
      }
      path_lengths_[id] += (proposed - positions_[id]).norm();
      positions_[id] = proposed;
      velocities_[id] = velocity;
      double error = std::remainder(yaw_commands_[id] - yaws_[id],
                                    2.0 * M_PI);
      yaws_[id] += std::clamp(error, -1.5 * dt, 1.5 * dt);
    }
    bool peer_contact = false;
    for (int first = 0; first < drone_count_; ++first) {
      for (int second = first + 1; second < drone_count_; ++second) {
        const double distance = (positions_[first] - positions_[second]).norm();
        min_inter_drone_ = std::min(min_inter_drone_, distance);
        peer_contact = peer_contact || distance < 2.0 * DRONE_RADIUS;
      }
    }
    if (peer_contact && !peer_contact_active_) ++collision_events_;
    peer_contact_active_ = peer_contact;
    for (const auto &position : positions_) {
      min_obstacle_clearance_ =
          std::min(min_obstacle_clearance_,
                   obstacleClearance(position, scenario_.obstacles) -
                       DRONE_RADIUS);
    }
    publishOdometry();
    if (elapsed_ - last_cloud_ >= 0.30 - 1.0e-9) {
      last_cloud_ = elapsed_;
      publishClouds();
    }
    if (static_cast<int>(std::round(elapsed_ * 10.0)) % 5 == 0) {
      publishMetrics();
    }
  }

  void publishOdometry() {
    const auto stamp = now();
    for (int id = 0; id < drone_count_; ++id) {
      nav_msgs::msg::Odometry message;
      message.header.stamp = stamp;
      message.header.frame_id = "map";
      message.child_frame_id = "drone_" + std::to_string(id) + "/base_link";
      message.pose.pose.position.x = positions_[id].x();
      message.pose.pose.position.y = positions_[id].y();
      message.pose.pose.position.z = positions_[id].z();
      message.pose.pose.orientation.w = std::cos(0.5 * yaws_[id]);
      message.pose.pose.orientation.z = std::sin(0.5 * yaws_[id]);
      message.twist.twist.linear.x = velocities_[id].x();
      message.twist.twist.linear.y = velocities_[id].y();
      message.twist.twist.linear.z = velocities_[id].z();
      odom_pubs_[id]->publish(message);
    }
  }

  void publishClouds() {
    const auto stamp = now();
    for (int id = 0; id < drone_count_; ++id) {
      const auto local = simulatePointCloud(
          positions_[id], 120, 21, 100.0 * M_PI / 180.0, lidar_range_,
          scenario_.obstacles);
      std::vector<Point3> world;
      std::vector<bool> hits;
      world.reserve(local.size());
      hits.reserve(local.size());
      for (const auto &point : local) {
        world.push_back(point + positions_[id]);
        hits.push_back(point.norm() < lidar_range_ - 0.03);
      }
      cloud_pubs_[id]->publish(makeCloud(stamp, world, hits));
    }
  }

  void publishMetrics() {
    Json::Value root;
    root["backend"] = "ros_3d_cpp_mock";
    root["scenario"] = scenario_.name;
    root["elapsed"] = elapsed_;
    root["collision_events"] = collision_events_;
    root["physics_contact_events"] = collision_events_;
    root["safety_interventions"] = safety_interventions_;
    root["min_inter_drone"] = min_inter_drone_;
    root["min_obstacle_clearance"] = min_obstacle_clearance_;
    root["sensor_source"] = "C++ analytic 3-D lidar";
    root["motion_source"] = "C++ acceleration-limited 3-D point mass";
    for (const auto &position : positions_) {
      Json::Value point(Json::arrayValue);
      point.append(position.x());
      point.append(position.y());
      point.append(position.z());
      root["positions"].append(point);
    }
    for (const double distance : path_lengths_) {
      root["path_lengths"].append(distance);
    }
    std_msgs::msg::String message;
    message.data = compactJson(root);
    metrics_pub_->publish(message);
  }

  std::string scenario_name_;
  int drone_count_{};
  double lidar_range_{};
  double max_speed_{};
  double max_acceleration_{};
  Scenario3D scenario_;
  std::vector<Point3> positions_;
  std::vector<Point3> velocities_;
  std::vector<Point3> commands_;
  std::vector<double> yaws_;
  std::vector<double> yaw_commands_;
  std::vector<double> path_lengths_;
  std::vector<bool> contact_active_;
  bool peer_contact_active_{};
  int collision_events_{};
  int safety_interventions_{};
  double min_inter_drone_{std::numeric_limits<double>::infinity()};
  double min_obstacle_clearance_{std::numeric_limits<double>::infinity()};
  double elapsed_{};
  double last_cloud_{-std::numeric_limits<double>::infinity()};
  std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_pubs_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
      cloud_pubs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr>
      cmd_subs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace racer_3d_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<racer_3d_cpp::Racer3DCppMockSimulator>());
  rclcpp::shutdown();
  return 0;
}
