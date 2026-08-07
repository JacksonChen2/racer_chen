#include "racer_3d_cpp/scenario.hpp"
#include "racer_3d_cpp/serialization.hpp"
#include "racer_3d_cpp/voxel_map.hpp"

#include <json/json.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <racer_3d_interfaces/msg/comm_statistics.hpp>
#include <racer_3d_interfaces/msg/link_quality_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace racer_3d_cpp {
namespace {

bool parseJson(const std::string &text, Json::Value &value) {
  Json::CharReaderBuilder builder;
  std::string errors;
  std::istringstream input(text);
  return Json::parseFromStream(builder, input, &value, &errors);
}

double jsonNumber(const Json::Value &root, const char *name, double fallback) {
  const auto &value = root[name];
  return value.isNumeric() ? value.asDouble() : fallback;
}

Json::Value finiteOrNull(double value) {
  return std::isfinite(value) ? Json::Value(value) : Json::Value();
}

}  // namespace

class Racer3DCppMonitor final : public rclcpp::Node {
 public:
  Racer3DCppMonitor() : Node("racer_3d_cpp_monitor") {
    scenario_name_ = declare_parameter<std::string>(
        "scenario_name", "acceptance_15x9x2");
    scenario_ = getScenario(scenario_name_);
    drone_count_ = declare_parameter<int>("drone_count", 3);
    duration_ = declare_parameter<double>("duration", 120.0);
    result_file_ = declare_parameter<std::string>(
        "result_file", "/tmp/racer_3d_cpp_result.json");
    minimum_coverage_ = declare_parameter<double>("minimum_coverage", 0.90);
    minimum_free_accuracy_ =
        declare_parameter<double>("minimum_free_accuracy", 0.95);
    minimum_occupied_precision_ =
        declare_parameter<double>("minimum_occupied_precision", 0.75);
    minimum_surface_recall_ =
        declare_parameter<double>("minimum_surface_recall", 0.35);
    minimum_inter_drone_ =
        declare_parameter<double>("minimum_inter_drone", 0.35);
    minimum_obstacle_clearance_ =
        declare_parameter<double>("minimum_obstacle_clearance", 0.02);
    require_physics_backend_ =
        declare_parameter<bool>("require_physics_backend", false);
    truth_mode_ =
        declare_parameter<std::string>("truth_mode", scenario_.truth_mode);
    const auto origin = declare_parameter<std::vector<double>>(
        "map_origin",
        {scenario_.map_min.x(), scenario_.map_min.y(), scenario_.map_min.z()});
    const auto size = declare_parameter<std::vector<double>>(
        "map_size", {scenario_.mapSize().x(), scenario_.mapSize().y(),
                     scenario_.mapSize().z()});
    map_ = std::make_unique<VoxelMap>(
        declare_parameter<double>("map_resolution", 0.20),
        Point3(origin[0], origin[1], origin[2]),
        Point3(size[0], size[1], size[2]));
    truth_occupied_.assign(map_->voxelCount(), false);
    if (truth_mode_ == "analytic_boxes") {
      for (int z = 0; z < map_->nz(); ++z) {
        for (int y = 0; y < map_->ny(); ++y) {
          for (int x = 0; x < map_->nx(); ++x) {
            const Point3 point = map_->gridToWorld({x, y, z});
            truth_occupied_[map_->flatIndex({x, y, z})] =
                std::any_of(scenario_.obstacles.begin(),
                            scenario_.obstacles.end(), [&](const Box3D &box) {
                              return box.contains(
                                  point, 0.5 * map_->resolution());
                            });
          }
        }
      }
    } else if (truth_mode_ != "observed_volume") {
      throw std::runtime_error("unsupported truth_mode: " + truth_mode_);
    }
    truth_surface_ = surfaceMask(truth_occupied_);
    path_lengths_.assign(drone_count_, 0.0);
    if (!require_physics_backend_) started_ = now().seconds();

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    completion_pub_ = create_publisher<std_msgs::msg::String>(
        "/racer_3d/mission_complete", qos);
    map_sub_ = create_subscription<std_msgs::msg::String>(
        "/racer_3d/evaluation/map_share_raw", qos,
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          decodeAndMergeMap(message->data, *map_);
        });
    comm_statistics_sub_ =
        create_subscription<racer_3d_interfaces::msg::CommStatistics>(
            "/racer_3d/comm/statistics", qos,
            [this](
                racer_3d_interfaces::msg::CommStatistics::ConstSharedPtr msg) {
              comm_statistics_ = *msg;
              comm_statistics_seen_ = true;
            });
    link_quality_sub_ =
        create_subscription<racer_3d_interfaces::msg::LinkQualityArray>(
            "/racer_3d/link_quality", qos,
            [this](
                racer_3d_interfaces::msg::LinkQualityArray::ConstSharedPtr msg) {
              link_quality_ = *msg;
            });
    metrics_sub_ = create_subscription<std_msgs::msg::String>(
        "/racer_3d/sim_metrics", qos,
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          Json::Value root;
          if (!parseJson(message->data, root)) return;
          sim_metrics_ = root;
          if (!backend_seen_) {
            RCLCPP_INFO(
                get_logger(),
                "connected to backend=%s sensor=%s",
                root.get("backend", "unknown").asCString(),
                root.get("sensor_source", "unknown").asCString());
          }
          backend_seen_ = true;
          if (!started_) started_ = now().seconds();
        });
    for (int id = 0; id < drone_count_; ++id) {
      const std::string ns = "/drone_" + std::to_string(id);
      odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
          ns + "/odom", qos,
          [this, id](nav_msgs::msg::Odometry::ConstSharedPtr message) {
            onOdom(id, *message);
          }));
      status_subs_.push_back(create_subscription<std_msgs::msg::String>(
          ns + "/status", qos,
          [this, id](std_msgs::msg::String::ConstSharedPtr message) {
            Json::Value root;
            if (parseJson(message->data, root)) status_[id] = root;
          }));
    }
    timer_ = create_wall_timer(
        std::chrono::milliseconds(500), [this]() { check(); });
    RCLCPP_INFO(get_logger(), "C++ acceptance monitor %.1fs target %.0f%%",
                duration_, 100.0 * minimum_coverage_);
  }

 private:
  struct Quality {
    double coverage{};
    std::optional<double> free_accuracy;
    std::optional<double> occupied_precision;
    std::optional<double> surface_recall;
    std::size_t known{};
    std::size_t total{};
    std::size_t truth_free{};
    std::size_t truth_surface{};
  };

  std::vector<bool> surfaceMask(const std::vector<bool> &occupied) const {
    std::vector<bool> surface(occupied.size(), false);
    constexpr int delta[6][3]{{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
                              {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    for (int z = 0; z < map_->nz(); ++z) {
      for (int y = 0; y < map_->ny(); ++y) {
        for (int x = 0; x < map_->nx(); ++x) {
          const GridIndex3 cell{x, y, z};
          if (!occupied[map_->flatIndex(cell)]) continue;
          for (const auto &move : delta) {
            const GridIndex3 neighbor{x + move[0], y + move[1], z + move[2]};
            if (map_->inBounds(neighbor) &&
                !occupied[map_->flatIndex(neighbor)]) {
              surface[map_->flatIndex(cell)] = true;
              break;
            }
          }
        }
      }
    }
    return surface;
  }

  void onOdom(int id, const nav_msgs::msg::Odometry &message) {
    const auto &p = message.pose.pose.position;
    const Point3 position(p.x, p.y, p.z);
    const auto previous = last_positions_.find(id);
    if (previous != last_positions_.end()) {
      const double distance = (position - previous->second).norm();
      if (distance < 0.25) path_lengths_[id] += distance;
    }
    last_positions_[id] = position;
    positions_[id] = position;
    min_obstacle_clearance_seen_ =
        std::min(min_obstacle_clearance_seen_,
                 obstacleClearance(position, scenario_.obstacles) -
                     DRONE_RADIUS);
    for (const auto &[peer_id, peer] : positions_) {
      if (peer_id != id) {
        min_inter_drone_seen_ =
            std::min(min_inter_drone_seen_, (position - peer).norm());
      }
    }
  }

  Quality quality() const {
    Quality result;
    const auto &states = map_->states();
    result.total = states.size();
    result.known = static_cast<std::size_t>(std::count_if(
        states.begin(), states.end(), [](std::int8_t value) {
          return value != UNKNOWN;
        }));
    if (truth_mode_ == "observed_volume") {
      result.coverage =
          static_cast<double>(result.known) / std::max<std::size_t>(1, result.total);
      return result;
    }
    std::size_t known_free_truth = 0U;
    std::size_t correct_free = 0U;
    std::size_t predicted_occupied = 0U;
    std::size_t correct_occupied = 0U;
    std::vector<bool> expanded(states.size(), false);
    for (std::size_t index = 0; index < states.size(); ++index) {
      const bool truth_free = !truth_occupied_[index];
      result.truth_free += truth_free ? 1U : 0U;
      if (truth_free && states[index] != UNKNOWN) {
        ++known_free_truth;
        if (states[index] == FREE) ++correct_free;
      }
      if (states[index] == OCCUPIED) {
        ++predicted_occupied;
        if (truth_occupied_[index]) ++correct_occupied;
      }
      result.truth_surface += truth_surface_[index] ? 1U : 0U;
    }
    for (int z = 0; z < map_->nz(); ++z) {
      for (int y = 0; y < map_->ny(); ++y) {
        for (int x = 0; x < map_->nx(); ++x) {
          bool nearby = false;
          for (int dz = -1; dz <= 1 && !nearby; ++dz) {
            for (int dy = -1; dy <= 1 && !nearby; ++dy) {
              for (int dx = -1; dx <= 1 && !nearby; ++dx) {
                if (std::abs(dx) + std::abs(dy) + std::abs(dz) > 1) continue;
                const GridIndex3 neighbor{x + dx, y + dy, z + dz};
                nearby = map_->inBounds(neighbor) &&
                         states[map_->flatIndex(neighbor)] == OCCUPIED;
              }
            }
          }
          expanded[map_->flatIndex({x, y, z})] = nearby;
        }
      }
    }
    std::size_t recalled = 0U;
    for (std::size_t index = 0; index < expanded.size(); ++index) {
      if (expanded[index] && truth_surface_[index]) ++recalled;
    }
    result.coverage = static_cast<double>(known_free_truth) /
                      std::max<std::size_t>(1, result.truth_free);
    result.free_accuracy = static_cast<double>(correct_free) /
                           std::max<std::size_t>(1, known_free_truth);
    result.occupied_precision = static_cast<double>(correct_occupied) /
                                std::max<std::size_t>(1, predicted_occupied);
    result.surface_recall = static_cast<double>(recalled) /
                            std::max<std::size_t>(1, result.truth_surface);
    return result;
  }

  void check() {
    if (finished_ || !started_) return;
    const double elapsed = now().seconds() - *started_;
    const auto current = quality();
    if (!completion_time_ && current.coverage >= minimum_coverage_) {
      completion_wall_time_ = elapsed;
      completion_time_ =
          jsonNumber(sim_metrics_, "elapsed", elapsed);
      std_msgs::msg::String message;
      message.data = "true";
      completion_pub_->publish(message);
      RCLCPP_INFO(get_logger(), "coverage reached at %.2fs",
                  *completion_time_);
    }
    const double deadline_elapsed =
        require_physics_backend_ && backend_seen_
            ? jsonNumber(sim_metrics_, "elapsed", 0.0)
            : elapsed;
    const bool settled =
        completion_wall_time_ && elapsed >= *completion_wall_time_ + 2.0;
    // A duration expressed as a decimal is not always reached exactly by an
    // accumulated fixed physics step (for example, 2.0 at 1 ms). Keep the
    // acceptance boundary deterministic without requiring an extra sim step.
    if (settled || deadline_elapsed + 1.0e-6 >= duration_) {
      finish(elapsed, current);
    }
  }

  void finish(double elapsed, const Quality &current) {
    const int collisions =
        sim_metrics_.get("collision_events", 0).asInt();
    const int contacts =
        sim_metrics_.get("physics_contact_events", collisions).asInt();
    const std::string backend =
        sim_metrics_.get("backend", "not_received").asString();
    const bool physics_ok =
        !require_physics_backend_ || backend == "isaac_sim_physx_3d";
    const double min_inter =
        std::min(min_inter_drone_seen_,
                 jsonNumber(sim_metrics_, "min_inter_drone",
                            std::numeric_limits<double>::infinity()));
    const double min_obstacle =
        std::min(min_obstacle_clearance_seen_,
                 jsonNumber(sim_metrics_, "min_obstacle_clearance",
                            std::numeric_limits<double>::infinity()));
    const bool quality_ok =
        truth_mode_ == "observed_volume" ||
        (*current.free_accuracy >= minimum_free_accuracy_ &&
         *current.occupied_precision >= minimum_occupied_precision_ &&
         *current.surface_recall >= minimum_surface_recall_);
    const bool all_agents = status_.size() == static_cast<std::size_t>(drone_count_);
    const bool passed =
        backend_seen_ && physics_ok && completion_time_.has_value() &&
        quality_ok && collisions == 0 && contacts == 0 &&
        min_inter >= minimum_inter_drone_ &&
        min_obstacle >= minimum_obstacle_clearance_ &&
        positions_.size() == static_cast<std::size_t>(drone_count_) &&
        all_agents;
    Json::Value root;
    root["passed"] = passed;
    root["implementation"] = "racer_3d_cpp";
    root["backend"] = backend;
    root["vehicle_model"] =
        sim_metrics_.get("vehicle_model", "not_received");
    root["vehicle_asset_usd"] =
        sim_metrics_.get("vehicle_asset_usd", Json::Value());
    root["motion_source"] =
        sim_metrics_.get("motion_source", "not_received");
    root["sensor_source"] =
        sim_metrics_.get("sensor_source", "not_received");
    root["clock_source"] =
        sim_metrics_.get("clock_source", "wall clock");
    root["physics_rate_hz"] =
        sim_metrics_.get("physics_rate_hz", Json::Value());
    root["odometry_rate_hz"] =
        sim_metrics_.get("odometry_rate_hz", Json::Value());
    root["imu_rate_hz"] =
        sim_metrics_.get("imu_rate_hz", Json::Value());
    root["sensor_rate_hz"] =
        sim_metrics_["sensor_parameters"].get("rate_hz", Json::Value());
    root["sensor_parameters"] =
        sim_metrics_.get("sensor_parameters", Json::Value());
    root["point_cloud_frames"] =
        sim_metrics_.get("point_cloud_frames", 0);
    root["safety_interventions"] =
        sim_metrics_.get("safety_interventions", 0);
    root["max_contact_force"] =
        sim_metrics_.get("max_contact_force", 0.0);
    root["scenario"] = scenario_.name;
    root["drone_count"] = drone_count_;
    root["elapsed_s"] = jsonNumber(sim_metrics_, "elapsed", elapsed);
    root["wall_elapsed_s"] = elapsed;
    const double sim_elapsed = root["elapsed_s"].asDouble();
    const double sensor_rate =
        sim_metrics_["sensor_parameters"].get("rate_hz", 0.0).asDouble();
    const double physics_rate =
        sim_metrics_.get("physics_rate_hz", 0.0).asDouble();
    const double expected_cloud_frames =
        sim_elapsed * sensor_rate * static_cast<double>(drone_count_);
    const double expected_control_steps =
        sim_elapsed * physics_rate * static_cast<double>(drone_count_);
    root["performance"]["realtime_factor"] =
        elapsed > 0.0 ? sim_elapsed / elapsed : Json::Value();
    root["performance"]["expected_point_cloud_frames"] =
        expected_cloud_frames;
    root["performance"]["point_cloud_delivery_ratio"] =
        expected_cloud_frames > 0.0
            ? root["point_cloud_frames"].asDouble() / expected_cloud_frames
            : Json::Value();
    root["performance"]["safety_intervention_fraction"] =
        expected_control_steps > 0.0
            ? root["safety_interventions"].asDouble() /
                  expected_control_steps
            : Json::Value();
    root["communication"]["statistics_available"] =
        comm_statistics_seen_;
    if (comm_statistics_seen_) {
      root["communication"]["attempted_packets"] =
          Json::UInt64(comm_statistics_.attempted_packets);
      root["communication"]["delivered_packets"] =
          Json::UInt64(comm_statistics_.delivered_packets);
      root["communication"]["dropped_no_link"] =
          Json::UInt64(comm_statistics_.dropped_no_link);
      root["communication"]["dropped_per"] =
          Json::UInt64(comm_statistics_.dropped_per);
      root["communication"]["dropped_queue"] =
          Json::UInt64(comm_statistics_.dropped_queue);
      root["communication"]["dropped_ttl"] =
          Json::UInt64(comm_statistics_.dropped_ttl);
      root["communication"]["retried_packets"] =
          Json::UInt64(comm_statistics_.retried_packets);
      root["communication"]["attempted_bytes"] =
          Json::UInt64(comm_statistics_.attempted_bytes);
      root["communication"]["delivered_bytes"] =
          Json::UInt64(comm_statistics_.delivered_bytes);
      root["communication"]["mean_delivery_delay_ms"] =
          comm_statistics_.mean_delivery_delay_ms;
      root["communication"]["delivery_ratio"] =
          comm_statistics_.attempted_packets > 0U
              ? static_cast<double>(comm_statistics_.delivered_packets) /
                    comm_statistics_.attempted_packets
              : Json::Value();
    }
    double snr_sum = 0.0;
    std::size_t snr_count = 0U;
    for (const auto &link : link_quality_.links) {
      root["communication"]["link_models"][link.model] =
          root["communication"]["link_models"].get(link.model, 0).asUInt64() +
          1U;
      if (std::isfinite(link.snr_db) && link.model != "unavailable") {
        snr_sum += link.snr_db;
        ++snr_count;
      }
    }
    root["communication"]["mean_current_snr_db"] =
        snr_count > 0U ? Json::Value(snr_sum / snr_count) : Json::Value();
    root["completion_time_s"] =
        completion_time_ ? Json::Value(*completion_time_) : Json::Value();
    root["completion_wall_time_s"] =
        completion_wall_time_ ? Json::Value(*completion_wall_time_) : Json::Value();
    double total_distance = 0.0;
    for (const double distance : path_lengths_) {
      root["flight_distance_m"].append(distance);
      total_distance += distance;
    }
    root["total_flight_distance_m"] = total_distance;
    root["volume_coverage"] = current.coverage;
    root["free_space_accuracy"] = current.free_accuracy
                                      ? Json::Value(*current.free_accuracy)
                                      : Json::Value();
    root["occupied_precision"] =
        current.occupied_precision ? Json::Value(*current.occupied_precision)
                                   : Json::Value();
    root["obstacle_surface_recall"] =
        current.surface_recall ? Json::Value(*current.surface_recall)
                               : Json::Value();
    root["known_voxels"] = static_cast<Json::UInt64>(current.known);
    root["total_voxels"] = static_cast<Json::UInt64>(current.total);
    root["map_quality_ground_truth_available"] =
        truth_mode_ == "analytic_boxes";
    root["collision_events"] = collisions;
    root["physics_contact_events"] = contacts;
    root["minimum_inter_drone_m"] = finiteOrNull(min_inter);
    root["minimum_obstacle_clearance_m"] = finiteOrNull(min_obstacle);
    root["all_agents_reporting"] = all_agents;
    for (const auto &[id, value] : status_) {
      root["agent_status"][std::to_string(id)] = value;
    }
    for (const auto &[id, position] : positions_) {
      Json::Value point(Json::arrayValue);
      point.append(position.x());
      point.append(position.y());
      point.append(position.z());
      root["final_positions"][std::to_string(id)] = point;
    }
    root["requirements"]["minimum_coverage"] = minimum_coverage_;
    root["requirements"]["minimum_inter_drone"] = minimum_inter_drone_;
    root["requirements"]["minimum_obstacle_clearance"] =
        minimum_obstacle_clearance_;
    root["requirements"]["require_physics_backend"] =
        require_physics_backend_;
    std::filesystem::path target(result_file_);
    if (!target.parent_path().empty()) {
      std::filesystem::create_directories(target.parent_path());
    }
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    const std::string output = Json::writeString(builder, root) + "\n";
    std::ofstream(target) << output;
    RCLCPP_INFO(get_logger(), "acceptance %s: %s",
                passed ? "PASS" : "FAIL", target.c_str());
    finished_ = true;
    rclcpp::shutdown();
  }

  std::string scenario_name_;
  Scenario3D scenario_;
  int drone_count_{};
  double duration_{};
  std::string result_file_;
  double minimum_coverage_{};
  double minimum_free_accuracy_{};
  double minimum_occupied_precision_{};
  double minimum_surface_recall_{};
  double minimum_inter_drone_{};
  double minimum_obstacle_clearance_{};
  bool require_physics_backend_{};
  std::string truth_mode_;
  std::unique_ptr<VoxelMap> map_;
  std::vector<bool> truth_occupied_;
  std::vector<bool> truth_surface_;
  std::optional<double> started_;
  std::optional<double> completion_time_;
  std::optional<double> completion_wall_time_;
  std::vector<double> path_lengths_;
  std::unordered_map<int, Point3> last_positions_;
  std::unordered_map<int, Point3> positions_;
  std::unordered_map<int, Json::Value> status_;
  double min_inter_drone_seen_{std::numeric_limits<double>::infinity()};
  double min_obstacle_clearance_seen_{std::numeric_limits<double>::infinity()};
  Json::Value sim_metrics_;
  bool backend_seen_{};
  bool finished_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr completion_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr metrics_sub_;
  rclcpp::Subscription<racer_3d_interfaces::msg::CommStatistics>::SharedPtr
      comm_statistics_sub_;
  rclcpp::Subscription<racer_3d_interfaces::msg::LinkQualityArray>::SharedPtr
      link_quality_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
      odom_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr>
      status_subs_;
  racer_3d_interfaces::msg::CommStatistics comm_statistics_;
  racer_3d_interfaces::msg::LinkQualityArray link_quality_;
  bool comm_statistics_seen_{false};
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace racer_3d_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<racer_3d_cpp::Racer3DCppMonitor>());
  return 0;
}
