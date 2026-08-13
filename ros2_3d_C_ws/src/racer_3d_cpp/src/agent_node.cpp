#include "racer_3d_cpp/allocation.hpp"
#include "racer_3d_cpp/hgrid.hpp"
#include "racer_3d_cpp/planning.hpp"
#include "racer_3d_cpp/safety.hpp"
#include "racer_3d_cpp/serialization.hpp"
#include "racer_3d_cpp/voxel_map.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <json/json.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace racer_3d_cpp {
namespace {

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Json::Value vectorJson(const Point3 &value) {
  Json::Value result(Json::arrayValue);
  result.append(value.x());
  result.append(value.y());
  result.append(value.z());
  return result;
}

Point3 jsonVector(const Json::Value &value, const Point3 &fallback = Point3::Zero()) {
  if (!value.isArray() || value.size() < 3U) return fallback;
  return Point3(value[0].asDouble(), value[1].asDouble(), value[2].asDouble());
}

std::string compactJson(const Json::Value &value) {
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "";
  return Json::writeString(builder, value);
}

bool parseJson(const std::string &text, Json::Value &value) {
  Json::CharReaderBuilder builder;
  std::string errors;
  std::istringstream input(text);
  return Json::parseFromStream(builder, input, &value, &errors);
}

std::optional<std::size_t> fieldOffset(
    const sensor_msgs::msg::PointCloud2 &message, const std::string &name) {
  for (const auto &field : message.fields) {
    if (field.name == name &&
        field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      return static_cast<std::size_t>(field.offset);
    }
  }
  return std::nullopt;
}

void readCloud(const sensor_msgs::msg::PointCloud2 &message,
               std::vector<Point3> &points, std::vector<bool> &hits) {
  points.clear();
  hits.clear();
  if (message.is_bigendian || message.point_step == 0U) return;
  const auto ox = fieldOffset(message, "x");
  const auto oy = fieldOffset(message, "y");
  const auto oz = fieldOffset(message, "z");
  const auto oi = fieldOffset(message, "intensity");
  if (!ox || !oy || !oz) return;
  const std::size_t count =
      std::min<std::size_t>(message.width * message.height,
                            message.data.size() / message.point_step);
  points.reserve(count);
  hits.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const auto *base = message.data.data() + index * message.point_step;
    float x{}, y{}, z{}, intensity{1.0F};
    if (*ox + sizeof(float) > message.point_step ||
        *oy + sizeof(float) > message.point_step ||
        *oz + sizeof(float) > message.point_step) {
      break;
    }
    std::memcpy(&x, base + *ox, sizeof(float));
    std::memcpy(&y, base + *oy, sizeof(float));
    std::memcpy(&z, base + *oz, sizeof(float));
    if (oi && *oi + sizeof(float) <= message.point_step) {
      std::memcpy(&intensity, base + *oi, sizeof(float));
    }
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      points.emplace_back(x, y, z);
      hits.push_back(intensity > 0.5F);
    }
  }
}

sensor_msgs::msg::PointCloud2 occupiedCloud(
    const VoxelMap &map, const rclcpp::Time &stamp) {
  sensor_msgs::msg::PointCloud2 message;
  message.header.stamp = stamp;
  message.header.frame_id = "map";
  message.height = 1U;
  message.is_bigendian = false;
  message.is_dense = true;
  message.point_step = 16U;
  const std::array<std::string, 4> names{"x", "y", "z", "intensity"};
  for (std::size_t index = 0; index < names.size(); ++index) {
    sensor_msgs::msg::PointField field;
    field.name = names[index];
    field.offset = static_cast<std::uint32_t>(4U * index);
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1U;
    message.fields.push_back(field);
  }
  const auto &states = map.states();
  std::vector<Point3> points;
  for (int z = 0; z < map.nz(); ++z) {
    for (int y = 0; y < map.ny(); ++y) {
      for (int x = 0; x < map.nx(); ++x) {
        if (states[map.flatIndex({x, y, z})] == OCCUPIED) {
          points.push_back(map.gridToWorld({x, y, z}));
        }
      }
    }
  }
  message.width = static_cast<std::uint32_t>(points.size());
  message.row_step = message.point_step * message.width;
  message.data.resize(message.row_step);
  for (std::size_t index = 0; index < points.size(); ++index) {
    float values[4]{static_cast<float>(points[index].x()),
                    static_cast<float>(points[index].y()),
                    static_cast<float>(points[index].z()), 1.0F};
    std::memcpy(message.data.data() + index * message.point_step, values,
                sizeof(values));
  }
  return message;
}

}  // namespace

class Racer3DCppAgent final : public rclcpp::Node {
 public:
  Racer3DCppAgent() : Node("racer_3d_cpp_agent") {
    scenario_name_ = declare_parameter<std::string>(
        "scenario_name", "acceptance_15x9x2");
    drone_id_ = declare_parameter<int>("drone_id", 0);
    drone_count_ = declare_parameter<int>("drone_count", 3);
    const auto flattened = declare_parameter<std::vector<double>>(
        "start_positions",
        {-6.4, -3.2, 0.45, -6.4, 0.0, 1.0, -6.4, 3.2, 1.55});
    for (std::size_t i = 0; i + 2 < flattened.size(); i += 3) {
      starts_.emplace_back(flattened[i], flattened[i + 1], flattened[i + 2]);
    }
    if (starts_.size() < static_cast<std::size_t>(drone_count_)) {
      throw std::runtime_error("start_positions has fewer poses than drone_count");
    }
    const auto origin_values = declare_parameter<std::vector<double>>(
        "map_origin", {-7.5, -4.5, 0.0});
    const auto size_values = declare_parameter<std::vector<double>>(
        "map_size", {15.0, 9.0, 2.0});
    const auto coarse_values = declare_parameter<std::vector<double>>(
        "coarse_grid_size", {5.0, 4.5, 2.0});
    if (origin_values.size() != 3U || size_values.size() != 3U ||
        coarse_values.size() != 3U) {
      throw std::runtime_error("map_origin/map_size/coarse_grid_size require xyz");
    }
    const Point3 origin(origin_values[0], origin_values[1], origin_values[2]);
    const Point3 size(size_values[0], size_values[1], size_values[2]);
    const Point3 coarse(coarse_values[0], coarse_values[1], coarse_values[2]);
    map_ = std::make_unique<VoxelMap>(
        declare_parameter<double>("map_resolution", 0.20), origin, size);
    hgrid_ = std::make_unique<HierarchicalGrid3D>(
        *map_, coarse, declare_parameter<int>("hgrid_levels", 2));
    minimum_sensor_range_ =
        declare_parameter<double>("minimum_sensor_range", 0.50);
    lidar_range_ = declare_parameter<double>("lidar_range", 7.0);
    if (!std::isfinite(minimum_sensor_range_) ||
        minimum_sensor_range_ < 0.0 ||
        !std::isfinite(lidar_range_) ||
        lidar_range_ <= minimum_sensor_range_) {
      throw std::runtime_error(
          "sensor ray ranges must satisfy 0 <= minimum < maximum");
    }
    const int maximum_sensor_rays =
        declare_parameter<int>("maximum_sensor_rays", 2400);
    if (maximum_sensor_rays <= 0) {
      throw std::runtime_error("maximum_sensor_rays must be positive");
    }
    maximum_sensor_rays_ = static_cast<std::size_t>(maximum_sensor_rays);
    clearance_ = declare_parameter<double>("planning_clearance", 0.22);
    control_clearance_ = declare_parameter<double>("control_clearance", 0.45);
    safe_distance_ = declare_parameter<double>("swarm_safe_distance", 0.65);
    emergency_distance_ = declare_parameter<double>("emergency_distance", 0.80);
    max_speed_ = declare_parameter<double>("max_speed", 0.35);
    max_acceleration_ = declare_parameter<double>("max_acceleration", 1.4);
    guaranteed_deceleration_ =
        declare_parameter<double>("guaranteed_deceleration", 0.6);
    safety_response_time_ =
        declare_parameter<double>("safety_response_time", 0.20);
    if (guaranteed_deceleration_ <= 0.0 || safety_response_time_ < 0.0) {
      throw std::runtime_error(
          "safety deceleration must be positive and response time nonnegative");
    }
    planning_period_ = declare_parameter<double>("planning_period", 1.0);
    pairwise_period_ = declare_parameter<double>("pairwise_period", 3.0);
    peer_timeout_ = declare_parameter<double>("peer_timeout", 3.0);
    completion_coverage_ =
        declare_parameter<double>("completion_coverage", 0.90);

    const auto reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    const std::string ns = "/drone_" + std::to_string(drone_id_);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        ns + "/cmd_vel_3d", reliable);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        ns + "/planned_path_3d", reliable);
    occupied_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        ns + "/occupied_voxels", reliable);
    map_pub_ = create_publisher<std_msgs::msg::String>(
        "/racer_3d/map_share", reliable);
    state_pub_ = create_publisher<std_msgs::msg::String>(
        "/racer_3d/swarm_state", reliable);
    pairwise_pub_ = create_publisher<std_msgs::msg::String>(
        "/racer_3d/pairwise", reliable);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        ns + "/status", reliable);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        ns + "/odom", sensor_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { onOdom(*msg); });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        ns + "/points", sensor_qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          onCloud(*msg);
        });
    map_sub_ = create_subscription<std_msgs::msg::String>(
        "/racer_3d/map_share", reliable,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          decodeAndMergeMap(msg->data, *map_, drone_id_);
        });
    state_sub_ = create_subscription<std_msgs::msg::String>(
        "/racer_3d/swarm_state", reliable,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          onPeerState(msg->data);
        });
    pairwise_sub_ = create_subscription<std_msgs::msg::String>(
        "/racer_3d/pairwise", reliable,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          onAllocation(msg->data);
        });
    completion_sub_ = create_subscription<std_msgs::msg::String>(
        "/racer_3d/mission_complete", reliable,
        [this](std_msgs::msg::String::ConstSharedPtr msg) {
          if (msg->data == "true") {
            mission_complete_ = true;
            current_plan_.reset();
          }
        });

    // ROS timers follow Isaac's /clock. This preserves the upstream timer
    // cadence even when the 1 kHz PhysX plant runs slower than wall time.
    control_timer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(0.05),
        [this]() { controlTick(); });
    planning_timer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(planning_period_),
        [this]() { planningTick(); });
    state_timer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(0.20),
        [this]() { publishState(); });
    map_timer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(1.0),
        [this]() { publishMap(); });
    pairwise_timer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(0.50),
        [this]() { pairwiseTick(); });
    RCLCPP_INFO(get_logger(), "C++ RACER 3D agent %d/%d ready", drone_id_,
                drone_count_);
  }

 private:
  double nowSeconds() const { return now().seconds(); }

  void onOdom(const nav_msgs::msg::Odometry &message) {
    const auto &p = message.pose.pose.position;
    const auto &v = message.twist.twist.linear;
    position_ = Point3(p.x, p.y, p.z);
    velocity_ = Point3(v.x, v.y, v.z);
    yaw_ = yawFromQuaternion(message.pose.pose.orientation);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2 &message) {
    if (!position_ || message.header.frame_id != "map") return;
    std::vector<Point3> points;
    std::vector<bool> hits;
    readCloud(message, points, hits);
    if (points.empty()) return;
    std::vector<Point3> filtered_points;
    std::vector<bool> filtered_hits;
    filtered_points.reserve(points.size());
    filtered_hits.reserve(hits.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
      const double distance = (points[index] - *position_).norm();
      if (!std::isfinite(distance) ||
          distance < minimum_sensor_range_) {
        continue;
      }
      filtered_points.push_back(points[index]);
      if (!hits.empty()) {
        filtered_hits.push_back(hits[index]);
      }
    }
    if (filtered_points.empty()) return;
    map_->updatePointCloud(
        *position_, filtered_points, lidar_range_, filtered_hits,
        maximum_sensor_rays_);
    sensor_ready_ = true;
  }

  void onPeerState(const std::string &text) {
    Json::Value root;
    if (!parseJson(text, root)) return;
    const int peer_id = root.get("drone_id", -1).asInt();
    if (peer_id < 0 || peer_id == drone_id_) return;
    PeerState state;
    state.drone_id = peer_id;
    state.stamp = root.get("stamp", 0.0).asDouble();
    state.received = nowSeconds();
    state.position = jsonVector(root["position"]);
    state.velocity = jsonVector(root["velocity"]);
    for (const auto &item : root["owned_cells"]) {
      state.owned_cells.push_back(item.asString());
    }
    for (const auto &item : root["trajectory"]) {
      if (item.isArray() && item.size() >= 4U) {
        state.trajectory.push_back(
            {item[0].asDouble(),
             Point3(item[1].asDouble(), item[2].asDouble(), item[3].asDouble())});
      }
    }
    peers_[peer_id] = std::move(state);
  }

  void onAllocation(const std::string &text) {
    Json::Value root;
    if (!parseJson(text, root) || root.get("to", -1).asInt() != drone_id_) {
      return;
    }
    const int sender = root.get("from", -1).asInt();
    std::set<std::string> receiver;
    for (const auto &item : root["cells"]) receiver.insert(item.asString());
    for (const auto &item : root["union"]) {
      const auto id = item.asString();
      if (hgrid_->cells().count(id) != 0U) {
        owners_[id] = receiver.count(id) != 0U ? drone_id_ : sender;
      }
    }
    refreshOwned();
    coverage_route_.clear();
    for (const auto &item : root["route"]) {
      if (hgrid_->cells().count(item.asString()) != 0U) {
        coverage_route_.push_back(item.asString());
      }
    }
  }

  std::unordered_map<int, PeerState> activePeers() const {
    std::unordered_map<int, PeerState> result;
    const double stamp = nowSeconds();
    for (const auto &[id, state] : peers_) {
      if (stamp - state.received < peer_timeout_) result.emplace(id, state);
    }
    return result;
  }

  void refreshOwned() {
    owned_cells_.clear();
    for (const auto &[id, owner] : owners_) {
      if (owner == drone_id_ && hgrid_->cells().count(id) != 0U) {
        owned_cells_.push_back(id);
      }
    }
    std::sort(owned_cells_.begin(), owned_cells_.end());
  }

  void reconcileOwnership() {
    const auto &active = hgrid_->cells();
    if (active.empty()) {
      owners_.clear();
      owned_cells_.clear();
      coverage_route_.clear();
      return;
    }
    const auto initial = hgrid_->initialOwners(
        std::vector<Point3>(starts_.begin(), starts_.begin() + drone_count_));
    const auto old = owners_;
    owners_.clear();
    for (const auto &[id, cell] : active) {
      auto found = old.find(id);
      if (found != old.end()) {
        owners_[id] = found->second;
      } else if (cell.level > 1) {
        const std::string parent =
            std::to_string(cell.level - 1) + ":" +
            std::to_string(cell.ix / 2) + ":" +
            std::to_string(cell.iy / 2) + ":" +
            std::to_string(cell.iz / 2);
        const auto inherited = old.find(parent);
        owners_[id] = inherited != old.end() ? inherited->second : initial.at(id);
      } else {
        owners_[id] = initial.at(id);
      }
    }
    refreshOwned();
    if (position_) {
      coverage_route_ = hgrid_->coverageRoute(owned_cells_, *position_);
    }
  }

  void pairwiseTick() {
    if (!position_) return;
    const double stamp = nowSeconds();
    if (stamp - last_pairwise_ < pairwise_period_) return;
    const auto peers = activePeers();
    std::vector<std::pair<int, int>> pairs;
    for (int first = 0; first < drone_count_; ++first) {
      for (int second = first + 1; second < drone_count_; ++second) {
        pairs.emplace_back(first, second);
      }
    }
    if (pairs.empty()) return;
    const auto pair = pairs[static_cast<std::size_t>(
        std::floor(stamp / pairwise_period_)) % pairs.size()];
    const auto peer_it = peers.find(pair.second);
    if (drone_id_ != pair.first || peer_it == peers.end()) return;
    std::set<std::string> ids(owned_cells_.begin(), owned_cells_.end());
    ids.insert(peer_it->second.owned_cells.begin(),
               peer_it->second.owned_cells.end());
    std::vector<HGridCell3D> cells;
    for (const auto &id : ids) {
      const auto found = hgrid_->cells().find(id);
      if (found != hgrid_->cells().end()) cells.push_back(found->second);
    }
    if (cells.empty()) return;
    const auto partition =
        capacityPartition(cells, *position_, peer_it->second.position);
    for (const auto &id : partition.first) owners_[id] = drone_id_;
    for (const auto &id : partition.second) owners_[id] = pair.second;
    owned_cells_ = partition.first;
    coverage_route_ = partition.firstRoute;
    Json::Value root;
    root["from"] = drone_id_;
    root["to"] = pair.second;
    for (const auto &id : partition.second) root["cells"].append(id);
    for (const auto &id : partition.first) root["sender_cells"].append(id);
    for (const auto &id : ids) root["union"].append(id);
    for (const auto &id : partition.secondRoute) root["route"].append(id);
    root["stamp"] = stamp;
    std_msgs::msg::String message;
    message.data = compactJson(root);
    pairwise_pub_->publish(message);
    last_pairwise_ = stamp;
  }

  bool planReusable() {
    if (!position_ || !current_plan_ || current_plan_->trajectory.empty()) {
      return false;
    }
    const double elapsed = nowSeconds() - plan_started_;
    if (elapsed > current_plan_->trajectory.back().time + 1.0 ||
        (*position_ - current_plan_->goal).norm() < 0.35) {
      return false;
    }
    for (const auto &sample : current_plan_->trajectory) {
      if (sample.time + 0.25 < elapsed) continue;
      const auto cell = map_->worldToGrid(sample.position);
      if (!cell || map_->stateAt(*cell) != FREE ||
          map_->distanceAt(sample.position, false) < clearance_) {
        return false;
      }
    }
    return true;
  }

  void planningTick() {
    if (!position_ || !sensor_ready_) return;
    hgrid_->update();
    reconcileOwnership();
    const double coverage = map_->coverage();
    if (mission_complete_ || coverage >= completion_coverage_) {
      mission_complete_ = true;
      current_plan_.reset();
    } else if (!planReusable()) {
      auto plan = planExploration(*map_, *hgrid_, owned_cells_,
                                  coverage_route_, *position_, clearance_,
                                  max_speed_, max_acceleration_);
      if (plan) {
        std::vector<TrajectorySample> absolute;
        const double stamp = nowSeconds();
        for (const auto &sample : plan->trajectory) {
          absolute.push_back({stamp + sample.time, sample.position});
        }
        for (const auto &[peer_id, peer] : activePeers()) {
          if (drone_id_ > peer_id &&
              predictedPathConflict(absolute, peer.trajectory, safe_distance_)) {
            yield_until_ = std::max(yield_until_, stamp + 0.8);
          }
        }
        current_plan_ = std::move(plan);
        plan_started_ = stamp;
        publishPath(*current_plan_);
      } else {
        current_plan_.reset();
      }
    }
    Json::Value status;
    status["drone_id"] = drone_id_;
    status["coverage"] = coverage;
    status["frontier_clusters"] =
        static_cast<Json::UInt64>(map_->frontierClusters().size());
    status["owned_hgrid_cells"] =
        static_cast<Json::UInt64>(owned_cells_.size());
    status["planning"] = static_cast<bool>(current_plan_);
    status["completed"] = mission_complete_;
    status["implementation"] = "racer_3d_cpp";
    std_msgs::msg::String message;
    message.data = compactJson(status);
    status_pub_->publish(message);
  }

  void controlTick() {
    geometry_msgs::msg::Twist message;
    if (!position_ || !current_plan_ || mission_complete_ ||
        nowSeconds() < yield_until_) {
      message.angular.z = yaw_;
      cmd_pub_->publish(message);
      return;
    }
    const double target_time = nowSeconds() - plan_started_ + 0.25;
    auto target = current_plan_->trajectory.back();
    for (const auto &sample : current_plan_->trajectory) {
      if (sample.time >= target_time) {
        target = sample;
        break;
      }
    }
    Point3 safe = limitNorm(1.4 * (target.position - *position_), max_speed_);
    std::vector<PeerState> peers;
    for (const auto &[id, state] : activePeers()) {
      (void)id;
      peers.push_back(state);
    }
    safe = cbfSwarmFilter(safe, *position_, peers, safe_distance_, max_speed_,
                          velocity_, 0.8, guaranteed_deceleration_,
                          safety_response_time_);
    std::vector<Point3> peer_positions;
    for (const auto &peer : peers) peer_positions.push_back(peer.position);
    // Emergency repulsion is followed by the obstacle barrier, so the final
    // command cannot be pushed into a wall by inter-vehicle separation.
    safe = emergencySeparation(safe, *position_, peer_positions,
                               emergency_distance_, max_speed_);
    safe = esdfObstacleFilter(safe, *position_, *map_, control_clearance_,
                              max_speed_, velocity_, 0.8,
                              guaranteed_deceleration_,
                              safety_response_time_);
    message.linear.x = safe.x();
    message.linear.y = safe.y();
    message.linear.z = safe.z();
    // Isaac's bridge intentionally interprets angular.z as absolute yaw.
    message.angular.z = current_plan_->yaw;
    cmd_pub_->publish(message);
  }

  void publishPath(const ExplorationPlan3D &plan) {
    nav_msgs::msg::Path message;
    message.header.stamp = now();
    message.header.frame_id = "map";
    for (const auto &sample : plan.trajectory) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = sample.position.x();
      pose.pose.position.y = sample.position.y();
      pose.pose.position.z = sample.position.z();
      pose.pose.orientation.w = 1.0;
      message.poses.push_back(pose);
    }
    path_pub_->publish(message);
  }

  void publishMap() {
    std_msgs::msg::String message;
    message.data = encodeMap(*map_, drone_id_, ++map_sequence_);
    map_pub_->publish(message);
    occupied_pub_->publish(occupiedCloud(*map_, now()));
  }

  void publishState() {
    if (!position_) return;
    Json::Value root;
    root["drone_id"] = drone_id_;
    root["stamp"] = nowSeconds();
    root["position"] = vectorJson(*position_);
    root["velocity"] = vectorJson(velocity_);
    for (const auto &id : owned_cells_) root["owned_cells"].append(id);
    if (current_plan_) {
      for (const auto &sample : current_plan_->trajectory) {
        Json::Value value(Json::arrayValue);
        value.append(plan_started_ + sample.time);
        value.append(sample.position.x());
        value.append(sample.position.y());
        value.append(sample.position.z());
        root["trajectory"].append(value);
      }
    }
    std_msgs::msg::String message;
    message.data = compactJson(root);
    state_pub_->publish(message);
  }

  std::string scenario_name_;
  int drone_id_{};
  int drone_count_{};
  std::vector<Point3> starts_;
  std::unique_ptr<VoxelMap> map_;
  std::unique_ptr<HierarchicalGrid3D> hgrid_;
  double minimum_sensor_range_{};
  double lidar_range_{};
  std::size_t maximum_sensor_rays_{};
  double clearance_{};
  double control_clearance_{};
  double safe_distance_{};
  double emergency_distance_{};
  double max_speed_{};
  double max_acceleration_{};
  double guaranteed_deceleration_{};
  double safety_response_time_{};
  double planning_period_{};
  double pairwise_period_{};
  double peer_timeout_{};
  double completion_coverage_{};
  std::optional<Point3> position_;
  Point3 velocity_{Point3::Zero()};
  double yaw_{};
  bool sensor_ready_{false};
  bool mission_complete_{false};
  std::unordered_map<int, PeerState> peers_;
  std::unordered_map<std::string, int> owners_;
  std::vector<std::string> owned_cells_;
  std::vector<std::string> coverage_route_;
  std::optional<ExplorationPlan3D> current_plan_;
  double plan_started_{};
  double yield_until_{};
  std::uint64_t map_sequence_{};
  double last_pairwise_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pairwise_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pairwise_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr completion_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr pairwise_timer_;
};

}  // namespace racer_3d_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<racer_3d_cpp::Racer3DCppAgent>());
  rclcpp::shutdown();
  return 0;
}
