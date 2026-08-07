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
#include <racer_3d_interfaces/msg/comm_packet.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
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

builtin_interfaces::msg::Duration durationMessage(double seconds) {
  builtin_interfaces::msg::Duration result;
  if (!std::isfinite(seconds) || seconds <= 0.0) return result;
  const double integral = std::floor(seconds);
  result.sec = static_cast<std::int32_t>(integral);
  result.nanosec = static_cast<std::uint32_t>(
      std::llround((seconds - integral) * 1.0e9));
  if (result.nanosec >= 1000000000U) {
    ++result.sec;
    result.nanosec -= 1000000000U;
  }
  return result;
}

std::vector<std::uint8_t> payloadBytes(const std::string &value) {
  return std::vector<std::uint8_t>(value.begin(), value.end());
}

std::string payloadText(
    const racer_3d_interfaces::msg::CommPacket &packet) {
  return std::string(packet.payload.begin(), packet.payload.end());
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
    const double map_resolution =
        declare_parameter<double>("map_resolution", 0.20);
    map_ = std::make_unique<VoxelMap>(map_resolution, origin, size);
    local_map_ = std::make_unique<VoxelMap>(map_resolution, origin, size);
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
    planning_period_ = declare_parameter<double>("planning_period", 1.0);
    pairwise_period_ = declare_parameter<double>("pairwise_period", 3.0);
    peer_timeout_ = declare_parameter<double>("peer_timeout", 3.0);
    safety_peer_timeout_ =
        declare_parameter<double>("safety_peer_timeout", 6.0);
    map_manifest_period_ =
        declare_parameter<double>("map_manifest_period", 2.0);
    allocation_retry_period_ =
        declare_parameter<double>("allocation_retry_period", 1.0);
    allocation_proposal_timeout_ =
        declare_parameter<double>("allocation_proposal_timeout", 10.0);
    const int map_chunk_voxels =
        declare_parameter<int>("map_chunk_voxels", 200);
    const int map_repair_chunks =
        declare_parameter<int>("map_repair_chunks_per_manifest", 32);
    if (map_chunk_voxels <= 0 || map_repair_chunks <= 0 ||
        safety_peer_timeout_ < peer_timeout_ || map_manifest_period_ <= 0.0 ||
        allocation_retry_period_ <= 0.0 || allocation_proposal_timeout_ <= 0.0) {
      throw std::runtime_error("invalid communication-aware agent parameters");
    }
    map_chunk_voxels_ = static_cast<std::size_t>(map_chunk_voxels);
    map_repair_chunks_per_manifest_ =
        static_cast<std::size_t>(map_repair_chunks);
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
    comm_tx_pub_ =
        create_publisher<racer_3d_interfaces::msg::CommPacket>(
            "/racer_3d/comm/tx",
            rclcpp::QoS(rclcpp::KeepLast(1000)).reliable());
    evaluation_map_pub_ = create_publisher<std_msgs::msg::String>(
        "/racer_3d/evaluation/map_share_raw", reliable);
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
    comm_rx_sub_ =
        create_subscription<racer_3d_interfaces::msg::CommPacket>(
            ns + "/comm/rx", rclcpp::QoS(rclcpp::KeepLast(1000)).reliable(),
            [this](racer_3d_interfaces::msg::CommPacket::ConstSharedPtr msg) {
              onCommPacket(*msg);
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
    manifest_timer_ = rclcpp::create_timer(
        this, get_clock(),
        rclcpp::Duration::from_seconds(map_manifest_period_),
        [this]() { publishManifest(); });
    allocation_retry_timer_ = rclcpp::create_timer(
        this, get_clock(),
        rclcpp::Duration::from_seconds(allocation_retry_period_),
        [this]() { allocationRetryTick(); });
    RCLCPP_INFO(get_logger(),
                "communication-aware C++ RACER agent %d/%d ready", drone_id_,
                drone_count_);
  }

 private:
  enum class AllocationStage { kProposal, kCommit };

  struct PendingAllocation {
    std::uint64_t transaction{};
    int peer{-1};
    PairwiseAllocationMessage allocation;
    std::vector<std::string> senderRoute;
    AllocationStage stage{AllocationStage::kProposal};
    double created{};
    double lastSent{};
  };

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
    local_map_->updatePointCloud(
        *position_, filtered_points, lidar_range_, filtered_hits,
        maximum_sensor_rays_);
    sensor_ready_ = true;
  }

  void sendPacket(std::uint8_t type, int receiver, const std::string &payload,
                  double ttl, std::uint8_t priority, bool reliable,
                  int origin_id = -1) {
    racer_3d_interfaces::msg::CommPacket packet;
    packet.source_stamp = now();
    packet.ttl = durationMessage(ttl);
    packet.sender_id = drone_id_;
    packet.receiver_id = receiver;
    packet.origin_id = origin_id < 0 ? drone_id_ : origin_id;
    packet.sequence = ++packet_sequence_;
    packet.payload_type = type;
    packet.priority = priority;
    packet.reliable = reliable;
    packet.payload = payloadBytes(payload);
    packet.payload_size = static_cast<std::uint32_t>(packet.payload.size());
    comm_tx_pub_->publish(packet);
  }

  void onCommPacket(
      const racer_3d_interfaces::msg::CommPacket &packet) {
    using Packet = racer_3d_interfaces::msg::CommPacket;
    if (packet.receiver_id != drone_id_ || packet.sender_id < 0 ||
        packet.sender_id == drone_id_ ||
        packet.payload_size != packet.payload.size()) {
      return;
    }
    const std::string payload = payloadText(packet);
    switch (packet.payload_type) {
      case Packet::TYPE_STATE: {
        const auto previous = last_state_packet_.find(packet.sender_id);
        if (previous != last_state_packet_.end() &&
            packet.sequence <= previous->second) {
          return;
        }
        last_state_packet_[packet.sender_id] = packet.sequence;
        onPeerState(payload);
        break;
      }
      case Packet::TYPE_MAP_CHUNK:
        onMapChunk(payload);
        break;
      case Packet::TYPE_MAP_MANIFEST:
        onMapManifest(payload, packet.sender_id);
        break;
      case Packet::TYPE_ALLOCATION_PROPOSAL:
        onAllocationProposal(payload, packet.sender_id);
        break;
      case Packet::TYPE_ALLOCATION_ACK:
        onAllocationAck(payload, packet.sender_id);
        break;
      case Packet::TYPE_ALLOCATION_COMMIT:
        onAllocationCommit(payload, packet.sender_id);
        break;
      case Packet::TYPE_ALLOCATION_COMMIT_ACK:
        onAllocationCommitAck(payload, packet.sender_id);
        break;
      default:
        break;
    }
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

  void onMapChunk(const std::string &text) {
    SparseMapChunk chunk;
    std::string error;
    if (!decodeSparseMapChunk(text, chunk, &error)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "invalid map chunk: %s", error.c_str());
      return;
    }
    const std::pair<int, std::uint64_t> key{chunk.originId, chunk.sequence};
    if (chunk_store_.count(key) != 0U) return;
    for (const auto index : chunk.indices) {
      if (index >= map_->voxelCount()) {
        RCLCPP_WARN(get_logger(), "map chunk contains out-of-range voxel");
        return;
      }
    }
    map_->mergeSparse(chunk.indices, chunk.states);
    chunk_store_[key] = text;
  }

  MapManifest currentManifest() const {
    MapManifest manifest;
    manifest.senderId = drone_id_;
    std::unordered_map<int, std::vector<std::uint64_t>> sequences;
    for (const auto &[key, payload] : chunk_store_) {
      (void)payload;
      sequences[key.first].push_back(key.second);
    }
    for (auto &[origin, values] : sequences) {
      std::sort(values.begin(), values.end());
      values.erase(std::unique(values.begin(), values.end()), values.end());
      if (values.empty()) continue;
      auto &ranges = manifest.ranges[origin];
      std::uint64_t first = values.front();
      std::uint64_t last = first;
      for (std::size_t index = 1; index < values.size(); ++index) {
        if (values[index] == last + 1U) {
          last = values[index];
        } else {
          ranges.emplace_back(first, last);
          first = last = values[index];
        }
      }
      ranges.emplace_back(first, last);
    }
    return manifest;
  }

  void publishManifest() {
    if (chunk_store_.empty()) return;
    sendPacket(racer_3d_interfaces::msg::CommPacket::TYPE_MAP_MANIFEST, -1,
               encodeMapManifest(currentManifest()), 5.0, 80U, false);
  }

  void onMapManifest(const std::string &text, int sender) {
    MapManifest manifest;
    if (!decodeMapManifest(text, manifest) || manifest.senderId != sender) {
      return;
    }
    std::size_t sent = 0U;
    for (const auto &[key, payload] : chunk_store_) {
      if (manifestContains(manifest, key.first, key.second)) continue;
      sendPacket(racer_3d_interfaces::msg::CommPacket::TYPE_MAP_CHUNK, sender,
                 payload, 120.0, 40U, false, key.first);
      if (++sent >= map_repair_chunks_per_manifest_) break;
    }
  }

  std::string allocationEnvelope(std::uint64_t transaction,
                                 const PairwiseAllocationMessage *allocation) {
    Json::Value root;
    root["transaction"] = Json::UInt64(transaction);
    if (allocation != nullptr) {
      root["allocation"] = encodePairwiseAllocation(*allocation);
    }
    return compactJson(root);
  }

  bool decodeAllocationEnvelope(const std::string &text,
                                std::uint64_t &transaction,
                                PairwiseAllocationMessage *allocation) {
    Json::Value root;
    if (!parseJson(text, root) || !root.isMember("transaction") ||
        !root["transaction"].isIntegral()) {
      return false;
    }
    transaction = root["transaction"].asUInt64();
    if (allocation != nullptr) {
      if (!root.isMember("allocation") || !root["allocation"].isString()) {
        return false;
      }
      return decodePairwiseAllocation(root["allocation"].asString(),
                                      *allocation);
    }
    return true;
  }

  void applyReceiverAllocation(const PairwiseAllocationMessage &allocation) {
    std::set<std::string> receiver;
    receiver.insert(allocation.cells.begin(), allocation.cells.end());
    for (const auto &id : allocation.unionCells) {
      if (hgrid_->cells().count(id) != 0U) {
        owners_[id] =
            receiver.count(id) != 0U ? drone_id_ : allocation.fromId;
      }
    }
    refreshOwned();
    coverage_route_.clear();
    for (const auto &id : allocation.route) {
      if (hgrid_->cells().count(id) != 0U) {
        coverage_route_.push_back(id);
      }
    }
  }

  void applySenderAllocation() {
    if (!pending_allocation_) return;
    std::set<std::string> sender(
        pending_allocation_->allocation.senderCells.begin(),
        pending_allocation_->allocation.senderCells.end());
    for (const auto &id : pending_allocation_->allocation.unionCells) {
      if (hgrid_->cells().count(id) != 0U) {
        owners_[id] = sender.count(id) != 0U
                          ? drone_id_
                          : pending_allocation_->peer;
      }
    }
    refreshOwned();
    coverage_route_ = pending_allocation_->senderRoute;
  }

  void onAllocationProposal(const std::string &text, int sender) {
    std::uint64_t transaction{};
    PairwiseAllocationMessage allocation;
    if (!decodeAllocationEnvelope(text, transaction, &allocation) ||
        allocation.fromId != sender || allocation.toId != drone_id_) {
      return;
    }
    received_allocations_[transaction] = allocation;
    sendPacket(racer_3d_interfaces::msg::CommPacket::TYPE_ALLOCATION_ACK,
               sender, allocationEnvelope(transaction, nullptr), 10.0, 220U,
               true);
  }

  void onAllocationAck(const std::string &text, int sender) {
    std::uint64_t transaction{};
    if (!decodeAllocationEnvelope(text, transaction, nullptr) ||
        !pending_allocation_ || pending_allocation_->peer != sender ||
        pending_allocation_->transaction != transaction ||
        pending_allocation_->stage != AllocationStage::kProposal) {
      return;
    }
    applySenderAllocation();
    pending_allocation_->stage = AllocationStage::kCommit;
    pending_allocation_->lastSent = nowSeconds();
    sendPacket(racer_3d_interfaces::msg::CommPacket::TYPE_ALLOCATION_COMMIT,
               sender,
               allocationEnvelope(transaction,
                                  &pending_allocation_->allocation),
               30.0, 230U, true);
  }

  void onAllocationCommit(const std::string &text, int sender) {
    std::uint64_t transaction{};
    PairwiseAllocationMessage allocation;
    if (!decodeAllocationEnvelope(text, transaction, &allocation) ||
        allocation.fromId != sender || allocation.toId != drone_id_) {
      return;
    }
    const auto found = received_allocations_.find(transaction);
    if (found != received_allocations_.end()) {
      applyReceiverAllocation(found->second);
      committed_allocations_.insert(transaction);
    } else if (committed_allocations_.count(transaction) == 0U) {
      // A proposal ACK may have been lost at the transport boundary. The
      // commit carries the complete allocation so it remains self-contained.
      applyReceiverAllocation(allocation);
      committed_allocations_.insert(transaction);
    }
    sendPacket(
        racer_3d_interfaces::msg::CommPacket::TYPE_ALLOCATION_COMMIT_ACK,
        sender, allocationEnvelope(transaction, nullptr), 10.0, 240U, true);
  }

  void onAllocationCommitAck(const std::string &text, int sender) {
    std::uint64_t transaction{};
    if (!decodeAllocationEnvelope(text, transaction, nullptr) ||
        !pending_allocation_ || pending_allocation_->peer != sender ||
        pending_allocation_->transaction != transaction ||
        pending_allocation_->stage != AllocationStage::kCommit) {
      return;
    }
    pending_allocation_.reset();
    last_pairwise_ = nowSeconds();
  }

  std::unordered_map<int, PeerState> activePeers() const {
    std::unordered_map<int, PeerState> result;
    const double stamp = nowSeconds();
    for (const auto &[id, state] : peers_) {
      if (stamp - state.received < peer_timeout_) result.emplace(id, state);
    }
    return result;
  }

  std::vector<PeerState> safetyPeers(double &inflated_distance) const {
    std::vector<PeerState> result;
    const double stamp = nowSeconds();
    inflated_distance = safe_distance_;
    for (const auto &[id, state] : peers_) {
      (void)id;
      const double age = std::max(0.0, stamp - state.received);
      if (age >= safety_peer_timeout_) continue;
      PeerState projected = state;
      projected.position += std::min(age, 1.0) * state.velocity;
      result.push_back(std::move(projected));
      inflated_distance = std::max(
          inflated_distance,
          safe_distance_ + max_speed_ * std::min(age, peer_timeout_));
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
    if (!position_ || pending_allocation_) return;
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
    PendingAllocation pending;
    pending.transaction =
        (static_cast<std::uint64_t>(static_cast<std::uint8_t>(drone_id_))
         << 56U) |
        (++allocation_sequence_ & 0x00ffffffffffffffULL);
    pending.peer = pair.second;
    pending.allocation.fromId = drone_id_;
    pending.allocation.toId = pair.second;
    pending.allocation.cells = partition.second;
    pending.allocation.senderCells = partition.first;
    pending.allocation.unionCells.assign(ids.begin(), ids.end());
    pending.allocation.route = partition.secondRoute;
    pending.allocation.stamp = stamp;
    pending.senderRoute = partition.firstRoute;
    pending.created = stamp;
    pending.lastSent = stamp;
    pending_allocation_ = std::move(pending);
    sendPacket(
        racer_3d_interfaces::msg::CommPacket::TYPE_ALLOCATION_PROPOSAL,
        pair.second,
        allocationEnvelope(pending_allocation_->transaction,
                           &pending_allocation_->allocation),
        10.0, 220U, true);
  }

  void allocationRetryTick() {
    if (!pending_allocation_) return;
    const double stamp = nowSeconds();
    if (pending_allocation_->stage == AllocationStage::kProposal &&
        stamp - pending_allocation_->created > allocation_proposal_timeout_) {
      pending_allocation_.reset();
      last_pairwise_ = stamp;
      return;
    }
    if (stamp - pending_allocation_->lastSent < allocation_retry_period_) {
      return;
    }
    if (activePeers().count(pending_allocation_->peer) == 0U) return;
    pending_allocation_->lastSent = stamp;
    const auto type = pending_allocation_->stage == AllocationStage::kProposal
                          ? racer_3d_interfaces::msg::CommPacket::
                                TYPE_ALLOCATION_PROPOSAL
                          : racer_3d_interfaces::msg::CommPacket::
                                TYPE_ALLOCATION_COMMIT;
    sendPacket(type, pending_allocation_->peer,
               allocationEnvelope(pending_allocation_->transaction,
                                  &pending_allocation_->allocation),
               pending_allocation_->stage == AllocationStage::kProposal
                   ? 10.0
                   : 30.0,
               230U, true);
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
    status["implementation"] = "racer_3d_cpp_sionna_hybrid";
    status["map_chunks"] = static_cast<Json::UInt64>(chunk_store_.size());
    status["active_peers"] =
        static_cast<Json::UInt64>(activePeers().size());
    status["allocation_transaction_pending"] =
        static_cast<bool>(pending_allocation_);
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
    double dynamic_safe_distance = safe_distance_;
    const auto peers = safetyPeers(dynamic_safe_distance);
    safe = cbfSwarmFilter(safe, *position_, peers, dynamic_safe_distance,
                          max_speed_);
    std::vector<Point3> peer_positions;
    for (const auto &peer : peers) peer_positions.push_back(peer.position);
    // Emergency repulsion is followed by the obstacle barrier, so the final
    // command cannot be pushed into a wall by inter-vehicle separation.
    safe = emergencySeparation(safe, *position_, peer_positions,
                               emergency_distance_, max_speed_);
    safe = esdfObstacleFilter(safe, *position_, *map_, control_clearance_,
                              max_speed_, velocity_);
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
    const auto &states = local_map_->states();
    if (last_local_states_.size() != states.size()) {
      last_local_states_.assign(states.size(), UNKNOWN);
    }
    std::vector<std::uint32_t> dirty;
    dirty.reserve(map_chunk_voxels_);
    for (std::size_t index = 0; index < states.size(); ++index) {
      if (states[index] != UNKNOWN && states[index] != last_local_states_[index]) {
        dirty.push_back(static_cast<std::uint32_t>(index));
      }
      last_local_states_[index] = states[index];
    }
    for (std::size_t begin = 0; begin < dirty.size();
         begin += map_chunk_voxels_) {
      const std::size_t end =
          std::min(dirty.size(), begin + map_chunk_voxels_);
      SparseMapChunk chunk;
      chunk.originId = drone_id_;
      chunk.sequence = ++map_chunk_sequence_;
      chunk.indices.insert(chunk.indices.end(), dirty.begin() + begin,
                           dirty.begin() + end);
      chunk.states.reserve(end - begin);
      for (std::size_t offset = begin; offset < end; ++offset) {
        chunk.states.push_back(states[dirty[offset]]);
      }
      const std::string payload = encodeSparseMapChunk(chunk);
      chunk_store_[{drone_id_, chunk.sequence}] = payload;
      sendPacket(racer_3d_interfaces::msg::CommPacket::TYPE_MAP_CHUNK, -1,
                 payload, 120.0, 40U, false, drone_id_);
    }
    std_msgs::msg::String evaluation;
    evaluation.data =
        encodeMap(*map_, drone_id_, ++evaluation_map_sequence_);
    evaluation_map_pub_->publish(evaluation);
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
    sendPacket(racer_3d_interfaces::msg::CommPacket::TYPE_STATE, -1,
               compactJson(root), 0.6, 250U, false);
  }

  std::string scenario_name_;
  int drone_id_{};
  int drone_count_{};
  std::vector<Point3> starts_;
  std::unique_ptr<VoxelMap> map_;
  std::unique_ptr<VoxelMap> local_map_;
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
  double planning_period_{};
  double pairwise_period_{};
  double peer_timeout_{};
  double safety_peer_timeout_{};
  double map_manifest_period_{};
  double allocation_retry_period_{};
  double allocation_proposal_timeout_{};
  std::size_t map_chunk_voxels_{};
  std::size_t map_repair_chunks_per_manifest_{};
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
  std::uint64_t packet_sequence_{};
  std::uint64_t map_chunk_sequence_{};
  std::uint64_t evaluation_map_sequence_{};
  std::uint64_t allocation_sequence_{};
  double last_pairwise_{};
  std::vector<std::int8_t> last_local_states_;
  std::map<std::pair<int, std::uint64_t>, std::string> chunk_store_;
  std::unordered_map<int, std::uint64_t> last_state_packet_;
  std::optional<PendingAllocation> pending_allocation_;
  std::unordered_map<std::uint64_t, PairwiseAllocationMessage>
      received_allocations_;
  std::unordered_set<std::uint64_t> committed_allocations_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_pub_;
  rclcpp::Publisher<racer_3d_interfaces::msg::CommPacket>::SharedPtr
      comm_tx_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr evaluation_map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<racer_3d_interfaces::msg::CommPacket>::SharedPtr
      comm_rx_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr pairwise_timer_;
  rclcpp::TimerBase::SharedPtr manifest_timer_;
  rclcpp::TimerBase::SharedPtr allocation_retry_timer_;
};

}  // namespace racer_3d_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<racer_3d_cpp::Racer3DCppAgent>());
  rclcpp::shutdown();
  return 0;
}
