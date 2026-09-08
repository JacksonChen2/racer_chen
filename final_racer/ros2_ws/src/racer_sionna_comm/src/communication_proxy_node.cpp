#include <racer_sionna_comm/link_model.hpp>

#include <racer_sionna_interfaces/msg/comm_statistics.hpp>
#include <racer_sionna_interfaces/msg/link_quality.hpp>
#include <racer_sionna_interfaces/msg/link_quality_array.hpp>

#include <racer_fidelity_msgs/msg/chunk_data.hpp>
#include <racer_fidelity_msgs/msg/chunk_stamps.hpp>
#include <racer_fidelity_msgs/msg/pair_opt.hpp>
#include <racer_fidelity_msgs/msg/pair_opt_response.hpp>
#include <racer_recovery_core/msg/recovery_command.hpp>
#include <racer_recovery_core/msg/recovery_status.hpp>

#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/serialization.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace racer_sionna_comm {
namespace {

using LinkQuality = racer_sionna_interfaces::msg::LinkQuality;
using LinkQualityArray = racer_sionna_interfaces::msg::LinkQualityArray;

struct TopicPolicy {
  std::string key;
  std::string type;
  int priority{};
  double ttl_s{};
  bool reliable{};
};

const std::vector<TopicPolicy> kTopicPolicies{
    {"drone_state", "racer_fidelity_msgs/msg/DroneState", 6, 0.30, false},
    {"pair_opt", "racer_fidelity_msgs/msg/PairOpt", 7, 2.0, true},
    {"pair_opt_res", "racer_fidelity_msgs/msg/PairOptResponse", 7, 2.0, true},
    {"trajectory", "racer_fidelity_msgs/msg/Bspline", 8, 1.0, true},
    {"recovery_status", "racer_recovery_core/msg/RecoveryStatus", 9, 2.0, true},
    {"recovery_command", "racer_recovery_core/msg/RecoveryCommand", 10, 5.0, true},
    {"chunk_stamps", "racer_fidelity_msgs/msg/ChunkStamps", 4, 3.0, true},
    {"chunk_data", "racer_fidelity_msgs/msg/ChunkData", 2, 10.0, true},
    {"global_assignment", "racer_fidelity_msgs/msg/GlobalGridAssignment", 10, 2.0, true},
};

struct LinkKey {
  int sender{};
  int receiver{};

  bool operator==(const LinkKey &other) const noexcept {
    return sender == other.sender && receiver == other.receiver;
  }
};

struct LinkKeyHash {
  std::size_t operator()(const LinkKey &key) const noexcept {
    return (static_cast<std::size_t>(static_cast<std::uint32_t>(key.sender))
            << 32U) ^
           static_cast<std::size_t>(static_cast<std::uint32_t>(key.receiver));
  }
};

enum class RouteStage {
  kDirect,
  kApUplink,
  kApDownlink,
  kBsControl,
  kBsUplink,
  kBsDownlink,
};

struct ChunkKey {
  int owner{};
  std::uint32_t index{};

  bool operator==(const ChunkKey &other) const noexcept {
    return owner == other.owner && index == other.index;
  }
};

struct ChunkKeyHash {
  std::size_t operator()(const ChunkKey &key) const noexcept {
    return (static_cast<std::size_t>(static_cast<std::uint32_t>(key.owner))
            << 32U) ^ static_cast<std::size_t>(key.index);
  }
};

double timeSeconds(const builtin_interfaces::msg::Time &value) {
  return static_cast<double>(value.sec) + 1.0e-9 * value.nanosec;
}

std::string txTopic(int sender, const TopicPolicy &policy) {
  return "/racer_sionna/tx/drone_" + std::to_string(sender) + "/" +
         policy.key;
}

std::string rxTopic(int receiver, const TopicPolicy &policy) {
  return "/racer_sionna/rx/drone_" + std::to_string(receiver) + "/" +
         policy.key;
}

}  // namespace

class CommunicationProxy final : public rclcpp::Node {
 public:
  CommunicationProxy()
      : Node("racer_sionna_communication_proxy"),
        mode_(declare_parameter<std::string>("mode", "sionna_hybrid")),
        drone_count_(declare_parameter<int>("drone_count", 5)),
        topology_(declare_parameter<std::string>("network_topology",
                                                 "distributed")),
        nearest_neighbor_count_(
            declare_parameter<int>("nearest_neighbor_count", 0)),
        lossless_nearest_neighbor_count_(
            declare_parameter<int>("lossless_nearest_neighbor_count", 0)),
        lossless_communication_range_m_(declare_parameter<double>(
            "lossless_communication_range_m", 0.0)),
        lossless_control_only_(
            declare_parameter<bool>("lossless_control_only", false)),
        directed_message_unicast_(
            declare_parameter<bool>("directed_message_unicast", false)),
        chunk_data_pre_enqueue_dedup_(declare_parameter<bool>(
            "chunk_data_pre_enqueue_dedup", false)),
        chunk_data_max_pending_per_link_(declare_parameter<int>(
            "chunk_data_max_pending_per_link", 0)),
        communication_range_m_(
            declare_parameter<double>("communication_range_m", 4.0)),
        ideal_coalesce_window_s_(
            1.0e-3 * declare_parameter<double>(
                           "ideal_coalesce_window_ms", 20.0)),
        active_link_hold_s_(
            declare_parameter<double>("active_link_hold_s", 1.0)),
        active_link_publish_period_s_(declare_parameter<double>(
            "active_link_publish_period_s", 0.02)),
        carrier_frequency_hz_(
            declare_parameter<double>("carrier_frequency_hz", 28.0e9)),
        bs_tx_power_dbm_(declare_parameter<double>("ap_tx_power_dbm", 33.0)),
        uav_tx_power_dbm_(declare_parameter<double>("tx_power_dbm", 23.0)),
        bs_array_rows_(declare_parameter<int>("ap_array_rows", 8)),
        bs_array_cols_(declare_parameter<int>("ap_array_cols", 8)),
        uav_array_rows_(declare_parameter<int>("uav_array_rows", 4)),
        uav_array_cols_(declare_parameter<int>("uav_array_cols", 4)),
        base_latency_s_(
            1.0e-3 * declare_parameter<double>("base_latency_ms", 20.0)),
        jitter_s_(1.0e-3 * declare_parameter<double>("jitter_ms", 10.0)),
        queue_capacity_bytes_(static_cast<std::size_t>(
            declare_parameter<int>("queue_capacity_bytes", 262144))),
        max_retries_(declare_parameter<int>("max_retries", 3)),
        bs_max_retries_(declare_parameter<int>("bs_max_retries", 3)),
        retry_backoff_s_(
            1.0e-3 * declare_parameter<double>("retry_backoff_ms", 8.0)),
        bs_min_turn_s_(1.0e-3 *
                       declare_parameter<double>("bs_min_turn_ms", 10.0)),
        bs_control_bytes_(static_cast<std::size_t>(
            declare_parameter<int>("bs_control_bytes", 32))),
        bs_max_downlink_chunks_per_turn_(declare_parameter<int>(
            "bs_max_downlink_chunks_per_turn", 32)),
        bs_max_uplink_chunks_per_turn_(declare_parameter<int>(
            "bs_max_uplink_chunks_per_turn", 32)),
        model_(LinkModelConfig{
            declare_parameter<double>("bandwidth_hz", 100.0e6),
            declare_parameter<double>("subcarrier_spacing_hz", 120.0e3),
            static_cast<int>(declare_parameter<int>("resource_blocks", 66)),
            declare_parameter<double>("data_re_efficiency", 0.82),
            declare_parameter<double>("target_initial_tbler", 0.10),
            declare_parameter<double>("tbler_slope_db", 1.35),
            static_cast<std::size_t>(
                declare_parameter<int>("transport_block_bytes", 1200)),
            static_cast<int>(
                declare_parameter<int>("fixed_mcs_index", -1))}) {
    random_seed_ = declare_parameter<int>("random_seed", 42);
    if (mode_ != "ideal" && mode_ != "sionna" &&
        mode_ != "sionna_hybrid") {
      throw std::runtime_error(
          "mode must be ideal, sionna, or sionna_hybrid");
    }
    if (topology_ != "distributed" && topology_ != "nearest_neighbors" &&
        topology_ != "distance_radius" && topology_ != "ap_assisted" &&
        topology_ != "bs_round_robin") {
      throw std::runtime_error(
          "network_topology must be distributed, nearest_neighbors, "
          "distance_radius, ap_assisted, or bs_round_robin");
    }
    if (drone_count_ < 1 || queue_capacity_bytes_ == 0U ||
        max_retries_ < 0 || bs_max_retries_ < 0 ||
        chunk_data_max_pending_per_link_ < 0 ||
        ideal_coalesce_window_s_ <= 0.0 || active_link_hold_s_ <= 0.0 ||
        active_link_publish_period_s_ <= 0.0 || base_latency_s_ < 0.0 ||
        jitter_s_ < 0.0 ||
        retry_backoff_s_ < 0.0 || bs_min_turn_s_ < 0.0 ||
        bs_control_bytes_ == 0U || bs_max_downlink_chunks_per_turn_ < 1 ||
        bs_max_uplink_chunks_per_turn_ < 1 || carrier_frequency_hz_ <= 0.0 ||
        bs_array_rows_ < 1 || bs_array_cols_ < 1 || uav_array_rows_ < 1 ||
        uav_array_cols_ < 1) {
      throw std::runtime_error("invalid communication proxy parameters");
    }
    nearest_neighbors_enabled_ = topology_ == "nearest_neighbors";
    distance_radius_enabled_ = topology_ == "distance_radius";
    if (nearest_neighbors_enabled_ &&
        (nearest_neighbor_count_ < 1 ||
         nearest_neighbor_count_ >= drone_count_)) {
      throw std::runtime_error(
          "nearest_neighbor_count must be in [1, drone_count - 1]");
    }
    if (distance_radius_enabled_ && communication_range_m_ <= 0.0) {
      throw std::runtime_error("communication_range_m must be positive");
    }
    ap_enabled_ = topology_ == "ap_assisted" ||
                  topology_ == "bs_round_robin";
    if (lossless_nearest_neighbor_count_ < 0 ||
        lossless_nearest_neighbor_count_ >= drone_count_) {
      throw std::runtime_error(
          "lossless_nearest_neighbor_count must be in [0, drone_count - 1]");
    }
    if ((lossless_nearest_neighbor_count_ > 0 ||
         lossless_communication_range_m_ > 0.0) &&
        (mode_ == "ideal" || topology_ != "distributed" || ap_enabled_)) {
      throw std::runtime_error(
          "lossless UAV-link overrides require a non-ideal "
          "distributed UAV-to-UAV topology");
    }
    if (!std::isfinite(lossless_communication_range_m_) ||
        lossless_communication_range_m_ < 0.0) {
      throw std::runtime_error(
          "lossless_communication_range_m must be finite and non-negative");
    }
    // AP-assisted modes model an actual two-hop radio path.  The lossless
    // distributed baselines instead forward serialized buffers directly.
    ideal_direct_enabled_ = mode_ == "ideal" && !ap_enabled_;
    bs_round_robin_enabled_ = topology_ == "bs_round_robin";
    ap_node_id_ = drone_count_;
    radio_node_count_ = drone_count_ + static_cast<int>(ap_enabled_);

    const auto data_qos = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable();
    publishers_.resize(kTopicPolicies.size());
    for (std::size_t topic_index = 0; topic_index < kTopicPolicies.size();
         ++topic_index) {
      const auto &policy = kTopicPolicies[topic_index];
      publishers_[topic_index].reserve(
          static_cast<std::size_t>(drone_count_));
      for (int receiver = 0; receiver < drone_count_; ++receiver) {
        publishers_[topic_index].push_back(create_generic_publisher(
            rxTopic(receiver, policy), policy.type, data_qos));
      }
      for (int sender = 0; sender < drone_count_; ++sender) {
        subscriptions_.push_back(create_generic_subscription(
            txTopic(sender, policy), policy.type, data_qos,
            [this, sender, topic_index](
                std::shared_ptr<rclcpp::SerializedMessage> message) {
              onTransmit(sender, topic_index, std::move(message));
            }));
      }
    }

    // Pre-create every physical queue. AP forwarding can then enqueue from a
    // scheduler callback without rehashing and invalidating the active queue.
    const auto directed_link_count = static_cast<std::size_t>(
        radio_node_count_ * std::max(0, radio_node_count_ - 1));
    queues_.reserve(directed_link_count);
    link_rngs_.reserve(directed_link_count);
    for (int sender = 0; sender < radio_node_count_; ++sender) {
      for (int receiver = 0; receiver < radio_node_count_; ++receiver) {
        if (sender == receiver) continue;
        const LinkKey key{sender, receiver};
        queues_.emplace(key, LinkQueue{});
        std::seed_seq seed{
            random_seed_, sender, receiver, 0x52414345, 0x53494f4e};
        link_rngs_.emplace(key, std::mt19937(seed));
      }
    }
    ap_latest_messages_.resize(static_cast<std::size_t>(drone_count_));
    for (auto &topics : ap_latest_messages_) {
      topics.resize(kTopicPolicies.size());
    }
    uav_chunks_.resize(static_cast<std::size_t>(drone_count_));
    uav_positions_.resize(static_cast<std::size_t>(drone_count_));
    uav_position_valid_.assign(static_cast<std::size_t>(drone_count_), false);
    odometry_subscriptions_.reserve(static_cast<std::size_t>(drone_count_));
    for (int drone = 0; drone < drone_count_; ++drone) {
      odometry_subscriptions_.push_back(
          create_subscription<nav_msgs::msg::Odometry>(
              "/drone_" + std::to_string(drone) + "/odom",
              rclcpp::SensorDataQoS(),
              [this, drone](nav_msgs::msg::Odometry::ConstSharedPtr message) {
                const auto &position = message->pose.pose.position;
                uav_positions_[static_cast<std::size_t>(drone)] =
                    {position.x, position.y, position.z};
                uav_position_valid_[static_cast<std::size_t>(drone)] = true;
              }));
    }

    if (ap_enabled_) {
      const auto status_index = topicIndex("recovery_status");
      const auto command_index = topicIndex("recovery_command");
      ap_recovery_status_publisher_ = create_generic_publisher(
          "/racer_ap_recovery/status_uplink",
          kTopicPolicies.at(status_index).type, data_qos);
      ap_recovery_command_subscription_ = create_generic_subscription(
          "/racer_ap_recovery/command_downlink",
          kTopicPolicies.at(command_index).type, data_qos,
          [this, command_index](
              std::shared_ptr<rclcpp::SerializedMessage> message) {
            onApRecoveryCommand(command_index, std::move(message));
          });
    }

    link_subscription_ = create_subscription<LinkQualityArray>(
        "/racer_sionna/link_quality", rclcpp::QoS(20).reliable(),
        [this](LinkQualityArray::ConstSharedPtr message) {
          onLinkQuality(*message);
        });
    statistics_publisher_ =
        create_publisher<racer_sionna_interfaces::msg::CommStatistics>(
            "/racer_sionna/comm_statistics", rclcpp::QoS(10).reliable());
    active_link_publisher_ = create_publisher<LinkQualityArray>(
        "/racer_sionna/active_links", rclcpp::QoS(20).reliable());
    scheduler_timer_ = create_wall_timer(
        std::chrono::milliseconds(2), [this]() { schedulerTick(); });
    active_link_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(active_link_publish_period_s_)),
        [this]() { publishActiveLinks(); });
    statistics_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this]() { publishStatistics(); });

    RCLCPP_INFO(
        get_logger(),
        "source-faithful communication proxy ready: mode=%s topology=%s "
        "drones=%d nearest_neighbors=%d lossless_nearest_neighbors=%d "
        "lossless_communication_range_m=%.3f communication_range_m=%.3f "
        "radio_nodes=%d bs_node_id=%d seed=%d phy=NR-LDPC "
        "waveform=CP-OFDM",
        mode_.c_str(), topology_.c_str(), drone_count_, nearest_neighbor_count_,
        lossless_nearest_neighbor_count_, lossless_communication_range_m_,
        communication_range_m_, radio_node_count_,
        ap_enabled_ ? ap_node_id_ : -1, random_seed_);
  }

 private:
  struct DeliveryFlow {
    int origin_sender{};
    std::size_t topic_index{};
    double born_at{};
    std::size_t bytes{};
    std::vector<bool> delivered;
    bool ap_received{false};
  };

  struct PendingPacket {
    std::shared_ptr<rclcpp::SerializedMessage> message;
    std::shared_ptr<DeliveryFlow> flow;
    std::size_t topic_index{};
    RouteStage route{RouteStage::kDirect};
    int final_receiver{-1};
    double born_at{};
    double enqueued_at{};
    double delivery_at{};
    std::size_t bytes{};
    int attempts{};
    bool transmitting{false};
    bool has_chunk_key{false};
    ChunkKey chunk_key{};
  };

  struct LinkQueue {
    std::deque<PendingPacket> packets;
    std::size_t bytes{};
  };

  struct CachedChunk {
    std::shared_ptr<rclcpp::SerializedMessage> message;
    std::size_t bytes{};
  };

  template <typename Message>
  bool deserialize(
      const std::shared_ptr<rclcpp::SerializedMessage> &serialized,
      Message &message) const {
    try {
      rclcpp::Serialization<Message> serializer;
      serializer.deserialize_message(serialized.get(), &message);
      return true;
    } catch (const std::exception &exception) {
      RCLCPP_WARN(get_logger(), "failed to inspect serialized map message: %s",
                  exception.what());
      return false;
    }
  }

  bool extractChunkKey(
      const std::shared_ptr<rclcpp::SerializedMessage> &message,
      ChunkKey &key) const {
    racer_fidelity_msgs::msg::ChunkData chunk;
    if (!deserialize(message, chunk) || chunk.chunk_drone_id < 1 ||
        chunk.chunk_drone_id > drone_count_ || chunk.idx == 0U) {
      return false;
    }
    key = {chunk.chunk_drone_id, chunk.idx};
    return true;
  }

  void observeChunkStamps(
      int sender,
      const std::shared_ptr<rclcpp::SerializedMessage> &message) {
    racer_fidelity_msgs::msg::ChunkStamps stamps;
    if (!deserialize(message, stamps) || sender < 0 ||
        sender >= drone_count_) {
      return;
    }
    auto &known = uav_chunks_[static_cast<std::size_t>(sender)];
    const auto owner_count = std::min(
        stamps.idx_lists.size(), static_cast<std::size_t>(drone_count_));
    for (std::size_t owner = 0; owner < owner_count; ++owner) {
      const auto &ranges = stamps.idx_lists[owner].ids;
      for (std::size_t offset = 0; offset + 1U < ranges.size(); offset += 2U) {
        const std::uint32_t first = ranges[offset];
        const std::uint32_t last = ranges[offset + 1U];
        if (first == 0U || last < first || last - first > 1000000U) continue;
        for (std::uint32_t index = first; index <= last; ++index) {
          known.insert({static_cast<int>(owner) + 1, index});
          if (index == std::numeric_limits<std::uint32_t>::max()) break;
        }
      }
    }
  }

  void observeUavTransmit(
      int sender, std::size_t topic_index,
      const std::shared_ptr<rclcpp::SerializedMessage> &message,
      ChunkKey *chunk_key) {
    if (kTopicPolicies[topic_index].key == "chunk_stamps") {
      observeChunkStamps(sender, message);
      return;
    }
    if (kTopicPolicies[topic_index].key != "chunk_data" ||
        !extractChunkKey(message, *chunk_key)) {
      return;
    }
    const std::size_t bytes = 64U + message->size();
    uav_chunks_[static_cast<std::size_t>(sender)].insert(*chunk_key);
    chunk_repository_.insert_or_assign(*chunk_key,
                                       CachedChunk{message, bytes});
  }

  void onLinkQuality(const LinkQualityArray &message) {
    for (const auto &link : message.links) {
      if (link.sender_id < 0 || link.sender_id >= radio_node_count_ ||
          link.receiver_id < 0 ||
          link.receiver_id >= radio_node_count_ ||
          link.sender_id == link.receiver_id) {
        continue;
      }
      links_[{link.sender_id, link.receiver_id}] = link;
      ++link_model_counts_[link.model];
    }
  }

  void onTransmit(int sender, std::size_t topic_index,
                  std::shared_ptr<rclcpp::SerializedMessage> message) {
    if (sender < 0 || sender >= drone_count_ ||
        topic_index >= kTopicPolicies.size()) {
      return;
    }
    const double stamp = now().seconds();
    const std::size_t bytes = 64U + message->size();
    const auto &topic_key = kTopicPolicies.at(topic_index).key;
    ChunkKey chunk_key{};
    observeUavTransmit(sender, topic_index, message, &chunk_key);
    const bool has_chunk_key = chunk_key.owner > 0 && chunk_key.index > 0U;

    if (ideal_direct_enabled_) {
      // Preserve source callback order: perfect mode forwards this complete
      // serialized logical message immediately, without queue or PER work.
      deliverIdealDirect(sender, topic_index, std::move(message), stamp,
                         has_chunk_key ? &chunk_key : nullptr);
      return;
    }

    auto flow = std::make_shared<DeliveryFlow>();
    flow->origin_sender = sender;
    flow->topic_index = topic_index;
    flow->born_at = stamp;
    flow->bytes = bytes;
    flow->delivered.assign(static_cast<std::size_t>(drone_count_), false);
    flow->delivered[static_cast<std::size_t>(sender)] = true;

    if (topic_key == "recovery_status") {
      ++logical_attempted_packets_;
      logical_attempted_bytes_ += bytes;
      if (ap_enabled_) {
        enqueue(sender, ap_node_id_, topic_index, message, flow,
                RouteStage::kApUplink, -1, stamp);
      }
      return;
    }
    if (topic_key == "recovery_command") return;

    const auto receivers = messageReceivers(sender, topic_index, message);
    const auto lossless_receivers =
        losslessEligible(topic_index)
            ? losslessNearestReceivers(sender, receivers)
            : std::vector<int>{};
    std::vector<bool> lossless_receiver_mask(
        static_cast<std::size_t>(drone_count_), false);
    for (const int receiver : lossless_receivers) {
      lossless_receiver_mask[static_cast<std::size_t>(receiver)] = true;
    }
    const auto intended_receivers =
        static_cast<std::uint64_t>(receivers.size());
    logical_attempted_packets_ += intended_receivers;
    logical_attempted_bytes_ += intended_receivers * bytes;
    for (const int receiver : receivers) {
      if (lossless_receiver_mask[static_cast<std::size_t>(receiver)]) {
        deliverLosslessNearest(
            sender, receiver, topic_index, message, flow, stamp,
            has_chunk_key ? &chunk_key : nullptr);
        continue;
      }
      enqueue(sender, receiver, topic_index, message, flow,
              RouteStage::kDirect, receiver, stamp,
              has_chunk_key ? &chunk_key : nullptr);
    }
    if (ap_enabled_ && !bs_round_robin_enabled_) {
      enqueue(sender, ap_node_id_, topic_index, message, flow,
              RouteStage::kApUplink, -1, stamp,
              has_chunk_key ? &chunk_key : nullptr);
    }
  }

  void deliverLosslessNearest(
      int sender, int receiver, std::size_t topic_index,
      const std::shared_ptr<rclcpp::SerializedMessage> &message,
      const std::shared_ptr<DeliveryFlow> &flow, double born_at,
      const ChunkKey *chunk_key = nullptr) {
    const double stamp = now().seconds();
    const std::size_t bytes = 64U + (message ? message->size() : 0U);
    ++attempted_packets_;
    attempted_bytes_ += bytes;
    ++direct_attempted_packets_;
    PendingPacket packet;
    packet.message = message;
    packet.flow = flow;
    packet.topic_index = topic_index;
    packet.route = RouteStage::kDirect;
    packet.final_receiver = receiver;
    packet.born_at = born_at;
    packet.enqueued_at = stamp;
    packet.delivery_at = stamp;
    packet.bytes = bytes;
    if (chunk_key != nullptr) {
      packet.has_chunk_key = true;
      packet.chunk_key = *chunk_key;
    }
    completeSuccessfulPacket({sender, receiver}, packet, stamp);
    ++lossless_nearest_forwarded_packets_;
    if (lossless_communication_range_m_ > 0.0 &&
        uav_position_valid_[static_cast<std::size_t>(sender)] &&
        uav_position_valid_[static_cast<std::size_t>(receiver)]) {
      const auto &source = uav_positions_[static_cast<std::size_t>(sender)];
      const auto &target = uav_positions_[static_cast<std::size_t>(receiver)];
      const double dx = source[0] - target[0];
      const double dy = source[1] - target[1];
      const double dz = source[2] - target[2];
      if (dx * dx + dy * dy + dz * dz <=
          lossless_communication_range_m_ *
                  lossless_communication_range_m_ +
              1.0e-12) {
        ++lossless_range_forwarded_packets_;
      }
    }
  }

  void deliverIdealDirect(
      int sender, std::size_t topic_index,
      std::shared_ptr<rclcpp::SerializedMessage> message, double born_at,
      const ChunkKey *chunk_key = nullptr) {
    const auto &topic_key = kTopicPolicies.at(topic_index).key;
    // Preserve the existing recovery routing contract: status is consumed by
    // the AP only, and commands originate at the AP.
    if (topic_key == "recovery_status" || topic_key == "recovery_command") {
      return;
    }
    const auto profile_started = std::chrono::steady_clock::now();
    const double stamp = now().seconds();
    const std::size_t bytes = 64U + (message ? message->size() : 0U);
    const auto receivers = messageReceivers(sender, topic_index, message);
    ++ideal_logical_messages_;
    logical_attempted_packets_ += receivers.size();
    logical_attempted_bytes_ += receivers.size() * bytes;

    for (const int receiver : receivers) {
      ++attempted_packets_;
      attempted_bytes_ += bytes;
      ++direct_attempted_packets_;
      ++delivered_packets_;
      delivered_bytes_ += bytes;
      ++direct_delivered_packets_;
      ideal_statistical_transport_blocks_ +=
          model_.transportBlockCount(bytes);
      ideal_statistical_bytes_ += bytes;
      const auto receiver_index = static_cast<std::size_t>(receiver);
      if (chunk_key != nullptr &&
          uav_chunks_[receiver_index].find(*chunk_key) !=
              uav_chunks_[receiver_index].end()) {
        ++duplicates_suppressed_;
        ++ideal_direct_forwarded_packets_;
        continue;
      }
      publishers_[topic_index][receiver_index]->publish(*message);
      if (chunk_key != nullptr) uav_chunks_[receiver_index].insert(*chunk_key);
      ++logical_delivered_packets_;
      logical_delivered_bytes_ += bytes;
      cumulative_end_to_end_delay_s_ += stamp - born_at;
      ++direct_delivery_wins_;
      ++ideal_direct_forwarded_packets_;
    }
    const double elapsed_ms = 1000.0 * std::chrono::duration<double>(
        std::chrono::steady_clock::now() - profile_started).count();
    ++perfect_forward_profile_calls_;
    perfect_forward_profile_receivers_ += receivers.size();
    perfect_forward_profile_total_ms_ += elapsed_ms;
    perfect_forward_profile_max_ms_ =
        std::max(perfect_forward_profile_max_ms_, elapsed_ms);
  }

  void markActiveLink(int sender, int receiver) {
    if (mode_ == "ideal" || sender < 0 || receiver < 0 ||
        sender >= radio_node_count_ || receiver >= radio_node_count_ ||
        sender == receiver) {
      return;
    }
    const LinkKey key{sender, receiver};
    if (active_links_.find(key) == active_links_.end()) {
      active_links_dirty_ = true;
    }
    active_links_[key] = std::chrono::steady_clock::now();
  }

  void publishActiveLinks() {
    if (mode_ == "ideal" || !active_link_publisher_) return;
    const auto wall_now = std::chrono::steady_clock::now();
    // A queued packet keeps its link active even if no new logical message
    // arrived during the hold interval.
    for (const auto &[key, queue] : queues_) {
      if (queue.packets.empty()) continue;
      if (active_links_.find(key) == active_links_.end()) {
        active_links_dirty_ = true;
      }
      active_links_[key] = wall_now;
    }
    std::vector<LinkKey> requested_links;
    requested_links.reserve(active_links_.size());
    for (auto iterator = active_links_.begin();
         iterator != active_links_.end();) {
      const double age_s = std::chrono::duration<double>(
          wall_now - iterator->second).count();
      if (age_s > active_link_hold_s_) {
        iterator = active_links_.erase(iterator);
        active_links_dirty_ = true;
        continue;
      }
      requested_links.push_back(iterator->first);
      ++iterator;
    }
    if (requested_links.empty()) return;
    // The 20 ms timer gives a newly active link low request latency. Once the
    // set is stable, a half-TTL keepalive is sufficient and avoids publishing
    // thousands of identical 90-link arrays during slow-wall-clock runs.
    const double keepalive_s = std::min(0.5, 0.5 * active_link_hold_s_);
    if (!active_links_dirty_ && have_active_link_publish_wall_ &&
        std::chrono::duration<double>(wall_now - last_active_link_publish_wall_)
                .count() < keepalive_s) {
      return;
    }
    LinkQualityArray request;
    request.stamp = now();
    request.links.reserve(requested_links.size());
    for (const auto &key : requested_links) {
      LinkQuality link;
      link.stamp = request.stamp;
      link.sender_id = key.sender;
      link.receiver_id = key.receiver;
      request.links.push_back(std::move(link));
    }
    active_link_publisher_->publish(request);
    active_links_dirty_ = false;
    have_active_link_publish_wall_ = true;
    last_active_link_publish_wall_ = wall_now;
    ++active_link_publications_;
    active_link_samples_published_ += request.links.size();
  }

  std::vector<int> intendedReceivers(int sender) {
    std::vector<int> receivers;
    receivers.reserve(static_cast<std::size_t>(std::max(0, drone_count_ - 1)));
    for (int receiver = 0; receiver < drone_count_; ++receiver) {
      if (receiver != sender) receivers.push_back(receiver);
    }
    if (!nearest_neighbors_enabled_ && !distance_radius_enabled_) {
      return receivers;
    }
    if (!uav_position_valid_[static_cast<std::size_t>(sender)]) {
      position_unavailable_receivers_ += receivers.size();
      return {};
    }
    const auto &source = uav_positions_[static_cast<std::size_t>(sender)];
    receivers.erase(
        std::remove_if(receivers.begin(), receivers.end(),
                       [this](int receiver) {
                         return !uav_position_valid_[
                             static_cast<std::size_t>(receiver)];
                       }),
        receivers.end());
    if (distance_radius_enabled_) {
      const double range_squared = communication_range_m_ * communication_range_m_;
      const auto before_filter = receivers.size();
      receivers.erase(
          std::remove_if(receivers.begin(), receivers.end(),
                         [this, &source, range_squared](int receiver) {
                           const auto &target = uav_positions_[
                               static_cast<std::size_t>(receiver)];
                           const double dx = source[0] - target[0];
                           const double dy = source[1] - target[1];
                           const double dz = source[2] - target[2];
                           return dx * dx + dy * dy + dz * dz >
                                  range_squared;
                         }),
          receivers.end());
      range_filtered_receivers_ += before_filter - receivers.size();
      return receivers;
    }
    std::sort(receivers.begin(), receivers.end(),
              [this, &source](int left, int right) {
                const auto squared_distance = [&source](const auto &point) {
                  const double dx = source[0] - point[0];
                  const double dy = source[1] - point[1];
                  const double dz = source[2] - point[2];
                  return dx * dx + dy * dy + dz * dz;
                };
                const double left_distance = squared_distance(
                    uav_positions_[static_cast<std::size_t>(left)]);
                const double right_distance = squared_distance(
                    uav_positions_[static_cast<std::size_t>(right)]);
                if (left_distance != right_distance) {
                  return left_distance < right_distance;
                }
                return left < right;
              });
    if (receivers.size() > static_cast<std::size_t>(nearest_neighbor_count_)) {
      nearest_filtered_receivers_ +=
          receivers.size() - static_cast<std::size_t>(nearest_neighbor_count_);
      receivers.resize(static_cast<std::size_t>(nearest_neighbor_count_));
    }
    return receivers;
  }

  int directedReceiver(
      int sender, std::size_t topic_index,
      const std::shared_ptr<rclcpp::SerializedMessage> &message) const {
    if (!directed_message_unicast_) return -1;
    const auto &key = kTopicPolicies.at(topic_index).key;
    int to_drone_id = 0;
    if (key == "pair_opt") {
      racer_fidelity_msgs::msg::PairOpt directed;
      if (!deserialize(message, directed)) return -2;
      to_drone_id = directed.to_drone_id;
    } else if (key == "pair_opt_res") {
      racer_fidelity_msgs::msg::PairOptResponse directed;
      if (!deserialize(message, directed)) return -2;
      to_drone_id = directed.to_drone_id;
    } else if (key == "chunk_data") {
      racer_fidelity_msgs::msg::ChunkData directed;
      if (!deserialize(message, directed)) return -2;
      // The separate oracle algorithm uses zero for an intentional fan-out.
      if (directed.to_drone_id == 0) return -1;
      to_drone_id = directed.to_drone_id;
    } else {
      return -1;
    }
    const int receiver = to_drone_id - 1;
    if (receiver < 0 || receiver >= drone_count_ || receiver == sender) {
      return -2;
    }
    return receiver;
  }

  std::vector<int> messageReceivers(
      int sender, std::size_t topic_index,
      const std::shared_ptr<rclcpp::SerializedMessage> &message) {
    const int receiver = directedReceiver(sender, topic_index, message);
    if (receiver == -2) {
      ++invalid_directed_messages_dropped_;
      return {};
    }
    if (receiver >= 0) {
      ++directed_unicast_messages_;
      return {receiver};
    }
    return intendedReceivers(sender);
  }

  bool losslessEligible(std::size_t topic_index) const {
    if (!lossless_control_only_) return true;
    const auto &key = kTopicPolicies.at(topic_index).key;
    return key == "drone_state" || key == "trajectory" ||
           key == "pair_opt" || key == "pair_opt_res" ||
           key == "global_assignment";
  }

  std::vector<int> losslessNearestReceivers(
      int sender, const std::vector<int> &candidates) {
    if ((lossless_nearest_neighbor_count_ <= 0 &&
         lossless_communication_range_m_ <= 0.0) || candidates.empty()) {
      return {};
    }
    if (!uav_position_valid_[static_cast<std::size_t>(sender)]) {
      ++lossless_nearest_position_unavailable_events_;
      return {};
    }
    const auto &source = uav_positions_[static_cast<std::size_t>(sender)];
    std::vector<int> receivers;
    receivers.reserve(static_cast<std::size_t>(drone_count_ - 1));
    for (int receiver = 0; receiver < drone_count_; ++receiver) {
      if (receiver == sender) continue;
      if (!uav_position_valid_[static_cast<std::size_t>(receiver)]) {
        ++lossless_nearest_position_unavailable_events_;
        return {};
      }
      receivers.push_back(receiver);
    }
    std::sort(receivers.begin(), receivers.end(),
              [this, &source](int left, int right) {
                const auto squared_distance = [&source](const auto &point) {
                  const double dx = source[0] - point[0];
                  const double dy = source[1] - point[1];
                  const double dz = source[2] - point[2];
                  return dx * dx + dy * dy + dz * dz;
                };
                const double left_distance = squared_distance(
                    uav_positions_[static_cast<std::size_t>(left)]);
                const double right_distance = squared_distance(
                    uav_positions_[static_cast<std::size_t>(right)]);
                if (left_distance != right_distance) {
                  return left_distance < right_distance;
                }
                return left < right;
              });
    std::unordered_set<int> selected;
    const auto nearest_limit = std::min(
        receivers.size(),
        static_cast<std::size_t>(lossless_nearest_neighbor_count_));
    for (std::size_t index = 0; index < nearest_limit; ++index) {
      selected.insert(receivers[index]);
    }
    if (lossless_communication_range_m_ > 0.0) {
      const double range_squared = lossless_communication_range_m_ *
                                   lossless_communication_range_m_;
      for (const int receiver : receivers) {
        const auto &target =
            uav_positions_[static_cast<std::size_t>(receiver)];
        const double dx = source[0] - target[0];
        const double dy = source[1] - target[1];
        const double dz = source[2] - target[2];
        if (dx * dx + dy * dy + dz * dz <= range_squared + 1.0e-12) {
          selected.insert(receiver);
        }
      }
    }
    receivers.erase(
        std::remove_if(receivers.begin(), receivers.end(),
                       [&candidates, &selected](int receiver) {
                         if (selected.find(receiver) == selected.end()) {
                           return true;
                         }
                         return std::find(candidates.begin(), candidates.end(),
                                          receiver) == candidates.end();
                       }),
        receivers.end());
    return receivers;
  }

  std::size_t topicIndex(const std::string &key) const {
    for (std::size_t index = 0; index < kTopicPolicies.size(); ++index) {
      if (kTopicPolicies[index].key == key) return index;
    }
    throw std::logic_error("missing communication topic policy: " + key);
  }

  void onApRecoveryCommand(
      std::size_t topic_index,
      std::shared_ptr<rclcpp::SerializedMessage> message) {
    if (!ap_enabled_ || !message) return;
    racer_recovery_core::msg::RecoveryCommand command;
    if (!deserialize(message, command) || command.drone_id < 1 ||
        command.drone_id > drone_count_) {
      RCLCPP_WARN(get_logger(), "drop malformed AP recovery command");
      return;
    }
    const int receiver = command.drone_id - 1;
    const double stamp = now().seconds();
    const std::size_t bytes = 64U + message->size();
    auto flow = std::make_shared<DeliveryFlow>();
    flow->origin_sender = ap_node_id_;
    flow->topic_index = topic_index;
    flow->born_at = stamp;
    flow->bytes = bytes;
    flow->delivered.assign(static_cast<std::size_t>(drone_count_), false);
    ++logical_attempted_packets_;
    logical_attempted_bytes_ += bytes;
    enqueue(ap_node_id_, receiver, topic_index, std::move(message), flow,
            RouteStage::kApDownlink, receiver, stamp);
  }

  int packetPriority(RouteStage route, std::size_t topic_index) const {
    if (route == RouteStage::kBsControl) return 100;
    return kTopicPolicies.at(topic_index).priority;
  }

  bool enqueue(int sender, int receiver, std::size_t topic_index,
               const std::shared_ptr<rclcpp::SerializedMessage> &message,
               const std::shared_ptr<DeliveryFlow> &flow, RouteStage route,
               int final_receiver, double born_at,
               const ChunkKey *chunk_key = nullptr,
               std::size_t override_bytes = 0U) {
    markActiveLink(sender, receiver);
    const double stamp = now().seconds();
    const std::size_t bytes = override_bytes > 0U
                                  ? override_bytes
                                  : 64U + (message ? message->size() : 0U);
    const LinkKey key{sender, receiver};
    auto found = queues_.find(key);
    if (found == queues_.end()) {
      ++dropped_queue_;
      return false;
    }
    auto &queue = found->second;
    const bool chunk_data =
        chunk_key != nullptr &&
        kTopicPolicies.at(topic_index).key == "chunk_data";
    if (chunk_data) {
      const bool receiver_has_chunk =
          receiver >= 0 && receiver < drone_count_ &&
          uav_chunks_[static_cast<std::size_t>(receiver)].find(*chunk_key) !=
              uav_chunks_[static_cast<std::size_t>(receiver)].end();
      const bool already_pending = std::any_of(
          queue.packets.begin(), queue.packets.end(),
          [chunk_key](const PendingPacket &packet) {
            return packet.has_chunk_key && packet.chunk_key == *chunk_key;
          });
      if (chunk_data_pre_enqueue_dedup_ &&
          (receiver_has_chunk || already_pending)) {
        ++chunk_enqueue_duplicates_suppressed_;
        return false;
      }
      if (chunk_data_max_pending_per_link_ > 0) {
        const auto pending_chunks = std::count_if(
            queue.packets.begin(), queue.packets.end(),
            [](const PendingPacket &packet) {
              return packet.has_chunk_key &&
                     kTopicPolicies.at(packet.topic_index).key == "chunk_data";
            });
        if (pending_chunks >= chunk_data_max_pending_per_link_) {
          ++chunk_enqueue_budget_drops_;
          return false;
        }
      }
    }
    ++attempted_packets_;
    attempted_bytes_ += bytes;
    if (route == RouteStage::kDirect) {
      ++direct_attempted_packets_;
    } else if (route == RouteStage::kApUplink) {
      ++ap_uplink_attempted_packets_;
    } else if (route == RouteStage::kApDownlink) {
      ++ap_downlink_attempted_packets_;
    } else if (route == RouteStage::kBsControl) {
      ++bs_control_attempted_packets_;
    } else if (route == RouteStage::kBsUplink) {
      ++bs_uplink_attempted_packets_;
    } else {
      ++bs_downlink_attempted_packets_;
    }
    if (bytes > queue_capacity_bytes_) {
      ++dropped_queue_;
      return false;
    }

    const int incoming_priority = packetPriority(route, topic_index);
    while (queue.bytes + bytes > queue_capacity_bytes_) {
      auto candidate = queue.packets.end();
      for (auto iterator = queue.packets.end();
           iterator != queue.packets.begin();) {
        --iterator;
        if (!iterator->transmitting &&
            packetPriority(iterator->route, iterator->topic_index) <
                incoming_priority) {
          candidate = iterator;
          break;
        }
      }
      if (candidate == queue.packets.end()) {
        ++dropped_queue_;
        return false;
      }
      queue.bytes -= std::min(queue.bytes, candidate->bytes);
      queue.packets.erase(candidate);
      ++dropped_queue_;
    }

    PendingPacket pending;
    pending.message = message;
    pending.flow = flow;
    pending.topic_index = topic_index;
    pending.route = route;
    pending.final_receiver = final_receiver;
    pending.born_at = born_at;
    pending.enqueued_at = stamp;
    pending.bytes = bytes;
    if (chunk_key != nullptr) {
      pending.has_chunk_key = true;
      pending.chunk_key = *chunk_key;
    }
    auto position = queue.packets.begin();
    if (position != queue.packets.end() && position->transmitting) {
      ++position;
    }
    while (position != queue.packets.end() &&
           packetPriority(position->route, position->topic_index) >=
               incoming_priority) {
      ++position;
    }
    queue.packets.insert(position, std::move(pending));
    queue.bytes += bytes;
    return true;
  }

  std::size_t chunkDataTopicIndex() const {
    for (std::size_t index = 0; index < kTopicPolicies.size(); ++index) {
      if (kTopicPolicies[index].key == "chunk_data") return index;
    }
    throw std::logic_error("chunk_data topic policy is missing");
  }

  std::vector<ChunkKey> sortedChunks(
      const std::unordered_set<ChunkKey, ChunkKeyHash> &available,
      const std::unordered_set<ChunkKey, ChunkKeyHash> &excluded) const {
    std::vector<ChunkKey> output;
    output.reserve(available.size());
    for (const auto &key : available) {
      if (excluded.find(key) == excluded.end() &&
          chunk_repository_.find(key) != chunk_repository_.end()) {
        output.push_back(key);
      }
    }
    std::sort(output.begin(), output.end(),
              [](const ChunkKey &left, const ChunkKey &right) {
                if (left.owner != right.owner) return left.owner < right.owner;
                return left.index < right.index;
              });
    return output;
  }

  std::shared_ptr<DeliveryFlow> makeCentralFlow(
      int origin, std::size_t topic_index, std::size_t bytes,
      double stamp) const {
    auto flow = std::make_shared<DeliveryFlow>();
    flow->origin_sender = origin;
    flow->topic_index = topic_index;
    flow->born_at = stamp;
    flow->bytes = bytes;
    flow->delivered.assign(static_cast<std::size_t>(drone_count_), false);
    return flow;
  }

  void beginBsTurn(double stamp) {
    if (!bs_round_robin_enabled_ || bs_turn_active_) return;
    bs_active_uav_ = bs_next_uav_;
    bs_next_uav_ = (bs_next_uav_ + 1) % drone_count_;
    bs_turn_active_ = true;
    bs_turn_started_at_ = stamp;
    bs_control_finished_ = false;
    ++bs_round_robin_turns_;
    ++bs_uav_turns_[bs_active_uav_];

    const bool control_enqueued = enqueue(
        ap_node_id_, bs_active_uav_, 0U, nullptr, nullptr,
        RouteStage::kBsControl, bs_active_uav_, stamp, nullptr,
        bs_control_bytes_);
    if (!control_enqueued) bs_control_finished_ = true;

    const auto candidates = sortedChunks(
        bs_chunks_, uav_chunks_[static_cast<std::size_t>(bs_active_uav_)]);
    const std::size_t limit = std::min(
        candidates.size(),
        static_cast<std::size_t>(bs_max_downlink_chunks_per_turn_));
    const auto topic_index = chunkDataTopicIndex();
    for (std::size_t offset = 0; offset < limit; ++offset) {
      const auto &key = candidates[offset];
      const auto &cached = chunk_repository_.at(key);
      auto flow = makeCentralFlow(ap_node_id_, topic_index, cached.bytes, stamp);
      if (enqueue(ap_node_id_, bs_active_uav_, topic_index, cached.message,
                  flow, RouteStage::kBsDownlink, bs_active_uav_, stamp,
                  &key)) {
        ++bs_missing_chunks_scheduled_downlink_;
      }
    }
  }

  void scheduleBsUpload(double stamp) {
    if (!bs_turn_active_ || bs_active_uav_ < 0) return;
    const auto candidates = sortedChunks(
        uav_chunks_[static_cast<std::size_t>(bs_active_uav_)], bs_chunks_);
    const std::size_t limit = std::min(
        candidates.size(),
        static_cast<std::size_t>(bs_max_uplink_chunks_per_turn_));
    const auto topic_index = chunkDataTopicIndex();
    for (std::size_t offset = 0; offset < limit; ++offset) {
      const auto &key = candidates[offset];
      const auto &cached = chunk_repository_.at(key);
      auto flow = makeCentralFlow(bs_active_uav_, topic_index, cached.bytes,
                                  stamp);
      if (enqueue(bs_active_uav_, ap_node_id_, topic_index, cached.message,
                  flow, RouteStage::kBsUplink, -1, stamp, &key)) {
        ++bs_incremental_chunks_scheduled_uplink_;
      }
    }
  }

  bool bsTurnQueuesEmpty() const {
    if (!bs_turn_active_ || bs_active_uav_ < 0) return true;
    const auto downlink = queues_.find({ap_node_id_, bs_active_uav_});
    const auto uplink = queues_.find({bs_active_uav_, ap_node_id_});
    return (downlink == queues_.end() || downlink->second.packets.empty()) &&
           (uplink == queues_.end() || uplink->second.packets.empty());
  }

  void finishBsPacket(const PendingPacket &packet, bool delivered,
                      double stamp) {
    if (packet.route == RouteStage::kBsControl) {
      bs_control_finished_ = true;
      if (delivered) {
        ++bs_upload_grants_delivered_;
        scheduleBsUpload(stamp);
      } else {
        ++bs_upload_grants_failed_;
      }
    }
  }

  void maybeFinishBsTurn(double stamp) {
    if (!bs_turn_active_ || !bs_control_finished_ ||
        stamp - bs_turn_started_at_ + 1.0e-12 < bs_min_turn_s_ ||
        !bsTurnQueuesEmpty()) {
      return;
    }
    bs_turn_active_ = false;
    bs_active_uav_ = -1;
  }

  bool bsLinkMayTransmit(const LinkKey &key) const {
    if (!bs_round_robin_enabled_ ||
        (key.sender != ap_node_id_ && key.receiver != ap_node_id_)) {
      return true;
    }
    if (!bs_turn_active_ || bs_active_uav_ < 0) return false;
    const int uav = key.sender == ap_node_id_ ? key.receiver : key.sender;
    return uav == bs_active_uav_;
  }

  const LinkQuality *linkFor(const LinkKey &key, double stamp) const {
    const auto found = links_.find(key);
    if (found == links_.end()) return nullptr;
    if (timeSeconds(found->second.valid_until) + 1.0e-9 < stamp) {
      return nullptr;
    }
    return &found->second;
  }

  bool usable(const LinkQuality *link) const {
    if (mode_ == "ideal") return true;
    return link != nullptr && std::isfinite(link->snr_db) &&
           link->model != "unavailable";
  }

  double snr(const LinkQuality *link) const {
    return mode_ == "ideal" ? 40.0 : static_cast<double>(link->snr_db);
  }

  bool expired(const PendingPacket &packet, double stamp) const {
    const double ttl = kTopicPolicies[packet.topic_index].ttl_s;
    return ttl > 0.0 && stamp - packet.born_at > ttl;
  }

  void removeFront(LinkQueue &queue) {
    if (queue.packets.empty()) return;
    queue.bytes -= std::min(queue.bytes, queue.packets.front().bytes);
    queue.packets.pop_front();
  }

  std::mt19937 &linkRng(const LinkKey &key) {
    return link_rngs_.at(key);
  }

  bool startFront(const LinkKey &key, LinkQueue &queue, double stamp,
                  double transmission_cursor) {
    if (queue.packets.empty()) return false;
    auto &front = queue.packets.front();
    if (front.transmitting || front.delivery_at > stamp) return true;
    const auto *link = linkFor(key, stamp);
    if (!usable(link)) {
      const PendingPacket failed = front;
      finishBsPacket(failed, false, stamp);
      removeFront(queue);
      ++dropped_no_link_;
      return false;
    }
    const double link_snr = snr(link);
    ++mcs_counts_[std::string(model_.selectMcs(link_snr).name)];
    cumulative_initial_tbler_ += model_.transportBlockErrorRate(link_snr);
    ++initial_tbler_samples_;
    const double random_jitter =
        jitter_s_ <= 0.0
            ? 0.0
            : std::uniform_real_distribution<double>(-jitter_s_, jitter_s_)(
                  linkRng(key));
    const double transmission_started_at =
        std::max(transmission_cursor, front.delivery_at);
    front.delivery_at =
        transmission_started_at +
        model_.serializationDelay(link_snr, front.bytes) +
        std::max(0.0, base_latency_s_ + random_jitter);
    front.transmitting = true;
    return true;
  }

  void deliverToUav(const PendingPacket &packet, int receiver,
                    double stamp) {
    if (!packet.flow || receiver < 0 || receiver >= drone_count_) return;
    const auto receiver_index = static_cast<std::size_t>(receiver);
    if (packet.has_chunk_key &&
        uav_chunks_[receiver_index].find(packet.chunk_key) !=
            uav_chunks_[receiver_index].end()) {
      ++duplicates_suppressed_;
      return;
    }
    if (packet.flow->delivered[receiver_index]) {
      ++duplicates_suppressed_;
      return;
    }
    publishers_[packet.topic_index][static_cast<std::size_t>(receiver)]->
        publish(*packet.message);
    packet.flow->delivered[receiver_index] = true;
    if (packet.has_chunk_key) {
      uav_chunks_[receiver_index].insert(packet.chunk_key);
    }
    if (packet.route != RouteStage::kBsDownlink) {
      ++logical_delivered_packets_;
      logical_delivered_bytes_ += packet.bytes;
      cumulative_end_to_end_delay_s_ += stamp - packet.flow->born_at;
    }
    if (packet.route == RouteStage::kApDownlink) {
      ++ap_relay_wins_;
    } else if (packet.route == RouteStage::kBsDownlink) {
      ++bs_missing_chunks_delivered_downlink_;
    } else {
      ++direct_delivery_wins_;
    }
  }

  void completeSuccessfulPacket(const LinkKey &key,
                                const PendingPacket &packet,
                                double stamp) {
    ++delivered_packets_;
    delivered_bytes_ += packet.bytes;
    cumulative_delay_s_ += stamp - packet.enqueued_at;
    if (packet.route == RouteStage::kDirect) {
      ++direct_delivered_packets_;
      deliverToUav(packet, packet.final_receiver, stamp);
      return;
    }
    if (packet.route == RouteStage::kApDownlink) {
      ++ap_downlink_delivered_packets_;
      deliverToUav(packet, packet.final_receiver, stamp);
      return;
    }

    if (packet.route == RouteStage::kBsControl) {
      ++bs_control_delivered_packets_;
      finishBsPacket(packet, true, stamp);
      return;
    }
    if (packet.route == RouteStage::kBsDownlink) {
      ++bs_downlink_delivered_packets_;
      deliverToUav(packet, packet.final_receiver, stamp);
      return;
    }
    if (packet.route == RouteStage::kBsUplink) {
      ++bs_uplink_delivered_packets_;
      if (packet.has_chunk_key &&
          bs_chunks_.insert(packet.chunk_key).second) {
        ++bs_incremental_chunks_received_uplink_;
      }
      return;
    }

    ++ap_uplink_delivered_packets_;
    if (kTopicPolicies.at(packet.topic_index).key == "recovery_status") {
      if (ap_recovery_status_publisher_ && packet.message) {
        ap_recovery_status_publisher_->publish(*packet.message);
        ++ap_global_updates_received_;
        ++logical_delivered_packets_;
        logical_delivered_bytes_ += packet.bytes;
        if (packet.flow) {
          cumulative_end_to_end_delay_s_ +=
              stamp - packet.flow->born_at;
        }
      }
      return;
    }
    if (!packet.flow || packet.flow->ap_received) return;
    packet.flow->ap_received = true;
    ++ap_global_updates_received_;
    ap_latest_messages_[static_cast<std::size_t>(
        packet.flow->origin_sender)][packet.topic_index] = packet.message;
    for (int receiver = 0; receiver < drone_count_; ++receiver) {
      if (receiver == packet.flow->origin_sender ||
          packet.flow->delivered[static_cast<std::size_t>(receiver)]) {
        continue;
      }
      if (enqueue(ap_node_id_, receiver, packet.topic_index,
                  packet.message, packet.flow, RouteStage::kApDownlink,
                  receiver, packet.flow->born_at,
                  packet.has_chunk_key ? &packet.chunk_key : nullptr)) {
        ++ap_selective_forwards_enqueued_;
      }
    }
    (void)key;
  }

  void schedulerTick() {
    const double stamp = now().seconds();
    beginBsTurn(stamp);
    for (auto &[key, queue] : queues_) {
      if (!bsLinkMayTransmit(key)) continue;
      double transmission_cursor = stamp;
      while (!queue.packets.empty()) {
        if (!startFront(key, queue, stamp, transmission_cursor)) continue;
        auto &front = queue.packets.front();
        if (front.delivery_at > stamp) break;
        if (expired(front, stamp)) {
          const PendingPacket failed = front;
          finishBsPacket(failed, false, stamp);
          ++dropped_ttl_;
          removeFront(queue);
          continue;
        }
        const auto *link = linkFor(key, stamp);
        if (!usable(link)) {
          const PendingPacket failed = front;
          finishBsPacket(failed, false, stamp);
          ++dropped_no_link_;
          removeFront(queue);
          continue;
        }
        const double per =
            mode_ == "ideal"
                ? 0.0
                : model_.packetErrorRate(snr(link), front.bytes);
        const bool failed =
            std::uniform_real_distribution<double>(0.0, 1.0)(linkRng(key)) <
            per;
        const bool reliable = front.route == RouteStage::kBsControl ||
                              kTopicPolicies[front.topic_index].reliable;
        const bool bs_route = front.route == RouteStage::kBsControl ||
                              front.route == RouteStage::kBsUplink ||
                              front.route == RouteStage::kBsDownlink;
        const int retry_limit = bs_route ? bs_max_retries_ : max_retries_;
        if (failed && reliable && front.attempts < retry_limit) {
          ++front.attempts;
          ++retried_packets_;
          front.transmitting = false;
          front.delivery_at = stamp + retry_backoff_s_;
          break;
        }
        if (failed) {
          const PendingPacket failed_packet = front;
          transmission_cursor = failed_packet.delivery_at;
          finishBsPacket(failed_packet, false, stamp);
          ++dropped_per_;
          removeFront(queue);
          continue;
        }
        // Copy the front because a successful AP uplink can append packets to
        // other queues before this physical packet is removed.
        const PendingPacket completed = front;
        transmission_cursor = completed.delivery_at;
        completeSuccessfulPacket(key, completed, stamp);
        removeFront(queue);
      }
    }
    maybeFinishBsTurn(stamp);
  }

  void publishStatistics() {
    racer_sionna_interfaces::msg::CommStatistics message;
    message.stamp = now();
    message.attempted_packets = attempted_packets_;
    message.delivered_packets = delivered_packets_;
    message.dropped_no_link = dropped_no_link_;
    message.dropped_per = dropped_per_;
    message.dropped_queue = dropped_queue_;
    message.dropped_ttl = dropped_ttl_;
    message.retried_packets = retried_packets_;
    message.attempted_bytes = attempted_bytes_;
    message.delivered_bytes = delivered_bytes_;
    for (const auto &[key, queue] : queues_) {
      (void)key;
      message.queued_packets += queue.packets.size();
      message.queued_bytes += queue.bytes;
    }
    message.mean_delivery_delay_ms =
        delivered_packets_ == 0U
            ? 0.0F
            : static_cast<float>(1000.0 * cumulative_delay_s_ /
                                 delivered_packets_);
    statistics_publisher_->publish(message);

    const double logical_delivery_ratio =
        logical_attempted_packets_ == 0U
            ? 0.0
            : static_cast<double>(logical_delivered_packets_) /
                  static_cast<double>(logical_attempted_packets_);
    const double mean_end_to_end_delay_ms =
        logical_delivered_packets_ == 0U
            ? 0.0
            : 1000.0 * cumulative_end_to_end_delay_s_ /
                  static_cast<double>(logical_delivered_packets_);
    const double mean_initial_tbler =
        initial_tbler_samples_ == 0U
            ? 0.0
            : cumulative_initial_tbler_ /
                  static_cast<double>(initial_tbler_samples_);
    const auto &phy = model_.config();
    const auto &fixed_mcs = model_.selectMcs(0.0);
    const bool uses_fixed_mcs = phy.fixed_mcs_index >= 0;
    std::ostringstream json;
    json << "{\"network_topology\":\"" << topology_
         << "\",\"nearest_neighbor_count\":" << nearest_neighbor_count_
         << ",\"lossless_nearest_neighbor_count\":"
         << lossless_nearest_neighbor_count_
         << ",\"lossless_communication_range_m\":"
         << lossless_communication_range_m_
         << ",\"lossless_control_only\":"
         << (lossless_control_only_ ? "true" : "false")
         << ",\"directed_message_unicast\":"
         << (directed_message_unicast_ ? "true" : "false")
         << ",\"chunk_data_pre_enqueue_dedup\":"
         << (chunk_data_pre_enqueue_dedup_ ? "true" : "false")
         << ",\"chunk_data_max_pending_per_link\":"
         << chunk_data_max_pending_per_link_
         << ",\"directed_unicast_messages\":"
         << directed_unicast_messages_
         << ",\"invalid_directed_messages_dropped\":"
         << invalid_directed_messages_dropped_
         << ",\"chunk_enqueue_duplicates_suppressed\":"
         << chunk_enqueue_duplicates_suppressed_
         << ",\"chunk_enqueue_budget_drops\":"
         << chunk_enqueue_budget_drops_
         << ",\"communication_range_m\":" << communication_range_m_
         << ",\"ideal_direct_enabled\":"
         << (ideal_direct_enabled_ ? "true" : "false")
         << ",\"ideal_coalesce_window_ms\":"
         << 1000.0 * ideal_coalesce_window_s_
         << ",\"ideal_coalesced_messages\":"
         << ideal_coalesced_messages_
         << ",\"perfect_forwarding_calls\":"
         << perfect_forward_profile_calls_
         << ",\"perfect_forwarding_receivers\":"
         << perfect_forward_profile_receivers_
         << ",\"perfect_forwarding_mean_ms\":"
         << (perfect_forward_profile_calls_ == 0U ? 0.0 :
             perfect_forward_profile_total_ms_ /
                 static_cast<double>(perfect_forward_profile_calls_))
         << ",\"perfect_forwarding_max_ms\":"
         << perfect_forward_profile_max_ms_
         << ",\"ideal_logical_messages\":" << ideal_logical_messages_
         << ",\"ideal_statistical_transport_blocks\":"
         << ideal_statistical_transport_blocks_
         << ",\"ideal_statistical_bytes\":"
         << ideal_statistical_bytes_
         << ",\"active_link_publications\":"
         << active_link_publications_
         << ",\"active_link_samples_published\":"
         << active_link_samples_published_
         << ",\"ideal_direct_forwarded_packets\":"
         << ideal_direct_forwarded_packets_
         << ",\"lossless_nearest_forwarded_packets\":"
         << lossless_nearest_forwarded_packets_
         << ",\"lossless_range_forwarded_packets\":"
         << lossless_range_forwarded_packets_
         << ",\"lossless_nearest_position_unavailable_events\":"
         << lossless_nearest_position_unavailable_events_
         << ",\"sionna_direct_attempted_packets\":"
         << (direct_attempted_packets_ - lossless_nearest_forwarded_packets_)
         << ",\"nearest_filtered_receivers\":" << nearest_filtered_receivers_
         << ",\"range_filtered_receivers\":" << range_filtered_receivers_
         << ",\"nearest_position_unavailable\":"
         << position_unavailable_receivers_
         << ",\"position_unavailable_receivers\":"
         << position_unavailable_receivers_
         << ",\"ap_enabled\":" << (ap_enabled_ ? "true" : "false")
         << ",\"ap_node_id\":" << (ap_enabled_ ? ap_node_id_ : -1)
         << ",\"attempted_packets\":" << attempted_packets_
         << ",\"delivered_packets\":" << delivered_packets_
         << ",\"dropped_no_link\":" << dropped_no_link_
         << ",\"dropped_per\":" << dropped_per_
         << ",\"dropped_queue\":" << dropped_queue_
         << ",\"dropped_ttl\":" << dropped_ttl_
         << ",\"retried_packets\":" << retried_packets_
         << ",\"attempted_bytes\":" << attempted_bytes_
         << ",\"delivered_bytes\":" << delivered_bytes_
         << ",\"queued_packets\":" << message.queued_packets
         << ",\"queued_bytes\":" << message.queued_bytes
         << ",\"mean_delivery_delay_ms\":"
         << message.mean_delivery_delay_ms
         << ",\"logical_attempted_packets\":"
         << logical_attempted_packets_
         << ",\"logical_delivered_packets\":"
         << logical_delivered_packets_
         << ",\"logical_attempted_bytes\":" << logical_attempted_bytes_
         << ",\"logical_delivered_bytes\":" << logical_delivered_bytes_
         << ",\"logical_delivery_ratio\":" << logical_delivery_ratio
         << ",\"mean_end_to_end_delay_ms\":" << mean_end_to_end_delay_ms
         << ",\"direct_attempted_packets\":" << direct_attempted_packets_
         << ",\"direct_delivered_packets\":" << direct_delivered_packets_
         << ",\"ap_uplink_attempted_packets\":"
         << ap_uplink_attempted_packets_
         << ",\"ap_uplink_delivered_packets\":"
         << ap_uplink_delivered_packets_
         << ",\"ap_downlink_attempted_packets\":"
         << ap_downlink_attempted_packets_
         << ",\"ap_downlink_delivered_packets\":"
         << ap_downlink_delivered_packets_
         << ",\"ap_global_updates_received\":"
         << ap_global_updates_received_
         << ",\"ap_selective_forwards_enqueued\":"
         << ap_selective_forwards_enqueued_
         << ",\"ap_relay_wins\":" << ap_relay_wins_
         << ",\"direct_delivery_wins\":" << direct_delivery_wins_
         << ",\"duplicates_suppressed\":" << duplicates_suppressed_
         << ",\"bs_round_robin_enabled\":"
         << (bs_round_robin_enabled_ ? "true" : "false")
         << ",\"bs_active_uav\":" << bs_active_uav_
         << ",\"bs_round_robin_turns\":" << bs_round_robin_turns_
         << ",\"bs_upload_grants_delivered\":"
         << bs_upload_grants_delivered_
         << ",\"bs_upload_grants_failed\":" << bs_upload_grants_failed_
         << ",\"bs_incremental_chunks_scheduled_uplink\":"
         << bs_incremental_chunks_scheduled_uplink_
         << ",\"bs_incremental_chunks_received_uplink\":"
         << bs_incremental_chunks_received_uplink_
         << ",\"bs_missing_chunks_scheduled_downlink\":"
         << bs_missing_chunks_scheduled_downlink_
         << ",\"bs_missing_chunks_delivered_downlink\":"
         << bs_missing_chunks_delivered_downlink_
         << ",\"bs_known_map_chunks\":" << bs_chunks_.size()
         << ",\"bs_control_attempted_packets\":"
         << bs_control_attempted_packets_
         << ",\"bs_control_delivered_packets\":"
         << bs_control_delivered_packets_
         << ",\"bs_uplink_attempted_packets\":"
         << bs_uplink_attempted_packets_
         << ",\"bs_uplink_delivered_packets\":"
         << bs_uplink_delivered_packets_
         << ",\"bs_downlink_attempted_packets\":"
         << bs_downlink_attempted_packets_
         << ",\"bs_downlink_delivered_packets\":"
         << bs_downlink_delivered_packets_
         << ",\"phy\":{\"carrier_frequency_hz\":"
         << carrier_frequency_hz_
         << ",\"bandwidth_hz\":" << phy.bandwidth_hz
         << ",\"subcarrier_spacing_hz\":"
         << phy.subcarrier_spacing_hz
         << ",\"resource_blocks\":" << phy.resource_blocks
         << ",\"waveform\":\"CP-OFDM\",\"fec\":\"5G_NR_LDPC\""
         << ",\"channel_small_scale_fading\":\"none\""
         << ",\"bs_tx_power_dbm\":" << bs_tx_power_dbm_
         << ",\"uav_tx_power_dbm\":" << uav_tx_power_dbm_
         << ",\"max_retries\":" << max_retries_
         << ",\"bs_max_retries\":" << bs_max_retries_
         << ",\"random_seed\":" << random_seed_
         << ",\"bs_upa\":\"" << bs_array_rows_ << "x"
         << bs_array_cols_ << "\""
         << ",\"uav_upa\":\"" << uav_array_rows_ << "x"
         << uav_array_cols_ << "\""
         << ",\"target_initial_tbler\":"
         << phy.target_initial_tbler
         << ",\"fixed_mcs_index\":" << phy.fixed_mcs_index
         << ",\"fixed_mcs_modulation\":\""
         << (uses_fixed_mcs ? fixed_mcs.name : "adaptive") << "\""
         << ",\"fixed_mcs_code_rate\":"
         << (uses_fixed_mcs ? fixed_mcs.code_rate : -1.0)
         << ",\"fixed_mcs_target_snr_db\":"
         << (uses_fixed_mcs ? fixed_mcs.target_snr_db : -1.0)
         << ",\"mean_selected_initial_tbler\":" << mean_initial_tbler
         << ",\"mcs_counts\":{\"QPSK\":" << mcs_counts_["QPSK"]
         << ",\"16QAM\":" << mcs_counts_["16QAM"]
         << ",\"64QAM\":" << mcs_counts_["64QAM"]
         << ",\"256QAM\":" << mcs_counts_["256QAM"] << "}}"
         << ",\"sionna_exact_samples\":"
         << link_model_counts_["sionna_exact"]
         << ",\"sionna_cache_corrected_samples\":"
         << link_model_counts_["sionna_cache_corrected"]
         << ",\"radio_map_cache_samples\":"
         << link_model_counts_["radio_map_cache"] << "}";
    RCLCPP_INFO(get_logger(), "RACER_SIONNA_STATS %s", json.str().c_str());
  }

  std::string mode_;
  int drone_count_{};
  std::string topology_;
  int nearest_neighbor_count_{};
  int lossless_nearest_neighbor_count_{};
  double lossless_communication_range_m_{};
  bool lossless_control_only_{false};
  bool directed_message_unicast_{false};
  bool chunk_data_pre_enqueue_dedup_{false};
  int chunk_data_max_pending_per_link_{};
  double communication_range_m_{};
  double ideal_coalesce_window_s_{};
  double active_link_hold_s_{};
  double active_link_publish_period_s_{};
  double carrier_frequency_hz_{};
  double bs_tx_power_dbm_{};
  double uav_tx_power_dbm_{};
  int bs_array_rows_{};
  int bs_array_cols_{};
  int uav_array_rows_{};
  int uav_array_cols_{};
  bool ap_enabled_{false};
  bool bs_round_robin_enabled_{false};
  bool nearest_neighbors_enabled_{false};
  bool distance_radius_enabled_{false};
  bool ideal_direct_enabled_{false};
  int ap_node_id_{};
  int radio_node_count_{};
  int random_seed_{};
  double base_latency_s_{};
  double jitter_s_{};
  std::size_t queue_capacity_bytes_{};
  int max_retries_{};
  int bs_max_retries_{};
  double retry_backoff_s_{};
  double bs_min_turn_s_{};
  std::size_t bs_control_bytes_{};
  int bs_max_downlink_chunks_per_turn_{};
  int bs_max_uplink_chunks_per_turn_{};
  LinkModel model_;

  std::unordered_map<LinkKey, LinkQuality, LinkKeyHash> links_;
  std::unordered_map<LinkKey, LinkQueue, LinkKeyHash> queues_;
  std::unordered_map<LinkKey, std::mt19937, LinkKeyHash> link_rngs_;
  std::unordered_map<LinkKey, std::chrono::steady_clock::time_point,
                     LinkKeyHash> active_links_;
  bool active_links_dirty_{false};
  bool have_active_link_publish_wall_{false};
  std::chrono::steady_clock::time_point last_active_link_publish_wall_{};
  std::unordered_map<std::string, std::uint64_t> link_model_counts_;
  std::unordered_map<std::string, std::uint64_t> mcs_counts_;
  std::unordered_map<ChunkKey, CachedChunk, ChunkKeyHash> chunk_repository_;
  std::vector<std::unordered_set<ChunkKey, ChunkKeyHash>> uav_chunks_;
  std::vector<std::array<double, 3>> uav_positions_;
  std::vector<bool> uav_position_valid_;
  std::unordered_set<ChunkKey, ChunkKeyHash> bs_chunks_;
  std::vector<std::vector<rclcpp::GenericPublisher::SharedPtr>> publishers_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::vector<std::vector<std::shared_ptr<rclcpp::SerializedMessage>>>
      ap_latest_messages_;
  rclcpp::Subscription<LinkQualityArray>::SharedPtr link_subscription_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
      odometry_subscriptions_;
  rclcpp::Publisher<racer_sionna_interfaces::msg::CommStatistics>::SharedPtr
      statistics_publisher_;
  rclcpp::Publisher<LinkQualityArray>::SharedPtr active_link_publisher_;
  rclcpp::GenericPublisher::SharedPtr ap_recovery_status_publisher_;
  rclcpp::GenericSubscription::SharedPtr ap_recovery_command_subscription_;
  rclcpp::TimerBase::SharedPtr scheduler_timer_;
  rclcpp::TimerBase::SharedPtr active_link_timer_;
  rclcpp::TimerBase::SharedPtr statistics_timer_;

  bool bs_turn_active_{false};
  bool bs_control_finished_{false};
  int bs_active_uav_{-1};
  int bs_next_uav_{0};
  double bs_turn_started_at_{};
  std::unordered_map<int, std::uint64_t> bs_uav_turns_;

  std::uint64_t attempted_packets_{};
  std::uint64_t delivered_packets_{};
  std::uint64_t dropped_no_link_{};
  std::uint64_t dropped_per_{};
  std::uint64_t dropped_queue_{};
  std::uint64_t dropped_ttl_{};
  std::uint64_t retried_packets_{};
  std::uint64_t attempted_bytes_{};
  std::uint64_t delivered_bytes_{};
  double cumulative_delay_s_{};

  std::uint64_t logical_attempted_packets_{};
  std::uint64_t logical_delivered_packets_{};
  std::uint64_t logical_attempted_bytes_{};
  std::uint64_t logical_delivered_bytes_{};
  double cumulative_end_to_end_delay_s_{};
  std::uint64_t direct_attempted_packets_{};
  std::uint64_t direct_delivered_packets_{};
  std::uint64_t ap_uplink_attempted_packets_{};
  std::uint64_t ap_uplink_delivered_packets_{};
  std::uint64_t ap_downlink_attempted_packets_{};
  std::uint64_t ap_downlink_delivered_packets_{};
  std::uint64_t ap_global_updates_received_{};
  std::uint64_t ap_selective_forwards_enqueued_{};
  std::uint64_t ap_relay_wins_{};
  std::uint64_t direct_delivery_wins_{};
  std::uint64_t duplicates_suppressed_{};
  std::uint64_t nearest_filtered_receivers_{};
  std::uint64_t range_filtered_receivers_{};
  std::uint64_t position_unavailable_receivers_{};
  std::uint64_t ideal_coalesced_messages_{};
  std::uint64_t ideal_direct_forwarded_packets_{};
  std::uint64_t ideal_logical_messages_{};
  std::uint64_t ideal_statistical_transport_blocks_{};
  std::uint64_t ideal_statistical_bytes_{};
  std::uint64_t perfect_forward_profile_calls_{};
  std::uint64_t perfect_forward_profile_receivers_{};
  double perfect_forward_profile_total_ms_{};
  double perfect_forward_profile_max_ms_{};
  std::uint64_t active_link_publications_{};
  std::uint64_t active_link_samples_published_{};
  std::uint64_t lossless_nearest_forwarded_packets_{};
  std::uint64_t lossless_range_forwarded_packets_{};
  std::uint64_t lossless_nearest_position_unavailable_events_{};
  std::uint64_t directed_unicast_messages_{};
  std::uint64_t invalid_directed_messages_dropped_{};
  std::uint64_t chunk_enqueue_duplicates_suppressed_{};
  std::uint64_t chunk_enqueue_budget_drops_{};
  double cumulative_initial_tbler_{};
  std::uint64_t initial_tbler_samples_{};
  std::uint64_t bs_round_robin_turns_{};
  std::uint64_t bs_upload_grants_delivered_{};
  std::uint64_t bs_upload_grants_failed_{};
  std::uint64_t bs_incremental_chunks_scheduled_uplink_{};
  std::uint64_t bs_incremental_chunks_received_uplink_{};
  std::uint64_t bs_missing_chunks_scheduled_downlink_{};
  std::uint64_t bs_missing_chunks_delivered_downlink_{};
  std::uint64_t bs_control_attempted_packets_{};
  std::uint64_t bs_control_delivered_packets_{};
  std::uint64_t bs_uplink_attempted_packets_{};
  std::uint64_t bs_uplink_delivered_packets_{};
  std::uint64_t bs_downlink_attempted_packets_{};
  std::uint64_t bs_downlink_delivered_packets_{};
};

}  // namespace racer_sionna_comm

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<racer_sionna_comm::CommunicationProxy>());
  rclcpp::shutdown();
  return 0;
}
