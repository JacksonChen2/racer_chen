#include <racer_sionna_comm/link_model.hpp>
#include <racer_sionna_comm/shared_ofdma_scheduler.hpp>

#include <racer_sionna_interfaces/msg/comm_statistics.hpp>
#include <racer_sionna_interfaces/msg/link_quality.hpp>
#include <racer_sionna_interfaces/msg/link_quality_array.hpp>

#include <racer_fidelity_msgs/msg/chunk_data.hpp>
#include <racer_fidelity_msgs/msg/chunk_stamps.hpp>
#include <racer_fidelity_msgs/msg/bspline.hpp>
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
#include <cstdio>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
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
        uav_udp_directed_unicast_(declare_parameter<bool>(
            "uav_udp_directed_unicast", true)),
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
        udp_header_bytes_(
            declare_parameter<int>("uav_udp_header_bytes", 8)),
        ofdma_diagnostic_logging_(declare_parameter<bool>(
            "uav_ofdma_diagnostic_logging", true)),
        ofdma_log_packet_stride_(
            declare_parameter<int>("uav_ofdma_log_packet_stride", 1000)),
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
        rl_bs_scheduler_enabled_(declare_parameter<bool>(
            "rl_bs_scheduler_enabled", false)),
        rl_bs_action_path_(declare_parameter<std::string>(
            "rl_bs_action_path", "/tmp/racer_agentic_crpo/action.txt")),
        rl_bs_state_path_(declare_parameter<std::string>(
            "rl_bs_state_path",
            "/tmp/racer_agentic_crpo/communication_state.json")),
        rl_bs_decision_period_s_(1.0e-3 * declare_parameter<double>(
            "rl_bs_decision_period_ms", 20.0)),
        ground_truth_occupied_voxels_path_(declare_parameter<std::string>(
            "ground_truth_occupied_voxels_path", "")),
        observed_occupied_voxels_path_(declare_parameter<std::string>(
            "observed_occupied_voxels_path", "")),
        require_ground_truth_map_(declare_parameter<bool>(
            "require_ground_truth_map", false)),
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
                declare_parameter<int>("fixed_mcs_index", -1))}),
        uav_broadcast_model_(LinkModelConfig{
            declare_parameter<double>("uav_broadcast_bandwidth_hz", 50.0e6),
            model_.config().subcarrier_spacing_hz,
            static_cast<int>(declare_parameter<int>(
                "uav_broadcast_resource_blocks", 33)),
            model_.config().data_re_efficiency,
            model_.config().target_initial_tbler,
            model_.config().tbler_slope_db,
            model_.config().transport_block_bytes,
            static_cast<int>(declare_parameter<int>(
                "uav_broadcast_fixed_mcs_index", 14))}),
        bs_model_(LinkModelConfig{
            declare_parameter<double>("bs_bandwidth_hz", 50.0e6),
            model_.config().subcarrier_spacing_hz,
            static_cast<int>(
                declare_parameter<int>("bs_resource_blocks", 33)),
            model_.config().data_re_efficiency,
            model_.config().target_initial_tbler,
            model_.config().tbler_slope_db,
            model_.config().transport_block_bytes,
            -1}),
        ofdma_scheduler_(model_.config().resource_blocks),
        uav_broadcast_ofdma_scheduler_(
            uav_broadcast_model_.config().resource_blocks) {
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
        udp_header_bytes_ <= 0 || ofdma_log_packet_stride_ <= 0 ||
        max_retries_ < 0 || bs_max_retries_ < 0 ||
        chunk_data_max_pending_per_link_ < 0 ||
        ideal_coalesce_window_s_ <= 0.0 || active_link_hold_s_ <= 0.0 ||
        active_link_publish_period_s_ <= 0.0 || base_latency_s_ < 0.0 ||
        jitter_s_ < 0.0 ||
        retry_backoff_s_ < 0.0 || bs_min_turn_s_ < 0.0 ||
        bs_control_bytes_ == 0U || bs_max_downlink_chunks_per_turn_ < 1 ||
        bs_max_uplink_chunks_per_turn_ < 1 || carrier_frequency_hz_ <= 0.0 ||
        rl_bs_decision_period_s_ <= 0.0 ||
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
    if (ap_enabled_) {
      const double assigned_bandwidth =
          uav_broadcast_model_.config().bandwidth_hz +
          bs_model_.config().bandwidth_hz;
      const int assigned_prbs =
          uav_broadcast_model_.config().resource_blocks +
          bs_model_.config().resource_blocks;
      if (std::abs(assigned_bandwidth - model_.config().bandwidth_hz) >
              1.0e-6 * model_.config().bandwidth_hz ||
          assigned_prbs != model_.config().resource_blocks ||
          uav_broadcast_model_.config().fixed_mcs_index < 0) {
        throw std::runtime_error(
            "BS-assisted PHY requires UAV and BS subbands to exactly "
            "partition bandwidth_hz/resource_blocks and UAV broadcast to "
            "use a fixed MCS");
      }
    }
    shared_uav_ofdma_enabled_ =
        mode_ != "ideal" && (topology_ == "distributed" || ap_enabled_);
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
    if (rl_bs_scheduler_enabled_ && !bs_round_robin_enabled_) {
      throw std::runtime_error(
          "rl_bs_scheduler_enabled requires network_topology=bs_round_robin");
    }
    ap_node_id_ = drone_count_;
    radio_node_count_ = drone_count_ + static_cast<int>(ap_enabled_);
    uav_udp_queues_.resize(static_cast<std::size_t>(drone_count_));
    uav_sender_rngs_.reserve(static_cast<std::size_t>(drone_count_));
    for (int sender = 0; sender < drone_count_; ++sender) {
      std::seed_seq seed{random_seed_, sender, 0x554450, 0x4f46444d};
      uav_sender_rngs_.emplace_back(seed);
    }

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
        // Distributed Sionna UAV traffic uses per-sender UDP queues below.
        // Keep the legacy per-link queues only for untouched non-distributed
        // and UAV-BS paths.
        const bool direct_uav_link =
            sender < drone_count_ && receiver < drone_count_;
        if (!(shared_uav_ofdma_enabled_ && direct_uav_link)) {
          queues_.emplace(key, LinkQueue{});
        }
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
    uav_observed_voxels_.resize(static_cast<std::size_t>(drone_count_));
    uav_positions_.resize(static_cast<std::size_t>(drone_count_));
    uav_velocities_.resize(static_cast<std::size_t>(drone_count_));
    uav_yaws_.assign(static_cast<std::size_t>(drone_count_), 0.0);
    uav_position_valid_.assign(static_cast<std::size_t>(drone_count_), false);
    last_uav_info_received_s_.assign(
        static_cast<std::size_t>(drone_count_),
        std::vector<double>(static_cast<std::size_t>(drone_count_), 0.0));
    last_bs_info_received_s_.assign(static_cast<std::size_t>(drone_count_),
                                    0.0);
    rl_relay_action_.assign(
        static_cast<std::size_t>(drone_count_),
        std::vector<bool>(static_cast<std::size_t>(drone_count_), false));
    rl_upload_action_.assign(static_cast<std::size_t>(drone_count_), false);
    trajectory_summaries_.resize(static_cast<std::size_t>(drone_count_));
    loadGroundTruthOccupiedVoxels();
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
                const auto &velocity = message->twist.twist.linear;
                uav_velocities_[static_cast<std::size_t>(drone)] =
                    {velocity.x, velocity.y, velocity.z};
                const auto &orientation = message->pose.pose.orientation;
                uav_yaws_[static_cast<std::size_t>(drone)] = std::atan2(
                    2.0 * (orientation.w * orientation.z +
                           orientation.x * orientation.y),
                    1.0 - 2.0 * (orientation.y * orientation.y +
                                 orientation.z * orientation.z));
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
        "waveform=CP-OFDM shared_uav_ofdma=%d uav_transport=UDP "
        "rl_bs_scheduler=%d total_bandwidth_mhz=%.1f "
        "uav_bandwidth_mhz=%.1f uav_mcs=%d bs_bandwidth_mhz=%.1f "
        "bs_mcs=adaptive",
        mode_.c_str(), topology_.c_str(), drone_count_, nearest_neighbor_count_,
        lossless_nearest_neighbor_count_, lossless_communication_range_m_,
        communication_range_m_, radio_node_count_,
        ap_enabled_ ? ap_node_id_ : -1, random_seed_,
        shared_uav_ofdma_enabled_ ? 1 : 0,
        rl_bs_scheduler_enabled_ ? 1 : 0,
        1.0e-6 * model_.config().bandwidth_hz,
        1.0e-6 * directRadioModel().config().bandwidth_hz,
        directRadioModel().config().fixed_mcs_index,
        1.0e-6 * bs_model_.config().bandwidth_hz);
  }

  ~CommunicationProxy() override { exportObservedOccupiedVoxels(); }

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

  struct UdpReceiverState {
    int receiver{};
    bool force_lossless{false};
    bool half_duplex_blocked{false};
    bool channel_unavailable{false};
    double received_bits{};
    double snr_linear_bit_sum{};
    double last_snr_db{-std::numeric_limits<double>::infinity()};
    int last_allocated_prbs{};
  };

  struct UdpDatagram {
    std::shared_ptr<rclcpp::SerializedMessage> message;
    std::shared_ptr<DeliveryFlow> flow;
    std::size_t topic_index{};
    double born_at{};
    double enqueued_at{};
    double delivery_at{};
    std::size_t bytes{};
    double total_bits{};
    double remaining_bits{};
    std::uint64_t packet_id{};
    std::uint64_t first_slot{};
    std::uint64_t last_slot{};
    std::uint64_t allocated_prb_slots{};
    int last_allocated_prbs{};
    bool started{false};
    bool has_chunk_key{false};
    ChunkKey chunk_key{};
    std::vector<UdpReceiverState> receivers;
  };

  struct UavUdpQueue {
    std::deque<UdpDatagram> datagrams;
    std::size_t bytes{};
  };

  struct CachedChunk {
    std::shared_ptr<rclcpp::SerializedMessage> message;
    std::size_t bytes{};
  };

  struct TrajectorySummary {
    std::array<double, 3> goal{};
    double length{};
    double expected_execution_time{};
    bool valid{false};
  };

  struct TaskQualitySample {
    double time_s{};
    double redundancy{};
    double map_iou{-1.0};
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

  void loadGroundTruthOccupiedVoxels() {
    if (ground_truth_occupied_voxels_path_.empty()) {
      if (require_ground_truth_map_) {
        throw std::runtime_error(
            "require_ground_truth_map=true needs "
            "ground_truth_occupied_voxels_path");
      }
      return;
    }
    std::ifstream input(ground_truth_occupied_voxels_path_);
    if (!input) {
      throw std::runtime_error(
          "cannot open ground-truth occupied voxel address file: " +
          ground_truth_occupied_voxels_path_);
    }
    std::uint64_t address{};
    while (input >> address) {
      if (address > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error(
            "ground-truth voxel address exceeds uint32 range");
      }
      ground_truth_occupied_voxels_.insert(
          static_cast<std::uint32_t>(address));
    }
    if (ground_truth_occupied_voxels_.empty()) {
      throw std::runtime_error(
          "ground-truth occupied voxel address file is empty");
    }
  }

  static void applyOccupiedVoxelUpdate(
      const racer_fidelity_msgs::msg::ChunkData &chunk,
      std::unordered_set<std::uint32_t> &occupied) {
    const auto count = std::min(chunk.voxel_adrs.size(), chunk.voxel_occ.size());
    for (std::size_t index = 0; index < count; ++index) {
      const auto address = chunk.voxel_adrs[index];
      if (chunk.voxel_occ[index] == 1U) {
        occupied.insert(address);
      } else {
        occupied.erase(address);
      }
    }
  }

  double redundantExplorationRatio() const {
    std::size_t sum{};
    std::unordered_set<std::uint32_t> union_voxels;
    for (const auto &observed : uav_observed_voxels_) {
      sum += observed.size();
      union_voxels.insert(observed.begin(), observed.end());
    }
    if (sum == 0U) return 0.0;
    return std::clamp(
        1.0 - static_cast<double>(union_voxels.size()) /
                  static_cast<double>(sum),
        0.0, 1.0);
  }

  double bsGlobalMapIou() const {
    if (ground_truth_occupied_voxels_.empty()) return -1.0;
    std::size_t intersection{};
    for (const auto address : bs_occupied_voxels_) {
      if (ground_truth_occupied_voxels_.count(address)) ++intersection;
    }
    const std::size_t union_size =
        bs_occupied_voxels_.size() + ground_truth_occupied_voxels_.size() -
        intersection;
    return union_size == 0U
               ? 1.0
               : static_cast<double>(intersection) /
                     static_cast<double>(union_size);
  }

  void exportObservedOccupiedVoxels() const {
    if (observed_occupied_voxels_path_.empty()) return;
    const std::filesystem::path output_path(observed_occupied_voxels_path_);
    if (!output_path.parent_path().empty()) {
      std::error_code error;
      std::filesystem::create_directories(output_path.parent_path(), error);
    }
    const auto temporary = output_path.string() + ".tmp";
    std::ofstream output(temporary, std::ios::trunc);
    if (!output) return;
    std::vector<std::uint32_t> addresses(bs_occupied_voxels_.begin(),
                                         bs_occupied_voxels_.end());
    std::sort(addresses.begin(), addresses.end());
    for (const auto address : addresses) output << address << '\n';
    output.close();
    if (output) std::rename(temporary.c_str(), output_path.string().c_str());
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
    if (kTopicPolicies[topic_index].key != "chunk_data") {
      return;
    }
    racer_fidelity_msgs::msg::ChunkData chunk;
    if (!deserialize(message, chunk) || chunk.chunk_drone_id < 1 ||
        chunk.chunk_drone_id > drone_count_ || chunk.idx == 0U) return;
    *chunk_key = {chunk.chunk_drone_id, chunk.idx};
    auto &observed = uav_observed_voxels_[
        static_cast<std::size_t>(chunk.chunk_drone_id - 1)];
    observed.insert(chunk.voxel_adrs.begin(), chunk.voxel_adrs.end());
    // An ideal run represents the perfect global-map aggregator used only to
    // create the offline reference/GT diagnostics. Real BS runs update this
    // map exclusively after successful UAV-to-BS delivery below.
    if (mode_ == "ideal") applyOccupiedVoxelUpdate(chunk, bs_occupied_voxels_);
    const std::size_t bytes = 64U + message->size();
    uav_chunks_[static_cast<std::size_t>(sender)].insert(*chunk_key);
    chunk_repository_.insert_or_assign(*chunk_key,
                                       CachedChunk{message, bytes});
  }

  void observeTrajectory(
      int sender, std::size_t topic_index,
      const std::shared_ptr<rclcpp::SerializedMessage> &message) {
    if (sender < 0 || sender >= drone_count_ ||
        kTopicPolicies.at(topic_index).key != "trajectory") {
      return;
    }
    racer_fidelity_msgs::msg::Bspline trajectory;
    if (!deserialize(message, trajectory) || trajectory.pos_pts.empty()) return;
    TrajectorySummary summary;
    const auto &goal = trajectory.pos_pts.back();
    summary.goal = {goal.x, goal.y, goal.z};
    for (std::size_t index = 1; index < trajectory.pos_pts.size(); ++index) {
      const auto &left = trajectory.pos_pts[index - 1U];
      const auto &right = trajectory.pos_pts[index];
      summary.length += std::hypot(
          std::hypot(right.x - left.x, right.y - left.y), right.z - left.z);
    }
    if (trajectory.knots.size() >= 2U) {
      summary.expected_execution_time = std::max(
          0.0, trajectory.knots.back() - trajectory.knots.front());
    }
    summary.valid = true;
    trajectory_summaries_[static_cast<std::size_t>(sender)] = summary;
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
    const std::size_t bytes = shared_uav_ofdma_enabled_
                                  ? udp_header_bytes_ + message->size()
                                  : 64U + message->size();
    const auto &topic_key = kTopicPolicies.at(topic_index).key;
    ChunkKey chunk_key{};
    observeUavTransmit(sender, topic_index, message, &chunk_key);
    observeTrajectory(sender, topic_index, message);
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
    if (shared_uav_ofdma_enabled_) {
      enqueueUavUdpDatagram(
          sender, topic_index, message, flow, receivers, lossless_receivers,
          stamp, has_chunk_key ? &chunk_key : nullptr);
      return;
    }
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
    const bool unicast_enabled = shared_uav_ofdma_enabled_
                                     ? uav_udp_directed_unicast_
                                     : directed_message_unicast_;
    if (!unicast_enabled) return -1;
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

  bool shouldLogUdpDatagram(std::uint64_t packet_id) const noexcept {
    return ofdma_diagnostic_logging_ &&
           packet_id % static_cast<std::uint64_t>(ofdma_log_packet_stride_) ==
               0U;
  }

  static std::size_t remainingBytes(const UdpDatagram &datagram) {
    return static_cast<std::size_t>(
        std::ceil(std::max(0.0, datagram.remaining_bits) / 8.0));
  }

  bool enqueueUavUdpDatagram(
      int sender, std::size_t topic_index,
      const std::shared_ptr<rclcpp::SerializedMessage> &message,
      const std::shared_ptr<DeliveryFlow> &flow,
      const std::vector<int> &receivers,
      const std::vector<int> &lossless_receivers, double born_at,
      const ChunkKey *chunk_key) {
    if (sender < 0 || sender >= drone_count_ || receivers.empty()) return false;
    const std::size_t bytes = udp_header_bytes_ + (message ? message->size() : 0U);
    auto &queue = uav_udp_queues_[static_cast<std::size_t>(sender)];
    const bool chunk_data =
        chunk_key != nullptr &&
        kTopicPolicies.at(topic_index).key == "chunk_data";
    const std::unordered_set<int> lossless_set(
        lossless_receivers.begin(), lossless_receivers.end());
    std::vector<UdpReceiverState> receiver_states;
    receiver_states.reserve(receivers.size());
    for (const int receiver : receivers) {
      if (receiver < 0 || receiver >= drone_count_ || receiver == sender) continue;
      if (chunk_data && chunk_data_pre_enqueue_dedup_ &&
          uav_chunks_[static_cast<std::size_t>(receiver)].find(*chunk_key) !=
              uav_chunks_[static_cast<std::size_t>(receiver)].end()) {
        ++chunk_enqueue_duplicates_suppressed_;
        continue;
      }
      if (chunk_data && chunk_data_max_pending_per_link_ > 0) {
        const auto pending = std::count_if(
            queue.datagrams.begin(), queue.datagrams.end(),
            [receiver](const UdpDatagram &datagram) {
              return datagram.has_chunk_key &&
                     std::any_of(
                         datagram.receivers.begin(), datagram.receivers.end(),
                         [receiver](const UdpReceiverState &state) {
                           return state.receiver == receiver;
                         });
            });
        if (pending >= chunk_data_max_pending_per_link_) {
          ++chunk_enqueue_budget_drops_;
          continue;
        }
      }
      receiver_states.push_back(
          {receiver, lossless_set.find(receiver) != lossless_set.end()});
    }
    if (receiver_states.empty()) return false;

    const auto receiver_count =
        static_cast<std::uint64_t>(receiver_states.size());
    attempted_packets_ += receiver_count;
    attempted_bytes_ += receiver_count * bytes;
    direct_attempted_packets_ += receiver_count;
    uav_udp_receiver_attempts_ += receiver_count;
    ++uav_udp_datagrams_enqueued_;
    uav_udp_physical_bytes_enqueued_ += bytes;
    if (receiver_states.size() == 1U) {
      ++uav_udp_unicast_datagrams_;
    } else {
      ++uav_udp_multicast_datagrams_;
    }

    const auto accountQueueDrop = [this](const UdpDatagram &datagram) {
      dropped_queue_ += datagram.receivers.size();
      ++uav_udp_datagrams_dropped_queue_;
    };
    if (bytes > queue_capacity_bytes_) {
      dropped_queue_ += receiver_count;
      ++uav_udp_datagrams_dropped_queue_;
      return false;
    }
    const int incoming_priority = packetPriority(RouteStage::kDirect, topic_index);
    while (queue.bytes + bytes > queue_capacity_bytes_) {
      auto candidate = queue.datagrams.end();
      for (auto iterator = queue.datagrams.end();
           iterator != queue.datagrams.begin();) {
        --iterator;
        if (!iterator->started &&
            packetPriority(RouteStage::kDirect, iterator->topic_index) <
                incoming_priority) {
          candidate = iterator;
          break;
        }
      }
      if (candidate == queue.datagrams.end()) {
        dropped_queue_ += receiver_count;
        ++uav_udp_datagrams_dropped_queue_;
        return false;
      }
      queue.bytes -= std::min(queue.bytes, candidate->bytes);
      accountQueueDrop(*candidate);
      queue.datagrams.erase(candidate);
    }

    UdpDatagram datagram;
    datagram.message = message;
    datagram.flow = flow;
    datagram.topic_index = topic_index;
    datagram.born_at = born_at;
    datagram.enqueued_at = now().seconds();
    datagram.bytes = bytes;
    datagram.total_bits = 8.0 * static_cast<double>(bytes);
    datagram.remaining_bits = datagram.total_bits;
    datagram.packet_id = ++uav_udp_next_packet_id_;
    datagram.receivers = std::move(receiver_states);
    if (chunk_key != nullptr) {
      datagram.has_chunk_key = true;
      datagram.chunk_key = *chunk_key;
    }
    for (const auto &receiver : datagram.receivers) {
      markActiveLink(sender, receiver.receiver);
    }

    auto position = queue.datagrams.begin();
    if (position != queue.datagrams.end() && position->started) ++position;
    while (position != queue.datagrams.end() &&
           packetPriority(RouteStage::kDirect, position->topic_index) >=
               incoming_priority) {
      ++position;
    }
    queue.datagrams.insert(position, std::move(datagram));
    queue.bytes += bytes;
    return true;
  }

  const LinkModel &directRadioModel() const {
    return ap_enabled_ ? uav_broadcast_model_ : model_;
  }

  SharedOfdmaScheduler &directOfdmaScheduler() {
    return ap_enabled_ ? uav_broadcast_ofdma_scheduler_ : ofdma_scheduler_;
  }

  bool usesBsRadio(RouteStage route) const {
    return route == RouteStage::kApUplink ||
           route == RouteStage::kApDownlink ||
           route == RouteStage::kBsControl ||
           route == RouteStage::kBsUplink ||
           route == RouteStage::kBsDownlink;
  }

  const LinkModel &radioModel(RouteStage route) const {
    return usesBsRadio(route) ? bs_model_ : directRadioModel();
  }

  double radioSnrDb(const LinkQuality *link, const LinkModel &radio) const {
    if (!usable(link)) return -std::numeric_limits<double>::infinity();
    if (mode_ == "ideal") return snr(link);
    // LinkQuality.snr_db is referenced to the configured total 100 MHz.
    // Each assisted radio occupies only its own 50 MHz subband, so recompute
    // the receiver noise bandwidth without inventing additional TX power.
    return snr(link) +
           10.0 * std::log10(model_.config().bandwidth_hz /
                             radio.config().bandwidth_hz);
  }

  double allocatedSnrDb(const LinkQuality *link, int allocated_prbs) const {
    if (!usable(link)) return -std::numeric_limits<double>::infinity();
    // Sionna reports SNR using the configured full channel bandwidth. A UAV's
    // single configured TX-power budget is concentrated over its assigned
    // PRBs, never duplicated per receiver.
    const auto &radio = directRadioModel();
    return radioSnrDb(link, radio) + 10.0 * std::log10(
        static_cast<double>(radio.config().resource_blocks) /
        static_cast<double>(allocated_prbs));
  }

  double rateSelectionSnr(const UdpDatagram &datagram, int sender,
                          int allocated_prbs, double stamp) const {
    if (directRadioModel().config().fixed_mcs_index >= 0) return 0.0;
    double worst = std::numeric_limits<double>::infinity();
    for (const auto &receiver : datagram.receivers) {
      if (receiver.force_lossless) continue;
      const auto *link = linkFor({sender, receiver.receiver}, stamp);
      if (usable(link)) {
        worst = std::min(worst, allocatedSnrDb(link, allocated_prbs));
      }
    }
    return std::isfinite(worst) ? worst : -100.0;
  }

  bool udpExpired(const UdpDatagram &datagram, double stamp) const {
    const double ttl = kTopicPolicies[datagram.topic_index].ttl_s;
    return ttl > 0.0 && stamp - datagram.born_at > ttl;
  }

  void removeUavUdpFront(UavUdpQueue &queue) {
    if (queue.datagrams.empty()) return;
    queue.bytes -= std::min(queue.bytes, queue.datagrams.front().bytes);
    queue.datagrams.pop_front();
  }

  void expireUavUdpQueues(double stamp) {
    for (int sender = 0; sender < drone_count_; ++sender) {
      auto &queue = uav_udp_queues_[static_cast<std::size_t>(sender)];
      while (!queue.datagrams.empty() && udpExpired(queue.datagrams.front(), stamp)) {
        const auto &datagram = queue.datagrams.front();
        dropped_ttl_ += datagram.receivers.size();
        ++uav_udp_datagrams_dropped_ttl_;
        if (shouldLogUdpDatagram(datagram.packet_id)) {
          RCLCPP_INFO(
              get_logger(),
              "RACER_UAV_OFDMA_DROP slot=%lu sender=%d udp_type=%s "
              "packet_id=%lu packet_size=%zu allocated_prbs=0 "
              "remaining_bytes=%zu receiver=-1 snr_db=nan per=nan "
              "result=ttl_expired",
              static_cast<unsigned long>(uav_ofdma_slot_index_), sender,
              kTopicPolicies[datagram.topic_index].key.c_str(),
              static_cast<unsigned long>(datagram.packet_id), datagram.bytes,
              remainingBytes(datagram));
        }
        removeUavUdpFront(queue);
      }
    }
  }

  void finishUavPhysicalTransmission(int sender, UavUdpQueue &queue,
                                     double slot_end) {
    UdpDatagram completed = std::move(queue.datagrams.front());
    removeUavUdpFront(queue);
    const double random_jitter =
        jitter_s_ <= 0.0
            ? 0.0
            : std::uniform_real_distribution<double>(-jitter_s_, jitter_s_)(
                  uav_sender_rngs_[static_cast<std::size_t>(sender)]);
    completed.delivery_at =
        slot_end + std::max(0.0, base_latency_s_ + random_jitter);
    ++uav_physical_transmissions_completed_;
    uav_udp_physical_bytes_transmitted_ += completed.bytes;
    completed_uav_datagrams_.push_back(std::move(completed));
  }

  void processUavOfdmaSlot(double slot_start) {
    expireUavUdpQueues(slot_start);
    std::vector<int> active_senders;
    active_senders.reserve(static_cast<std::size_t>(drone_count_));
    for (int sender = 0; sender < drone_count_; ++sender) {
      if (!uav_udp_queues_[static_cast<std::size_t>(sender)].datagrams.empty()) {
        active_senders.push_back(sender);
      }
    }
    ++uav_ofdma_slots_processed_;
    if (active_senders.empty()) return;

    const auto allocations = directOfdmaScheduler().allocate(active_senders);
    if (allocations.empty()) return;
    ++uav_ofdma_active_slots_;
    uav_ofdma_peak_active_senders_ = std::max<std::uint64_t>(
        uav_ofdma_peak_active_senders_, allocations.size());
    std::vector<bool> transmitting(
        static_cast<std::size_t>(drone_count_), false);
    int slot_prbs = 0;
    for (const auto &allocation : allocations) {
      transmitting[static_cast<std::size_t>(allocation.sender)] = true;
      slot_prbs += allocation.prb_count;
    }
    uav_ofdma_max_allocated_prbs_per_slot_ = std::max(
        uav_ofdma_max_allocated_prbs_per_slot_, slot_prbs);

    for (const auto &allocation : allocations) {
      auto &queue =
          uav_udp_queues_[static_cast<std::size_t>(allocation.sender)];
      if (queue.datagrams.empty()) continue;
      auto &datagram = queue.datagrams.front();
      for (const auto &receiver : datagram.receivers) {
        markActiveLink(allocation.sender, receiver.receiver);
      }
      const double rate_snr = rateSelectionSnr(
          datagram, allocation.sender, allocation.prb_count, slot_start);
      const auto &radio = directRadioModel();
      const double available_bits = radio.bitsPerSlot(
          rate_snr, allocation.prb_count);
      const double transmitted_bits =
          std::min(datagram.remaining_bits, available_bits);
      if (!datagram.started) {
        datagram.started = true;
        datagram.first_slot = uav_ofdma_slot_index_;
        ++uav_physical_transmissions_started_;
        ++uav_tx_power_applications_;
        const std::string selected_mcs(radio.selectMcs(rate_snr).name);
        ++mcs_counts_[selected_mcs];
        ++uav_mcs_counts_[selected_mcs];
      }
      datagram.last_slot = uav_ofdma_slot_index_;
      datagram.last_allocated_prbs = allocation.prb_count;
      datagram.allocated_prb_slots +=
          static_cast<std::uint64_t>(allocation.prb_count);
      ++uav_ofdma_prb_allocation_events_;
      uav_ofdma_allocated_prb_slots_ += allocation.prb_count;
      direct_u2u_prb_slots_ += allocation.prb_count;

      for (auto &receiver : datagram.receivers) {
        receiver.last_allocated_prbs = allocation.prb_count;
        if (transmitting[static_cast<std::size_t>(receiver.receiver)]) {
          receiver.half_duplex_blocked = true;
          continue;
        }
        if (receiver.force_lossless) {
          receiver.received_bits += transmitted_bits;
          continue;
        }
        const auto *link =
            linkFor({allocation.sender, receiver.receiver}, slot_start);
        if (!usable(link)) {
          receiver.channel_unavailable = true;
          continue;
        }
        const double receiver_snr =
            allocatedSnrDb(link, allocation.prb_count);
        receiver.last_snr_db = receiver_snr;
        receiver.received_bits += transmitted_bits;
        receiver.snr_linear_bit_sum +=
            std::pow(10.0, receiver_snr / 10.0) * transmitted_bits;
      }
      datagram.remaining_bits =
          std::max(0.0, datagram.remaining_bits - transmitted_bits);
      if (shouldLogUdpDatagram(datagram.packet_id)) {
        RCLCPP_INFO(
            get_logger(),
            "RACER_UAV_OFDMA_SLOT slot=%lu sender=%d udp_type=%s "
            "packet_id=%lu packet_size=%zu prb_start=%d allocated_prbs=%d "
            "remaining_bytes=%zu receivers=%zu tx_power_dbm=%.3f",
            static_cast<unsigned long>(uav_ofdma_slot_index_),
            allocation.sender,
            kTopicPolicies[datagram.topic_index].key.c_str(),
            static_cast<unsigned long>(datagram.packet_id), datagram.bytes,
            allocation.first_prb, allocation.prb_count,
            remainingBytes(datagram), datagram.receivers.size(),
            uav_tx_power_dbm_);
      }
      if (datagram.remaining_bits <= 1.0e-9) {
        finishUavPhysicalTransmission(
            allocation.sender, queue, slot_start + radio.slotDuration());
      }
    }
  }

  void deliverCompletedUavDatagrams(double stamp) {
    for (auto iterator = completed_uav_datagrams_.begin();
         iterator != completed_uav_datagrams_.end();) {
      if (iterator->delivery_at > stamp + 1.0e-12) {
        ++iterator;
        continue;
      }
      UdpDatagram datagram = std::move(*iterator);
      iterator = completed_uav_datagrams_.erase(iterator);
      bool any_success = false;
      for (auto &receiver : datagram.receivers) {
        bool success = false;
        double receiver_snr = receiver.last_snr_db;
        double per = 1.0;
        const char *result = "per_failure";
        if (receiver.half_duplex_blocked) {
          result = "half_duplex_blocked";
          ++uav_udp_receiver_half_duplex_failures_;
        } else if (receiver.force_lossless) {
          success = true;
          per = 0.0;
          receiver_snr = std::numeric_limits<double>::infinity();
          result = "success_lossless_override";
          ++lossless_nearest_forwarded_packets_;
        } else if (receiver.channel_unavailable ||
                   receiver.received_bits + 1.0e-6 < datagram.total_bits) {
          result = "no_link";
          ++dropped_no_link_;
          ++uav_udp_receiver_no_link_failures_;
        } else {
          const double mean_linear_snr =
              receiver.snr_linear_bit_sum /
              std::max(1.0, receiver.received_bits);
          receiver_snr = 10.0 * std::log10(std::max(1.0e-30, mean_linear_snr));
          const auto &radio = directRadioModel();
          per = radio.packetErrorRate(receiver_snr, datagram.bytes);
          cumulative_initial_tbler_ +=
              radio.transportBlockErrorRate(receiver_snr);
          ++initial_tbler_samples_;
          success =
              std::uniform_real_distribution<double>(0.0, 1.0)(
                  linkRng({datagram.flow->origin_sender, receiver.receiver})) >=
              per;
          result = success ? "success" : "per_failure";
          if (!success) {
            ++dropped_per_;
            ++uav_udp_receiver_per_failures_;
          }
        }

        if (success) {
          any_success = true;
          ++delivered_packets_;
          delivered_bytes_ += datagram.bytes;
          cumulative_delay_s_ += stamp - datagram.enqueued_at;
          ++direct_delivered_packets_;
          ++uav_udp_receiver_successes_;
          PendingPacket delivered;
          delivered.message = datagram.message;
          delivered.flow = datagram.flow;
          delivered.topic_index = datagram.topic_index;
          delivered.route = RouteStage::kDirect;
          delivered.final_receiver = receiver.receiver;
          delivered.born_at = datagram.born_at;
          delivered.enqueued_at = datagram.enqueued_at;
          delivered.delivery_at = datagram.delivery_at;
          delivered.bytes = datagram.bytes;
          delivered.has_chunk_key = datagram.has_chunk_key;
          delivered.chunk_key = datagram.chunk_key;
          deliverToUav(delivered, receiver.receiver, stamp);
        }
        if (shouldLogUdpDatagram(datagram.packet_id)) {
          RCLCPP_INFO(
              get_logger(),
              "RACER_UAV_OFDMA_RX slot=%lu sender=%d udp_type=%s "
              "packet_id=%lu packet_size=%zu allocated_prbs=%d "
              "remaining_bytes=0 receiver=%d snr_db=%.3f per=%.6f "
              "result=%s",
              static_cast<unsigned long>(datagram.last_slot),
              datagram.flow->origin_sender,
              kTopicPolicies[datagram.topic_index].key.c_str(),
              static_cast<unsigned long>(datagram.packet_id), datagram.bytes,
              receiver.last_allocated_prbs, receiver.receiver, receiver_snr,
              per, result);
        }
      }
      if (any_success) ++uav_physical_transmissions_with_success_;
    }
  }

  void advanceUavOfdma(double stamp) {
    if (!uav_ofdma_timeline_initialized_) {
      uav_ofdma_timeline_initialized_ = true;
      uav_ofdma_next_slot_start_ = stamp;
    }
    const double slot = directRadioModel().slotDuration();
    while (uav_ofdma_next_slot_start_ + slot <= stamp + 1.0e-12) {
      processUavOfdmaSlot(uav_ofdma_next_slot_start_);
      uav_ofdma_next_slot_start_ += slot;
      ++uav_ofdma_slot_index_;
    }
    deliverCompletedUavDatagrams(stamp);
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

  bool hasPendingChunk(const LinkKey &link, const ChunkKey &chunk,
                       RouteStage route) const {
    const auto found = queues_.find(link);
    if (found == queues_.end()) return false;
    return std::any_of(
        found->second.packets.begin(), found->second.packets.end(),
        [&chunk, route](const PendingPacket &packet) {
          return packet.route == route && packet.has_chunk_key &&
                 packet.chunk_key == chunk;
        });
  }

  std::vector<ChunkKey> ownedMissingChunks(
      int owner,
      const std::unordered_set<ChunkKey, ChunkKeyHash> &available,
      const std::unordered_set<ChunkKey, ChunkKeyHash> &known) const {
    auto candidates = sortedChunks(available, known);
    candidates.erase(
        std::remove_if(candidates.begin(), candidates.end(),
                       [owner](const ChunkKey &key) {
                         return key.owner != owner + 1;
                       }),
        candidates.end());
    return candidates;
  }

  void scheduleRlUpload(int owner, double stamp) {
    const auto candidates = ownedMissingChunks(
        owner, uav_chunks_[static_cast<std::size_t>(owner)], bs_chunks_);
    const std::size_t limit = std::min(
        candidates.size(),
        static_cast<std::size_t>(bs_max_uplink_chunks_per_turn_));
    const auto topic_index = chunkDataTopicIndex();
    for (std::size_t offset = 0; offset < limit; ++offset) {
      const auto &key = candidates[offset];
      const LinkKey link{owner, ap_node_id_};
      if (hasPendingChunk(link, key, RouteStage::kBsUplink)) continue;
      const auto &cached = chunk_repository_.at(key);
      auto flow = makeCentralFlow(owner, topic_index, cached.bytes, stamp);
      if (enqueue(owner, ap_node_id_, topic_index, cached.message, flow,
                  RouteStage::kBsUplink, -1, stamp, &key)) {
        ++bs_incremental_chunks_scheduled_uplink_;
      }
    }
  }

  void scheduleRlRelay(int owner, int receiver, double stamp) {
    if (owner == receiver) return;
    const auto candidates = ownedMissingChunks(
        owner, bs_chunks_, uav_chunks_[static_cast<std::size_t>(receiver)]);
    const std::size_t limit = std::min(
        candidates.size(),
        static_cast<std::size_t>(bs_max_downlink_chunks_per_turn_));
    const auto topic_index = chunkDataTopicIndex();
    for (std::size_t offset = 0; offset < limit; ++offset) {
      const auto &key = candidates[offset];
      const LinkKey link{ap_node_id_, receiver};
      if (hasPendingChunk(link, key, RouteStage::kBsDownlink)) continue;
      const auto &cached = chunk_repository_.at(key);
      auto flow = makeCentralFlow(owner, topic_index, cached.bytes, stamp);
      if (enqueue(ap_node_id_, receiver, topic_index, cached.message, flow,
                  RouteStage::kBsDownlink, receiver, stamp, &key)) {
        ++bs_missing_chunks_scheduled_downlink_;
      }
    }
  }

  bool readRlAction() {
    std::ifstream stream(rl_bs_action_path_);
    std::uint64_t epoch{};
    int action_drone_count{};
    if (!(stream >> epoch >> action_drone_count) ||
        action_drone_count != drone_count_ || epoch <= rl_action_epoch_) {
      return false;
    }
    std::vector<std::vector<bool>> relay(
        static_cast<std::size_t>(drone_count_),
        std::vector<bool>(static_cast<std::size_t>(drone_count_), false));
    std::vector<bool> upload(static_cast<std::size_t>(drone_count_), false);
    int bit{};
    for (int owner = 0; owner < drone_count_; ++owner) {
      for (int receiver = 0; receiver < drone_count_; ++receiver) {
        if (owner == receiver) continue;
        if (!(stream >> bit) || (bit != 0 && bit != 1)) return false;
        relay[static_cast<std::size_t>(owner)]
             [static_cast<std::size_t>(receiver)] = bit != 0;
      }
    }
    for (int owner = 0; owner < drone_count_; ++owner) {
      if (!(stream >> bit) || (bit != 0 && bit != 1)) return false;
      upload[static_cast<std::size_t>(owner)] = bit != 0;
    }
    rl_action_epoch_ = epoch;
    rl_relay_action_ = std::move(relay);
    rl_upload_action_ = std::move(upload);
    return true;
  }

  bool beginRlDecision(double stamp) {
    if (!rl_bs_scheduler_enabled_ ||
        (rl_have_decision_stamp_ &&
         stamp - rl_last_decision_s_ + 1.0e-12 < rl_bs_decision_period_s_)) {
      return false;
    }
    rl_have_decision_stamp_ = true;
    rl_last_decision_s_ = stamp;
    if (readRlAction()) {
      for (int owner = 0; owner < drone_count_; ++owner) {
        if (rl_upload_action_[static_cast<std::size_t>(owner)]) {
          scheduleRlUpload(owner, stamp);
        }
        for (int receiver = 0; receiver < drone_count_; ++receiver) {
          if (rl_relay_action_[static_cast<std::size_t>(owner)]
                              [static_cast<std::size_t>(receiver)]) {
            scheduleRlRelay(owner, receiver, stamp);
          }
        }
      }
    }
    return true;
  }

  std::size_t missingBytes(
      int owner,
      const std::unordered_set<ChunkKey, ChunkKeyHash> &known) const {
    std::size_t bytes{};
    const auto &source = uav_chunks_[static_cast<std::size_t>(owner)];
    for (const auto &key : source) {
      if (key.owner != owner + 1 || known.find(key) != known.end()) continue;
      const auto found = chunk_repository_.find(key);
      if (found != chunk_repository_.end()) bytes += found->second.bytes;
    }
    return bytes;
  }

  std::size_t queuedRouteBytes(const LinkKey &link, RouteStage route,
                               int owner = -1) const {
    const auto found = queues_.find(link);
    if (found == queues_.end()) return 0U;
    std::size_t bytes{};
    for (const auto &packet : found->second.packets) {
      if (packet.route != route) continue;
      if (owner >= 0 &&
          (!packet.has_chunk_key || packet.chunk_key.owner != owner + 1)) {
        continue;
      }
      bytes += packet.bytes;
    }
    return bytes;
  }

  double telemetrySnr(int sender, int receiver, double stamp) const {
    if (sender == receiver) return 0.0;
    const auto *link = linkFor({sender, receiver}, stamp);
    if (!usable(link)) return -120.0;
    const bool bs_link = ap_enabled_ &&
                         (sender == ap_node_id_ || receiver == ap_node_id_);
    return radioSnrDb(link, bs_link ? bs_model_ : directRadioModel());
  }

  void writeRlState(double stamp) {
    if (!rl_bs_scheduler_enabled_) return;
    ++rl_state_sequence_;
    const std::filesystem::path output_path(rl_bs_state_path_);
    if (!output_path.parent_path().empty()) {
      std::error_code error;
      std::filesystem::create_directories(output_path.parent_path(), error);
    }
    const auto temporary = output_path.string() + ".tmp";
    std::ofstream json(temporary, std::ios::trunc);
    if (!json) return;
    json << "{\"sequence\":" << rl_state_sequence_
         << ",\"action_epoch\":" << rl_action_epoch_
         << ",\"sim_time_s\":" << stamp
         << ",\"positions\":[";
    for (int drone = 0; drone < drone_count_; ++drone) {
      if (drone) json << ',';
      const auto &value = uav_positions_[static_cast<std::size_t>(drone)];
      json << '[' << value[0] << ',' << value[1] << ',' << value[2] << ']';
    }
    json << "],\"velocities\":[";
    for (int drone = 0; drone < drone_count_; ++drone) {
      if (drone) json << ',';
      const auto &value = uav_velocities_[static_cast<std::size_t>(drone)];
      json << '[' << value[0] << ',' << value[1] << ',' << value[2] << ']';
    }
    json << "],\"yaws\":[";
    for (int drone = 0; drone < drone_count_; ++drone) {
      if (drone) json << ',';
      json << uav_yaws_[static_cast<std::size_t>(drone)];
    }
    json << "],\"fsm_states\":[";
    for (int drone = 0; drone < drone_count_; ++drone) {
      if (drone) json << ',';
      json << (uav_position_valid_[static_cast<std::size_t>(drone)]
                   ? "\"EXPLORE\""
                   : "\"UNKNOWN\"");
    }
    json << "],\"channel_snr_db\":[";
    for (int sender = 0; sender < radio_node_count_; ++sender) {
      if (sender) json << ',';
      json << '[';
      for (int receiver = 0; receiver < radio_node_count_; ++receiver) {
        if (receiver) json << ',';
        json << telemetrySnr(sender, receiver, stamp);
      }
      json << ']';
    }
    json << "],\"pair_aoi_s\":[";
    for (int owner = 0; owner < drone_count_; ++owner) {
      if (owner) json << ',';
      json << '[';
      for (int receiver = 0; receiver < drone_count_; ++receiver) {
        if (receiver) json << ',';
        const double aoi = owner == receiver
                               ? 0.0
                               : std::max(0.0, stamp - last_uav_info_received_s_
                                                        [static_cast<std::size_t>(owner)]
                                                        [static_cast<std::size_t>(receiver)]);
        json << aoi;
      }
      json << ']';
    }
    json << "],\"bs_aoi_s\":[";
    for (int owner = 0; owner < drone_count_; ++owner) {
      if (owner) json << ',';
      json << std::max(0.0, stamp - last_bs_info_received_s_
                                        [static_cast<std::size_t>(owner)]);
    }
    json << "],\"pair_missing_bytes\":[";
    for (int owner = 0; owner < drone_count_; ++owner) {
      if (owner) json << ',';
      json << '[';
      for (int receiver = 0; receiver < drone_count_; ++receiver) {
        if (receiver) json << ',';
        json << (owner == receiver
                     ? 0U
                     : missingBytes(owner, uav_chunks_[static_cast<std::size_t>(receiver)]));
      }
      json << ']';
    }
    json << "],\"bs_missing_bytes\":[";
    for (int owner = 0; owner < drone_count_; ++owner) {
      if (owner) json << ',';
      json << missingBytes(owner, bs_chunks_);
    }
    json << "],\"uplink_queue_bytes\":[";
    for (int owner = 0; owner < drone_count_; ++owner) {
      if (owner) json << ',';
      json << queuedRouteBytes({owner, ap_node_id_}, RouteStage::kBsUplink,
                               owner);
    }
    json << "],\"relay_queue_bytes\":[";
    for (int owner = 0; owner < drone_count_; ++owner) {
      if (owner) json << ',';
      json << '[';
      for (int receiver = 0; receiver < drone_count_; ++receiver) {
        if (receiver) json << ',';
        json << (owner == receiver
                     ? 0U
                     : queuedRouteBytes({ap_node_id_, receiver},
                                        RouteStage::kBsDownlink, owner));
      }
      json << ']';
    }
    json << "],\"information_version_gap\":[";
    for (int owner = 0; owner < drone_count_; ++owner) {
      if (owner) json << ',';
      json << '[';
      for (int receiver = 0; receiver < drone_count_; ++receiver) {
        if (receiver) json << ',';
        std::size_t count{};
        for (const auto &key : uav_chunks_[static_cast<std::size_t>(owner)]) {
          if (key.owner == owner + 1 &&
              uav_chunks_[static_cast<std::size_t>(receiver)].find(key) ==
                  uav_chunks_[static_cast<std::size_t>(receiver)].end()) {
            ++count;
          }
        }
        json << (owner == receiver ? 0U : count);
      }
      std::size_t bs_count{};
      for (const auto &key : uav_chunks_[static_cast<std::size_t>(owner)]) {
        if (key.owner == owner + 1 && bs_chunks_.find(key) == bs_chunks_.end()) {
          ++bs_count;
        }
      }
      json << ',' << bs_count << ']';
    }
    json << "],\"trajectory_summary\":[";
    for (int drone = 0; drone < drone_count_; ++drone) {
      if (drone) json << ',';
      const auto &summary = trajectory_summaries_[static_cast<std::size_t>(drone)];
      const auto &goal = summary.valid
                             ? summary.goal
                             : uav_positions_[static_cast<std::size_t>(drone)];
      json << "{\"goal_position\":[" << goal[0] << ',' << goal[1] << ','
           << goal[2] << "],\"trajectory_length\":" << summary.length
           << ",\"expected_execution_time\":"
           << summary.expected_execution_time << '}';
    }
    json << "],\"map_summary\":{\"representation\":"
            "\"chunk_version_counts\",\"bs_known_chunks\":"
         << bs_chunks_.size() << ",\"local_known_chunks\":[";
    for (int drone = 0; drone < drone_count_; ++drone) {
      if (drone) json << ',';
      json << uav_chunks_[static_cast<std::size_t>(drone)].size();
    }
    json << "]},\"coverage\":0.0"
         << ",\"redundant_exploration_ratio\":"
         << redundantExplorationRatio()
         << ",\"bs_global_map_iou\":" << bsGlobalMapIou()
         << ",\"bs_uplink_prb_slots\":" << bs_uplink_prb_slots_
         << ",\"bs_downlink_prb_slots\":" << bs_downlink_prb_slots_
         << ",\"direct_u2u_prb_slots\":" << direct_u2u_prb_slots_
         << '}';
    json.close();
    if (json) std::rename(temporary.c_str(), output_path.string().c_str());
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
    if (rl_bs_scheduler_enabled_) return true;
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
    const auto &radio = radioModel(front.route);
    const double link_snr = radioSnrDb(link, radio);
    const std::string selected_mcs(radio.selectMcs(link_snr).name);
    ++mcs_counts_[selected_mcs];
    if (usesBsRadio(front.route)) {
      ++bs_mcs_counts_[selected_mcs];
    } else {
      ++uav_mcs_counts_[selected_mcs];
    }
    cumulative_initial_tbler_ += radio.transportBlockErrorRate(link_snr);
    ++initial_tbler_samples_;
    const double random_jitter =
        jitter_s_ <= 0.0
            ? 0.0
            : std::uniform_real_distribution<double>(-jitter_s_, jitter_s_)(
                  linkRng(key));
    double transmission_started_at =
        std::max(transmission_cursor, front.delivery_at);
    if (usesBsRadio(front.route)) {
      // UAV<->BS UL/DL share one 50 MHz resource pool. Reserving a common
      // serialization timeline prevents concurrent queues from each claiming
      // the full BS subband.
      transmission_started_at =
          std::max(transmission_started_at, bs_radio_available_at_);
    }
    const double serialization_delay =
        radio.serializationDelay(link_snr, front.bytes);
    if (usesBsRadio(front.route)) {
      bs_radio_available_at_ = transmission_started_at + serialization_delay;
    }
    front.delivery_at =
        transmission_started_at +
        serialization_delay +
        std::max(0.0, base_latency_s_ + random_jitter);
    const auto occupied_slots = static_cast<std::uint64_t>(std::max(
        1.0, std::ceil(serialization_delay / radio.slotDuration())));
    const auto prb_slots = occupied_slots *
        static_cast<std::uint64_t>(radio.config().resource_blocks);
    if (front.route == RouteStage::kBsUplink ||
        front.route == RouteStage::kApUplink) {
      bs_uplink_prb_slots_ += prb_slots;
    } else if (front.route == RouteStage::kBsDownlink ||
               front.route == RouteStage::kBsControl ||
               front.route == RouteStage::kApDownlink) {
      bs_downlink_prb_slots_ += prb_slots;
    } else if (front.route == RouteStage::kDirect) {
      direct_u2u_prb_slots_ += prb_slots;
    }
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
    const int information_owner =
        packet.has_chunk_key
            ? packet.chunk_key.owner - 1
            : (packet.flow ? packet.flow->origin_sender : -1);
    if (information_owner >= 0 && information_owner < drone_count_) {
      last_uav_info_received_s_[static_cast<std::size_t>(information_owner)]
                               [receiver_index] = stamp;
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
      const int information_owner =
          packet.has_chunk_key ? packet.chunk_key.owner - 1
                               : (packet.flow ? packet.flow->origin_sender : -1);
      if (information_owner >= 0 && information_owner < drone_count_) {
        last_bs_info_received_s_[static_cast<std::size_t>(information_owner)] =
            stamp;
      }
      if (packet.has_chunk_key &&
          bs_chunks_.insert(packet.chunk_key).second) {
        ++bs_incremental_chunks_received_uplink_;
      }
      if (kTopicPolicies.at(packet.topic_index).key == "chunk_data" &&
          packet.message) {
        racer_fidelity_msgs::msg::ChunkData chunk;
        if (deserialize(packet.message, chunk)) {
          applyOccupiedVoxelUpdate(chunk, bs_occupied_voxels_);
        }
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
    if (shared_uav_ofdma_enabled_) {
      advanceUavOfdma(stamp);
      if (!ap_enabled_) return;
    }
    const bool rl_state_due = beginRlDecision(stamp);
    if (!rl_bs_scheduler_enabled_) beginBsTurn(stamp);
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
        const auto &radio = radioModel(front.route);
        const double per =
            mode_ == "ideal"
                ? 0.0
                : radio.packetErrorRate(radioSnrDb(link, radio), front.bytes);
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
    if (!rl_bs_scheduler_enabled_) maybeFinishBsTurn(stamp);
    if (rl_state_due) writeRlState(stamp);
  }

  void publishStatistics() {
    racer_sionna_interfaces::msg::CommStatistics message;
    message.stamp = now();
    const double task_stamp = now().seconds();
    task_quality_history_.push_back(
        {task_stamp, redundantExplorationRatio(), bsGlobalMapIou()});
    exportObservedOccupiedVoxels();
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
    for (const auto &queue : uav_udp_queues_) {
      message.queued_packets += queue.datagrams.size();
      message.queued_bytes += queue.bytes;
    }
    message.queued_packets += completed_uav_datagrams_.size();
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
    const auto &total_phy = model_.config();
    const auto &phy = directRadioModel().config();
    const auto &bs_phy = bs_model_.config();
    const auto &fixed_mcs = directRadioModel().selectMcs(0.0);
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
         << ",\"uav_udp_directed_unicast\":"
         << (uav_udp_directed_unicast_ ? "true" : "false")
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
         << ",\"uav_transport\":\"UDP\""
         << ",\"shared_uav_ofdma_enabled\":"
         << (shared_uav_ofdma_enabled_ ? "true" : "false")
         << ",\"uav_udp_header_bytes\":" << udp_header_bytes_
         << ",\"uav_udp_retransmissions\":0"
         << ",\"uav_ofdma_total_prbs\":"
         << directRadioModel().config().resource_blocks
         << ",\"uav_ofdma_slot_duration_ms\":"
         << 1000.0 * directRadioModel().slotDuration()
         << ",\"uav_ofdma_scheduler\":\"round_robin_equal_share\""
         << ",\"uav_ofdma_half_duplex\":true"
         << ",\"uav_ofdma_diagnostic_logging\":"
         << (ofdma_diagnostic_logging_ ? "true" : "false")
         << ",\"uav_ofdma_log_packet_stride\":"
         << ofdma_log_packet_stride_
         << ",\"uav_udp_datagrams_enqueued\":"
         << uav_udp_datagrams_enqueued_
         << ",\"uav_udp_unicast_datagrams\":"
         << uav_udp_unicast_datagrams_
         << ",\"uav_udp_multicast_datagrams\":"
         << uav_udp_multicast_datagrams_
         << ",\"uav_udp_datagrams_dropped_queue\":"
         << uav_udp_datagrams_dropped_queue_
         << ",\"uav_udp_datagrams_dropped_ttl\":"
         << uav_udp_datagrams_dropped_ttl_
         << ",\"uav_udp_receiver_attempts\":"
         << uav_udp_receiver_attempts_
         << ",\"uav_udp_receiver_successes\":"
         << uav_udp_receiver_successes_
         << ",\"uav_udp_receiver_per_failures\":"
         << uav_udp_receiver_per_failures_
         << ",\"uav_udp_receiver_no_link_failures\":"
         << uav_udp_receiver_no_link_failures_
         << ",\"uav_udp_receiver_half_duplex_failures\":"
         << uav_udp_receiver_half_duplex_failures_
         << ",\"uav_udp_physical_bytes_enqueued\":"
         << uav_udp_physical_bytes_enqueued_
         << ",\"uav_udp_physical_bytes_transmitted\":"
         << uav_udp_physical_bytes_transmitted_
         << ",\"uav_physical_transmissions_started\":"
         << uav_physical_transmissions_started_
         << ",\"uav_physical_transmissions_completed\":"
         << uav_physical_transmissions_completed_
         << ",\"uav_physical_transmissions_with_success\":"
         << uav_physical_transmissions_with_success_
         << ",\"uav_tx_power_applications\":"
         << uav_tx_power_applications_
         << ",\"uav_ofdma_slots_processed\":"
         << uav_ofdma_slots_processed_
         << ",\"uav_ofdma_active_slots\":"
         << uav_ofdma_active_slots_
         << ",\"uav_ofdma_prb_allocation_events\":"
         << uav_ofdma_prb_allocation_events_
         << ",\"uav_ofdma_allocated_prb_slots\":"
         << uav_ofdma_allocated_prb_slots_
         << ",\"uav_ofdma_peak_active_senders\":"
         << uav_ofdma_peak_active_senders_
         << ",\"uav_ofdma_max_allocated_prbs_per_slot\":"
         << uav_ofdma_max_allocated_prbs_per_slot_
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
         << ",\"rl_bs_scheduler_enabled\":"
         << (rl_bs_scheduler_enabled_ ? "true" : "false")
         << ",\"rl_bs_action_epoch\":" << rl_action_epoch_
         << ",\"rl_bs_state_sequence\":" << rl_state_sequence_
         << ",\"bs_uplink_prb_slots\":" << bs_uplink_prb_slots_
         << ",\"bs_downlink_prb_slots\":" << bs_downlink_prb_slots_
         << ",\"direct_u2u_prb_slots\":" << direct_u2u_prb_slots_
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
         << ",\"redundant_exploration_ratio\":"
         << redundantExplorationRatio()
         << ",\"bs_global_map_iou\":" << bsGlobalMapIou()
         << ",\"ground_truth_occupied_voxels\":"
         << ground_truth_occupied_voxels_.size()
         << ",\"task_quality_history\":[";
    for (std::size_t index = 0; index < task_quality_history_.size(); ++index) {
      if (index) json << ',';
      const auto &sample = task_quality_history_[index];
      json << "{\"time_s\":" << sample.time_s
           << ",\"redundant_exploration_ratio\":" << sample.redundancy
           << ",\"bs_global_map_iou\":" << sample.map_iou << '}';
    }
    json << ']'
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
         << ",\"bandwidth_hz\":" << total_phy.bandwidth_hz
         << ",\"total_bandwidth_hz\":" << total_phy.bandwidth_hz
         << ",\"spectrum_partition_enabled\":"
         << (ap_enabled_ ? "true" : "false")
         << ",\"subcarrier_spacing_hz\":"
         << total_phy.subcarrier_spacing_hz
         << ",\"resource_blocks\":" << total_phy.resource_blocks
         << ",\"uav_broadcast_bandwidth_hz\":" << phy.bandwidth_hz
         << ",\"uav_broadcast_resource_blocks\":" << phy.resource_blocks
         << ",\"bs_bandwidth_hz\":" << bs_phy.bandwidth_hz
         << ",\"bs_resource_blocks\":" << bs_phy.resource_blocks
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
         << ",\"bs_mcs_mode\":\"adaptive\""
         << ",\"bs_fixed_mcs_index\":" << bs_phy.fixed_mcs_index
         << ",\"mean_selected_initial_tbler\":" << mean_initial_tbler
         << ",\"mcs_counts\":{\"QPSK\":" << mcs_counts_["QPSK"]
         << ",\"16QAM\":" << mcs_counts_["16QAM"]
         << ",\"64QAM\":" << mcs_counts_["64QAM"]
         << ",\"256QAM\":" << mcs_counts_["256QAM"] << "}"
         << ",\"uav_mcs_counts\":{\"QPSK\":" << uav_mcs_counts_["QPSK"]
         << ",\"16QAM\":" << uav_mcs_counts_["16QAM"]
         << ",\"64QAM\":" << uav_mcs_counts_["64QAM"]
         << ",\"256QAM\":" << uav_mcs_counts_["256QAM"] << "}"
         << ",\"bs_mcs_counts\":{\"QPSK\":" << bs_mcs_counts_["QPSK"]
         << ",\"16QAM\":" << bs_mcs_counts_["16QAM"]
         << ",\"64QAM\":" << bs_mcs_counts_["64QAM"]
         << ",\"256QAM\":" << bs_mcs_counts_["256QAM"] << "}}"
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
  bool uav_udp_directed_unicast_{true};
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
  int udp_header_bytes_{};
  bool ofdma_diagnostic_logging_{true};
  int ofdma_log_packet_stride_{1000};
  int max_retries_{};
  int bs_max_retries_{};
  double retry_backoff_s_{};
  double bs_min_turn_s_{};
  std::size_t bs_control_bytes_{};
  int bs_max_downlink_chunks_per_turn_{};
  int bs_max_uplink_chunks_per_turn_{};
  bool rl_bs_scheduler_enabled_{false};
  std::string rl_bs_action_path_;
  std::string rl_bs_state_path_;
  double rl_bs_decision_period_s_{};
  std::string ground_truth_occupied_voxels_path_;
  std::string observed_occupied_voxels_path_;
  bool require_ground_truth_map_{false};
  LinkModel model_;
  LinkModel uav_broadcast_model_;
  LinkModel bs_model_;
  SharedOfdmaScheduler ofdma_scheduler_;
  SharedOfdmaScheduler uav_broadcast_ofdma_scheduler_;
  bool shared_uav_ofdma_enabled_{false};
  double bs_radio_available_at_{};

  std::unordered_map<LinkKey, LinkQuality, LinkKeyHash> links_;
  std::unordered_map<LinkKey, LinkQueue, LinkKeyHash> queues_;
  std::vector<UavUdpQueue> uav_udp_queues_;
  std::deque<UdpDatagram> completed_uav_datagrams_;
  std::vector<std::mt19937> uav_sender_rngs_;
  std::unordered_map<LinkKey, std::mt19937, LinkKeyHash> link_rngs_;
  std::unordered_map<LinkKey, std::chrono::steady_clock::time_point,
                     LinkKeyHash> active_links_;
  bool active_links_dirty_{false};
  bool have_active_link_publish_wall_{false};
  std::chrono::steady_clock::time_point last_active_link_publish_wall_{};
  std::unordered_map<std::string, std::uint64_t> link_model_counts_;
  std::unordered_map<std::string, std::uint64_t> mcs_counts_;
  std::unordered_map<std::string, std::uint64_t> uav_mcs_counts_;
  std::unordered_map<std::string, std::uint64_t> bs_mcs_counts_;
  std::unordered_map<ChunkKey, CachedChunk, ChunkKeyHash> chunk_repository_;
  std::vector<std::unordered_set<ChunkKey, ChunkKeyHash>> uav_chunks_;
  std::vector<std::unordered_set<std::uint32_t>> uav_observed_voxels_;
  std::vector<std::array<double, 3>> uav_positions_;
  std::vector<std::array<double, 3>> uav_velocities_;
  std::vector<double> uav_yaws_;
  std::vector<bool> uav_position_valid_;
  std::vector<std::vector<double>> last_uav_info_received_s_;
  std::vector<double> last_bs_info_received_s_;
  std::vector<TrajectorySummary> trajectory_summaries_;
  std::unordered_set<ChunkKey, ChunkKeyHash> bs_chunks_;
  std::unordered_set<std::uint32_t> bs_occupied_voxels_;
  std::unordered_set<std::uint32_t> ground_truth_occupied_voxels_;
  std::vector<TaskQualitySample> task_quality_history_;
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
  bool rl_have_decision_stamp_{false};
  double rl_last_decision_s_{};
  std::uint64_t rl_action_epoch_{};
  std::uint64_t rl_state_sequence_{};
  std::vector<std::vector<bool>> rl_relay_action_;
  std::vector<bool> rl_upload_action_;

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
  bool uav_ofdma_timeline_initialized_{false};
  double uav_ofdma_next_slot_start_{};
  std::uint64_t uav_ofdma_slot_index_{};
  std::uint64_t uav_udp_next_packet_id_{};
  std::uint64_t uav_udp_datagrams_enqueued_{};
  std::uint64_t uav_udp_unicast_datagrams_{};
  std::uint64_t uav_udp_multicast_datagrams_{};
  std::uint64_t uav_udp_datagrams_dropped_queue_{};
  std::uint64_t uav_udp_datagrams_dropped_ttl_{};
  std::uint64_t uav_udp_receiver_attempts_{};
  std::uint64_t uav_udp_receiver_successes_{};
  std::uint64_t uav_udp_receiver_per_failures_{};
  std::uint64_t uav_udp_receiver_no_link_failures_{};
  std::uint64_t uav_udp_receiver_half_duplex_failures_{};
  std::uint64_t uav_udp_physical_bytes_enqueued_{};
  std::uint64_t uav_udp_physical_bytes_transmitted_{};
  std::uint64_t uav_physical_transmissions_started_{};
  std::uint64_t uav_physical_transmissions_completed_{};
  std::uint64_t uav_physical_transmissions_with_success_{};
  std::uint64_t uav_tx_power_applications_{};
  std::uint64_t uav_ofdma_slots_processed_{};
  std::uint64_t uav_ofdma_active_slots_{};
  std::uint64_t uav_ofdma_prb_allocation_events_{};
  std::uint64_t uav_ofdma_allocated_prb_slots_{};
  std::uint64_t uav_ofdma_peak_active_senders_{};
  int uav_ofdma_max_allocated_prbs_per_slot_{};
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
  std::uint64_t bs_uplink_prb_slots_{};
  std::uint64_t bs_downlink_prb_slots_{};
  std::uint64_t direct_u2u_prb_slots_{};
};

}  // namespace racer_sionna_comm

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<racer_sionna_comm::CommunicationProxy>());
  rclcpp::shutdown();
  return 0;
}
