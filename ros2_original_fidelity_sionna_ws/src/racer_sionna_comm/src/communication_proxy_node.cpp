#include <racer_sionna_comm/link_model.hpp>

#include <racer_sionna_interfaces/msg/comm_statistics.hpp>
#include <racer_sionna_interfaces/msg/link_quality.hpp>
#include <racer_sionna_interfaces/msg/link_quality_array.hpp>

#include <racer_fidelity_msgs/msg/chunk_data.hpp>
#include <racer_fidelity_msgs/msg/chunk_stamps.hpp>

#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/serialization.hpp>

#include <algorithm>
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
    {"chunk_stamps", "racer_fidelity_msgs/msg/ChunkStamps", 4, 3.0, true},
    {"chunk_data", "racer_fidelity_msgs/msg/ChunkData", 2, 10.0, true},
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
                declare_parameter<int>("transport_block_bytes", 1200))}) {
    random_seed_ = declare_parameter<int>("random_seed", 42);
    if (mode_ != "ideal" && mode_ != "sionna" &&
        mode_ != "sionna_hybrid") {
      throw std::runtime_error(
          "mode must be ideal, sionna, or sionna_hybrid");
    }
    if (topology_ != "distributed" && topology_ != "ap_assisted" &&
        topology_ != "bs_round_robin") {
      throw std::runtime_error(
          "network_topology must be distributed, ap_assisted, or "
          "bs_round_robin");
    }
    if (drone_count_ < 1 || queue_capacity_bytes_ == 0U ||
        max_retries_ < 0 || base_latency_s_ < 0.0 || jitter_s_ < 0.0 ||
        retry_backoff_s_ < 0.0 || bs_min_turn_s_ < 0.0 ||
        bs_control_bytes_ == 0U || bs_max_downlink_chunks_per_turn_ < 1 ||
        bs_max_uplink_chunks_per_turn_ < 1 || carrier_frequency_hz_ <= 0.0 ||
        bs_array_rows_ < 1 || bs_array_cols_ < 1 || uav_array_rows_ < 1 ||
        uav_array_cols_ < 1) {
      throw std::runtime_error("invalid communication proxy parameters");
    }
    ap_enabled_ = topology_ == "ap_assisted" ||
                  topology_ == "bs_round_robin";
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

    link_subscription_ = create_subscription<LinkQualityArray>(
        "/racer_sionna/link_quality", rclcpp::QoS(20).reliable(),
        [this](LinkQualityArray::ConstSharedPtr message) {
          onLinkQuality(*message);
        });
    statistics_publisher_ =
        create_publisher<racer_sionna_interfaces::msg::CommStatistics>(
            "/racer_sionna/comm_statistics", rclcpp::QoS(10).reliable());
    scheduler_timer_ = create_wall_timer(
        std::chrono::milliseconds(2), [this]() { schedulerTick(); });
    statistics_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this]() { publishStatistics(); });

    RCLCPP_INFO(
        get_logger(),
        "source-faithful communication proxy ready: mode=%s topology=%s "
        "drones=%d radio_nodes=%d bs_node_id=%d seed=%d phy=NR-LDPC "
        "waveform=CP-OFDM",
        mode_.c_str(), topology_.c_str(), drone_count_, radio_node_count_,
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
    ChunkKey chunk_key{};
    observeUavTransmit(sender, topic_index, message, &chunk_key);
    const bool has_chunk_key = chunk_key.owner > 0 && chunk_key.index > 0U;
    auto flow = std::make_shared<DeliveryFlow>();
    flow->origin_sender = sender;
    flow->topic_index = topic_index;
    flow->born_at = stamp;
    flow->bytes = bytes;
    flow->delivered.assign(static_cast<std::size_t>(drone_count_), false);
    flow->delivered[static_cast<std::size_t>(sender)] = true;

    const auto intended_receivers = static_cast<std::uint64_t>(
        std::max(0, drone_count_ - 1));
    logical_attempted_packets_ += intended_receivers;
    logical_attempted_bytes_ += intended_receivers * bytes;
    for (int receiver = 0; receiver < drone_count_; ++receiver) {
      if (receiver == sender) continue;
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
    const double stamp = now().seconds();
    const std::size_t bytes = override_bytes > 0U
                                  ? override_bytes
                                  : 64U + (message ? message->size() : 0U);
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

    const LinkKey key{sender, receiver};
    auto found = queues_.find(key);
    if (found == queues_.end()) {
      ++dropped_queue_;
      return false;
    }
    auto &queue = found->second;
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
        if (failed && reliable && front.attempts < max_retries_) {
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
    std::ostringstream json;
    json << "{\"network_topology\":\"" << topology_
         << "\",\"ap_enabled\":" << (ap_enabled_ ? "true" : "false")
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
         << ",\"bs_upa\":\"" << bs_array_rows_ << "x"
         << bs_array_cols_ << "\""
         << ",\"uav_upa\":\"" << uav_array_rows_ << "x"
         << uav_array_cols_ << "\""
         << ",\"target_initial_tbler\":"
         << phy.target_initial_tbler
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
  double carrier_frequency_hz_{};
  double bs_tx_power_dbm_{};
  double uav_tx_power_dbm_{};
  int bs_array_rows_{};
  int bs_array_cols_{};
  int uav_array_rows_{};
  int uav_array_cols_{};
  bool ap_enabled_{false};
  bool bs_round_robin_enabled_{false};
  int ap_node_id_{};
  int radio_node_count_{};
  int random_seed_{};
  double base_latency_s_{};
  double jitter_s_{};
  std::size_t queue_capacity_bytes_{};
  int max_retries_{};
  double retry_backoff_s_{};
  double bs_min_turn_s_{};
  std::size_t bs_control_bytes_{};
  int bs_max_downlink_chunks_per_turn_{};
  int bs_max_uplink_chunks_per_turn_{};
  LinkModel model_;

  std::unordered_map<LinkKey, LinkQuality, LinkKeyHash> links_;
  std::unordered_map<LinkKey, LinkQueue, LinkKeyHash> queues_;
  std::unordered_map<LinkKey, std::mt19937, LinkKeyHash> link_rngs_;
  std::unordered_map<std::string, std::uint64_t> link_model_counts_;
  std::unordered_map<std::string, std::uint64_t> mcs_counts_;
  std::unordered_map<ChunkKey, CachedChunk, ChunkKeyHash> chunk_repository_;
  std::vector<std::unordered_set<ChunkKey, ChunkKeyHash>> uav_chunks_;
  std::unordered_set<ChunkKey, ChunkKeyHash> bs_chunks_;
  std::vector<std::vector<rclcpp::GenericPublisher::SharedPtr>> publishers_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::vector<std::vector<std::shared_ptr<rclcpp::SerializedMessage>>>
      ap_latest_messages_;
  rclcpp::Subscription<LinkQualityArray>::SharedPtr link_subscription_;
  rclcpp::Publisher<racer_sionna_interfaces::msg::CommStatistics>::SharedPtr
      statistics_publisher_;
  rclcpp::TimerBase::SharedPtr scheduler_timer_;
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
