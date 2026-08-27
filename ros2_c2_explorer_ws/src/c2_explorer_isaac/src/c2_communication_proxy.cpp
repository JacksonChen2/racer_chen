#include <c2_explorer_msgs/msg/comm_statistics.hpp>
#include <c2_explorer_msgs/msg/link_quality_array.hpp>

#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

struct TopicPolicy {
  std::string key;
  std::string type;
  double ttl_s;
};

const std::vector<TopicPolicy> kTopics{
    {"drone_state", "c2_explorer_msgs/msg/DroneState", 0.30},
    {"meeting_opt", "c2_explorer_msgs/msg/MeetingOpt", 2.0},
    {"meeting_opt_res", "c2_explorer_msgs/msg/MeetingOptResponse", 2.0},
    {"rendezvous", "c2_explorer_msgs/msg/Rendezvous", 2.0},
    {"trajectory", "c2_explorer_msgs/msg/Bspline", 1.0},
    {"heartbeat", "c2_explorer_msgs/msg/Heartbeat", 0.30},
    {"chunk_stamps", "c2_explorer_msgs/msg/ChunkStamps", 3.0},
    {"chunk_data", "c2_explorer_msgs/msg/ChunkData", 10.0},
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

double timeSeconds(const builtin_interfaces::msg::Time &value) {
  return static_cast<double>(value.sec) + 1.0e-9 * value.nanosec;
}

std::string txTopic(int sender, const TopicPolicy &policy) {
  return "/c2_communication/tx/drone_" + std::to_string(sender) + "/" +
         policy.key;
}

std::string rxTopic(int receiver, const TopicPolicy &policy) {
  return "/c2_communication/rx/drone_" + std::to_string(receiver) + "/" +
         policy.key;
}

struct Mcs {
  int modulation_order;
  double code_rate;
  double threshold_db;
};

constexpr Mcs kMcsTable[] = {
    {2, 0.4902, 1.0},
    {4, 0.4785, 7.0},
    {6, 0.6504, 14.0},
    {8, 0.7363, 21.0},
};

}  // namespace

class C2CommunicationProxy final : public rclcpp::Node {
 public:
  C2CommunicationProxy()
      : Node("c2_communication_proxy"),
        mode_(declare_parameter<std::string>("mode", "ideal")),
        drone_count_(declare_parameter<int>("drone_count", 5)),
        bandwidth_hz_(declare_parameter<double>("bandwidth_hz", 100.0e6)),
        subcarrier_spacing_hz_(
            declare_parameter<double>("subcarrier_spacing_hz", 120.0e3)),
        resource_blocks_(declare_parameter<int>("resource_blocks", 66)),
        data_re_efficiency_(
            declare_parameter<double>("data_re_efficiency", 0.82)),
        target_tbler_(
            declare_parameter<double>("target_initial_tbler", 0.10)),
        tbler_slope_db_(declare_parameter<double>("tbler_slope_db", 1.35)),
        transport_block_bytes_(static_cast<std::size_t>(
            declare_parameter<int>("transport_block_bytes", 1200))),
        base_latency_s_(
            1.0e-3 * declare_parameter<double>("base_latency_ms", 0.5)),
        jitter_s_(1.0e-3 * declare_parameter<double>("jitter_ms", 0.1)),
        queue_capacity_bytes_(static_cast<std::size_t>(
            declare_parameter<int>("queue_capacity_bytes", 16777216))),
        max_retries_(declare_parameter<int>("max_retries", 0)),
        retry_backoff_s_(
            1.0e-3 * declare_parameter<double>("retry_backoff_ms", 0.125)),
        random_seed_(declare_parameter<int>("random_seed", 42)) {
    if ((mode_ != "ideal" && mode_ != "sionna") || drone_count_ < 2 ||
        bandwidth_hz_ <= 0.0 || subcarrier_spacing_hz_ <= 0.0 ||
        resource_blocks_ <= 0 || data_re_efficiency_ <= 0.0 ||
        data_re_efficiency_ > 1.0 || target_tbler_ <= 0.0 ||
        target_tbler_ >= 1.0 || tbler_slope_db_ <= 0.0 ||
        transport_block_bytes_ == 0U || queue_capacity_bytes_ == 0U ||
        max_retries_ < 0 || base_latency_s_ < 0.0 || jitter_s_ < 0.0 ||
        retry_backoff_s_ < 0.0) {
      throw std::runtime_error("invalid C2 communication parameters");
    }
    if (12.0 * resource_blocks_ * subcarrier_spacing_hz_ > bandwidth_hz_) {
      throw std::runtime_error("configured NR resource blocks exceed bandwidth");
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable();
    publishers_.resize(kTopics.size());
    for (std::size_t topic = 0; topic < kTopics.size(); ++topic) {
      for (int receiver = 0; receiver < drone_count_; ++receiver) {
        publishers_[topic].push_back(create_generic_publisher(
            rxTopic(receiver, kTopics[topic]), kTopics[topic].type, qos));
      }
      for (int sender = 0; sender < drone_count_; ++sender) {
        subscriptions_.push_back(create_generic_subscription(
            txTopic(sender, kTopics[topic]), kTopics[topic].type, qos,
            [this, sender, topic](
                std::shared_ptr<rclcpp::SerializedMessage> message) {
              onTransmit(sender, topic, std::move(message));
            }));
      }
    }

    for (int sender = 0; sender < drone_count_; ++sender) {
      for (int receiver = 0; receiver < drone_count_; ++receiver) {
        if (sender == receiver) continue;
        const LinkKey key{sender, receiver};
        queues_.emplace(key, LinkQueue{});
        std::seed_seq seed{random_seed_, sender, receiver, 0x43324558,
                           0x53494f4e};
        random_.emplace(key, std::mt19937(seed));
      }
    }

    link_subscription_ =
        create_subscription<c2_explorer_msgs::msg::LinkQualityArray>(
            "/racer_sionna/link_quality", rclcpp::QoS(20).reliable(),
            [this](c2_explorer_msgs::msg::LinkQualityArray::ConstSharedPtr msg) {
              onLinkQuality(*msg);
            });
    stats_publisher_ =
        create_publisher<c2_explorer_msgs::msg::CommStatistics>(
            "/c2_communication/statistics", rclcpp::QoS(10).reliable());
    scheduler_timer_ = create_wall_timer(
        std::chrono::milliseconds(2), [this]() { schedulerTick(); });
    statistics_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this]() { publishStatistics(); });
    RCLCPP_INFO(get_logger(),
                "C2 communication proxy ready: mode=%s drones=%d seed=%d "
                "phy=5G-NR-LDPC/CP-OFDM retries=%d",
                mode_.c_str(), drone_count_, random_seed_, max_retries_);
  }

 private:
  struct Pending {
    std::shared_ptr<rclcpp::SerializedMessage> message;
    std::size_t topic{};
    double born_at{};
    double delivery_at{};
    std::size_t bytes{};
  };

  struct LinkQueue {
    std::deque<Pending> packets;
    std::size_t bytes{};
    double next_available{};
  };

  const Mcs &selectMcs(double snr_db) const {
    const Mcs *selected = &kMcsTable[0];
    for (const auto &candidate : kMcsTable) {
      if (snr_db + 1.0e-12 < candidate.threshold_db) break;
      selected = &candidate;
    }
    return *selected;
  }

  double transportBlockErrorRate(double snr_db) const {
    const auto &mcs = selectMcs(snr_db);
    const double odds = (1.0 - target_tbler_) / target_tbler_;
    const double exponent = std::clamp(
        (snr_db - mcs.threshold_db) / tbler_slope_db_, -60.0, 60.0);
    return std::clamp(1.0 / (1.0 + odds * std::exp(exponent)), 0.0, 1.0);
  }

  double bitRate(double snr_db) const {
    const auto &mcs = selectMcs(snr_db);
    const double slots_per_second =
        1000.0 * subcarrier_spacing_hz_ / 15.0e3;
    const double resource_elements =
        12.0 * resource_blocks_ * 14.0 * slots_per_second *
        data_re_efficiency_;
    return std::max(1.0e3, resource_elements * mcs.modulation_order *
                                  mcs.code_rate);
  }

  double serializationDelay(double snr_db, std::size_t bytes) const {
    const double slot = 1.0e-3 * 15.0e3 / subcarrier_spacing_hz_;
    const double raw = 8.0 * static_cast<double>(bytes) / bitRate(snr_db);
    return std::max(slot, std::ceil(raw / slot) * slot);
  }

  double packetErrorRate(double snr_db, std::size_t bytes) const {
    const double tbler = transportBlockErrorRate(snr_db);
    const std::size_t blocks = std::max<std::size_t>(
        1U, (bytes + transport_block_bytes_ - 1U) /
                transport_block_bytes_);
    return std::clamp(
        1.0 - std::pow(1.0 - tbler, static_cast<double>(blocks)), 0.0, 1.0);
  }

  void onLinkQuality(const c2_explorer_msgs::msg::LinkQualityArray &message) {
    for (const auto &link : message.links) {
      if (link.sender_id < 0 || link.sender_id >= drone_count_ ||
          link.receiver_id < 0 || link.receiver_id >= drone_count_ ||
          link.sender_id == link.receiver_id) {
        continue;
      }
      links_[{link.sender_id, link.receiver_id}] = link;
      if (link.model == "sionna_exact") {
        ++sionna_exact_samples_;
      } else if (link.model == "sionna_cache_corrected") {
        ++sionna_cache_corrected_samples_;
      } else if (link.model == "radio_map_cache") {
        ++radio_map_cache_samples_;
      } else if (link.model == "unavailable") {
        ++unavailable_link_samples_;
      }
    }
  }

  void onTransmit(int sender, std::size_t topic,
                  std::shared_ptr<rclcpp::SerializedMessage> message) {
    if (sender < 0 || sender >= drone_count_ || topic >= kTopics.size() ||
        !message) {
      return;
    }
    const double stamp = now().seconds();
    const std::size_t bytes = 64U + message->size();
    for (int receiver = 0; receiver < drone_count_; ++receiver) {
      if (receiver == sender) continue;
      if (mode_ == "ideal") {
        ++attempted_packets_;
        attempted_bytes_ += bytes;
        publishers_[topic][receiver]->publish(*message);
        ++delivered_packets_;
        delivered_bytes_ += bytes;
        ++ideal_forwarded_packets_;
      } else {
        enqueueSionna(sender, receiver, topic, message, bytes, stamp);
      }
    }
  }

  void enqueueSionna(
      int sender, int receiver, std::size_t topic,
      const std::shared_ptr<rclcpp::SerializedMessage> &message,
      std::size_t bytes, double stamp) {
    const LinkKey key{sender, receiver};
    auto &queue = queues_.at(key);
    if (queue.bytes + bytes > queue_capacity_bytes_) {
      ++attempted_packets_;
      attempted_bytes_ += bytes;
      ++dropped_queue_;
      return;
    }
    const auto link = links_.find(key);
    if (link == links_.end() || link->second.model == "unavailable" ||
        timeSeconds(link->second.valid_until) + 1.0e-9 < stamp ||
        !std::isfinite(link->second.snr_db)) {
      ++attempted_packets_;
      attempted_bytes_ += bytes;
      ++dropped_no_link_;
      return;
    }

    const double snr = link->second.snr_db;
    const double per = packetErrorRate(snr, bytes);
    const double air_time = serializationDelay(snr, bytes);
    double available = std::max(stamp, queue.next_available);
    bool success = false;
    auto &engine = random_.at(key);
    std::uniform_real_distribution<double> draw(0.0, 1.0);
    for (int attempt = 0; attempt <= max_retries_; ++attempt) {
      ++attempted_packets_;
      attempted_bytes_ += bytes;
      available += air_time;
      if (draw(engine) >= per) {
        success = true;
        break;
      }
      if (attempt < max_retries_) {
        ++retried_packets_;
        available += retry_backoff_s_;
      }
    }
    queue.next_available = available;
    if (!success) {
      ++dropped_per_;
      return;
    }

    std::uniform_real_distribution<double> jitter(-jitter_s_, jitter_s_);
    const double delivery =
        available + std::max(0.0, base_latency_s_ + jitter(engine));
    if (delivery - stamp > kTopics[topic].ttl_s) {
      ++dropped_ttl_;
      return;
    }
    queue.packets.push_back(Pending{message, topic, stamp, delivery, bytes});
    queue.bytes += bytes;
  }

  void schedulerTick() {
    if (mode_ != "sionna") return;
    const double stamp = now().seconds();
    for (auto &[key, queue] : queues_) {
      while (!queue.packets.empty() &&
             queue.packets.front().delivery_at <= stamp + 1.0e-9) {
        Pending packet = std::move(queue.packets.front());
        queue.packets.pop_front();
        queue.bytes -= packet.bytes;
        if (stamp - packet.born_at > kTopics[packet.topic].ttl_s) {
          ++dropped_ttl_;
          continue;
        }
        publishers_[packet.topic][key.receiver]->publish(*packet.message);
        ++delivered_packets_;
        delivered_bytes_ += packet.bytes;
        cumulative_delay_s_ += stamp - packet.born_at;
      }
    }
  }

  void publishStatistics() {
    std::uint64_t queued_packets = 0;
    std::uint64_t queued_bytes = 0;
    for (const auto &[key, queue] : queues_) {
      (void)key;
      queued_packets += queue.packets.size();
      queued_bytes += queue.bytes;
    }
    c2_explorer_msgs::msg::CommStatistics message;
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
    message.queued_packets = queued_packets;
    message.queued_bytes = queued_bytes;
    message.mean_delivery_delay_ms =
        delivered_packets_ == 0
            ? 0.0F
            : static_cast<float>(1000.0 * cumulative_delay_s_ /
                                 delivered_packets_);
    stats_publisher_->publish(message);
    RCLCPP_INFO(
        get_logger(),
        "C2_COMMUNICATION_STATS {\"mode\":\"%s\",\"attempted_packets\":%llu,"
        "\"delivered_packets\":%llu,\"dropped_no_link\":%llu,"
        "\"dropped_per\":%llu,\"dropped_queue\":%llu,"
        "\"dropped_ttl\":%llu,\"retried_packets\":%llu,"
        "\"queued_packets\":%llu,\"queued_bytes\":%llu,"
        "\"mean_delivery_delay_ms\":%.6f,\"ideal_forwarded_packets\":%llu,"
        "\"sionna_exact_samples\":%llu,"
        "\"sionna_cache_corrected_samples\":%llu,"
        "\"radio_map_cache_samples\":%llu,"
        "\"unavailable_link_samples\":%llu}",
        mode_.c_str(), static_cast<unsigned long long>(attempted_packets_),
        static_cast<unsigned long long>(delivered_packets_),
        static_cast<unsigned long long>(dropped_no_link_),
        static_cast<unsigned long long>(dropped_per_),
        static_cast<unsigned long long>(dropped_queue_),
        static_cast<unsigned long long>(dropped_ttl_),
        static_cast<unsigned long long>(retried_packets_),
        static_cast<unsigned long long>(queued_packets),
        static_cast<unsigned long long>(queued_bytes),
        message.mean_delivery_delay_ms,
        static_cast<unsigned long long>(ideal_forwarded_packets_),
        static_cast<unsigned long long>(sionna_exact_samples_),
        static_cast<unsigned long long>(sionna_cache_corrected_samples_),
        static_cast<unsigned long long>(radio_map_cache_samples_),
        static_cast<unsigned long long>(unavailable_link_samples_));
  }

  std::string mode_;
  int drone_count_;
  double bandwidth_hz_;
  double subcarrier_spacing_hz_;
  int resource_blocks_;
  double data_re_efficiency_;
  double target_tbler_;
  double tbler_slope_db_;
  std::size_t transport_block_bytes_;
  double base_latency_s_;
  double jitter_s_;
  std::size_t queue_capacity_bytes_;
  int max_retries_;
  double retry_backoff_s_;
  int random_seed_;

  std::vector<std::vector<rclcpp::GenericPublisher::SharedPtr>> publishers_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  rclcpp::Subscription<c2_explorer_msgs::msg::LinkQualityArray>::SharedPtr
      link_subscription_;
  rclcpp::Publisher<c2_explorer_msgs::msg::CommStatistics>::SharedPtr
      stats_publisher_;
  rclcpp::TimerBase::SharedPtr scheduler_timer_, statistics_timer_;
  std::unordered_map<LinkKey, c2_explorer_msgs::msg::LinkQuality, LinkKeyHash>
      links_;
  std::unordered_map<LinkKey, LinkQueue, LinkKeyHash> queues_;
  std::unordered_map<LinkKey, std::mt19937, LinkKeyHash> random_;

  std::uint64_t attempted_packets_{0};
  std::uint64_t delivered_packets_{0};
  std::uint64_t dropped_no_link_{0};
  std::uint64_t dropped_per_{0};
  std::uint64_t dropped_queue_{0};
  std::uint64_t dropped_ttl_{0};
  std::uint64_t retried_packets_{0};
  std::uint64_t attempted_bytes_{0};
  std::uint64_t delivered_bytes_{0};
  std::uint64_t ideal_forwarded_packets_{0};
  std::uint64_t sionna_exact_samples_{0};
  std::uint64_t sionna_cache_corrected_samples_{0};
  std::uint64_t radio_map_cache_samples_{0};
  std::uint64_t unavailable_link_samples_{0};
  double cumulative_delay_s_{0.0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<C2CommunicationProxy>());
  rclcpp::shutdown();
  return 0;
}
