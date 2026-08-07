#include <racer_3d_interfaces/msg/comm_packet.hpp>
#include <racer_3d_interfaces/msg/comm_statistics.hpp>
#include <racer_3d_interfaces/msg/link_quality_array.hpp>

#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

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

namespace racer_3d_sionna_comm {
namespace {

using CommPacket = racer_3d_interfaces::msg::CommPacket;
using LinkQuality = racer_3d_interfaces::msg::LinkQuality;

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

double durationSeconds(const builtin_interfaces::msg::Duration &value) {
  return static_cast<double>(value.sec) + 1.0e-9 * value.nanosec;
}

std::size_t wireBytes(const CommPacket &packet) {
  constexpr std::size_t kEnvelopeBytes = 64U;
  return kEnvelopeBytes + packet.payload.size();
}

}  // namespace

class CommunicationEmulator final : public rclcpp::Node {
 public:
  CommunicationEmulator() : Node("racer_3d_communication_emulator") {
    mode_ = declare_parameter<std::string>("mode", "sionna_hybrid");
    drone_count_ = declare_parameter<int>("drone_count", 3);
    range_only_m_ = declare_parameter<double>("range_only_m", 12.0);
    bandwidth_hz_ = declare_parameter<double>("bandwidth_hz", 20.0e6);
    mac_efficiency_ = declare_parameter<double>("mac_efficiency", 0.55);
    base_latency_s_ =
        1.0e-3 * declare_parameter<double>("base_latency_ms", 20.0);
    jitter_s_ = 1.0e-3 * declare_parameter<double>("jitter_ms", 10.0);
    queue_capacity_bytes_ = static_cast<std::size_t>(
        declare_parameter<int>("queue_capacity_bytes", 262144));
    mtu_bytes_ = static_cast<std::size_t>(
        declare_parameter<int>("mtu_bytes", 1200));
    snr_midpoint_db_ = declare_parameter<double>("snr_midpoint_db", 7.0);
    snr_slope_db_ = declare_parameter<double>("snr_slope_db", 1.8);
    max_retries_ = declare_parameter<int>("max_retries", 3);
    retry_backoff_s_ =
        1.0e-3 * declare_parameter<double>("retry_backoff_ms", 8.0);
    const int random_seed = declare_parameter<int>("random_seed", 42);
    rng_.seed(static_cast<std::mt19937::result_type>(random_seed));

    if (mode_ != "ideal" && mode_ != "range_only" &&
        mode_ != "sionna" && mode_ != "sionna_hybrid") {
      throw std::runtime_error(
          "mode must be ideal, range_only, sionna, or sionna_hybrid");
    }
    if (drone_count_ <= 0 || bandwidth_hz_ <= 0.0 ||
        mac_efficiency_ <= 0.0 || queue_capacity_bytes_ == 0U ||
        mtu_bytes_ == 0U || snr_slope_db_ <= 0.0 || max_retries_ < 0) {
      throw std::runtime_error("invalid communication-emulator parameters");
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable();
    tx_sub_ = create_subscription<CommPacket>(
        "/racer_3d/comm/tx", qos,
        [this](CommPacket::ConstSharedPtr message) { onTransmit(*message); });
    link_sub_ =
        create_subscription<racer_3d_interfaces::msg::LinkQualityArray>(
            "/racer_3d/link_quality", rclcpp::QoS(10).reliable(),
            [this](
                racer_3d_interfaces::msg::LinkQualityArray::ConstSharedPtr msg) {
              onLinkQuality(*msg);
            });
    for (int id = 0; id < drone_count_; ++id) {
      rx_publishers_.push_back(create_publisher<CommPacket>(
          "/drone_" + std::to_string(id) + "/comm/rx", qos));
    }
    statistics_pub_ =
        create_publisher<racer_3d_interfaces::msg::CommStatistics>(
            "/racer_3d/comm/statistics", rclcpp::QoS(10).reliable());
    scheduler_timer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(0.002),
        [this]() { schedulerTick(); });
    statistics_timer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(1.0),
        [this]() { publishStatistics(); });

    RCLCPP_INFO(get_logger(),
                "communication emulator ready: mode=%s drones=%d seed=%d",
                mode_.c_str(), drone_count_, random_seed);
  }

 private:
  struct PendingPacket {
    CommPacket packet;
    double enqueued_at{};
    double delivery_at{};
    std::size_t bytes{};
    int attempts{};
    bool transmitting{false};
  };

  struct LinkQueue {
    std::deque<PendingPacket> packets;
    std::size_t bytes{};
  };

  void onLinkQuality(
      const racer_3d_interfaces::msg::LinkQualityArray &message) {
    for (const auto &link : message.links) {
      if (link.sender_id < 0 || link.receiver_id < 0 ||
          link.sender_id == link.receiver_id) {
        continue;
      }
      links_[{link.sender_id, link.receiver_id}] = link;
    }
  }

  void onTransmit(const CommPacket &message) {
    if (message.sender_id < 0 || message.sender_id >= drone_count_) {
      ++dropped_no_link_;
      return;
    }
    if (message.receiver_id == -1) {
      for (int receiver = 0; receiver < drone_count_; ++receiver) {
        if (receiver != message.sender_id) {
          CommPacket copy = message;
          copy.receiver_id = receiver;
          enqueue(std::move(copy));
        }
      }
      return;
    }
    if (message.receiver_id < 0 || message.receiver_id >= drone_count_ ||
        message.receiver_id == message.sender_id) {
      ++dropped_no_link_;
      return;
    }
    enqueue(message);
  }

  bool expired(const CommPacket &packet, double stamp) const {
    const double ttl = durationSeconds(packet.ttl);
    if (ttl <= 0.0) {
      return false;
    }
    return stamp - timeSeconds(packet.source_stamp) > ttl;
  }

  void enqueue(CommPacket packet) {
    const double stamp = now().seconds();
    ++attempted_packets_;
    const std::size_t bytes = wireBytes(packet);
    attempted_bytes_ += bytes;
    if (expired(packet, stamp)) {
      ++dropped_ttl_;
      return;
    }
    packet.payload_size = static_cast<std::uint32_t>(packet.payload.size());
    const LinkKey key{packet.sender_id, packet.receiver_id};
    auto &queue = queues_[key];
    if (bytes > queue_capacity_bytes_) {
      ++dropped_queue_;
      return;
    }
    // Protect control-plane traffic from a burst of low-priority map chunks.
    // A packet may evict queued (never in-flight) lower-priority packets; it
    // can never interrupt the frame currently being serialized on the link.
    while (queue.bytes + bytes > queue_capacity_bytes_) {
      auto candidate = queue.packets.end();
      for (auto iterator = queue.packets.end();
           iterator != queue.packets.begin();) {
        --iterator;
        if (!iterator->transmitting &&
            iterator->packet.priority < packet.priority) {
          candidate = iterator;
          break;
        }
      }
      if (candidate == queue.packets.end()) {
        ++dropped_queue_;
        return;
      }
      queue.bytes -= std::min(queue.bytes, candidate->bytes);
      queue.packets.erase(candidate);
      ++dropped_queue_;
    }

    PendingPacket pending;
    pending.packet = std::move(packet);
    pending.enqueued_at = stamp;
    pending.bytes = bytes;
    auto position = queue.packets.begin();
    if (position != queue.packets.end() && position->transmitting) {
      ++position;
    }
    while (position != queue.packets.end() &&
           position->packet.priority >= pending.packet.priority) {
      ++position;
    }
    queue.packets.insert(position, std::move(pending));
    queue.bytes += bytes;
  }

  const LinkQuality *linkFor(const LinkKey &key, double stamp) const {
    const auto found = links_.find(key);
    if (found == links_.end()) {
      return nullptr;
    }
    if (timeSeconds(found->second.valid_until) + 1.0e-9 < stamp) {
      return nullptr;
    }
    return &found->second;
  }

  bool usable(const LinkQuality *link) const {
    if (mode_ == "ideal") {
      return true;
    }
    if (link == nullptr || !std::isfinite(link->distance_m)) {
      return false;
    }
    if (mode_ == "range_only") {
      return link->distance_m <= range_only_m_;
    }
    return std::isfinite(link->snr_db) &&
           link->model != "unavailable";
  }

  double snr(const LinkQuality *link) const {
    return mode_ == "ideal" ? 40.0 : static_cast<double>(link->snr_db);
  }

  double linkRate(double snr_db) const {
    double spectral_efficiency = 0.25;
    if (snr_db >= 4.0) spectral_efficiency = 0.5;
    if (snr_db >= 7.0) spectral_efficiency = 1.0;
    if (snr_db >= 10.0) spectral_efficiency = 2.0;
    if (snr_db >= 16.0) spectral_efficiency = 4.0;
    if (snr_db >= 23.0) spectral_efficiency = 6.0;
    return std::max(1.0e3,
                    bandwidth_hz_ * spectral_efficiency * mac_efficiency_);
  }

  double packetErrorRate(double snr_db, std::size_t bytes) const {
    if (mode_ == "ideal") {
      return 0.0;
    }
    const double exponent = std::clamp(
        (snr_db - snr_midpoint_db_) / snr_slope_db_, -60.0, 60.0);
    const double frame_per = 1.0 / (1.0 + std::exp(exponent));
    const std::size_t frames =
        std::max<std::size_t>(1U, (bytes + mtu_bytes_ - 1U) / mtu_bytes_);
    return std::clamp(1.0 - std::pow(1.0 - frame_per, frames), 0.0, 1.0);
  }

  bool startFront(const LinkKey &key, LinkQueue &queue, double stamp) {
    if (queue.packets.empty()) return false;
    auto &front = queue.packets.front();
    if (front.transmitting) return true;
    if (front.delivery_at > stamp) return true;
    const auto *link = linkFor(key, stamp);
    if (!usable(link)) {
      removeFront(queue);
      ++dropped_no_link_;
      return false;
    }
    const double rate = linkRate(snr(link));
    const double serialization = 8.0 * front.bytes / rate;
    const double jitter =
        jitter_s_ <= 0.0
            ? 0.0
            : std::uniform_real_distribution<double>(-jitter_s_, jitter_s_)(rng_);
    front.delivery_at =
        stamp + serialization + std::max(0.0, base_latency_s_ + jitter);
    front.transmitting = true;
    return true;
  }

  void removeFront(LinkQueue &queue) {
    if (queue.packets.empty()) return;
    queue.bytes -= std::min(queue.bytes, queue.packets.front().bytes);
    queue.packets.pop_front();
  }

  void schedulerTick() {
    const double stamp = now().seconds();
    for (auto &[key, queue] : queues_) {
      while (!queue.packets.empty()) {
        if (!startFront(key, queue, stamp)) {
          continue;
        }
        auto &front = queue.packets.front();
        if (front.delivery_at > stamp) {
          break;
        }
        if (expired(front.packet, stamp)) {
          ++dropped_ttl_;
          removeFront(queue);
          continue;
        }
        const auto *link = linkFor(key, stamp);
        if (!usable(link)) {
          ++dropped_no_link_;
          removeFront(queue);
          continue;
        }
        const double per = packetErrorRate(snr(link), front.bytes);
        const bool failed =
            std::uniform_real_distribution<double>(0.0, 1.0)(rng_) < per;
        if (failed && front.packet.reliable && front.attempts < max_retries_) {
          ++front.attempts;
          ++retried_packets_;
          front.transmitting = false;
          front.delivery_at = stamp + retry_backoff_s_;
          break;
        }
        if (failed) {
          ++dropped_per_;
          removeFront(queue);
          continue;
        }

        rx_publishers_[static_cast<std::size_t>(front.packet.receiver_id)]->
            publish(front.packet);
        ++delivered_packets_;
        delivered_bytes_ += front.bytes;
        cumulative_delay_s_ += stamp - front.enqueued_at;
        removeFront(queue);
      }
    }
  }

  void publishStatistics() {
    racer_3d_interfaces::msg::CommStatistics message;
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
    std::size_t queued_packets = 0U;
    std::size_t queued_bytes = 0U;
    for (const auto &[key, queue] : queues_) {
      (void)key;
      queued_packets += queue.packets.size();
      queued_bytes += queue.bytes;
    }
    message.queued_packets = queued_packets;
    message.queued_bytes = queued_bytes;
    message.mean_delivery_delay_ms =
        delivered_packets_ == 0U
            ? 0.0F
            : static_cast<float>(
                  1000.0 * cumulative_delay_s_ / delivered_packets_);
    statistics_pub_->publish(message);
  }

  std::string mode_;
  int drone_count_{};
  double range_only_m_{};
  double bandwidth_hz_{};
  double mac_efficiency_{};
  double base_latency_s_{};
  double jitter_s_{};
  std::size_t queue_capacity_bytes_{};
  std::size_t mtu_bytes_{};
  double snr_midpoint_db_{};
  double snr_slope_db_{};
  int max_retries_{};
  double retry_backoff_s_{};
  std::mt19937 rng_;

  std::unordered_map<LinkKey, LinkQuality, LinkKeyHash> links_;
  std::unordered_map<LinkKey, LinkQueue, LinkKeyHash> queues_;
  std::vector<rclcpp::Publisher<CommPacket>::SharedPtr> rx_publishers_;
  rclcpp::Subscription<CommPacket>::SharedPtr tx_sub_;
  rclcpp::Subscription<
      racer_3d_interfaces::msg::LinkQualityArray>::SharedPtr link_sub_;
  rclcpp::Publisher<
      racer_3d_interfaces::msg::CommStatistics>::SharedPtr statistics_pub_;
  rclcpp::TimerBase::SharedPtr scheduler_timer_;
  rclcpp::TimerBase::SharedPtr statistics_timer_;

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
};

}  // namespace racer_3d_sionna_comm

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<racer_3d_sionna_comm::CommunicationEmulator>());
  rclcpp::shutdown();
  return 0;
}
