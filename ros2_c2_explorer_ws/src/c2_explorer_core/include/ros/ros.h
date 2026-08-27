#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <atomic>
#include <any>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <typeindex>
#include <unordered_map>
#include <utility>

namespace ros {

namespace detail {
// These objects must be process-wide, including across the algorithm shared
// library and its executable.  Out-of-line definitions avoid creating one
// hidden ROS 2 node per DSO while retaining ROS 1's single callback queue.
std::mutex &globalMutex();
std::shared_ptr<rclcpp::Node> &globalNode();
std::string &requestedNodeName();
std::atomic<unsigned long> &serviceClientIndex();

inline std::string parameterName(std::string value) {
  std::replace(value.begin(), value.end(), '/', '.');
  while (!value.empty() && value.front() == '.') value.erase(value.begin());
  while (!value.empty() && value.back() == '.') value.pop_back();
  return value;
}

inline rclcpp::Logger logger() {
  std::lock_guard<std::mutex> lock(globalMutex());
  return globalNode() ? globalNode()->get_logger()
                     : rclcpp::get_logger("c2_explorer");
}

inline bool throttle(double period, const char *file, int line) {
  static std::mutex mutex;
  static std::unordered_map<std::string,
      std::chrono::steady_clock::time_point> last_by_callsite;
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex);
  const std::string callsite = std::string(file) + ":" + std::to_string(line);
  auto &last = last_by_callsite[callsite];
  if (last.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now - last).count() < period) {
    return false;
  }
  last = now;
  return true;
}

inline std::shared_ptr<rclcpp::Node> ensureNode() {
  std::lock_guard<std::mutex> lock(globalMutex());
  if (!globalNode()) {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    globalNode() = std::make_shared<rclcpp::Node>(requestedNodeName(), options);
  }
  return globalNode();
}

// ROS1 communicates a service callback's boolean return value separately from
// its response payload.  ROS2 callbacks cannot reject a request that way, so
// the ported LKH servers encode that transport-level success bit in the
// otherwise-unused `empty` response field.  Preserve ServiceClient::call()'s
// ROS1 semantics for those two legacy solver services while leaving arbitrary
// response types unchanged.
template <typename Response, typename = void>
struct HasLegacyServiceSuccess : std::false_type {};

template <typename Response>
struct HasLegacyServiceSuccess<
    Response, std::void_t<decltype(std::declval<Response>().empty)>>
    : std::true_type {};

template <typename Response>
inline bool legacyServiceSucceeded(const Response &response) {
  if constexpr (HasLegacyServiceSuccess<Response>::value) {
    return response.empty != 0;
  }
  return true;
}
}  // namespace detail

class Duration;

class Time {
 public:
  Time() : nanoseconds_(0) {}
  explicit Time(double seconds)
      : nanoseconds_(static_cast<std::int64_t>(std::llround(seconds * 1.0e9))) {}
  Time(const builtin_interfaces::msg::Time &value)
      : nanoseconds_(rclcpp::Time(value, RCL_ROS_TIME).nanoseconds()) {}

  static Time now() {
    const auto node = detail::ensureNode();
    Time result;
    result.nanoseconds_ = node->now().nanoseconds();
    return result;
  }

  double toSec() const { return static_cast<double>(nanoseconds_) * 1.0e-9; }
  std::int64_t toNSec() const { return nanoseconds_; }
  bool isZero() const { return nanoseconds_ == 0; }

  operator builtin_interfaces::msg::Time() const {
    builtin_interfaces::msg::Time value;
    std::int64_t sec = nanoseconds_ / 1000000000LL;
    std::int64_t rem = nanoseconds_ % 1000000000LL;
    if (rem < 0) {
      --sec;
      rem += 1000000000LL;
    }
    value.sec = static_cast<std::int32_t>(sec);
    value.nanosec = static_cast<std::uint32_t>(rem);
    return value;
  }

  friend Duration operator-(const Time &first, const Time &second);
  friend Time operator+(const Time &time, const Duration &duration);
  friend bool operator<(const Time &first, const Time &second) {
    return first.nanoseconds_ < second.nanoseconds_;
  }
  friend bool operator>(const Time &first, const Time &second) {
    return second < first;
  }
  friend bool operator<=(const Time &first, const Time &second) {
    return !(second < first);
  }
  friend bool operator>=(const Time &first, const Time &second) {
    return !(first < second);
  }
  friend bool operator==(const Time &first, const Time &second) {
    return first.nanoseconds_ == second.nanoseconds_;
  }
  friend bool operator!=(const Time &first, const Time &second) {
    return !(first == second);
  }

 private:
  std::int64_t nanoseconds_;
};

class Duration {
 public:
  Duration() = default;
  explicit Duration(double seconds) : seconds_(seconds) {}
  double toSec() const { return seconds_; }
  void sleep() const {
    if (seconds_ > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(seconds_));
    }
  }

 private:
  double seconds_{0.0};
  friend Duration operator-(const Time &first, const Time &second);
  friend Time operator+(const Time &time, const Duration &duration);
};

inline Duration operator-(const Time &first, const Time &second) {
  return Duration(static_cast<double>(first.nanoseconds_ - second.nanoseconds_) * 1.0e-9);
}

inline Time operator+(const Time &time, const Duration &duration) {
  return Time(time.toSec() + duration.seconds_);
}

struct TimerEvent {};
struct TransportHints {
  TransportHints &tcpNoDelay(bool = true) { return *this; }
  TransportHints &bestEffort(bool value = true) {
    best_effort_ = value;
    return *this;
  }
  bool best_effort_{false};
};

class Publisher {
 public:
  Publisher() = default;

  template <typename Message>
  void publish(const Message &message) const {
    if (!publish_) return;
    publish_(&message, std::type_index(typeid(Message)));
  }

  explicit operator bool() const { return static_cast<bool>(publish_); }
  std::size_t getNumSubscribers() const {
    return subscription_count_ ? subscription_count_() : 0;
  }

 private:
  std::function<void(const void *, std::type_index)> publish_;
  std::function<std::size_t()> subscription_count_;
  template <typename Message>
  friend Publisher makePublisher(const std::shared_ptr<rclcpp::Node> &,
                                 const std::string &, std::size_t);
};

template <typename Message>
Publisher makePublisher(const std::shared_ptr<rclcpp::Node> &node,
                        const std::string &topic, std::size_t depth) {
  Publisher result;
  auto publisher = node->create_publisher<Message>(
      topic, rclcpp::QoS(rclcpp::KeepLast(std::max<std::size_t>(1, depth))).reliable());
  result.publish_ = [publisher](const void *message, std::type_index type) {
    if (type != std::type_index(typeid(Message))) {
      throw std::runtime_error("legacy publisher message type mismatch");
    }
    publisher->publish(*static_cast<const Message *>(message));
  };
  result.subscription_count_ = [publisher]() {
    return publisher->get_subscription_count();
  };
  return result;
}

class Subscriber {
 public:
  Subscriber() = default;
  void shutdown() { subscription_.reset(); }
 private:
  rclcpp::SubscriptionBase::SharedPtr subscription_;
  friend class NodeHandle;
};

class Timer {
 public:
  Timer() = default;
 private:
  rclcpp::TimerBase::SharedPtr timer_;
  friend class NodeHandle;
};

class ServiceClient {
 public:
  ServiceClient() = default;

  template <typename LegacyService>
  bool call(LegacyService &service) const {
    if (!call_) return false;
    return call_(&service, std::type_index(typeid(LegacyService)));
  }

 private:
  std::function<bool(void *, std::type_index)> call_;
  friend class NodeHandle;
};

class NodeHandle {
 public:
  NodeHandle() : node_(detail::ensureNode()) {}
  explicit NodeHandle(const std::string &) : node_(detail::ensureNode()) {}
  explicit NodeHandle(std::shared_ptr<rclcpp::Node> node) : node_(std::move(node)) {
    std::lock_guard<std::mutex> lock(detail::globalMutex());
    detail::globalNode() = node_;
  }

  std::shared_ptr<rclcpp::Node> ros2_node() const { return node_; }

  template <typename Value>
  void param(const std::string &legacy_name, Value &value,
             const Value &default_value) const {
    const std::string name = detail::parameterName(legacy_name);
    if (name.empty()) {
      value = default_value;
      return;
    }
    try {
      if (!node_->has_parameter(name)) {
        value = node_->declare_parameter<Value>(name, default_value);
      } else if (!node_->get_parameter(name, value)) {
        value = default_value;
      }
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      if (!node_->get_parameter(name, value)) value = default_value;
    }
  }

  template <typename Message>
  Publisher advertise(const std::string &topic, std::size_t depth,
                      bool = false) const {
    return makePublisher<Message>(node_, topic, depth);
  }

  template <typename Message, typename Object>
  Subscriber subscribe(
      const std::string &topic, std::size_t depth,
      void (Object::*callback)(const std::shared_ptr<const Message> &),
      Object *object, const TransportHints &hints = TransportHints()) const {
    Subscriber result;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(std::max<std::size_t>(1, depth)));
    if (hints.best_effort_) qos.best_effort(); else qos.reliable();
    result.subscription_ = node_->create_subscription<Message>(
        topic, qos,
        [object, callback](const std::shared_ptr<const Message> message) {
          (object->*callback)(message);
        });
    return result;
  }

  template <typename Message>
  Subscriber subscribe(
      const std::string &topic, std::size_t depth,
      void (*callback)(const std::shared_ptr<const Message> &),
      const TransportHints &hints = TransportHints()) const {
    Subscriber result;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(std::max<std::size_t>(1, depth)));
    if (hints.best_effort_) qos.best_effort(); else qos.reliable();
    result.subscription_ = node_->create_subscription<Message>(topic, qos, callback);
    return result;
  }

  template <typename Message>
  Subscriber subscribe(
      const std::string &topic, std::size_t depth,
      void (*callback)(const Message &),
      const TransportHints &hints = TransportHints()) const {
    Subscriber result;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(std::max<std::size_t>(1, depth)));
    if (hints.best_effort_) qos.best_effort(); else qos.reliable();
    result.subscription_ = node_->create_subscription<Message>(
        topic, qos, [callback](const std::shared_ptr<const Message> message) {
          callback(*message);
        });
    return result;
  }

  template <typename Message>
  Subscriber subscribe(
      const std::string &topic, std::size_t depth, void (*callback)(Message),
      const TransportHints &hints = TransportHints()) const {
    Subscriber result;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(std::max<std::size_t>(1, depth)));
    if (hints.best_effort_) qos.best_effort(); else qos.reliable();
    result.subscription_ = node_->create_subscription<Message>(
        topic, qos, [callback](const std::shared_ptr<const Message> message) {
          callback(*message);
        });
    return result;
  }

  template <typename Object>
  Timer createTimer(const Duration &duration,
                    void (Object::*callback)(const TimerEvent &),
                    Object *object, bool = false, bool = true) const {
    Timer result;
    const auto ns = std::max<std::int64_t>(1,
        static_cast<std::int64_t>(std::llround(duration.toSec() * 1.0e9)));
    result.timer_ = rclcpp::create_timer(
        node_, node_->get_clock(), rclcpp::Duration::from_nanoseconds(ns),
        [object, callback]() {
          const TimerEvent event;
          (object->*callback)(event);
        });
    return result;
  }

  Timer createTimer(const Duration &duration,
                    void (*callback)(const TimerEvent &),
                    bool = false, bool = true) const {
    Timer result;
    const auto ns = std::max<std::int64_t>(1,
        static_cast<std::int64_t>(std::llround(duration.toSec() * 1.0e9)));
    result.timer_ = rclcpp::create_timer(
        node_, node_->get_clock(), rclcpp::Duration::from_nanoseconds(ns),
        [callback]() {
          const TimerEvent event;
          callback(event);
        });
    return result;
  }

  template <typename LegacyService>
  ServiceClient serviceClient(const std::string &name, bool = false) const {
    using RosService = typename LegacyService::RosService;
    ServiceClient result;
    // A ROS1 service call is synchronous and can safely be made from a timer
    // callback.  Waiting on a client owned by the main ROS2 node would deadlock:
    // that node's single-threaded executor cannot process the response while
    // the legacy callback is blocked.  Keep the transport-only client on its
    // own node and spin that node until the response arrives.
    const auto index = detail::serviceClientIndex().fetch_add(1);
    rclcpp::NodeOptions client_options;
    client_options.use_global_arguments(false);
    auto client_node = std::make_shared<rclcpp::Node>(
        "c2_ros1_service_client_" + std::to_string(index), "", client_options);
    auto client = client_node->create_client<RosService>(name);
    result.call_ = [client_node, client](void *raw, std::type_index type) {
      if (type != std::type_index(typeid(LegacyService))) return false;
      auto &legacy = *static_cast<LegacyService *>(raw);
      if (!client->wait_for_service(std::chrono::seconds(5))) return false;
      auto request = std::make_shared<typename RosService::Request>(legacy.request);
      auto future = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(
              client_node, future, std::chrono::seconds(30)) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        return false;
      }
      legacy.response = *future.get();
      return detail::legacyServiceSucceeded(legacy.response);
    };
    return result;
  }

 private:
  std::shared_ptr<rclcpp::Node> node_;
};

inline void init(int &argc, char **argv, const std::string &name) {
  if (!rclcpp::ok()) rclcpp::init(argc, argv);
  std::lock_guard<std::mutex> lock(detail::globalMutex());
  detail::requestedNodeName() = name;
}

inline bool ok() { return rclcpp::ok(); }
inline void spinOnce() { rclcpp::spin_some(detail::ensureNode()); }
inline void spin() {
  // ros::spin() in the ROS1 baseline drains one callback queue on one thread.
  // Keeping that serialization is important: the original FSM, HGrid, map,
  // frontier, and pair-optimization callbacks intentionally share mutable
  // state without locks.  The transport-only service clients above own and
  // spin separate nodes, so blocking LKH calls do not require a multithreaded
  // executor for the algorithm node.
  rclcpp::executors::SingleThreadedExecutor executor;
  const auto node = detail::ensureNode();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  std::lock_guard<std::mutex> lock(detail::globalMutex());
  detail::globalNode().reset();
}

}  // namespace ros

#define ROS_INFO(...) RCLCPP_INFO(::ros::detail::logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(::ros::detail::logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(::ros::detail::logger(), __VA_ARGS__)
#define ROS_INFO_THROTTLE(period, ...) \
  do { if (::ros::detail::throttle(period, __FILE__, __LINE__)) ROS_INFO(__VA_ARGS__); } while (0)
#define ROS_WARN_THROTTLE(period, ...) \
  do { if (::ros::detail::throttle(period, __FILE__, __LINE__)) ROS_WARN(__VA_ARGS__); } while (0)
#define ROS_ERROR_COND(condition, ...) \
  do { if (condition) ROS_ERROR(__VA_ARGS__); } while (0)
#define ROS_ASSERT(condition) assert(condition)
#define ROS_BREAK() std::abort()
#define ROS_INFO_ONCE(...) \
  do { static bool once = false; if (!once) { once = true; ROS_INFO(__VA_ARGS__); } } while (0)

#define ROS_INFO_STREAM(value) \
  do { std::ostringstream stream; stream << value; \
       RCLCPP_INFO(::ros::detail::logger(), "%s", stream.str().c_str()); } while (0)
#define ROS_WARN_STREAM(value) \
  do { std::ostringstream stream; stream << value; \
       RCLCPP_WARN(::ros::detail::logger(), "%s", stream.str().c_str()); } while (0)
#define ROS_INFO_STREAM_THROTTLE(period, value) \
  do { if (::ros::detail::throttle(period, __FILE__, __LINE__)) ROS_INFO_STREAM(value); } while (0)
#define ROS_WARN_STREAM_THROTTLE(period, value) \
  do { if (::ros::detail::throttle(period, __FILE__, __LINE__)) ROS_WARN_STREAM(value); } while (0)
#define ROS_DEBUG_STREAM_THROTTLE(period, value) \
  do { if (::ros::detail::throttle(period, __FILE__, __LINE__)) { \
       std::ostringstream stream; stream << value; \
       RCLCPP_DEBUG(::ros::detail::logger(), "%s", stream.str().c_str()); } } while (0)
