#pragma once

// A deliberately small compatibility layer used while migrating VINS-Mono's
// algorithm code from roscpp to rclcpp.  It keeps the old callback signatures
// readable, but all communication is implemented by ROS 2.

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

// ROS 1 spelling of message namespaces.  The aliases are intentionally kept
// in the global namespace so existing VINS source files need no algorithmic
// changes.
namespace std_msgs {
using Header = msg::Header;
using Bool = msg::Bool;
using Float32 = msg::Float32;
using ColorRGBA = msg::ColorRGBA;
using BoolConstPtr = Bool::ConstSharedPtr;
using HeaderConstPtr = Header::ConstSharedPtr;
}
namespace sensor_msgs {
using Image = msg::Image;
using Imu = msg::Imu;
using PointCloud = msg::PointCloud;
using ChannelFloat32 = msg::ChannelFloat32;
using ImageConstPtr = Image::ConstSharedPtr;
using ImagePtr = Image::SharedPtr;
using ImuConstPtr = Imu::ConstSharedPtr;
using ImuPtr = Imu::SharedPtr;
using PointCloudConstPtr = PointCloud::ConstSharedPtr;
using PointCloudPtr = PointCloud::SharedPtr;
}
namespace nav_msgs {
using Odometry = msg::Odometry;
using Path = msg::Path;
using OdometryConstPtr = Odometry::ConstSharedPtr;
using PathConstPtr = Path::ConstSharedPtr;
}
namespace geometry_msgs {
using Point32 = msg::Point32;
using Point = msg::Point;
using PointStamped = msg::PointStamped;
using PoseStamped = msg::PoseStamped;
using TransformStamped = msg::TransformStamped;
}
namespace visualization_msgs {
using Marker = msg::Marker;
using MarkerArray = msg::MarkerArray;
}

// ---------------------------------------------------------------------------
// Time helpers: VINS-Mono historically called stamp.toSec() on message
// headers; ROS 2's builtin_interfaces::msg::Time has no such method, so the
// short notation is preserved through these helpers.
// ---------------------------------------------------------------------------
inline double stampToSec(const builtin_interfaces::msg::Time &stamp)
{
  return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

inline builtin_interfaces::msg::Time secToStamp(double seconds)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(seconds);
  stamp.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(stamp.sec)) * 1e9);
  return stamp;
}

#define ROSCONSOLE_DEFAULT_NAME "ros2_compat"

namespace ros {

class Time {
public:
  Time() = default;
  explicit Time(double seconds) : seconds_(seconds) {}
  Time(const builtin_interfaces::msg::Time &time)
      : seconds_(static_cast<double>(time.sec) + 1e-9 * time.nanosec) {}
  explicit Time(const rclcpp::Time &time) : seconds_(time.seconds()) {}
  static Time now() { return Time(rclcpp::Clock(RCL_SYSTEM_TIME).now()); }
  double toSec() const { return seconds_; }
  int32_t sec() const { return static_cast<int32_t>(seconds_); }
  uint32_t nanosec() const {
    const double fractional = seconds_ - static_cast<double>(sec());
    return static_cast<uint32_t>(fractional * 1e9);
  }
  bool operator<(const Time &other) const { return seconds_ < other.seconds_; }
  bool operator>(const Time &other) const { return seconds_ > other.seconds_; }
  operator builtin_interfaces::msg::Time() const {
    builtin_interfaces::msg::Time out;
    out.sec = sec();
    out.nanosec = nanosec();
    return out;
  }
private:
  double seconds_{0.0};
};

class Duration {
public:
  explicit Duration(double seconds = 0.0) : seconds_(seconds) {}
  double toSec() const { return seconds_; }
  void sleep() const {
    if (seconds_ > 0.0)
      std::this_thread::sleep_for(std::chrono::duration<double>(seconds_));
  }
  operator builtin_interfaces::msg::Duration() const {
    builtin_interfaces::msg::Duration out;
    out.sec = static_cast<int32_t>(seconds_);
    out.nanosec = static_cast<uint32_t>((seconds_ - out.sec) * 1e9);
    return out;
  }
private:
  double seconds_{0.0};
};

class Rate {
public:
  explicit Rate(double hz) : period_(hz > 0.0 ? 1.0 / hz : 0.0) {}
  void sleep() const {
    if (period_ > 0.0)
      std::this_thread::sleep_for(std::chrono::duration<double>(period_));
  }
private:
  double period_;
};

class TransportHints {
public:
  TransportHints &tcpNoDelay() { return *this; }
};

class Publisher {
public:
  Publisher() = default;
  template <typename MessageT>
  void publish(const MessageT &message) const {
    if (publish_fn_)
      publish_fn_(static_cast<const void *>(&message));
  }
  template <typename MessageT>
  void publish(const std::shared_ptr<MessageT> &message) const {
    if (message)
      publish(*message);
  }
  explicit operator bool() const { return static_cast<bool>(publish_fn_); }
  template <typename MessageT>
  static Publisher make(const typename rclcpp::Publisher<MessageT>::SharedPtr &publisher) {
    Publisher out;
    out.publish_fn_ = [publisher](const void *value) {
      publisher->publish(*static_cast<const MessageT *>(value));
    };
    return out;
  }
private:
  std::function<void(const void *)> publish_fn_;
};

class Subscriber {
public:
  Subscriber() = default;
  explicit Subscriber(std::shared_ptr<void> handle) : handle_(std::move(handle)) {}
private:
  std::shared_ptr<void> handle_;
};

namespace console {
namespace levels {
enum Level { Debug, Info, Warn, Error, Fatal };
}
inline void set_logger_level(const std::string &name, levels::Level level) {
  (void)name;
  (void)level;
  rclcpp::Logger logger = rclcpp::get_logger(name.empty() ? ROSCONSOLE_DEFAULT_NAME : name);
  switch (level) {
    case levels::Debug: logger.set_level(rclcpp::Logger::Level::Debug); break;
    case levels::Info: logger.set_level(rclcpp::Logger::Level::Info); break;
    case levels::Warn: logger.set_level(rclcpp::Logger::Level::Warn); break;
    case levels::Error: logger.set_level(rclcpp::Logger::Level::Error); break;
    case levels::Fatal: logger.set_level(rclcpp::Logger::Level::Fatal); break;
  }
}
}

inline std::string &node_name_storage();
inline const std::string &node_name();

class NodeHandle {
public:
  explicit NodeHandle(const std::string &namespace_name = std::string()) {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    if (!default_node_)
      default_node_ = rclcpp::Node::make_shared(ros::node_name());
    node_ = default_node_;

    if (namespace_name.empty() || namespace_name == "~") {
      // ROS 1 private node handle: relative topics live under /node_name/
      prefix_ = "/" + ros::node_name() + "/";
    } else if (namespace_name[0] == '/') {
      prefix_ = namespace_name;
      if (prefix_.back() != '/')
        prefix_ += "/";
    } else {
      prefix_ = "/" + namespace_name + "/";
    }
  }

  // Resolve a topic name the way roscpp did: absolute names pass through,
  // relative names are prefixed with the private namespace.
  std::string resolve(const std::string &topic) const {
    if (topic.empty() || topic[0] == '/')
      return topic;
    return prefix_ + topic;
  }

  template <typename T>
  bool getParam(const std::string &name, T &value) {
    if (!node_->has_parameter(name))
      node_->declare_parameter<T>(name, T{});
    try {
      value = node_->get_parameter(name).get_value<T>();
      return true;
    } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
      return false;
    }
  }

  template <typename T>
  void param(const std::string &name, T &value, const T &default_value) {
    if (!node_->has_parameter(name))
      node_->declare_parameter<T>(name, default_value);
    value = node_->get_parameter(name).get_value<T>();
  }

  template <typename MessageT>
  Publisher advertise(const std::string &topic, size_t depth) {
    auto publisher = node_->create_publisher<MessageT>(resolve(topic), depth);
    return Publisher::make<MessageT>(publisher);
  }

  template <typename MessageT>
  Subscriber subscribe(const std::string &topic, size_t depth,
                       std::function<void(const std::shared_ptr<const MessageT> &)> callback,
                       const TransportHints & = TransportHints()) {
    auto subscription = node_->create_subscription<MessageT>(
        resolve(topic), rclcpp::QoS(depth), std::move(callback));
    auto handle = std::shared_ptr<void>(subscription, subscription.get());
    subscriptions_.push_back(handle);
    return Subscriber(std::move(handle));
  }

  template <typename MessageT>
  Subscriber subscribe(const std::string &topic, size_t depth,
                       void (*callback)(const std::shared_ptr<const MessageT> &)) {
    return subscribe<MessageT>(topic, depth,
      std::function<void(const std::shared_ptr<const MessageT> &)>(callback));
  }

  template <typename MessageT>
  Subscriber subscribe(const std::string &topic, size_t depth,
                       void (*callback)(const std::shared_ptr<const MessageT> &),
                       const TransportHints &hints) {
    return subscribe<MessageT>(topic, depth,
      std::function<void(const std::shared_ptr<const MessageT> &)>(callback), hints);
  }

  void shutdown() { rclcpp::shutdown(); }
  rclcpp::Node::SharedPtr node() const { return node_; }

  static rclcpp::Node::SharedPtr default_node() { return default_node_; }
private:
  rclcpp::Node::SharedPtr node_;
  std::string prefix_;
  std::vector<std::shared_ptr<void>> subscriptions_;
  inline static rclcpp::Node::SharedPtr default_node_{};
};

// The node name from ros::init; all NodeHandles created afterwards share it.
inline std::string &node_name_storage() {
  static std::string storage = "node";
  return storage;
}
inline const std::string &node_name() { return node_name_storage(); }

inline void set_node_name(const std::string &name) {
  if (!name.empty())
    node_name_storage() = name;
}

inline void init(int &argc, char **argv, const std::string &name) {
  set_node_name(name);
  if (!rclcpp::ok())
    rclcpp::init(argc, argv);
}
inline void spin() {
  if (NodeHandle::default_node())
    rclcpp::spin(NodeHandle::default_node());
}
inline void spinOnce() {
  if (NodeHandle::default_node())
    rclcpp::spin_some(NodeHandle::default_node());
}
inline bool ok() { return rclcpp::ok(); }
inline void shutdown() { rclcpp::shutdown(); }

namespace package {
inline std::string getPath(const std::string &package_name) {
  try {
    return ament_index_cpp::get_package_share_directory(package_name);
  } catch (...) {
    return std::string();
  }
}
}

}  // namespace ros

// Lightweight logging macros retain printf-style formatting used by VINS.
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger(ROSCONSOLE_DEFAULT_NAME), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger(ROSCONSOLE_DEFAULT_NAME), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger(ROSCONSOLE_DEFAULT_NAME), __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger(ROSCONSOLE_DEFAULT_NAME), __VA_ARGS__)
#define ROS_INFO_STREAM(value) do { std::ostringstream _ros_stream; _ros_stream << value; ROS_INFO("%s", _ros_stream.str().c_str()); } while (0)
#define ROS_WARN_STREAM(value) do { std::ostringstream _ros_stream; _ros_stream << value; ROS_WARN("%s", _ros_stream.str().c_str()); } while (0)
#define ROS_ERROR_STREAM(value) do { std::ostringstream _ros_stream; _ros_stream << value; ROS_ERROR("%s", _ros_stream.str().c_str()); } while (0)
#define ROS_DEBUG_STREAM(value) do { std::ostringstream _ros_stream; _ros_stream << value; ROS_DEBUG("%s", _ros_stream.str().c_str()); } while (0)
#define ROS_ASSERT(condition) do { if (!(condition)) { ROS_ERROR("assertion failed: %s", #condition); std::abort(); } } while (0)
#define ROS_ASSERT_MSG(condition, ...) do { if (!(condition)) { ROS_ERROR(__VA_ARGS__); std::abort(); } } while (0)
#define ROS_BREAK() do { std::abort(); } while (0)
