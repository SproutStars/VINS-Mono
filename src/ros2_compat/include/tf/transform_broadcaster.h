#pragma once

#include <ros2_compat/compat.hpp>

namespace tf {
class Vector3 {
public:
  Vector3() = default;
  Vector3(double x, double y, double z) : x_(x), y_(y), z_(z) {}
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  void setX(double x) { x_ = x; }
  void setY(double y) { y_ = y; }
  void setZ(double z) { z_ = z; }
private:
  double x_{0.0}, y_{0.0}, z_{0.0};
};

class Quaternion {
public:
  Quaternion() = default;
  Quaternion(double x, double y, double z, double w) : x_(x), y_(y), z_(z), w_(w) {}
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double w() const { return w_; }
  void setX(double x) { x_ = x; }
  void setY(double y) { y_ = y; }
  void setZ(double z) { z_ = z; }
  void setW(double w) { w_ = w; }
private:
  double x_{0.0}, y_{0.0}, z_{0.0}, w_{1.0};
};

class Transform {
public:
  void setOrigin(const Vector3 &origin) { origin_ = origin; }
  void setRotation(const Quaternion &rotation) { rotation_ = rotation; }
  const Vector3 &origin() const { return origin_; }
  const Quaternion &rotation() const { return rotation_; }
private:
  Vector3 origin_;
  Quaternion rotation_;
};

class StampedTransform {
public:
  StampedTransform(const Transform &transform, const ros::Time &stamp,
                   const std::string &parent, const std::string &child)
      : transform_(transform), stamp_(stamp), parent_(parent), child_(child) {}
  const Transform &transform() const { return transform_; }
  const ros::Time &stamp() const { return stamp_; }
  const std::string &parent() const { return parent_; }
  const std::string &child() const { return child_; }
private:
  Transform transform_;
  ros::Time stamp_;
  std::string parent_, child_;
};

class TransformBroadcaster {
public:
  // Create the broadcaster from the shared default node created by NodeHandle.
  TransformBroadcaster()
      : broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(ros::NodeHandle::default_node())) {}

  void sendTransform(const StampedTransform &stamped) {
    geometry_msgs::TransformStamped message;
    message.header.stamp = stamped.stamp();
    message.header.frame_id = stamped.parent();
    message.child_frame_id = stamped.child();
    const auto &origin = stamped.transform().origin();
    message.transform.translation.x = origin.x();
    message.transform.translation.y = origin.y();
    message.transform.translation.z = origin.z();
    const auto &rotation = stamped.transform().rotation();
    message.transform.rotation.x = rotation.x();
    message.transform.rotation.y = rotation.y();
    message.transform.rotation.z = rotation.z();
    message.transform.rotation.w = rotation.w();
    broadcaster_->sendTransform(message);
  }
private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};
}  // namespace tf
