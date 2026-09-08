#pragma once

#include <ros2_compat/compat.hpp>

namespace image_transport {
using Publisher = ros::Publisher;
class ImageTransport {
public:
  explicit ImageTransport(const ros::NodeHandle &node) : node_(node) {}
  Publisher advertise(const std::string &topic, size_t depth) {
    return node_.template advertise<sensor_msgs::Image>(topic, depth);
  }
private:
  ros::NodeHandle node_;
};
}  // namespace image_transport
