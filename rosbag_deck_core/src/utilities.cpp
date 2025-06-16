#include "rosbag_deck_core/utilities.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <set>

namespace rosbag_deck_core {

// Utility functions
Timestamp to_timestamp(int64_t nanoseconds_since_epoch) {
  return Timestamp(Duration(nanoseconds_since_epoch));
}

int64_t from_timestamp(const Timestamp &timestamp) {
  return std::chrono::duration_cast<Duration>(timestamp.time_since_epoch())
      .count();
}

// Template specializations for message serialization/deserialization
template <typename MessageT>
std::shared_ptr<std::vector<uint8_t>>
serialize_message(const MessageT &message) {
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<MessageT> serialization;
  serialization.serialize_message(&message, &serialized_msg);

  auto data = std::make_shared<std::vector<uint8_t>>(
      serialized_msg.get_rcl_serialized_message().buffer,
      serialized_msg.get_rcl_serialized_message().buffer +
          serialized_msg.size());

  return data;
}

template <typename MessageT>
MessageT deserialize_message(const std::vector<uint8_t> &serialized_data) {
  rclcpp::SerializedMessage serialized_msg;
  serialized_msg.reserve(serialized_data.size());

  // Copy data to serialized message
  auto &rcl_msg = serialized_msg.get_rcl_serialized_message();
  rcl_msg.buffer_length = serialized_data.size();
  rcl_msg.buffer_capacity = serialized_data.size();
  rcl_msg.buffer = const_cast<uint8_t *>(serialized_data.data());

  MessageT message;
  rclcpp::Serialization<MessageT> serialization;
  serialization.deserialize_message(&serialized_msg, &message);

  return message;
}

bool is_message_type_supported(const std::string &message_type) {
  // Check if the message type is one of the commonly supported ROS 2 types
  // This is a static check for well-known message types
  static const std::set<std::string> supported_types = {
    "sensor_msgs/msg/PointCloud2",
    "sensor_msgs/msg/Image", 
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/CameraInfo",
    "sensor_msgs/msg/LaserScan",
    "sensor_msgs/msg/Imu",
    "geometry_msgs/msg/Twist",
    "geometry_msgs/msg/TwistStamped", 
    "geometry_msgs/msg/Pose",
    "geometry_msgs/msg/PoseStamped",
    "geometry_msgs/msg/Transform",
    "geometry_msgs/msg/TransformStamped",
    "std_msgs/msg/String",
    "std_msgs/msg/Header",
    "std_msgs/msg/Bool",
    "std_msgs/msg/Int32",
    "std_msgs/msg/Float64",
    "nav_msgs/msg/Odometry",
    "tf2_msgs/msg/TFMessage"
  };
  
  return supported_types.find(message_type) != supported_types.end();
}

} // namespace rosbag_deck_core