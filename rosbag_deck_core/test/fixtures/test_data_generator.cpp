#include "test_data_generator.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace rosbag_deck_core {
namespace test {

TestDataGenerator::TestDataGenerator(uint32_t seed)
    : rng_(seed),
      float_dist_(-1.0f, 1.0f),
      byte_dist_(0, 255),
      char_dist_('a', 'z'),
      base_time_(rclcpp::Clock().now()) {
}

sensor_msgs::msg::PointCloud2 TestDataGenerator::generate_pointcloud(
    size_t width, size_t height, const std::string& frame_id) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = generate_timestamp();
  cloud.height = height;
  cloud.width = width;
  cloud.is_dense = true;
  
  // Setup fields
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(width * height);
  
  // Fill with random data
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  
  for (size_t i = 0; i < width * height; ++i) {
    *iter_x = float_dist_(rng_);
    *iter_y = float_dist_(rng_);
    *iter_z = float_dist_(rng_);
    ++iter_x; ++iter_y; ++iter_z;
  }
  
  return cloud;
}

sensor_msgs::msg::Image TestDataGenerator::generate_image(
    uint32_t width, uint32_t height, const std::string& encoding) {
  sensor_msgs::msg::Image image;
  image.header.stamp = generate_timestamp();
  image.header.frame_id = "camera_frame";
  image.height = height;
  image.width = width;
  image.encoding = encoding;
  
  if (encoding == "rgb8") {
    image.is_bigendian = false;
    image.step = width * 3;
    image.data.resize(height * image.step);
    
    for (auto& byte : image.data) {
      byte = byte_dist_(rng_);
    }
  }
  
  return image;
}

geometry_msgs::msg::Twist TestDataGenerator::generate_twist(
    double linear_range, double angular_range) {
  geometry_msgs::msg::Twist twist;
  std::uniform_real_distribution<double> linear_dist(-linear_range, linear_range);
  std::uniform_real_distribution<double> angular_dist(-angular_range, angular_range);
  
  twist.linear.x = linear_dist(rng_);
  twist.linear.y = linear_dist(rng_);
  twist.linear.z = linear_dist(rng_);
  twist.angular.x = angular_dist(rng_);
  twist.angular.y = angular_dist(rng_);
  twist.angular.z = angular_dist(rng_);
  
  return twist;
}

std_msgs::msg::String TestDataGenerator::generate_string(size_t length) {
  std_msgs::msg::String msg;
  msg.data.reserve(length);
  
  for (size_t i = 0; i < length; ++i) {
    msg.data += char_dist_(rng_);
  }
  
  return msg;
}

template<typename MessageT>
std::shared_ptr<std::vector<uint8_t>> serialize_message_impl(const MessageT& msg) {
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<MessageT> serializer;
  serializer.serialize_message(&msg, &serialized_msg);
  
  auto data = std::make_shared<std::vector<uint8_t>>(
      serialized_msg.get_rcl_serialized_message().buffer,
      serialized_msg.get_rcl_serialized_message().buffer + 
      serialized_msg.size());
  
  return data;
}

std::shared_ptr<std::vector<uint8_t>> TestDataGenerator::serialize_message(
    const sensor_msgs::msg::PointCloud2& msg) {
  return serialize_message_impl(msg);
}

std::shared_ptr<std::vector<uint8_t>> TestDataGenerator::serialize_message(
    const sensor_msgs::msg::Image& msg) {
  return serialize_message_impl(msg);
}

std::shared_ptr<std::vector<uint8_t>> TestDataGenerator::serialize_message(
    const geometry_msgs::msg::Twist& msg) {
  return serialize_message_impl(msg);
}

std::shared_ptr<std::vector<uint8_t>> TestDataGenerator::serialize_message(
    const std_msgs::msg::String& msg) {
  return serialize_message_impl(msg);
}

BagMessage TestDataGenerator::create_test_bag_message(
    const std::string& topic_name,
    const std::string& message_type,
    size_t frame_index,
    rclcpp::Time timestamp) {
  BagMessage msg;
  msg.topic_name = topic_name;
  msg.message_type = message_type;
  msg.frame_index = frame_index;
  msg.original_timestamp = Timestamp(std::chrono::nanoseconds(timestamp.nanoseconds()));
  msg.virtual_timestamp = msg.original_timestamp;
  msg.serialization_format = "cdr";
  msg.serialized_data = std::make_shared<std::vector<uint8_t>>(generate_random_data(1024));
  
  msg.metadata["test_key"] = "test_value";
  msg.metadata["frame_index"] = std::to_string(frame_index);
  
  return msg;
}

std::vector<uint8_t> TestDataGenerator::generate_random_data(size_t size) {
  std::vector<uint8_t> data(size);
  for (auto& byte : data) {
    byte = byte_dist_(rng_);
  }
  return data;
}

rclcpp::Time TestDataGenerator::generate_timestamp(uint64_t nanoseconds_offset) {
  return base_time_ + rclcpp::Duration::from_nanoseconds(nanoseconds_offset);
}

} // namespace test
} // namespace rosbag_deck_core