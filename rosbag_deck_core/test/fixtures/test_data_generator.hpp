#pragma once

#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <memory>

namespace rosbag_deck_core {
namespace test {

class TestDataGenerator {
public:
  TestDataGenerator(uint32_t seed = 42);
  
  // Generate test messages
  sensor_msgs::msg::PointCloud2 generate_pointcloud(
      size_t width = 100, 
      size_t height = 1,
      const std::string& frame_id = "test_frame");
  
  sensor_msgs::msg::Image generate_image(
      uint32_t width = 640,
      uint32_t height = 480,
      const std::string& encoding = "rgb8");
  
  geometry_msgs::msg::Twist generate_twist(
      double linear_range = 1.0,
      double angular_range = 1.0);
  
  std_msgs::msg::String generate_string(
      size_t length = 100);
  
  // Generate serialized message data
  std::shared_ptr<std::vector<uint8_t>> serialize_message(
      const sensor_msgs::msg::PointCloud2& msg);
  
  std::shared_ptr<std::vector<uint8_t>> serialize_message(
      const sensor_msgs::msg::Image& msg);
  
  std::shared_ptr<std::vector<uint8_t>> serialize_message(
      const geometry_msgs::msg::Twist& msg);
  
  std::shared_ptr<std::vector<uint8_t>> serialize_message(
      const std_msgs::msg::String& msg);
  
  // Generate BagMessage for testing
  BagMessage create_test_bag_message(
      const std::string& topic_name,
      const std::string& message_type,
      size_t frame_index,
      rclcpp::Time timestamp);
  
  // Generate random data
  std::vector<uint8_t> generate_random_data(size_t size);
  
  // Time utilities
  rclcpp::Time generate_timestamp(uint64_t nanoseconds_offset = 0);
  
private:
  std::mt19937 rng_;
  std::uniform_real_distribution<float> float_dist_;
  std::uniform_int_distribution<uint8_t> byte_dist_;
  std::uniform_int_distribution<char> char_dist_;
  
  rclcpp::Time base_time_;
};

} // namespace test
} // namespace rosbag_deck_core