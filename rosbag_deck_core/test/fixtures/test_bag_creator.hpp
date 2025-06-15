#pragma once

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>

namespace rosbag_deck_core {
namespace test {

class TestBagCreator {
public:
  TestBagCreator();
  ~TestBagCreator();

  // Create a simple test bag with specified number of messages
  std::string create_test_bag(const std::string& bag_name,
                              size_t num_messages = 100,
                              const std::vector<std::string>& topics = {"/test_topic"});

  // Create a multi-topic test bag
  std::string create_multi_topic_bag(const std::string& bag_name,
                                     size_t messages_per_topic = 50);

  // Create a corrupted test bag for error handling tests
  std::string create_corrupted_bag(const std::string& bag_name);

  // Create a large test bag for performance testing
  std::string create_large_bag(const std::string& bag_name,
                               size_t num_messages = 10000);

  // Get the test data directory
  std::string get_test_data_dir() const { return test_data_dir_; }

  // Clean up all created test bags
  void cleanup();

private:
  std::string test_data_dir_;
  std::vector<std::string> created_bags_;
  
  // Helper to write a PointCloud2 message
  void write_pointcloud_message(rosbag2_cpp::Writer& writer,
                                const std::string& topic,
                                rclcpp::Time timestamp,
                                size_t point_count = 100);
  
  // Helper to write a Twist message
  void write_twist_message(rosbag2_cpp::Writer& writer,
                           const std::string& topic,
                           rclcpp::Time timestamp);
  
  // Helper to write a String message
  void write_string_message(rosbag2_cpp::Writer& writer,
                            const std::string& topic,
                            rclcpp::Time timestamp,
                            const std::string& data);
};

} // namespace test
} // namespace rosbag_deck_core