#include "test_bag_creator.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cstdlib>
#include <iostream>

namespace rosbag_deck_core {
namespace test {

TestBagCreator::TestBagCreator() {
  // Get test data directory from environment or use default
  const char* test_dir = std::getenv("ROS_HOME");
  if (test_dir) {
    test_data_dir_ = std::string(test_dir) + "/test_bags";
  } else {
    test_data_dir_ = "/tmp/rosbag_deck_test_bags";
  }
  
  // Create test directory if it doesn't exist
  std::filesystem::create_directories(test_data_dir_);
}

TestBagCreator::~TestBagCreator() {
  // Cleanup is optional, can be called explicitly
}

std::string TestBagCreator::create_test_bag(const std::string& bag_name,
                                             size_t num_messages,
                                             const std::vector<std::string>& topics) {
  std::string bag_path = test_data_dir_ + "/" + bag_name;
  
  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;
  storage_options.storage_id = "sqlite3";
  
  writer.open(storage_options);
  
  // Create topics
  for (const auto& topic : topics) {
    rosbag2_storage::TopicMetadata topic_metadata;
    topic_metadata.name = topic;
    topic_metadata.type = "sensor_msgs/msg/PointCloud2";
    topic_metadata.serialization_format = "cdr";
    writer.create_topic(topic_metadata);
  }
  
  // Write messages
  auto start_time = rclcpp::Clock().now();
  for (size_t i = 0; i < num_messages; ++i) {
    auto timestamp = start_time + rclcpp::Duration::from_seconds(i * 0.1);
    for (const auto& topic : topics) {
      write_pointcloud_message(writer, topic, timestamp, 100 + i);
    }
  }
  
  writer.close();
  created_bags_.push_back(bag_path);
  return bag_path;
}

std::string TestBagCreator::create_multi_topic_bag(const std::string& bag_name,
                                                   size_t messages_per_topic) {
  std::string bag_path = test_data_dir_ + "/" + bag_name;
  
  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;
  storage_options.storage_id = "sqlite3";
  
  writer.open(storage_options);
  
  // Create multiple topics with different types
  std::vector<std::pair<std::string, std::string>> topic_types = {
    {"/pointcloud", "sensor_msgs/msg/PointCloud2"},
    {"/cmd_vel", "geometry_msgs/msg/Twist"},
    {"/status", "std_msgs/msg/String"}
  };
  
  for (const auto& [topic, type] : topic_types) {
    rosbag2_storage::TopicMetadata topic_metadata;
    topic_metadata.name = topic;
    topic_metadata.type = type;
    topic_metadata.serialization_format = "cdr";
    writer.create_topic(topic_metadata);
  }
  
  // Write messages
  auto start_time = rclcpp::Clock().now();
  for (size_t i = 0; i < messages_per_topic; ++i) {
    auto timestamp = start_time + rclcpp::Duration::from_seconds(i * 0.1);
    
    write_pointcloud_message(writer, "/pointcloud", timestamp, 100);
    write_twist_message(writer, "/cmd_vel", timestamp);
    write_string_message(writer, "/status", timestamp, 
                         "Status message " + std::to_string(i));
  }
  
  writer.close();
  created_bags_.push_back(bag_path);
  return bag_path;
}

std::string TestBagCreator::create_corrupted_bag(const std::string& bag_name) {
  // Create a normal bag first
  std::string bag_path = create_test_bag(bag_name, 10);
  
  // Corrupt it by truncating the file
  std::filesystem::resize_file(bag_path + "/metadata.yaml", 50);
  
  return bag_path;
}

std::string TestBagCreator::create_large_bag(const std::string& bag_name,
                                            size_t num_messages) {
  return create_test_bag(bag_name, num_messages, {"/large_topic"});
}

void TestBagCreator::cleanup() {
  for (const auto& bag_path : created_bags_) {
    try {
      std::filesystem::remove_all(bag_path);
    } catch (const std::exception& e) {
      std::cerr << "Failed to remove test bag: " << bag_path 
                << " - " << e.what() << std::endl;
    }
  }
  created_bags_.clear();
}

void TestBagCreator::write_pointcloud_message(rosbag2_cpp::Writer& writer,
                                              const std::string& topic,
                                              rclcpp::Time timestamp,
                                              size_t point_count) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = timestamp;
  cloud.header.frame_id = "test_frame";
  cloud.height = 1;
  cloud.width = point_count;
  cloud.is_dense = true;
  
  // Setup point cloud fields
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(point_count);
  
  // Fill with dummy data
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  
  for (size_t i = 0; i < point_count; ++i) {
    *iter_x = static_cast<float>(i) * 0.01f;
    *iter_y = static_cast<float>(i) * 0.02f;
    *iter_z = static_cast<float>(i) * 0.03f;
    ++iter_x; ++iter_y; ++iter_z;
  }
  
  // Serialize and write
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
  serializer.serialize_message(&cloud, &serialized_msg);
  
  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  bag_message->topic_name = topic;
  bag_message->time_stamp = timestamp.nanoseconds();
  bag_message->serialized_data = std::make_shared<rcutils_uint8_array_t>();
  bag_message->serialized_data->buffer = serialized_msg.get_rcl_serialized_message().buffer;
  bag_message->serialized_data->buffer_length = serialized_msg.get_rcl_serialized_message().buffer_length;
  bag_message->serialized_data->buffer_capacity = serialized_msg.get_rcl_serialized_message().buffer_capacity;
  
  writer.write(bag_message);
}

void TestBagCreator::write_twist_message(rosbag2_cpp::Writer& writer,
                                        const std::string& topic,
                                        rclcpp::Time timestamp) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.5;
  
  // Serialize and write
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<geometry_msgs::msg::Twist> serializer;
  serializer.serialize_message(&twist, &serialized_msg);
  
  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  bag_message->topic_name = topic;
  bag_message->time_stamp = timestamp.nanoseconds();
  bag_message->serialized_data = std::make_shared<rcutils_uint8_array_t>();
  bag_message->serialized_data->buffer = serialized_msg.get_rcl_serialized_message().buffer;
  bag_message->serialized_data->buffer_length = serialized_msg.get_rcl_serialized_message().buffer_length;
  bag_message->serialized_data->buffer_capacity = serialized_msg.get_rcl_serialized_message().buffer_capacity;
  
  writer.write(bag_message);
}

void TestBagCreator::write_string_message(rosbag2_cpp::Writer& writer,
                                         const std::string& topic,
                                         rclcpp::Time timestamp,
                                         const std::string& data) {
  std_msgs::msg::String msg;
  msg.data = data;
  
  // Serialize and write
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<std_msgs::msg::String> serializer;
  serializer.serialize_message(&msg, &serialized_msg);
  
  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  bag_message->topic_name = topic;
  bag_message->time_stamp = timestamp.nanoseconds();
  bag_message->serialized_data = std::make_shared<rcutils_uint8_array_t>();
  bag_message->serialized_data->buffer = serialized_msg.get_rcl_serialized_message().buffer;
  bag_message->serialized_data->buffer_length = serialized_msg.get_rcl_serialized_message().buffer_length;
  bag_message->serialized_data->buffer_capacity = serialized_msg.get_rcl_serialized_message().buffer_capacity;
  
  writer.write(bag_message);
}

} // namespace test
} // namespace rosbag_deck_core