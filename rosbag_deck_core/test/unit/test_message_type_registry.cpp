#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include "../fixtures/test_bag_creator.hpp"
#include "../fixtures/test_data_generator.hpp"

using namespace rosbag_deck_core;
using namespace rosbag_deck_core::test;
using namespace testing;

class MessageTypeRegistryTest : public ::testing::Test {
protected:
  void SetUp() override {
    bag_creator_ = std::make_unique<TestBagCreator>();
    registry_ = std::make_unique<MessageTypeRegistry>();
  }

  void TearDown() override {
    bag_creator_->cleanup();
  }

  std::unique_ptr<TestBagCreator> bag_creator_;
  std::unique_ptr<MessageTypeRegistry> registry_;
};

TEST_F(MessageTypeRegistryTest, RegisterTypesFromSingleBag) {
  // Create a test bag with known types
  auto bag_path = bag_creator_->create_test_bag("registry_single.db3", 10, {"/laser"});
  
  // Register types from bag
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Verify types were registered
  auto discovered_types = registry_->get_discovered_types();
  EXPECT_THAT(discovered_types, Contains("sensor_msgs/msg/PointCloud2"));
  
  // Verify topic mappings
  auto topic_types = registry_->get_topic_type_map();
  EXPECT_EQ(topic_types["/laser"], "sensor_msgs/msg/PointCloud2");
}

TEST_F(MessageTypeRegistryTest, RegisterTypesFromMultipleBags) {
  // Create bags with different types
  auto bag1 = bag_creator_->create_test_bag("registry_multi1.db3", 5, {"/topic1"});
  auto bag2 = bag_creator_->create_multi_topic_bag("registry_multi2.db3", 5);
  
  // Register types from both bags
  registry_->register_types_from_bag_metadata({bag1, bag2});
  
  // Verify all types were discovered
  auto discovered_types = registry_->get_discovered_types();
  EXPECT_THAT(discovered_types, Contains("sensor_msgs/msg/PointCloud2"));
  EXPECT_THAT(discovered_types, Contains("geometry_msgs/msg/Twist"));
  EXPECT_THAT(discovered_types, Contains("std_msgs/msg/String"));
  
  // Verify topic mappings from multi-topic bag
  auto topic_types = registry_->get_topic_type_map();
  EXPECT_EQ(topic_types["/pointcloud"], "sensor_msgs/msg/PointCloud2");
  EXPECT_EQ(topic_types["/cmd_vel"], "geometry_msgs/msg/Twist");
  EXPECT_EQ(topic_types["/status"], "std_msgs/msg/String");
}

TEST_F(MessageTypeRegistryTest, TopicFiltering) {
  auto bag_path = bag_creator_->create_multi_topic_bag("registry_filter.db3", 10);
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Initially all topics should be enabled
  EXPECT_TRUE(registry_->is_topic_enabled("/pointcloud"));
  EXPECT_TRUE(registry_->is_topic_enabled("/cmd_vel"));
  EXPECT_TRUE(registry_->is_topic_enabled("/status"));
  
  // Set topic filter
  std::vector<std::string> enabled_topics = {"/pointcloud", "/status"};
  registry_->set_topic_filter(enabled_topics);
  
  // Check filtered topics
  EXPECT_TRUE(registry_->is_topic_enabled("/pointcloud"));
  EXPECT_FALSE(registry_->is_topic_enabled("/cmd_vel"));
  EXPECT_TRUE(registry_->is_topic_enabled("/status"));
  
  // Verify get_enabled_topics
  auto filtered_topics = registry_->get_enabled_topics();
  EXPECT_THAT(filtered_topics, UnorderedElementsAre("/pointcloud", "/status"));
}

TEST_F(MessageTypeRegistryTest, TypeFiltering) {
  auto bag_path = bag_creator_->create_multi_topic_bag("registry_type_filter.db3", 10);
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Initially all types should be enabled
  EXPECT_TRUE(registry_->is_message_type_supported("sensor_msgs/msg/PointCloud2"));
  EXPECT_TRUE(registry_->is_message_type_supported("geometry_msgs/msg/Twist"));
  EXPECT_TRUE(registry_->is_message_type_supported("std_msgs/msg/String"));
  
  // Set type filter
  std::vector<std::string> enabled_types = {"sensor_msgs/msg/PointCloud2", "std_msgs/msg/String"};
  registry_->set_type_filter(enabled_types);
  
  // Check filtered types
  EXPECT_TRUE(registry_->is_message_type_supported("sensor_msgs/msg/PointCloud2"));
  EXPECT_FALSE(registry_->is_message_type_supported("geometry_msgs/msg/Twist"));
  EXPECT_TRUE(registry_->is_message_type_supported("std_msgs/msg/String"));
  
  // Verify get_enabled_types
  auto filtered_types = registry_->get_enabled_types();
  EXPECT_THAT(filtered_types, UnorderedElementsAre("sensor_msgs/msg/PointCloud2", "std_msgs/msg/String"));
}

TEST_F(MessageTypeRegistryTest, CombinedFiltering) {
  auto bag_path = bag_creator_->create_multi_topic_bag("registry_combined.db3", 10);
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Set both topic and type filters
  registry_->set_topic_filter({"/pointcloud", "/cmd_vel"});
  registry_->set_type_filter({"sensor_msgs/msg/PointCloud2"});
  
  // Should pass both filters
  EXPECT_TRUE(registry_->should_process_message("/pointcloud", "sensor_msgs/msg/PointCloud2"));
  
  // Should fail topic filter
  EXPECT_FALSE(registry_->should_process_message("/status", "sensor_msgs/msg/PointCloud2"));
  
  // Should fail type filter
  EXPECT_FALSE(registry_->should_process_message("/cmd_vel", "geometry_msgs/msg/Twist"));
  
  // Should fail both filters
  EXPECT_FALSE(registry_->should_process_message("/status", "std_msgs/msg/String"));
}

TEST_F(MessageTypeRegistryTest, ClearFilters) {
  auto bag_path = bag_creator_->create_multi_topic_bag("registry_clear.db3", 10);
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Set restrictive filters
  registry_->set_topic_filter({"/pointcloud"});
  registry_->set_type_filter({"sensor_msgs/msg/PointCloud2"});
  
  // Verify filtering is active
  EXPECT_FALSE(registry_->is_topic_enabled("/cmd_vel"));
  EXPECT_FALSE(registry_->is_message_type_supported("geometry_msgs/msg/Twist"));
  
  // Clear filters
  registry_->clear_topic_filter();
  registry_->clear_type_filter();
  
  // All topics and types should be enabled again
  EXPECT_TRUE(registry_->is_topic_enabled("/cmd_vel"));
  EXPECT_TRUE(registry_->is_message_type_supported("geometry_msgs/msg/Twist"));
}

TEST_F(MessageTypeRegistryTest, UnknownTopicAndTypeHandling) {
  auto bag_path = bag_creator_->create_test_bag("registry_unknown.db3", 5, {"/known"});
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Unknown topic should not be enabled
  EXPECT_FALSE(registry_->is_topic_enabled("/unknown_topic"));
  
  // Unknown type should not be supported
  EXPECT_FALSE(registry_->is_message_type_supported("unknown_msgs/msg/UnknownType"));
  
  // Unknown combinations should not be processed
  EXPECT_FALSE(registry_->should_process_message("/unknown", "unknown_msgs/msg/Type"));
}

TEST_F(MessageTypeRegistryTest, EmptyBagHandling) {
  auto bag_path = bag_creator_->create_test_bag("registry_empty.db3", 0, {});
  
  // Should handle empty bags gracefully
  EXPECT_NO_THROW(registry_->register_types_from_bag_metadata({bag_path}));
  
  // Should have no discovered types
  auto discovered_types = registry_->get_discovered_types();
  EXPECT_TRUE(discovered_types.empty());
  
  auto topic_types = registry_->get_topic_type_map();
  EXPECT_TRUE(topic_types.empty());
}

TEST_F(MessageTypeRegistryTest, InvalidBagHandling) {
  // Try to register from non-existent bag
  EXPECT_NO_THROW(registry_->register_types_from_bag_metadata({"/nonexistent/bag.db3"}));
  
  // Should have no discovered types
  auto discovered_types = registry_->get_discovered_types();
  EXPECT_TRUE(discovered_types.empty());
}

TEST_F(MessageTypeRegistryTest, DuplicateRegistration) {
  auto bag_path = bag_creator_->create_test_bag("registry_duplicate.db3", 10, {"/test"});
  
  // Register the same bag multiple times
  registry_->register_types_from_bag_metadata({bag_path});
  registry_->register_types_from_bag_metadata({bag_path});
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Should only have one entry per type/topic
  auto discovered_types = registry_->get_discovered_types();
  EXPECT_EQ(discovered_types.size(), 1);
  EXPECT_THAT(discovered_types, Contains("sensor_msgs/msg/PointCloud2"));
  
  auto topic_types = registry_->get_topic_type_map();
  EXPECT_EQ(topic_types.size(), 1);
  EXPECT_EQ(topic_types["/test"], "sensor_msgs/msg/PointCloud2");
}

TEST_F(MessageTypeRegistryTest, GetTopicsByType) {
  auto bag_path = bag_creator_->create_multi_topic_bag("registry_by_type.db3", 10);
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Get topics for specific type
  auto pointcloud_topics = registry_->get_topics_by_type("sensor_msgs/msg/PointCloud2");
  EXPECT_THAT(pointcloud_topics, Contains("/pointcloud"));
  
  auto twist_topics = registry_->get_topics_by_type("geometry_msgs/msg/Twist");
  EXPECT_THAT(twist_topics, Contains("/cmd_vel"));
  
  auto string_topics = registry_->get_topics_by_type("std_msgs/msg/String");
  EXPECT_THAT(string_topics, Contains("/status"));
  
  // Non-existent type should return empty vector
  auto unknown_topics = registry_->get_topics_by_type("unknown_msgs/msg/Unknown");
  EXPECT_TRUE(unknown_topics.empty());
}

TEST_F(MessageTypeRegistryTest, ThreadSafety) {
  auto bag_path = bag_creator_->create_multi_topic_bag("registry_thread_safe.db3", 20);
  registry_->register_types_from_bag_metadata({bag_path});
  
  std::vector<std::thread> threads;
  std::atomic<bool> stop_flag{false};
  
  // Thread 1: Topic filtering operations
  threads.emplace_back([this, &stop_flag]() {
    std::vector<std::string> topics = {"/pointcloud", "/cmd_vel"};
    while (!stop_flag) {
      registry_->set_topic_filter(topics);
      registry_->clear_topic_filter();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  
  // Thread 2: Type filtering operations
  threads.emplace_back([this, &stop_flag]() {
    std::vector<std::string> types = {"sensor_msgs/msg/PointCloud2"};
    while (!stop_flag) {
      registry_->set_type_filter(types);
      registry_->clear_type_filter();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  
  // Thread 3: Query operations
  threads.emplace_back([this, &stop_flag]() {
    while (!stop_flag) {
      registry_->is_topic_enabled("/pointcloud");
      registry_->is_message_type_supported("sensor_msgs/msg/PointCloud2");
      registry_->should_process_message("/cmd_vel", "geometry_msgs/msg/Twist");
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  
  // Let threads run for a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stop_flag = true;
  
  // Wait for all threads to complete
  for (auto& thread : threads) {
    thread.join();
  }
  
  // Registry should still be in a valid state
  EXPECT_NO_THROW(registry_->get_discovered_types());
}