#include <gtest/gtest.h>
#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include "../fixtures/test_bag_creator.hpp"
#include <thread>

using namespace rosbag_deck_core;
using namespace rosbag_deck_core::test;

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

TEST_F(MessageTypeRegistryTest, DefaultConstructor) {
  // Registry should start empty
  auto types = registry_->get_registered_types();
  EXPECT_TRUE(types.empty());
}

TEST_F(MessageTypeRegistryTest, RegisterTypesFromSingleBag) {
  // Create a test bag with known types
  auto bag_path = bag_creator_->create_test_bag("registry_single.db3", 10, {"/laser"});
  
  // Register types from bag
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Verify types were registered
  auto types = registry_->get_registered_types();
  EXPECT_TRUE(std::find(types.begin(), types.end(), "sensor_msgs/msg/PointCloud2") != types.end());
  
  // Verify type is registered
  EXPECT_TRUE(registry_->is_type_registered("sensor_msgs/msg/PointCloud2"));
}

TEST_F(MessageTypeRegistryTest, RegisterTypesFromMultipleBags) {
  // Create bags with different types
  auto bag1 = bag_creator_->create_test_bag("registry_multi1.db3", 5, {"/topic1"});
  auto bag2 = bag_creator_->create_multi_topic_bag("registry_multi2.db3", 5);
  
  // Register types from both bags
  registry_->register_types_from_bag_metadata({bag1, bag2});
  
  // Verify all types were discovered
  auto types = registry_->get_registered_types();
  EXPECT_TRUE(std::find(types.begin(), types.end(), "sensor_msgs/msg/PointCloud2") != types.end());
  EXPECT_TRUE(std::find(types.begin(), types.end(), "geometry_msgs/msg/Twist") != types.end());
  EXPECT_TRUE(std::find(types.begin(), types.end(), "std_msgs/msg/String") != types.end());
  
  // Verify all types are registered
  EXPECT_TRUE(registry_->is_type_registered("sensor_msgs/msg/PointCloud2"));
  EXPECT_TRUE(registry_->is_type_registered("geometry_msgs/msg/Twist"));
  EXPECT_TRUE(registry_->is_type_registered("std_msgs/msg/String"));
}

TEST_F(MessageTypeRegistryTest, TopicFiltering) {
  auto bag_path = bag_creator_->create_multi_topic_bag("registry_filter.db3", 10);
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Initially all topics should be enabled (no filter)
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
  
  // Clear filter
  registry_->clear_topic_filter();
  
  // All topics should be enabled again
  EXPECT_TRUE(registry_->is_topic_enabled("/pointcloud"));
  EXPECT_TRUE(registry_->is_topic_enabled("/cmd_vel"));
  EXPECT_TRUE(registry_->is_topic_enabled("/status"));
}

TEST_F(MessageTypeRegistryTest, TypeFiltering) {
  auto bag_path = bag_creator_->create_multi_topic_bag("registry_type_filter.db3", 10);
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Initially all types should be enabled (no filter)
  EXPECT_TRUE(registry_->is_type_enabled("sensor_msgs/msg/PointCloud2"));
  EXPECT_TRUE(registry_->is_type_enabled("geometry_msgs/msg/Twist"));
  EXPECT_TRUE(registry_->is_type_enabled("std_msgs/msg/String"));
  
  // Set type filter
  std::vector<std::string> enabled_types = {"sensor_msgs/msg/PointCloud2", "std_msgs/msg/String"};
  registry_->set_type_filter(enabled_types);
  
  // Check filtered types
  EXPECT_TRUE(registry_->is_type_enabled("sensor_msgs/msg/PointCloud2"));
  EXPECT_FALSE(registry_->is_type_enabled("geometry_msgs/msg/Twist"));
  EXPECT_TRUE(registry_->is_type_enabled("std_msgs/msg/String"));
  
  // Clear filter
  registry_->clear_type_filter();
  
  // All types should be enabled again
  EXPECT_TRUE(registry_->is_type_enabled("sensor_msgs/msg/PointCloud2"));
  EXPECT_TRUE(registry_->is_type_enabled("geometry_msgs/msg/Twist"));
  EXPECT_TRUE(registry_->is_type_enabled("std_msgs/msg/String"));
}

TEST_F(MessageTypeRegistryTest, UnknownTopicAndTypeHandling) {
  auto bag_path = bag_creator_->create_test_bag("registry_unknown.db3", 5, {"/known"});
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Without filters, unknown topics are allowed (permissive default behavior)
  EXPECT_TRUE(registry_->is_topic_enabled("/unknown_topic"));
  
  // Without filters, unknown types are allowed (permissive default behavior) 
  EXPECT_TRUE(registry_->is_type_enabled("unknown_msgs/msg/UnknownType"));
  
  // But unknown types should not be registered in the type registry
  EXPECT_FALSE(registry_->is_type_registered("unknown_msgs/msg/UnknownType"));
  
  // Test filtering behavior: set a filter that only allows known items
  registry_->set_topic_filter({"/known"});
  EXPECT_TRUE(registry_->is_topic_enabled("/known"));
  EXPECT_FALSE(registry_->is_topic_enabled("/unknown_topic"));
  
  // Set type filter for known types
  registry_->set_type_filter({"sensor_msgs/msg/PointCloud2"});
  EXPECT_TRUE(registry_->is_type_enabled("sensor_msgs/msg/PointCloud2"));
  EXPECT_FALSE(registry_->is_type_enabled("unknown_msgs/msg/UnknownType"));
}

TEST_F(MessageTypeRegistryTest, EmptyBagHandling) {
  auto bag_path = bag_creator_->create_test_bag("registry_empty.db3", 0, {});
  
  // Should handle empty bags gracefully
  EXPECT_NO_THROW(registry_->register_types_from_bag_metadata({bag_path}));
  
  // Should have no registered types
  auto types = registry_->get_registered_types();
  EXPECT_TRUE(types.empty());
}

TEST_F(MessageTypeRegistryTest, InvalidBagHandling) {
  // Try to register from non-existent bag
  EXPECT_NO_THROW(registry_->register_types_from_bag_metadata({"/nonexistent/bag.db3"}));
  
  // Should have no registered types
  auto types = registry_->get_registered_types();
  EXPECT_TRUE(types.empty());
}

TEST_F(MessageTypeRegistryTest, DuplicateRegistration) {
  auto bag_path = bag_creator_->create_test_bag("registry_duplicate.db3", 10, {"/test"});
  
  // Register the same bag multiple times
  registry_->register_types_from_bag_metadata({bag_path});
  registry_->register_types_from_bag_metadata({bag_path});
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Should only have one entry per type
  auto types = registry_->get_registered_types();
  EXPECT_EQ(types.size(), 1);
  EXPECT_TRUE(std::find(types.begin(), types.end(), "sensor_msgs/msg/PointCloud2") != types.end());
}

TEST_F(MessageTypeRegistryTest, CanDeserialize) {
  auto bag_path = bag_creator_->create_multi_topic_bag("registry_deserialize.db3", 5);
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Should be able to deserialize registered types
  EXPECT_TRUE(registry_->can_deserialize("sensor_msgs/msg/PointCloud2"));
  EXPECT_TRUE(registry_->can_deserialize("geometry_msgs/msg/Twist"));
  EXPECT_TRUE(registry_->can_deserialize("std_msgs/msg/String"));
  
  // Should not be able to deserialize unregistered types
  EXPECT_FALSE(registry_->can_deserialize("unknown_msgs/msg/Unknown"));
}

TEST_F(MessageTypeRegistryTest, TypeHash) {
  auto bag_path = bag_creator_->create_test_bag("registry_hash.db3", 5, {"/test"});
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Should have hash for registered types
  auto hash = registry_->get_type_hash("sensor_msgs/msg/PointCloud2");
  EXPECT_FALSE(hash.empty());
  
  // Hash should be consistent
  auto hash2 = registry_->get_type_hash("sensor_msgs/msg/PointCloud2");
  EXPECT_EQ(hash, hash2);
  
  // Unknown types should return empty hash
  auto unknown_hash = registry_->get_type_hash("unknown_msgs/msg/Unknown");
  EXPECT_TRUE(unknown_hash.empty());
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
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  });
  
  // Thread 2: Type filtering operations
  threads.emplace_back([this, &stop_flag]() {
    std::vector<std::string> types = {"sensor_msgs/msg/PointCloud2"};
    while (!stop_flag) {
      registry_->set_type_filter(types);
      registry_->clear_type_filter();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  });
  
  // Thread 3: Query operations
  threads.emplace_back([this, &stop_flag]() {
    while (!stop_flag) {
      registry_->is_topic_enabled("/pointcloud");
      registry_->is_type_enabled("sensor_msgs/msg/PointCloud2");
      registry_->is_type_registered("geometry_msgs/msg/Twist");
      registry_->can_deserialize("std_msgs/msg/String");
      registry_->get_type_hash("sensor_msgs/msg/PointCloud2");
      std::this_thread::sleep_for(std::chrono::microseconds(5));
    }
  });
  
  // Let threads run for a short time
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stop_flag = true;
  
  // Wait for all threads to complete
  for (auto& thread : threads) {
    thread.join();
  }
  
  // Registry should still be in a valid state
  EXPECT_NO_THROW(registry_->get_registered_types());
}

TEST_F(MessageTypeRegistryTest, FilterCombinations) {
  auto bag_path = bag_creator_->create_multi_topic_bag("registry_combinations.db3", 5);
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Test various filter combinations
  registry_->set_topic_filter({"/pointcloud"});
  registry_->set_type_filter({"sensor_msgs/msg/PointCloud2"});
  
  EXPECT_TRUE(registry_->is_topic_enabled("/pointcloud"));
  EXPECT_FALSE(registry_->is_topic_enabled("/cmd_vel"));
  EXPECT_TRUE(registry_->is_type_enabled("sensor_msgs/msg/PointCloud2"));
  EXPECT_FALSE(registry_->is_type_enabled("geometry_msgs/msg/Twist"));
  
  // Clear one filter
  registry_->clear_topic_filter();
  
  EXPECT_TRUE(registry_->is_topic_enabled("/pointcloud"));
  EXPECT_TRUE(registry_->is_topic_enabled("/cmd_vel"));
  EXPECT_TRUE(registry_->is_type_enabled("sensor_msgs/msg/PointCloud2"));
  EXPECT_FALSE(registry_->is_type_enabled("geometry_msgs/msg/Twist"));
  
  // Clear remaining filter
  registry_->clear_type_filter();
  
  EXPECT_TRUE(registry_->is_topic_enabled("/pointcloud"));
  EXPECT_TRUE(registry_->is_topic_enabled("/cmd_vel"));
  EXPECT_TRUE(registry_->is_type_enabled("sensor_msgs/msg/PointCloud2"));
  EXPECT_TRUE(registry_->is_type_enabled("geometry_msgs/msg/Twist"));
}

TEST_F(MessageTypeRegistryTest, EmptyFilters) {
  auto bag_path = bag_creator_->create_multi_topic_bag("registry_empty_filters.db3", 5);
  registry_->register_types_from_bag_metadata({bag_path});
  
  // Set empty filters
  registry_->set_topic_filter({});
  registry_->set_type_filter({});
  
  // With empty filters, nothing should be enabled
  EXPECT_FALSE(registry_->is_topic_enabled("/pointcloud"));
  EXPECT_FALSE(registry_->is_topic_enabled("/cmd_vel"));
  EXPECT_FALSE(registry_->is_type_enabled("sensor_msgs/msg/PointCloud2"));
  EXPECT_FALSE(registry_->is_type_enabled("geometry_msgs/msg/Twist"));
}