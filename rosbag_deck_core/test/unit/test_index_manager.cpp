#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include "../fixtures/test_bag_creator.hpp"
#include "../fixtures/test_data_generator.hpp"
#include "../fixtures/mock_callbacks.hpp"
#include <chrono>

using namespace rosbag_deck_core;
using namespace rosbag_deck_core::test;
using namespace testing;

class IndexManagerTest : public ::testing::Test {
protected:
  void SetUp() override {
    bag_creator_ = std::make_unique<TestBagCreator>();
    data_generator_ = std::make_unique<TestDataGenerator>();
  }

  void TearDown() override {
    bag_creator_->cleanup();
  }

  std::unique_ptr<TestBagCreator> bag_creator_;
  std::unique_ptr<TestDataGenerator> data_generator_;
};

TEST_F(IndexManagerTest, BuildIndexSingleBag) {
  // Create a test bag
  auto bag_path = bag_creator_->create_test_bag("test_single.db3", 10, {"/test_topic"});
  
  // Build index
  IndexManager manager;
  manager.build_index({bag_path});
  
  // Verify index
  auto& index = manager.get_index();
  EXPECT_EQ(index.size(), 10);
  
  // Check index entries
  for (size_t i = 0; i < index.size(); ++i) {
    EXPECT_EQ(index[i].frame_index, i);
    EXPECT_EQ(index[i].topic_name, "/test_topic");
    EXPECT_EQ(index[i].message_type, "sensor_msgs/msg/PointCloud2");
  }
}

TEST_F(IndexManagerTest, BuildIndexMultipleBags) {
  // Create multiple test bags
  auto bag1 = bag_creator_->create_test_bag("test_multi1.db3", 5, {"/topic1"});
  auto bag2 = bag_creator_->create_test_bag("test_multi2.db3", 5, {"/topic2"});
  
  // Build index
  IndexManager manager;
  manager.build_index({bag1, bag2});
  
  // Verify combined index
  auto& index = manager.get_index();
  EXPECT_EQ(index.size(), 10);
  
  // Check that both topics are present
  int topic1_count = 0;
  int topic2_count = 0;
  for (const auto& entry : index) {
    if (entry.topic_name == "/topic1") topic1_count++;
    if (entry.topic_name == "/topic2") topic2_count++;
  }
  EXPECT_EQ(topic1_count, 5);
  EXPECT_EQ(topic2_count, 5);
}

TEST_F(IndexManagerTest, BuildIndexEmptyBag) {
  // Create an empty bag
  auto bag_path = bag_creator_->create_test_bag("test_empty.db3", 0, {"/empty_topic"});
  
  // Build index
  IndexManager manager;
  manager.build_index({bag_path});
  
  // Verify empty index
  auto& index = manager.get_index();
  EXPECT_TRUE(index.empty());
}

TEST_F(IndexManagerTest, BuildIndexMultipleTopics) {
  // Create a bag with multiple topics
  auto bag_path = bag_creator_->create_multi_topic_bag("test_multi_topic.db3", 10);
  
  // Build index
  IndexManager manager;
  manager.build_index({bag_path});
  
  // Verify index contains all messages
  auto& index = manager.get_index();
  EXPECT_EQ(index.size(), 30); // 10 messages * 3 topics
  
  // Verify topic distribution
  std::map<std::string, int> topic_counts;
  for (const auto& entry : index) {
    topic_counts[entry.topic_name]++;
  }
  
  EXPECT_EQ(topic_counts["/pointcloud"], 10);
  EXPECT_EQ(topic_counts["/cmd_vel"], 10);
  EXPECT_EQ(topic_counts["/status"], 10);
}

TEST_F(IndexManagerTest, GetFrameInfo) {
  // Create a test bag
  auto bag_path = bag_creator_->create_test_bag("test_frame_info.db3", 10, {"/test"});
  
  // Build index
  IndexManager manager;
  manager.build_index({bag_path});
  
  // Test valid frame indices
  for (size_t i = 0; i < 10; ++i) {
    auto info_opt = manager.get_frame_info(i);
    ASSERT_TRUE(info_opt.has_value());
    auto& info = info_opt.value();
    EXPECT_EQ(info.frame_index, i);
    EXPECT_EQ(info.topic_name, "/test");
  }
  
  // Test invalid frame index
  auto invalid_info = manager.get_frame_info(100);
  EXPECT_FALSE(invalid_info.has_value());
}

TEST_F(IndexManagerTest, GetTopics) {
  // Create a bag with multiple topics
  auto bag_path = bag_creator_->create_multi_topic_bag("test_topics.db3", 5);
  
  // Build index
  IndexManager manager;
  manager.build_index({bag_path});
  
  // Get topics
  auto topics = manager.get_topics();
  EXPECT_EQ(topics.size(), 3);
  EXPECT_THAT(topics, UnorderedElementsAre("/pointcloud", "/cmd_vel", "/status"));
}

TEST_F(IndexManagerTest, GetTopicInfo) {
  // Create a bag with known topics
  auto bag_path = bag_creator_->create_multi_topic_bag("test_topic_info.db3", 10);
  
  // Build index
  IndexManager manager;
  manager.build_index({bag_path});
  
  // Test valid topic
  auto info_opt = manager.get_topic_info("/pointcloud");
  ASSERT_TRUE(info_opt.has_value());
  auto& info = info_opt.value();
  EXPECT_EQ(info.topic_name, "/pointcloud");
  EXPECT_EQ(info.message_type, "sensor_msgs/msg/PointCloud2");
  EXPECT_EQ(info.message_count, 10);
  
  // Test invalid topic
  auto invalid_info = manager.get_topic_info("/nonexistent");
  EXPECT_FALSE(invalid_info.has_value());
}

TEST_F(IndexManagerTest, ClearIndex) {
  // Create and build index
  auto bag_path = bag_creator_->create_test_bag("test_clear.db3", 10);
  IndexManager manager;
  manager.build_index({bag_path});
  
  // Verify index is populated
  EXPECT_FALSE(manager.get_index().empty());
  
  // Clear index
  manager.clear();
  
  // Verify index is empty
  EXPECT_TRUE(manager.get_index().empty());
  EXPECT_TRUE(manager.get_topics().empty());
}

TEST_F(IndexManagerTest, BuildIndexWithInvalidPath) {
  IndexManager manager;
  
  // Expect no crash with invalid path
  EXPECT_NO_THROW(manager.build_index({"/nonexistent/path.db3"}));
  
  // Index should be empty
  EXPECT_TRUE(manager.get_index().empty());
}

TEST_F(IndexManagerTest, IndexOrdering) {
  // Create a test bag
  auto bag_path = bag_creator_->create_test_bag("test_ordering.db3", 20, {"/ordered"});
  
  // Build index
  IndexManager manager;
  manager.build_index({bag_path});
  
  // Verify messages are ordered by timestamp
  auto& index = manager.get_index();
  for (size_t i = 1; i < index.size(); ++i) {
    EXPECT_LE(index[i-1].original_timestamp, index[i].original_timestamp);
  }
}