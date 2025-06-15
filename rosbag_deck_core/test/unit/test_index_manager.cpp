#include <gtest/gtest.h>
#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include "../fixtures/test_bag_creator.hpp"
#include <chrono>

using namespace rosbag_deck_core;
using namespace rosbag_deck_core::test;

class IndexManagerTest : public ::testing::Test {
protected:
  void SetUp() override {
    bag_creator_ = std::make_unique<TestBagCreator>();
    index_manager_ = std::make_unique<IndexManager>();
  }

  void TearDown() override {
    bag_creator_->cleanup();
  }

  std::unique_ptr<TestBagCreator> bag_creator_;
  std::unique_ptr<IndexManager> index_manager_;
};

TEST_F(IndexManagerTest, BuildIndexSingleBag) {
  // Create a test bag
  auto bag_path = bag_creator_->create_test_bag("test_single.db3", 10, {"/test_topic"});
  
  // Build index
  index_manager_->build_index({bag_path});
  
  // Verify basic properties
  EXPECT_EQ(index_manager_->total_frames(), 10);
  EXPECT_EQ(index_manager_->bag_paths().size(), 1);
  EXPECT_EQ(index_manager_->bag_paths()[0], bag_path);
  
  // Verify topic information
  auto topics = index_manager_->topic_names();
  EXPECT_TRUE(std::find(topics.begin(), topics.end(), "/test_topic") != topics.end());
  
  auto topic_types = index_manager_->topic_types();
  EXPECT_EQ(topic_types.at("/test_topic"), "sensor_msgs/msg/PointCloud2");
}

TEST_F(IndexManagerTest, BuildIndexMultipleBags) {
  // Create multiple test bags
  auto bag1 = bag_creator_->create_test_bag("test_multi1.db3", 5, {"/topic1"});
  auto bag2 = bag_creator_->create_test_bag("test_multi2.db3", 7, {"/topic2"});
  
  // Build index
  index_manager_->build_index({bag1, bag2});
  
  // Verify combined index
  EXPECT_EQ(index_manager_->total_frames(), 12);
  EXPECT_EQ(index_manager_->bag_paths().size(), 2);
  
  // Check that both topics are present
  auto topics = index_manager_->topic_names();
  EXPECT_TRUE(std::find(topics.begin(), topics.end(), "/topic1") != topics.end());
  EXPECT_TRUE(std::find(topics.begin(), topics.end(), "/topic2") != topics.end());
}

TEST_F(IndexManagerTest, BuildIndexEmptyBag) {
  // Create an empty bag
  auto bag_path = bag_creator_->create_test_bag("test_empty.db3", 0, {});
  
  // Build index
  index_manager_->build_index({bag_path});
  
  // Verify empty index
  EXPECT_EQ(index_manager_->total_frames(), 0);
}

TEST_F(IndexManagerTest, GetEntryValidIndex) {
  // Create a test bag
  auto bag_path = bag_creator_->create_test_bag("test_entry.db3", 5, {"/test"});
  index_manager_->build_index({bag_path});
  
  // Test valid indices
  for (size_t i = 0; i < index_manager_->total_frames(); ++i) {
    EXPECT_NO_THROW({
      const auto& entry = index_manager_->get_entry(i);
      EXPECT_EQ(entry.frame_index, i);
      EXPECT_EQ(entry.topic_name, "/test");
      EXPECT_EQ(entry.message_type, "sensor_msgs/msg/PointCloud2");
    });
  }
}

TEST_F(IndexManagerTest, GetEntryInvalidIndex) {
  // Create a test bag
  auto bag_path = bag_creator_->create_test_bag("test_invalid.db3", 5, {"/test"});
  index_manager_->build_index({bag_path});
  
  // Test invalid index (should throw or handle gracefully)
  EXPECT_THROW(index_manager_->get_entry(100), std::out_of_range);
}

TEST_F(IndexManagerTest, FindFrameByTime) {
  // Create a test bag with known timing
  auto bag_path = bag_creator_->create_test_bag("test_timing.db3", 10, {"/timed"});
  index_manager_->build_index({bag_path});
  
  if (index_manager_->total_frames() > 0) {
    // Test finding frame at start time
    auto start_time = index_manager_->start_time();
    size_t start_frame = index_manager_->find_frame_by_time(start_time);
    EXPECT_EQ(start_frame, 0);
    
    // Test finding frame at end time
    auto end_time = index_manager_->end_time();
    size_t end_frame = index_manager_->find_frame_by_time(end_time);
    EXPECT_LT(end_frame, index_manager_->total_frames());
    
    // Test finding frame in middle
    auto middle_time = start_time + std::chrono::nanoseconds((end_time - start_time).count() / 2);
    size_t middle_frame = index_manager_->find_frame_by_time(middle_time);
    EXPECT_GT(middle_frame, 0);
    EXPECT_LT(middle_frame, index_manager_->total_frames());
  }
}

TEST_F(IndexManagerTest, TimeRangeProperties) {
  // Create a test bag
  auto bag_path = bag_creator_->create_test_bag("test_time_range.db3", 10, {"/time_test"});
  index_manager_->build_index({bag_path});
  
  if (index_manager_->total_frames() > 0) {
    // Start time should be before or equal to end time
    EXPECT_LE(index_manager_->start_time(), index_manager_->end_time());
    
    // For multiple frames, start and end should be different
    if (index_manager_->total_frames() > 1) {
      EXPECT_LT(index_manager_->start_time(), index_manager_->end_time());
    }
  }
}

TEST_F(IndexManagerTest, MultiTopicBag) {
  // Create a bag with multiple topics
  auto bag_path = bag_creator_->create_multi_topic_bag("test_multi_topic.db3", 5);
  index_manager_->build_index({bag_path});
  
  // Should have multiple topics
  auto topics = index_manager_->topic_names();
  EXPECT_GE(topics.size(), 2);
  
  // Should have multiple message types
  auto topic_types = index_manager_->topic_types();
  EXPECT_GE(topic_types.size(), 2);
  
  // Verify some expected topics and types
  EXPECT_TRUE(std::find(topics.begin(), topics.end(), "/pointcloud") != topics.end());
  EXPECT_TRUE(std::find(topics.begin(), topics.end(), "/cmd_vel") != topics.end());
  EXPECT_TRUE(std::find(topics.begin(), topics.end(), "/status") != topics.end());
  
  EXPECT_EQ(topic_types.at("/pointcloud"), "sensor_msgs/msg/PointCloud2");
  EXPECT_EQ(topic_types.at("/cmd_vel"), "geometry_msgs/msg/Twist");
  EXPECT_EQ(topic_types.at("/status"), "std_msgs/msg/String");
}

TEST_F(IndexManagerTest, InvalidBagPath) {
  // Try to build index with non-existent bag
  EXPECT_NO_THROW(index_manager_->build_index({"/nonexistent/bag.db3"}));
  
  // Should have empty index
  EXPECT_EQ(index_manager_->total_frames(), 0);
}

TEST_F(IndexManagerTest, IndexConsistency) {
  // Create a test bag
  auto bag_path = bag_creator_->create_test_bag("test_consistency.db3", 20, {"/consistent"});
  index_manager_->build_index({bag_path});
  
  // Verify frame indices are sequential
  for (size_t i = 0; i < index_manager_->total_frames(); ++i) {
    const auto& entry = index_manager_->get_entry(i);
    EXPECT_EQ(entry.frame_index, i);
  }
  
  // Verify timestamps are non-decreasing
  for (size_t i = 1; i < index_manager_->total_frames(); ++i) {
    const auto& prev_entry = index_manager_->get_entry(i - 1);
    const auto& curr_entry = index_manager_->get_entry(i);
    EXPECT_LE(prev_entry.timestamp, curr_entry.timestamp);
  }
}

TEST_F(IndexManagerTest, TopicTypeMapping) {
  // Create bags with different message types
  auto bag_path = bag_creator_->create_multi_topic_bag("test_types.db3", 3);
  index_manager_->build_index({bag_path});
  
  auto topic_types = index_manager_->topic_types();
  
  // Verify each topic has a type mapping
  for (const auto& topic : index_manager_->topic_names()) {
    EXPECT_TRUE(topic_types.find(topic) != topic_types.end());
    EXPECT_FALSE(topic_types.at(topic).empty());
  }
}

TEST_F(IndexManagerTest, LargeBagHandling) {
  // Create a larger bag to test scalability
  auto bag_path = bag_creator_->create_large_bag("test_large.db3", 100);
  index_manager_->build_index({bag_path});
  
  EXPECT_EQ(index_manager_->total_frames(), 100);
  
  // Test random access to verify index integrity
  std::vector<size_t> test_indices = {0, 25, 50, 75, 99};
  for (size_t idx : test_indices) {
    EXPECT_NO_THROW({
      const auto& entry = index_manager_->get_entry(idx);
      EXPECT_EQ(entry.frame_index, idx);
    });
  }
}