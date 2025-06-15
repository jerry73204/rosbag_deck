#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include "../fixtures/test_bag_creator.hpp"
#include "../fixtures/test_data_generator.hpp"
#include "../fixtures/mock_callbacks.hpp"
#include <chrono>
#include <thread>

using namespace rosbag_deck_core;
using namespace rosbag_deck_core::test;
using namespace testing;

class IntegrationTest : public ::testing::Test {
protected:
  void SetUp() override {
    bag_creator_ = std::make_unique<TestBagCreator>();
    data_generator_ = std::make_unique<TestDataGenerator>();
    mock_handler_ = std::make_shared<MockCallbackHandler>();
  }

  void TearDown() override {
    bag_creator_->cleanup();
  }

  std::unique_ptr<TestBagCreator> bag_creator_;
  std::unique_ptr<TestDataGenerator> data_generator_;
  std::shared_ptr<MockCallbackHandler> mock_handler_;
};

TEST_F(IntegrationTest, FullPlaybackWorkflow) {
  // Create a test bag with realistic data
  auto bag_path = bag_creator_->create_multi_topic_bag("integration_full.db3", 20);
  
  // Create core instance
  auto core = std::make_unique<RosbagDeckCore>();
  
  // Set up callbacks
  core->set_message_callback([this](const BagMessage& msg) {
    mock_handler_->record_message(msg);
  });
  core->set_status_callback([this](const PlaybackStatus& status) {
    mock_handler_->record_status(status);
  });
  
  // Open bag
  ASSERT_TRUE(core->open({bag_path}));
  
  // Verify bag info
  auto bag_info = core->get_bag_info();
  EXPECT_EQ(bag_info.total_frames, 60); // 20 messages * 3 topics
  EXPECT_GE(bag_info.topics.size(), 3);
  
  // Test complete playback workflow
  EXPECT_CALL(*mock_handler_, on_message(_)).Times(AtLeast(60));
  EXPECT_CALL(*mock_handler_, on_status(_)).Times(AtLeast(5));
  
  // 1. Seek to middle
  EXPECT_TRUE(core->seek_to_frame(30));
  
  // 2. Step through some frames
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(core->step_forward());
  }
  
  // 3. Change playback speed
  EXPECT_TRUE(core->set_playback_speed(2.0));
  
  // 4. Play for a short time
  core->play();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  core->pause();
  
  // 5. Step backward
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(core->step_backward());
  }
  
  // 6. Rewind to beginning
  core->rewind();
  EXPECT_EQ(core->get_playback_status().current_frame, 0);
  
  // 7. Play through several frames
  core->play();
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  core->stop();
  
  // Verify messages were received from all topics
  auto messages = mock_handler_->get_message_history();
  EXPECT_GT(messages.size(), 30);
  
  std::set<std::string> received_topics;
  std::set<std::string> received_types;
  for (const auto& msg : messages) {
    received_topics.insert(msg.topic_name);
    received_types.insert(msg.message_type);
  }
  
  EXPECT_GE(received_topics.size(), 3);
  EXPECT_GE(received_types.size(), 3);
  
  core->close();
}

TEST_F(IntegrationTest, MultipleTopicFiltering) {
  auto bag_path = bag_creator_->create_multi_topic_bag("integration_filtering.db3", 15);
  auto core = std::make_unique<RosbagDeckCore>();
  
  core->set_message_callback([this](const BagMessage& msg) {
    mock_handler_->record_message(msg);
  });
  
  ASSERT_TRUE(core->open({bag_path}));
  
  // Test 1: Filter to only PointCloud2 messages
  core->set_type_filter({"sensor_msgs/msg/PointCloud2"});
  
  // Step through all frames
  for (size_t i = 0; i < 45; ++i) { // 15 * 3 topics
    core->step_forward();
  }
  
  auto pointcloud_messages = mock_handler_->get_message_history();
  for (const auto& msg : pointcloud_messages) {
    EXPECT_EQ(msg.message_type, "sensor_msgs/msg/PointCloud2");
  }
  
  mock_handler_->clear_history();
  core->rewind();
  
  // Test 2: Filter to specific topics
  core->clear_type_filter();
  core->set_topic_filter({"/cmd_vel", "/status"});
  
  for (size_t i = 0; i < 45; ++i) {
    core->step_forward();
  }
  
  auto filtered_messages = mock_handler_->get_message_history();
  for (const auto& msg : filtered_messages) {
    EXPECT_TRUE(msg.topic_name == "/cmd_vel" || msg.topic_name == "/status");
  }
  
  core->close();
}

TEST_F(IntegrationTest, LargeBagPlayback) {
  // Create a large bag for performance testing
  auto large_bag = bag_creator_->create_large_bag("integration_large.db3", 500);
  auto core = std::make_unique<RosbagDeckCore>();
  
  int message_count = 0;
  core->set_message_callback([&message_count](const BagMessage& msg) {
    message_count++;
  });
  
  ASSERT_TRUE(core->open({large_bag}));
  
  auto bag_info = core->get_bag_info();
  EXPECT_EQ(bag_info.total_frames, 500);
  
  // Test seeking performance
  auto start_time = std::chrono::high_resolution_clock::now();
  
  // Seek to various positions
  std::vector<size_t> seek_positions = {100, 250, 400, 50, 350, 150};
  for (size_t pos : seek_positions) {
    EXPECT_TRUE(core->seek_to_frame(pos));
    EXPECT_EQ(core->get_playback_status().current_frame, pos);
  }
  
  auto seek_time = std::chrono::high_resolution_clock::now() - start_time;
  
  // Seeking should be fast (less than 100ms for all operations)
  EXPECT_LT(seek_time.count(), 100000000); // 100ms in nanoseconds
  
  // Test stepping performance
  start_time = std::chrono::high_resolution_clock::now();
  
  core->seek_to_frame(100);
  for (int i = 0; i < 50; ++i) {
    EXPECT_TRUE(core->step_forward());
  }
  
  auto step_time = std::chrono::high_resolution_clock::now() - start_time;
  
  // Stepping should be reasonably fast
  EXPECT_LT(step_time.count(), 500000000); // 500ms in nanoseconds
  EXPECT_GE(message_count, 50);
  
  core->close();
}

TEST_F(IntegrationTest, MultipleBagCombination) {
  // Create multiple bags with different characteristics
  auto bag1 = bag_creator_->create_test_bag("integration_multi1.db3", 10, {"/lidar"});
  auto bag2 = bag_creator_->create_test_bag("integration_multi2.db3", 15, {"/camera"});
  auto bag3 = bag_creator_->create_multi_topic_bag("integration_multi3.db3", 8);
  
  auto core = std::make_unique<RosbagDeckCore>();
  
  core->set_message_callback([this](const BagMessage& msg) {
    mock_handler_->record_message(msg);
  });
  
  // Open all bags together
  ASSERT_TRUE(core->open({bag1, bag2, bag3}));
  
  auto bag_info = core->get_bag_info();
  EXPECT_EQ(bag_info.total_frames, 10 + 15 + 24); // 10 + 15 + (8*3)
  
  // Verify all topics are available
  auto topics = core->get_available_topics();
  EXPECT_THAT(topics, Contains("/lidar"));
  EXPECT_THAT(topics, Contains("/camera"));
  EXPECT_THAT(topics, Contains("/pointcloud"));
  EXPECT_THAT(topics, Contains("/cmd_vel"));
  EXPECT_THAT(topics, Contains("/status"));
  
  // Play through all frames
  EXPECT_CALL(*mock_handler_, on_message(_)).Times(AtLeast(40));
  
  for (size_t i = 0; i < bag_info.total_frames; ++i) {
    core->step_forward();
  }
  
  // Verify messages from all bags were received
  auto messages = mock_handler_->get_message_history();
  
  std::set<std::string> received_topics;
  for (const auto& msg : messages) {
    received_topics.insert(msg.topic_name);
  }
  
  EXPECT_GE(received_topics.size(), 5);
  EXPECT_THAT(received_topics, Contains("/lidar"));
  EXPECT_THAT(received_topics, Contains("/camera"));
  
  core->close();
}

TEST_F(IntegrationTest, ErrorRecoveryAndRobustness) {
  auto valid_bag = bag_creator_->create_test_bag("integration_valid.db3", 10, {"/valid"});
  auto core = std::make_unique<RosbagDeckCore>();
  
  // Test 1: Opening invalid bag first
  EXPECT_FALSE(core->open({"/nonexistent/bag.db3"}));
  EXPECT_FALSE(core->is_open());
  
  // Should be able to open valid bag after failure
  EXPECT_TRUE(core->open({valid_bag}));
  EXPECT_TRUE(core->is_open());
  
  // Test 2: Operations on closed bag
  core->close();
  EXPECT_FALSE(core->is_open());
  
  // These operations should fail gracefully
  EXPECT_FALSE(core->seek_to_frame(5));
  EXPECT_FALSE(core->step_forward());
  EXPECT_FALSE(core->step_backward());
  
  auto status = core->get_playback_status();
  EXPECT_EQ(status.state, PlaybackState::STOPPED);
  
  // Test 3: Reopen and verify functionality
  EXPECT_TRUE(core->open({valid_bag}));
  EXPECT_TRUE(core->seek_to_frame(5));
  EXPECT_TRUE(core->step_forward());
  
  core->close();
}

TEST_F(IntegrationTest, ConcurrentAccess) {
  auto bag_path = bag_creator_->create_multi_topic_bag("integration_concurrent.db3", 30);
  auto core = std::make_unique<RosbagDeckCore>();
  
  std::atomic<int> message_count{0};
  std::atomic<int> status_count{0};
  
  core->set_message_callback([&message_count](const BagMessage& msg) {
    message_count++;
  });
  core->set_status_callback([&status_count](const PlaybackStatus& status) {
    status_count++;
  });
  
  ASSERT_TRUE(core->open({bag_path}));
  
  std::vector<std::thread> threads;
  std::atomic<bool> stop_flag{false};
  
  // Thread 1: Continuous playback operations
  threads.emplace_back([&core, &stop_flag]() {
    while (!stop_flag) {
      core->play();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      core->pause();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
  
  // Thread 2: Seeking operations
  threads.emplace_back([&core, &stop_flag]() {
    size_t frame = 0;
    while (!stop_flag) {
      core->seek_to_frame(frame % 90); // 30 * 3 topics
      frame += 5;
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  });
  
  // Thread 3: Status queries
  threads.emplace_back([&core, &stop_flag]() {
    while (!stop_flag) {
      auto status = core->get_playback_status();
      auto info = core->get_bag_info();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
  
  // Thread 4: Filter operations
  threads.emplace_back([&core, &stop_flag]() {
    std::vector<std::string> topics1 = {"/pointcloud"};
    std::vector<std::string> topics2 = {"/cmd_vel", "/status"};
    bool use_filter1 = true;
    
    while (!stop_flag) {
      if (use_filter1) {
        core->set_topic_filter(topics1);
      } else {
        core->set_topic_filter(topics2);
      }
      use_filter1 = !use_filter1;
      std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
  });
  
  // Let threads run for a reasonable time
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  stop_flag = true;
  
  // Wait for all threads to complete
  for (auto& thread : threads) {
    thread.join();
  }
  
  // Verify that operations completed without crashes
  EXPECT_TRUE(core->is_open());
  EXPECT_GT(message_count.load(), 0);
  EXPECT_GT(status_count.load(), 0);
  
  // Final verification of core state
  auto final_status = core->get_playback_status();
  EXPECT_GE(final_status.current_frame, 0);
  EXPECT_LT(final_status.current_frame, 90);
  
  core->close();
}

TEST_F(IntegrationTest, MessageTimestampConsistency) {
  auto bag_path = bag_creator_->create_test_bag("integration_timestamps.db3", 25, {"/test"});
  auto core = std::make_unique<RosbagDeckCore>();
  
  std::vector<Timestamp> received_timestamps;
  core->set_message_callback([&received_timestamps](const BagMessage& msg) {
    received_timestamps.push_back(msg.original_timestamp);
  });
  
  ASSERT_TRUE(core->open({bag_path}));
  
  // Step through all frames sequentially
  for (size_t i = 0; i < 25; ++i) {
    core->step_forward();
  }
  
  // Verify timestamps are in ascending order
  ASSERT_GE(received_timestamps.size(), 20);
  for (size_t i = 1; i < received_timestamps.size(); ++i) {
    EXPECT_LE(received_timestamps[i-1], received_timestamps[i])
        << "Timestamp ordering violation at index " << i;
  }
  
  // Test random seeking and verify frame consistency
  received_timestamps.clear();
  std::vector<size_t> seek_frames = {10, 5, 20, 0, 15, 24};
  
  for (size_t frame : seek_frames) {
    core->seek_to_frame(frame);
    core->step_forward();
  }
  
  // Each seek + step should produce exactly one message
  EXPECT_EQ(received_timestamps.size(), seek_frames.size());
  
  core->close();
}