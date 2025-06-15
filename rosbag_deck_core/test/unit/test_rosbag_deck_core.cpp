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

class RosbagDeckCoreTest : public ::testing::Test {
protected:
  void SetUp() override {
    bag_creator_ = std::make_unique<TestBagCreator>();
    data_generator_ = std::make_unique<TestDataGenerator>();
    mock_handler_ = std::make_shared<MockCallbackHandler>();
    
    // Create test bags
    single_topic_bag_ = bag_creator_->create_test_bag("core_single.db3", 15, {"/laser"});
    multi_topic_bag_ = bag_creator_->create_multi_topic_bag("core_multi.db3", 10);
    
    // Create core instance
    core_ = std::make_unique<RosbagDeckCore>();
    
    // Set up callbacks
    core_->set_message_callback([this](const BagMessage& msg) {
      mock_handler_->record_message(msg);
    });
    core_->set_status_callback([this](const PlaybackStatus& status) {
      mock_handler_->record_status(status);
    });
  }

  void TearDown() override {
    if (core_) {
      core_->close();
    }
    bag_creator_->cleanup();
  }

  std::unique_ptr<TestBagCreator> bag_creator_;
  std::unique_ptr<TestDataGenerator> data_generator_;
  std::shared_ptr<MockCallbackHandler> mock_handler_;
  std::unique_ptr<RosbagDeckCore> core_;
  std::string single_topic_bag_;
  std::string multi_topic_bag_;
};

TEST_F(RosbagDeckCoreTest, OpenSingleBag) {
  EXPECT_TRUE(core_->open({single_topic_bag_}));
  EXPECT_TRUE(core_->is_open());
  
  // Verify bag info
  auto info = core_->get_bag_info();
  EXPECT_EQ(info.total_frames, 15);
  EXPECT_EQ(info.topics.size(), 1);
  EXPECT_THAT(info.topics, Contains("/laser"));
}

TEST_F(RosbagDeckCoreTest, OpenMultipleBags) {
  auto bag2 = bag_creator_->create_test_bag("core_second.db3", 8, {"/camera"});
  
  EXPECT_TRUE(core_->open({single_topic_bag_, bag2}));
  EXPECT_TRUE(core_->is_open());
  
  // Total frames should be sum of both bags
  auto info = core_->get_bag_info();
  EXPECT_EQ(info.total_frames, 23);
  EXPECT_GE(info.topics.size(), 2);
}

TEST_F(RosbagDeckCoreTest, OpenInvalidBag) {
  EXPECT_FALSE(core_->open({"/nonexistent/bag.db3"}));
  EXPECT_FALSE(core_->is_open());
}

TEST_F(RosbagDeckCoreTest, CloseAndReopen) {
  // Open bag
  EXPECT_TRUE(core_->open({single_topic_bag_}));
  EXPECT_TRUE(core_->is_open());
  
  // Close bag
  core_->close();
  EXPECT_FALSE(core_->is_open());
  
  // Reopen same bag
  EXPECT_TRUE(core_->open({single_topic_bag_}));
  EXPECT_TRUE(core_->is_open());
}

TEST_F(RosbagDeckCoreTest, PlaybackBasicOperations) {
  core_->open({single_topic_bag_});
  
  // Initial state should be stopped
  auto initial_status = core_->get_playback_status();
  EXPECT_EQ(initial_status.state, PlaybackState::STOPPED);
  EXPECT_EQ(initial_status.current_frame, 0);
  
  // Start playback
  core_->play();
  auto playing_status = core_->get_playback_status();
  EXPECT_EQ(playing_status.state, PlaybackState::PLAYING);
  
  // Pause playback
  core_->pause();
  auto paused_status = core_->get_playback_status();
  EXPECT_EQ(paused_status.state, PlaybackState::PAUSED);
  
  // Stop playback
  core_->stop();
  auto stopped_status = core_->get_playback_status();
  EXPECT_EQ(stopped_status.state, PlaybackState::STOPPED);
}

TEST_F(RosbagDeckCoreTest, SeekOperations) {
  core_->open({single_topic_bag_});
  
  // Seek to middle
  EXPECT_TRUE(core_->seek_to_frame(7));
  auto status = core_->get_playback_status();
  EXPECT_EQ(status.current_frame, 7);
  
  // Seek to beginning
  EXPECT_TRUE(core_->seek_to_frame(0));
  status = core_->get_playback_status();
  EXPECT_EQ(status.current_frame, 0);
  
  // Seek to end
  EXPECT_TRUE(core_->seek_to_frame(14));
  status = core_->get_playback_status();
  EXPECT_EQ(status.current_frame, 14);
  
  // Seek beyond end should fail
  EXPECT_FALSE(core_->seek_to_frame(100));
}

TEST_F(RosbagDeckCoreTest, StepOperations) {
  core_->open({single_topic_bag_});
  
  // Step forward from beginning
  EXPECT_TRUE(core_->step_forward());
  auto status = core_->get_playback_status();
  EXPECT_EQ(status.current_frame, 1);
  
  // Step forward again
  EXPECT_TRUE(core_->step_forward());
  status = core_->get_playback_status();
  EXPECT_EQ(status.current_frame, 2);
  
  // Step backward
  EXPECT_TRUE(core_->step_backward());
  status = core_->get_playback_status();
  EXPECT_EQ(status.current_frame, 1);
}

TEST_F(RosbagDeckCoreTest, MessagePublishing) {
  core_->open({single_topic_bag_});
  
  EXPECT_CALL(*mock_handler_, on_message(_))
      .Times(AtLeast(1));
  
  // Step through frames to trigger message publishing
  for (int i = 0; i < 5; ++i) {
    core_->step_forward();
  }
  
  // Verify messages were published
  auto messages = mock_handler_->get_message_history();
  EXPECT_GT(messages.size(), 0);
  
  if (!messages.empty()) {
    EXPECT_EQ(messages[0].topic_name, "/laser");
    EXPECT_EQ(messages[0].message_type, "sensor_msgs/msg/PointCloud2");
  }
}

TEST_F(RosbagDeckCoreTest, StatusUpdates) {
  core_->open({single_topic_bag_});
  
  EXPECT_CALL(*mock_handler_, on_status(_))
      .Times(AtLeast(3));
  
  // Perform operations that should trigger status updates
  core_->play();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  core_->pause();
  core_->stop();
  
  // Verify status updates were received
  auto statuses = mock_handler_->get_status_history();
  EXPECT_GT(statuses.size(), 0);
}

TEST_F(RosbagDeckCoreTest, PlaybackSpeed) {
  core_->open({single_topic_bag_});
  
  // Test setting different speeds
  EXPECT_TRUE(core_->set_playback_speed(0.5));
  auto status = core_->get_playback_status();
  EXPECT_DOUBLE_EQ(status.playback_speed, 0.5);
  
  EXPECT_TRUE(core_->set_playback_speed(2.0));
  status = core_->get_playback_status();
  EXPECT_DOUBLE_EQ(status.playback_speed, 2.0);
  
  // Test invalid speeds
  EXPECT_FALSE(core_->set_playback_speed(0.0));
  EXPECT_FALSE(core_->set_playback_speed(-1.0));
}

TEST_F(RosbagDeckCoreTest, TopicFiltering) {
  core_->open({multi_topic_bag_});
  
  // Get all topics
  auto all_topics = core_->get_available_topics();
  EXPECT_GE(all_topics.size(), 3);
  
  // Enable only specific topics
  std::vector<std::string> filtered_topics = {"/pointcloud", "/cmd_vel"};
  core_->set_topic_filter(filtered_topics);
  
  // Step through frames and collect messages
  EXPECT_CALL(*mock_handler_, on_message(_))
      .Times(AtLeast(1));
  
  for (int i = 0; i < 10; ++i) {
    core_->step_forward();
  }
  
  // Verify only filtered topics are published
  auto messages = mock_handler_->get_message_history();
  for (const auto& msg : messages) {
    EXPECT_THAT(filtered_topics, Contains(msg.topic_name));
  }
}

TEST_F(RosbagDeckCoreTest, TypeFiltering) {
  core_->open({multi_topic_bag_});
  
  // Enable only specific message types
  std::vector<std::string> filtered_types = {"sensor_msgs/msg/PointCloud2"};
  core_->set_type_filter(filtered_types);
  
  // Step through frames and collect messages
  EXPECT_CALL(*mock_handler_, on_message(_))
      .Times(AtLeast(1));
  
  for (int i = 0; i < 10; ++i) {
    core_->step_forward();
  }
  
  // Verify only filtered types are published
  auto messages = mock_handler_->get_message_history();
  for (const auto& msg : messages) {
    EXPECT_THAT(filtered_types, Contains(msg.message_type));
  }
}

TEST_F(RosbagDeckCoreTest, GetFrameInfo) {
  core_->open({single_topic_bag_});
  
  // Get info for valid frame
  auto frame_info = core_->get_frame_info(5);
  ASSERT_TRUE(frame_info.has_value());
  EXPECT_EQ(frame_info->frame_index, 5);
  EXPECT_EQ(frame_info->topic_name, "/laser");
  
  // Get info for invalid frame
  auto invalid_info = core_->get_frame_info(100);
  EXPECT_FALSE(invalid_info.has_value());
}

TEST_F(RosbagDeckCoreTest, GetTopicInfo) {
  core_->open({multi_topic_bag_});
  
  // Get info for valid topic
  auto topic_info = core_->get_topic_info("/pointcloud");
  ASSERT_TRUE(topic_info.has_value());
  EXPECT_EQ(topic_info->topic_name, "/pointcloud");
  EXPECT_EQ(topic_info->message_type, "sensor_msgs/msg/PointCloud2");
  EXPECT_GT(topic_info->message_count, 0);
  
  // Get info for invalid topic
  auto invalid_info = core_->get_topic_info("/nonexistent");
  EXPECT_FALSE(invalid_info.has_value());
}

TEST_F(RosbagDeckCoreTest, RewindOperation) {
  core_->open({single_topic_bag_});
  
  // Seek to middle
  core_->seek_to_frame(10);
  EXPECT_EQ(core_->get_playback_status().current_frame, 10);
  
  // Rewind
  core_->rewind();
  EXPECT_EQ(core_->get_playback_status().current_frame, 0);
}

TEST_F(RosbagDeckCoreTest, ConcurrentOperations) {
  core_->open({single_topic_bag_});
  
  // Launch multiple threads performing different operations
  std::vector<std::thread> threads;
  std::atomic<bool> stop_flag{false};
  
  // Thread 1: Status queries
  threads.emplace_back([this, &stop_flag]() {
    while (!stop_flag) {
      core_->get_playback_status();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
  
  // Thread 2: Seek operations
  threads.emplace_back([this, &stop_flag]() {
    int frame = 0;
    while (!stop_flag) {
      core_->seek_to_frame(frame % 15);
      frame++;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
  
  // Thread 3: Play/pause operations
  threads.emplace_back([this, &stop_flag]() {
    while (!stop_flag) {
      core_->play();
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      core_->pause();
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  });
  
  // Let threads run for a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  stop_flag = true;
  
  // Wait for all threads to complete
  for (auto& thread : threads) {
    thread.join();
  }
  
  // Core should still be in a valid state
  EXPECT_TRUE(core_->is_open());
}

TEST_F(RosbagDeckCoreTest, LargeMessageHandling) {
  // Create a bag with large messages
  auto large_bag = bag_creator_->create_large_bag("large_core.db3", 100);
  
  EXPECT_TRUE(core_->open({large_bag}));
  
  // Should be able to handle large message counts
  auto info = core_->get_bag_info();
  EXPECT_EQ(info.total_frames, 100);
  
  // Seek operations should work with large bags
  EXPECT_TRUE(core_->seek_to_frame(50));
  EXPECT_TRUE(core_->seek_to_frame(99));
}