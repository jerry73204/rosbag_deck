#include <gtest/gtest.h>
#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include "../fixtures/test_bag_creator.hpp"
#include "../fixtures/mock_callbacks.hpp"
#include <chrono>
#include <thread>

using namespace rosbag_deck_core;
using namespace rosbag_deck_core::test;

class RosbagDeckCoreTest : public ::testing::Test {
protected:
  void SetUp() override {
    bag_creator_ = std::make_unique<TestBagCreator>();
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
      core_->stop_playback();
    }
    bag_creator_->cleanup();
  }

  std::unique_ptr<TestBagCreator> bag_creator_;
  std::shared_ptr<MockCallbackHandler> mock_handler_;
  std::unique_ptr<RosbagDeckCore> core_;
  std::string single_topic_bag_;
  std::string multi_topic_bag_;
};

TEST_F(RosbagDeckCoreTest, BasicConstruction) {
  // Core should be constructible
  EXPECT_NO_THROW(RosbagDeckCore core);
}

TEST_F(RosbagDeckCoreTest, BuildIndexSingleBag) {
  EXPECT_TRUE(core_->build_index({single_topic_bag_}));
  
  auto info = core_->get_bag_info();
  EXPECT_TRUE(info.success);
  EXPECT_EQ(info.total_frames, 15);
  EXPECT_TRUE(std::find(info.topic_names.begin(), info.topic_names.end(), "/laser") != info.topic_names.end());
}

TEST_F(RosbagDeckCoreTest, BuildIndexMultipleBags) {
  auto bag2 = bag_creator_->create_test_bag("core_second.db3", 8, {"/camera"});
  
  EXPECT_TRUE(core_->build_index({single_topic_bag_, bag2}));
  
  auto info = core_->get_bag_info();
  EXPECT_TRUE(info.success);
  EXPECT_EQ(info.total_frames, 23); // 15 + 8
  EXPECT_GE(info.topic_names.size(), 2);
}

TEST_F(RosbagDeckCoreTest, BuildIndexInvalidBag) {
  EXPECT_FALSE(core_->build_index({"/nonexistent/bag.db3"}));
  
  auto info = core_->get_bag_info();
  EXPECT_FALSE(info.success);
}

TEST_F(RosbagDeckCoreTest, GetBagInfo) {
  core_->build_index({multi_topic_bag_});
  
  auto info = core_->get_bag_info();
  EXPECT_TRUE(info.success);
  EXPECT_GT(info.total_frames, 0);
  EXPECT_GT(info.total_duration.count(), 0);
  EXPECT_GE(info.topic_names.size(), 3);
  EXPECT_LE(info.start_time, info.end_time);
}

TEST_F(RosbagDeckCoreTest, PlaybackBasicOperations) {
  core_->build_index({single_topic_bag_});
  
  // Initial state
  auto initial_status = core_->get_status();
  EXPECT_FALSE(initial_status.is_playing);
  EXPECT_EQ(initial_status.current_frame, 0);
  
  // Start playback
  core_->start_playback();
  auto playing_status = core_->get_status();
  EXPECT_TRUE(playing_status.is_playing);
  
  // Stop playback
  core_->stop_playback();
  auto stopped_status = core_->get_status();
  EXPECT_FALSE(stopped_status.is_playing);
}

TEST_F(RosbagDeckCoreTest, SeekOperations) {
  core_->build_index({single_topic_bag_});
  
  // Seek to middle frame
  EXPECT_TRUE(core_->seek_to_frame(7));
  auto status = core_->get_status();
  EXPECT_EQ(status.current_frame, 7);
  
  // Seek to beginning
  EXPECT_TRUE(core_->seek_to_frame(0));
  status = core_->get_status();
  EXPECT_EQ(status.current_frame, 0);
  
  // Seek to end
  EXPECT_TRUE(core_->seek_to_frame(14));
  status = core_->get_status();
  EXPECT_EQ(status.current_frame, 14);
  
  // Seek beyond end should fail
  EXPECT_FALSE(core_->seek_to_frame(100));
}

TEST_F(RosbagDeckCoreTest, SeekToTime) {
  core_->build_index({single_topic_bag_});
  
  auto info = core_->get_bag_info();
  if (info.total_frames > 0) {
    // Seek to start time
    EXPECT_TRUE(core_->seek_to_time(info.start_time));
    auto status = core_->get_status();
    EXPECT_EQ(status.current_frame, 0);
    
    // Seek to middle time
    auto middle_time = info.start_time + info.total_duration / 2;
    EXPECT_TRUE(core_->seek_to_time(middle_time));
    status = core_->get_status();
    EXPECT_GT(status.current_frame, 0);
    EXPECT_LT(status.current_frame, info.total_frames);
  }
}

TEST_F(RosbagDeckCoreTest, StepOperations) {
  core_->build_index({single_topic_bag_});
  
  // Step forward from beginning
  EXPECT_TRUE(core_->step_forward());
  auto status = core_->get_status();
  EXPECT_EQ(status.current_frame, 1);
  
  // Step forward again
  EXPECT_TRUE(core_->step_forward());
  status = core_->get_status();
  EXPECT_EQ(status.current_frame, 2);
  
  // Step backward
  EXPECT_TRUE(core_->step_backward());
  status = core_->get_status();
  EXPECT_EQ(status.current_frame, 1);
}

TEST_F(RosbagDeckCoreTest, StepAtBoundaries) {
  core_->build_index({single_topic_bag_});
  
  // Step backward at beginning
  EXPECT_FALSE(core_->step_backward());
  auto status = core_->get_status();
  EXPECT_EQ(status.current_frame, 0);
  
  // Seek to end and try step forward
  core_->seek_to_frame(14);
  EXPECT_FALSE(core_->step_forward());
  status = core_->get_status();
  EXPECT_EQ(status.current_frame, 14);
}

TEST_F(RosbagDeckCoreTest, ConfigurationSettings) {
  // Test cache size setting
  EXPECT_NO_THROW(core_->set_cache_size(100));
  EXPECT_NO_THROW(core_->set_cache_size(1000));
  
  // Test preload settings
  EXPECT_NO_THROW(core_->set_preload_settings(10, 5));
  
  // Test playback rate
  EXPECT_NO_THROW(core_->set_playback_rate(0.5));
  EXPECT_NO_THROW(core_->set_playback_rate(2.0));
  EXPECT_NO_THROW(core_->set_playback_rate(1.0));
  
  // Test loop playback
  EXPECT_NO_THROW(core_->set_loop_playback(true));
  EXPECT_NO_THROW(core_->set_loop_playback(false));
}

TEST_F(RosbagDeckCoreTest, TopicFiltering) {
  core_->build_index({multi_topic_bag_});
  
  // Get all topics
  auto all_topics = core_->get_available_topics();
  EXPECT_GE(all_topics.size(), 3);
  
  // Set topic filter
  std::vector<std::string> filtered_topics = {"/pointcloud", "/cmd_vel"};
  core_->set_topic_filter(filtered_topics);
  
  // Clear filter
  core_->clear_topic_filter();
  
  // Get available topics again
  auto topics_after_clear = core_->get_available_topics();
  EXPECT_EQ(topics_after_clear.size(), all_topics.size());
}

TEST_F(RosbagDeckCoreTest, TypeFiltering) {
  core_->build_index({multi_topic_bag_});
  
  // Get all types
  auto all_types = core_->get_available_types();
  EXPECT_GE(all_types.size(), 3);
  
  // Set type filter
  std::vector<std::string> filtered_types = {"sensor_msgs/msg/PointCloud2"};
  core_->set_type_filter(filtered_types);
  
  // Clear filter
  core_->clear_type_filter();
  
  // Get available types again
  auto types_after_clear = core_->get_available_types();
  EXPECT_EQ(types_after_clear.size(), all_types.size());
}

TEST_F(RosbagDeckCoreTest, CallbackRegistration) {
  bool message_received = false;
  bool status_received = false;
  
  // Set callbacks
  core_->set_message_callback([&message_received](const BagMessage& /*msg*/) {
    message_received = true;
  });
  
  core_->set_status_callback([&status_received](const PlaybackStatus& /*status*/) {
    status_received = true;
  });
  
  core_->build_index({single_topic_bag_});
  
  // Step forward should trigger callbacks
  core_->step_forward();
  
  // Give callbacks time to execute
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  
  EXPECT_TRUE(status_received); // Status should always be updated
  // Message callback may or may not be called depending on implementation
}

TEST_F(RosbagDeckCoreTest, TimestampUtilities) {
  // Test timestamp conversion functions
  int64_t test_nanos = 1234567890123456789LL;
  
  auto timestamp = RosbagDeckCore::to_timestamp(test_nanos);
  auto converted_back = RosbagDeckCore::from_timestamp(timestamp);
  
  EXPECT_EQ(test_nanos, converted_back);
}

TEST_F(RosbagDeckCoreTest, MessageTypeSupport) {
  // Test message type support checking
  EXPECT_TRUE(RosbagDeckCore::is_message_type_supported("sensor_msgs/msg/PointCloud2"));
  EXPECT_TRUE(RosbagDeckCore::is_message_type_supported("geometry_msgs/msg/Twist"));
  EXPECT_TRUE(RosbagDeckCore::is_message_type_supported("std_msgs/msg/String"));
  
  // Unknown types should not be supported initially
  EXPECT_FALSE(RosbagDeckCore::is_message_type_supported("unknown_msgs/msg/Unknown"));
}

TEST_F(RosbagDeckCoreTest, InternalComponentAccess) {
  core_->build_index({multi_topic_bag_});
  
  // Should be able to access internal components
  EXPECT_NO_THROW((void)core_->get_type_registry());
  EXPECT_NO_THROW((void)core_->get_index_manager());
  
  // Components should be in valid state
  auto& index_mgr = core_->get_index_manager();
  EXPECT_GT(index_mgr.total_frames(), 0);
}

TEST_F(RosbagDeckCoreTest, MultiplePlaybackCycles) {
  core_->build_index({single_topic_bag_});
  
  // Multiple start/stop cycles
  for (int cycle = 0; cycle < 3; ++cycle) {
    core_->start_playback();
    EXPECT_TRUE(core_->get_status().is_playing);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    core_->stop_playback();
    EXPECT_FALSE(core_->get_status().is_playing);
  }
}

TEST_F(RosbagDeckCoreTest, ConcurrentOperations) {
  core_->build_index({single_topic_bag_});
  
  std::vector<std::thread> threads;
  std::atomic<bool> stop_flag{false};
  
  // Thread 1: Status queries
  threads.emplace_back([this, &stop_flag]() {
    while (!stop_flag) {
      core_->get_status();
      core_->get_bag_info();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });
  
  // Thread 2: Seek operations
  threads.emplace_back([this, &stop_flag]() {
    int frame = 0;
    while (!stop_flag) {
      core_->seek_to_frame(frame % 15);
      frame++;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
  
  // Thread 3: Step operations
  threads.emplace_back([this, &stop_flag]() {
    while (!stop_flag) {
      core_->step_forward();
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
  });
  
  // Let threads run for a short time
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stop_flag = true;
  
  // Wait for all threads to complete
  for (auto& thread : threads) {
    thread.join();
  }
  
  // Core should still be in a valid state
  EXPECT_NO_THROW(core_->get_status());
}

TEST_F(RosbagDeckCoreTest, EmptyBagHandling) {
  auto empty_bag = bag_creator_->create_test_bag("core_empty.db3", 0, {});
  
  EXPECT_TRUE(core_->build_index({empty_bag}));
  
  auto info = core_->get_bag_info();
  EXPECT_TRUE(info.success);
  EXPECT_EQ(info.total_frames, 0);
  
  // Operations on empty bag should handle gracefully
  EXPECT_FALSE(core_->step_forward());
  EXPECT_FALSE(core_->step_backward());
  EXPECT_FALSE(core_->seek_to_frame(0));
}

TEST_F(RosbagDeckCoreTest, LargeBagHandling) {
  auto large_bag = bag_creator_->create_large_bag("core_large.db3", 100);
  
  EXPECT_TRUE(core_->build_index({large_bag}));
  
  auto info = core_->get_bag_info();
  EXPECT_TRUE(info.success);
  EXPECT_EQ(info.total_frames, 100);
  
  // Should be able to seek to various positions
  EXPECT_TRUE(core_->seek_to_frame(50));
  EXPECT_TRUE(core_->seek_to_frame(99));
  EXPECT_TRUE(core_->seek_to_frame(0));
}