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

class BagWorkerTest : public ::testing::Test {
protected:
  void SetUp() override {
    bag_creator_ = std::make_unique<TestBagCreator>();
    data_generator_ = std::make_unique<TestDataGenerator>();
    mock_handler_ = std::make_shared<MockCallbackHandler>();
    
    // Create a test bag for most tests
    test_bag_path_ = bag_creator_->create_test_bag("worker_test.db3", 20, {"/test_topic"});
    
    // Setup worker with mock callbacks
    worker_ = std::make_unique<BagWorker>();
    worker_->set_message_callback([this](const BagMessage& msg) {
      mock_handler_->record_message(msg);
    });
    worker_->set_status_callback([this](const PlaybackStatus& status) {
      mock_handler_->record_status(status);
    });
  }

  void TearDown() override {
    if (worker_) {
      worker_->stop();
    }
    bag_creator_->cleanup();
  }

  std::unique_ptr<TestBagCreator> bag_creator_;
  std::unique_ptr<TestDataGenerator> data_generator_;
  std::shared_ptr<MockCallbackHandler> mock_handler_;
  std::unique_ptr<BagWorker> worker_;
  std::string test_bag_path_;
};

TEST_F(BagWorkerTest, InitializeWithBag) {
  // Initialize worker with test bag
  EXPECT_TRUE(worker_->initialize({test_bag_path_}));
  
  // Verify initialization
  EXPECT_TRUE(worker_->is_initialized());
  auto total_frames = worker_->get_total_frames();
  EXPECT_EQ(total_frames, 20);
}

TEST_F(BagWorkerTest, InitializeWithInvalidBag) {
  // Try to initialize with non-existent bag
  EXPECT_FALSE(worker_->initialize({"/nonexistent/bag.db3"}));
  EXPECT_FALSE(worker_->is_initialized());
}

TEST_F(BagWorkerTest, InitializeWithMultipleBags) {
  auto bag2 = bag_creator_->create_test_bag("worker_test2.db3", 15, {"/topic2"});
  
  // Initialize with multiple bags
  EXPECT_TRUE(worker_->initialize({test_bag_path_, bag2}));
  EXPECT_TRUE(worker_->is_initialized());
  
  // Total frames should be sum of both bags
  EXPECT_EQ(worker_->get_total_frames(), 35);
}

TEST_F(BagWorkerTest, SeekToFrame) {
  worker_->initialize({test_bag_path_});
  
  // Seek to middle frame
  EXPECT_TRUE(worker_->seek_to_frame(10));
  EXPECT_EQ(worker_->get_current_frame(), 10);
  
  // Seek to first frame
  EXPECT_TRUE(worker_->seek_to_frame(0));
  EXPECT_EQ(worker_->get_current_frame(), 0);
  
  // Seek to last frame
  EXPECT_TRUE(worker_->seek_to_frame(19));
  EXPECT_EQ(worker_->get_current_frame(), 19);
}

TEST_F(BagWorkerTest, SeekToInvalidFrame) {
  worker_->initialize({test_bag_path_});
  
  // Seek beyond end
  EXPECT_FALSE(worker_->seek_to_frame(100));
  
  // Current frame should not change
  EXPECT_EQ(worker_->get_current_frame(), 0);
}

TEST_F(BagWorkerTest, PlayFromStart) {
  worker_->initialize({test_bag_path_});
  
  // Set up expectations for messages
  EXPECT_CALL(*mock_handler_, on_message(_))
      .Times(AtLeast(1));
  EXPECT_CALL(*mock_handler_, on_status(_))
      .Times(AtLeast(1));
  
  // Start playback
  worker_->play();
  
  // Wait a bit for messages to be processed
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Stop playback
  worker_->pause();
  
  // Verify some messages were received
  auto messages = mock_handler_->get_message_history();
  EXPECT_GT(messages.size(), 0);
}

TEST_F(BagWorkerTest, PlayPauseCycle) {
  worker_->initialize({test_bag_path_});
  
  // Start playback
  worker_->play();
  EXPECT_TRUE(worker_->is_playing());
  
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  
  // Pause playback
  worker_->pause();
  EXPECT_FALSE(worker_->is_playing());
  
  // Resume playback
  worker_->play();
  EXPECT_TRUE(worker_->is_playing());
  
  worker_->pause();
}

TEST_F(BagWorkerTest, StepForward) {
  worker_->initialize({test_bag_path_});
  
  size_t initial_frame = worker_->get_current_frame();
  
  // Step forward
  EXPECT_TRUE(worker_->step_forward());
  EXPECT_EQ(worker_->get_current_frame(), initial_frame + 1);
  
  // Step forward again
  EXPECT_TRUE(worker_->step_forward());
  EXPECT_EQ(worker_->get_current_frame(), initial_frame + 2);
}

TEST_F(BagWorkerTest, StepBackward) {
  worker_->initialize({test_bag_path_});
  
  // Seek to middle first
  worker_->seek_to_frame(10);
  
  // Step backward
  EXPECT_TRUE(worker_->step_backward());
  EXPECT_EQ(worker_->get_current_frame(), 9);
  
  // Step backward again
  EXPECT_TRUE(worker_->step_backward());
  EXPECT_EQ(worker_->get_current_frame(), 8);
}

TEST_F(BagWorkerTest, StepAtBoundaries) {
  worker_->initialize({test_bag_path_});
  
  // Step backward at beginning
  worker_->seek_to_frame(0);
  EXPECT_FALSE(worker_->step_backward());
  EXPECT_EQ(worker_->get_current_frame(), 0);
  
  // Step forward at end
  worker_->seek_to_frame(19);
  EXPECT_FALSE(worker_->step_forward());
  EXPECT_EQ(worker_->get_current_frame(), 19);
}

TEST_F(BagWorkerTest, SetPlaybackSpeed) {
  worker_->initialize({test_bag_path_});
  
  // Test various playback speeds
  EXPECT_TRUE(worker_->set_playback_speed(0.5));
  EXPECT_TRUE(worker_->set_playback_speed(2.0));
  EXPECT_TRUE(worker_->set_playback_speed(1.0));
  
  // Test invalid speeds
  EXPECT_FALSE(worker_->set_playback_speed(0.0));
  EXPECT_FALSE(worker_->set_playback_speed(-1.0));
}

TEST_F(BagWorkerTest, RewindOperation) {
  worker_->initialize({test_bag_path_});
  
  // Play some frames
  worker_->seek_to_frame(10);
  
  // Rewind
  worker_->rewind();
  
  // Should be back at beginning
  EXPECT_EQ(worker_->get_current_frame(), 0);
}

TEST_F(BagWorkerTest, StatusCallbacks) {
  worker_->initialize({test_bag_path_});
  
  // Expect status updates
  EXPECT_CALL(*mock_handler_, on_status(_))
      .Times(AtLeast(3)); // Initial, play, pause
  
  worker_->play();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  worker_->pause();
  
  // Verify status history
  auto statuses = mock_handler_->get_status_history();
  EXPECT_GT(statuses.size(), 0);
  
  // Check for state changes
  bool found_playing = false;
  bool found_paused = false;
  for (const auto& status : statuses) {
    if (status.state == PlaybackState::PLAYING) found_playing = true;
    if (status.state == PlaybackState::PAUSED) found_paused = true;
  }
  EXPECT_TRUE(found_playing || found_paused);
}

TEST_F(BagWorkerTest, MessageCallbacks) {
  worker_->initialize({test_bag_path_});
  
  // Expect message callbacks
  EXPECT_CALL(*mock_handler_, on_message(_))
      .Times(AtLeast(1));
  
  // Step through a few frames
  worker_->step_forward();
  worker_->step_forward();
  
  // Verify messages were received
  auto messages = mock_handler_->get_message_history();
  EXPECT_GT(messages.size(), 0);
  
  // Verify message content
  if (!messages.empty()) {
    EXPECT_EQ(messages[0].topic_name, "/test_topic");
    EXPECT_EQ(messages[0].message_type, "sensor_msgs/msg/PointCloud2");
  }
}

TEST_F(BagWorkerTest, ThreadSafety) {
  worker_->initialize({test_bag_path_});
  
  // Launch multiple threads performing different operations
  std::vector<std::thread> threads;
  
  // Thread 1: Play/pause cycles
  threads.emplace_back([this]() {
    for (int i = 0; i < 10; ++i) {
      worker_->play();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      worker_->pause();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
  
  // Thread 2: Seeking operations
  threads.emplace_back([this]() {
    for (int i = 0; i < 20; ++i) {
      worker_->seek_to_frame(i);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });
  
  // Thread 3: Step operations
  threads.emplace_back([this]() {
    for (int i = 0; i < 10; ++i) {
      worker_->step_forward();
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
      worker_->step_backward();
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
  });
  
  // Wait for all threads to complete
  for (auto& thread : threads) {
    thread.join();
  }
  
  // Worker should still be in a valid state
  EXPECT_TRUE(worker_->is_initialized());
}

TEST_F(BagWorkerTest, MultipleTopicPlayback) {
  auto multi_bag = bag_creator_->create_multi_topic_bag("multi_worker.db3", 10);
  worker_->initialize({multi_bag});
  
  EXPECT_CALL(*mock_handler_, on_message(_))
      .Times(AtLeast(10));
  
  // Play through several frames
  for (int i = 0; i < 5; ++i) {
    worker_->step_forward();
  }
  
  // Verify different topic messages were received
  auto messages = mock_handler_->get_message_history();
  
  std::set<std::string> received_topics;
  for (const auto& msg : messages) {
    received_topics.insert(msg.topic_name);
  }
  
  EXPECT_GT(received_topics.size(), 1);
}

TEST_F(BagWorkerTest, ResourceCleanup) {
  {
    auto temp_worker = std::make_unique<BagWorker>();
    temp_worker->initialize({test_bag_path_});
    temp_worker->play();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // Worker destructor should clean up properly
  }
  
  // This test mainly verifies no crashes during cleanup
  SUCCEED();
}