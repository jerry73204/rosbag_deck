#include <gtest/gtest.h>
#include <benchmark/benchmark.h>
#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include "../fixtures/test_bag_creator.hpp"
#include "../fixtures/test_data_generator.hpp"
#include <chrono>
#include <iostream>

using namespace rosbag_deck_core;
using namespace rosbag_deck_core::test;

class PerformanceTest : public ::testing::Test {
protected:
  void SetUp() override {
    bag_creator_ = std::make_unique<TestBagCreator>();
    
    // Create various sized bags for performance testing
    small_bag_ = bag_creator_->create_test_bag("perf_small.db3", 100, {"/small"});
    medium_bag_ = bag_creator_->create_test_bag("perf_medium.db3", 1000, {"/medium"});
    large_bag_ = bag_creator_->create_large_bag("perf_large.db3", 5000);
    multi_topic_bag_ = bag_creator_->create_multi_topic_bag("perf_multi.db3", 1000);
  }

  void TearDown() override {
    bag_creator_->cleanup();
  }

  template<typename Func>
  std::chrono::microseconds measure_time(Func&& func) {
    auto start = std::chrono::high_resolution_clock::now();
    func();
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  }

  std::unique_ptr<TestBagCreator> bag_creator_;
  std::string small_bag_;
  std::string medium_bag_;
  std::string large_bag_;
  std::string multi_topic_bag_;
};

TEST_F(PerformanceTest, BagOpeningPerformance) {
  auto core = std::make_unique<RosbagDeckCore>();
  
  // Test small bag opening time
  auto small_time = measure_time([&]() {
    ASSERT_TRUE(core->open({small_bag_}));
    core->close();
  });
  
  // Test medium bag opening time
  auto medium_time = measure_time([&]() {
    ASSERT_TRUE(core->open({medium_bag_}));
    core->close();
  });
  
  // Test large bag opening time
  auto large_time = measure_time([&]() {
    ASSERT_TRUE(core->open({large_bag_}));
    core->close();
  });
  
  std::cout << "Bag opening performance:\n";
  std::cout << "  Small (100 msgs): " << small_time.count() << " μs\n";
  std::cout << "  Medium (1000 msgs): " << medium_time.count() << " μs\n";
  std::cout << "  Large (5000 msgs): " << large_time.count() << " μs\n";
  
  // Performance expectations (these may need adjustment based on hardware)
  EXPECT_LT(small_time.count(), 50000);   // < 50ms
  EXPECT_LT(medium_time.count(), 200000);  // < 200ms
  EXPECT_LT(large_time.count(), 1000000);  // < 1s
}

TEST_F(PerformanceTest, SeekingPerformance) {
  auto core = std::make_unique<RosbagDeckCore>();
  ASSERT_TRUE(core->open({large_bag_}));
  
  const size_t num_seeks = 100;
  std::vector<size_t> seek_positions;
  
  // Generate random seek positions
  for (size_t i = 0; i < num_seeks; ++i) {
    seek_positions.push_back(rand() % 5000);
  }
  
  auto seek_time = measure_time([&]() {
    for (size_t pos : seek_positions) {
      core->seek_to_frame(pos);
    }
  });
  
  double avg_seek_time = static_cast<double>(seek_time.count()) / num_seeks;
  
  std::cout << "Seeking performance:\n";
  std::cout << "  " << num_seeks << " seeks in " << seek_time.count() << " μs\n";
  std::cout << "  Average seek time: " << avg_seek_time << " μs\n";
  
  // Each seek should be fast
  EXPECT_LT(avg_seek_time, 1000); // < 1ms per seek
  
  core->close();
}

TEST_F(PerformanceTest, SteppingPerformance) {
  auto core = std::make_unique<RosbagDeckCore>();
  ASSERT_TRUE(core->open({medium_bag_}));
  
  const size_t num_steps = 500;
  
  // Test forward stepping
  auto forward_time = measure_time([&]() {
    for (size_t i = 0; i < num_steps; ++i) {
      core->step_forward();
    }
  });
  
  // Reset to middle for backward stepping
  core->seek_to_frame(750);
  
  // Test backward stepping
  auto backward_time = measure_time([&]() {
    for (size_t i = 0; i < num_steps; ++i) {
      core->step_backward();
    }
  });
  
  double avg_forward = static_cast<double>(forward_time.count()) / num_steps;
  double avg_backward = static_cast<double>(backward_time.count()) / num_steps;
  
  std::cout << "Stepping performance:\n";
  std::cout << "  Forward steps: " << avg_forward << " μs/step\n";
  std::cout << "  Backward steps: " << avg_backward << " μs/step\n";
  
  // Stepping should be very fast
  EXPECT_LT(avg_forward, 500);  // < 0.5ms per step
  EXPECT_LT(avg_backward, 500); // < 0.5ms per step
  
  core->close();
}

TEST_F(PerformanceTest, MessageCachePerformance) {
  auto cache = std::make_unique<MessageCache>();
  auto data_generator = std::make_unique<TestDataGenerator>();
  
  // Create test messages
  std::vector<BagMessage> messages;
  for (size_t i = 0; i < 1000; ++i) {
    messages.push_back(data_generator->create_test_bag_message(
      "/test", "sensor_msgs/msg/PointCloud2", i, 
      data_generator->generate_timestamp(i * 1000000)
    ));
  }
  
  // Test cache storing performance
  auto store_time = measure_time([&]() {
    for (size_t i = 0; i < messages.size(); ++i) {
      cache->store_message(i, messages[i]);
    }
  });
  
  // Test cache retrieval performance
  auto retrieve_time = measure_time([&]() {
    for (size_t i = 0; i < messages.size(); ++i) {
      auto msg = cache->get_message(i);
      ASSERT_TRUE(msg.has_value());
    }
  });
  
  double avg_store = static_cast<double>(store_time.count()) / messages.size();
  double avg_retrieve = static_cast<double>(retrieve_time.count()) / messages.size();
  
  std::cout << "Cache performance:\n";
  std::cout << "  Store: " << avg_store << " μs/message\n";
  std::cout << "  Retrieve: " << avg_retrieve << " μs/message\n";
  
  // Cache operations should be very fast
  EXPECT_LT(avg_store, 50);    // < 50μs per store
  EXPECT_LT(avg_retrieve, 10); // < 10μs per retrieve
}

TEST_F(PerformanceTest, PlaybackThroughput) {
  auto core = std::make_unique<RosbagDeckCore>();
  ASSERT_TRUE(core->open({medium_bag_}));
  
  std::atomic<size_t> message_count{0};
  core->set_message_callback([&message_count](const BagMessage& msg) {
    message_count++;
  });
  
  // Test maximum playback speed
  core->set_playback_speed(10.0); // 10x speed
  
  auto start_time = std::chrono::high_resolution_clock::now();
  core->play();
  
  // Let it play for a fixed time
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  core->pause();
  
  auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  
  double throughput = static_cast<double>(message_count.load()) / elapsed_ms * 1000.0; // messages per second
  
  std::cout << "Playback throughput:\n";
  std::cout << "  Messages processed: " << message_count.load() << "\n";
  std::cout << "  Elapsed time: " << elapsed_ms << " ms\n";
  std::cout << "  Throughput: " << throughput << " messages/second\n";
  
  // Should be able to process at least 1000 messages per second
  EXPECT_GT(throughput, 1000.0);
  
  core->close();
}

TEST_F(PerformanceTest, MultiTopicFilteringPerformance) {
  auto core = std::make_unique<RosbagDeckCore>();
  ASSERT_TRUE(core->open({multi_topic_bag_}));
  
  std::atomic<size_t> filtered_messages{0};
  core->set_message_callback([&filtered_messages](const BagMessage& msg) {
    filtered_messages++;
  });
  
  // Test with no filtering
  auto no_filter_time = measure_time([&]() {
    for (size_t i = 0; i < 1000; ++i) {
      core->step_forward();
    }
  });
  
  core->rewind();
  filtered_messages = 0;
  
  // Test with topic filtering
  core->set_topic_filter({"/pointcloud"});
  auto topic_filter_time = measure_time([&]() {
    for (size_t i = 0; i < 1000; ++i) {
      core->step_forward();
    }
  });
  
  core->rewind();
  filtered_messages = 0;
  
  // Test with type filtering
  core->clear_topic_filter();
  core->set_type_filter({"sensor_msgs/msg/PointCloud2"});
  auto type_filter_time = measure_time([&]() {
    for (size_t i = 0; i < 1000; ++i) {
      core->step_forward();
    }
  });
  
  std::cout << "Filtering performance:\n";
  std::cout << "  No filter: " << no_filter_time.count() << " μs\n";
  std::cout << "  Topic filter: " << topic_filter_time.count() << " μs\n";
  std::cout << "  Type filter: " << type_filter_time.count() << " μs\n";
  
  // Filtering shouldn't add significant overhead (< 20% increase)
  EXPECT_LT(topic_filter_time.count(), no_filter_time.count() * 1.2);
  EXPECT_LT(type_filter_time.count(), no_filter_time.count() * 1.2);
  
  core->close();
}

TEST_F(PerformanceTest, MemoryUsageUnderLoad) {
  auto core = std::make_unique<RosbagDeckCore>();
  ASSERT_TRUE(core->open({large_bag_}));
  
  // Measure memory usage during intensive operations
  size_t initial_memory = get_memory_usage(); // This would need to be implemented
  
  // Perform memory-intensive operations
  for (int iteration = 0; iteration < 5; ++iteration) {
    // Seek to random positions
    for (int i = 0; i < 100; ++i) {
      core->seek_to_frame(rand() % 5000);
    }
    
    // Step through many frames
    for (int i = 0; i < 200; ++i) {
      core->step_forward();
    }
    
    // Change filters frequently
    if (iteration % 2 == 0) {
      core->set_topic_filter({"/large_topic"});
    } else {
      core->clear_topic_filter();
    }
  }
  
  size_t final_memory = get_memory_usage();
  
  std::cout << "Memory usage:\n";
  std::cout << "  Initial: " << initial_memory << " bytes\n";
  std::cout << "  Final: " << final_memory << " bytes\n";
  std::cout << "  Increase: " << (final_memory - initial_memory) << " bytes\n";
  
  // Memory increase should be reasonable (< 100MB for this test)
  EXPECT_LT(final_memory - initial_memory, 100 * 1024 * 1024);
  
  core->close();
}

// Helper function for memory measurement (simplified implementation)
size_t get_memory_usage() {
  // This is a simplified implementation
  // In a real test, you would use platform-specific APIs to get actual memory usage
  std::ifstream statm("/proc/self/statm");
  if (statm.is_open()) {
    size_t size, resident, share, text, lib, data, dt;
    statm >> size >> resident >> share >> text >> lib >> data >> dt;
    return resident * sysconf(_SC_PAGESIZE); // RSS in bytes
  }
  return 0;
}

TEST_F(PerformanceTest, ConcurrentAccessPerformance) {
  auto core = std::make_unique<RosbagDeckCore>();
  ASSERT_TRUE(core->open({medium_bag_}));
  
  std::atomic<size_t> operations_completed{0};
  std::vector<std::thread> threads;
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  // Launch multiple threads performing different operations
  for (int t = 0; t < 4; ++t) {
    threads.emplace_back([&core, &operations_completed, t]() {
      for (int i = 0; i < 100; ++i) {
        switch (t % 4) {
          case 0: // Seeking
            core->seek_to_frame(i * 10 % 1000);
            break;
          case 1: // Stepping
            core->step_forward();
            break;
          case 2: // Status queries
            core->get_playback_status();
            break;
          case 3: // Filter changes
            if (i % 10 == 0) {
              core->set_topic_filter({"/medium"});
            } else if (i % 10 == 5) {
              core->clear_topic_filter();
            }
            break;
        }
        operations_completed++;
      }
    });
  }
  
  // Wait for all threads to complete
  for (auto& thread : threads) {
    thread.join();
  }
  
  auto elapsed = std::chrono::high_resolution_clock::now() - start_time;
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  
  double ops_per_second = static_cast<double>(operations_completed.load()) / elapsed_ms * 1000.0;
  
  std::cout << "Concurrent access performance:\n";
  std::cout << "  Operations completed: " << operations_completed.load() << "\n";
  std::cout << "  Elapsed time: " << elapsed_ms << " ms\n";
  std::cout << "  Operations per second: " << ops_per_second << "\n";
  
  // Should maintain good performance under concurrent load
  EXPECT_GT(ops_per_second, 1000.0); // At least 1000 ops/second
  
  core->close();
}