#include <gtest/gtest.h>
#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include "../fixtures/test_data_generator.hpp"
#include <chrono>
#include <thread>
#include <vector>

using namespace rosbag_deck_core;
using namespace rosbag_deck_core::test;

class MessageCacheTest : public ::testing::Test {
protected:
  void SetUp() override {
    data_generator_ = std::make_unique<TestDataGenerator>();
    cache_ = std::make_unique<MessageCache>(10); // Small cache for testing
  }

  BagMessage create_test_message(size_t frame_index, const std::string& topic = "/test") {
    return data_generator_->create_test_bag_message(
        topic, 
        "sensor_msgs/msg/PointCloud2", 
        frame_index, 
        data_generator_->generate_timestamp(frame_index * 1000000) // 1ms increments
    );
  }

  std::unique_ptr<TestDataGenerator> data_generator_;
  std::unique_ptr<MessageCache> cache_;
};

TEST_F(MessageCacheTest, BasicConstructor) {
  // Test different cache sizes
  EXPECT_NO_THROW(MessageCache cache(1));
  EXPECT_NO_THROW(MessageCache cache(100));
  EXPECT_NO_THROW(MessageCache cache(1000));
}

TEST_F(MessageCacheTest, PutAndGetMessage) {
  auto message = create_test_message(42);
  
  // Store message
  cache_->put_message(42, message);
  
  // Retrieve message
  BagMessage retrieved;
  EXPECT_TRUE(cache_->get_message(42, retrieved));
  
  EXPECT_EQ(retrieved.frame_index, 42);
  EXPECT_EQ(retrieved.topic_name, "/test");
  EXPECT_EQ(retrieved.message_type, "sensor_msgs/msg/PointCloud2");
}

TEST_F(MessageCacheTest, GetNonExistentMessage) {
  BagMessage retrieved;
  EXPECT_FALSE(cache_->get_message(999, retrieved));
}

TEST_F(MessageCacheTest, IsFrameCached) {
  auto message = create_test_message(5);
  
  // Initially not cached
  EXPECT_FALSE(cache_->is_frame_cached(5));
  
  // After putting, should be cached
  cache_->put_message(5, message);
  EXPECT_TRUE(cache_->is_frame_cached(5));
}

TEST_F(MessageCacheTest, CacheSize) {
  EXPECT_EQ(cache_->size(), 0);
  
  // Add some messages
  for (size_t i = 0; i < 5; ++i) {
    auto message = create_test_message(i);
    cache_->put_message(i, message);
  }
  
  EXPECT_EQ(cache_->size(), 5);
}

TEST_F(MessageCacheTest, CacheSizeLimit) {
  const size_t cache_limit = 10;
  
  // Fill cache beyond limit
  for (size_t i = 0; i < cache_limit + 5; ++i) {
    auto message = create_test_message(i);
    cache_->put_message(i, message);
  }
  
  // Cache should not exceed limit
  EXPECT_LE(cache_->size(), cache_limit);
}

TEST_F(MessageCacheTest, OverwriteExistingMessage) {
  auto message1 = create_test_message(5, "/topic1");
  auto message2 = create_test_message(5, "/topic2");
  
  // Store first message
  cache_->put_message(5, message1);
  
  BagMessage retrieved1;
  EXPECT_TRUE(cache_->get_message(5, retrieved1));
  EXPECT_EQ(retrieved1.topic_name, "/topic1");
  
  // Overwrite with second message
  cache_->put_message(5, message2);
  
  BagMessage retrieved2;
  EXPECT_TRUE(cache_->get_message(5, retrieved2));
  EXPECT_EQ(retrieved2.topic_name, "/topic2");
}

TEST_F(MessageCacheTest, ClearCache) {
  // Store some messages
  for (size_t i = 0; i < 5; ++i) {
    auto message = create_test_message(i);
    cache_->put_message(i, message);
  }
  
  EXPECT_EQ(cache_->size(), 5);
  
  // Clear cache
  cache_->clear();
  
  EXPECT_EQ(cache_->size(), 0);
  
  // Verify messages are gone
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_FALSE(cache_->is_frame_cached(i));
  }
}

TEST_F(MessageCacheTest, EvictOutsideWindow) {
  // Fill cache with messages
  for (size_t i = 0; i < 10; ++i) {
    auto message = create_test_message(i);
    cache_->put_message(i, message);
  }
  
  EXPECT_EQ(cache_->size(), 10);
  
  // Evict messages outside window [3, 7]
  cache_->evict_outside_window(5, 4); // center=5, window_size=4 -> [3, 7]
  
  // Messages 0, 1, 2, 8, 9 should be evicted
  EXPECT_FALSE(cache_->is_frame_cached(0));
  EXPECT_FALSE(cache_->is_frame_cached(1));
  EXPECT_FALSE(cache_->is_frame_cached(2));
  EXPECT_FALSE(cache_->is_frame_cached(8));
  EXPECT_FALSE(cache_->is_frame_cached(9));
  
  // Messages 3, 4, 5, 6, 7 should remain
  EXPECT_TRUE(cache_->is_frame_cached(3));
  EXPECT_TRUE(cache_->is_frame_cached(4));
  EXPECT_TRUE(cache_->is_frame_cached(5));
  EXPECT_TRUE(cache_->is_frame_cached(6));
  EXPECT_TRUE(cache_->is_frame_cached(7));
}

TEST_F(MessageCacheTest, ThreadSafety) {
  const size_t num_threads = 4;
  const size_t messages_per_thread = 25;
  std::vector<std::thread> threads;
  
  // Launch multiple threads storing messages
  for (size_t t = 0; t < num_threads; ++t) {
    threads.emplace_back([this, t, messages_per_thread]() {
      for (size_t i = 0; i < messages_per_thread; ++i) {
        size_t frame_index = t * messages_per_thread + i;
        auto message = create_test_message(frame_index);
        cache_->put_message(frame_index, message);
      }
    });
  }
  
  // Wait for all threads to complete
  for (auto& thread : threads) {
    thread.join();
  }
  
  // Cache should be in consistent state
  EXPECT_LE(cache_->size(), 10); // Cache limit
  
  // Test concurrent read/write
  std::atomic<bool> stop_flag{false};
  std::vector<std::thread> rw_threads;
  
  // Writer thread
  rw_threads.emplace_back([this, &stop_flag]() {
    size_t counter = 0;
    while (!stop_flag) {
      auto message = create_test_message(counter % 20);
      cache_->put_message(counter % 20, message);
      counter++;
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  });
  
  // Reader threads
  for (int i = 0; i < 2; ++i) {
    rw_threads.emplace_back([this, &stop_flag]() {
      while (!stop_flag) {
        for (size_t frame = 0; frame < 20; ++frame) {
          BagMessage message;
          cache_->get_message(frame, message);
          cache_->is_frame_cached(frame);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
      }
    });
  }
  
  // Let threads run for a short time
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stop_flag = true;
  
  for (auto& thread : rw_threads) {
    thread.join();
  }
  
  // Should not crash and cache should be in valid state
  EXPECT_LE(cache_->size(), 10);
}

TEST_F(MessageCacheTest, LargeMessageHandling) {
  // Create a large message (1MB of data)
  auto message = create_test_message(0);
  message.serialized_data = std::make_shared<std::vector<uint8_t>>(1024 * 1024, 0xFF);
  
  // Store large message
  cache_->put_message(0, message);
  
  // Retrieve and verify
  BagMessage retrieved;
  EXPECT_TRUE(cache_->get_message(0, retrieved));
  EXPECT_EQ(retrieved.serialized_data->size(), 1024 * 1024);
  EXPECT_EQ((*retrieved.serialized_data)[0], 0xFF);
}

TEST_F(MessageCacheTest, WindowEvictionEdgeCases) {
  // Test window eviction with edge cases
  
  // Fill cache
  for (size_t i = 0; i < 10; ++i) {
    auto message = create_test_message(i);
    cache_->put_message(i, message);
  }
  
  // Test window size 0 (should evict everything)
  cache_->evict_outside_window(5, 0);
  EXPECT_EQ(cache_->size(), 0);
  
  // Refill cache
  for (size_t i = 0; i < 10; ++i) {
    auto message = create_test_message(i);
    cache_->put_message(i, message);
  }
  
  // Test very large window (should evict nothing)
  cache_->evict_outside_window(5, 1000);
  EXPECT_EQ(cache_->size(), 10);
  
  // Test window at boundary
  cache_->evict_outside_window(0, 2); // window [0, 1]
  EXPECT_TRUE(cache_->is_frame_cached(0));
  EXPECT_TRUE(cache_->is_frame_cached(1));
  for (size_t i = 2; i < 10; ++i) {
    EXPECT_FALSE(cache_->is_frame_cached(i));
  }
}

TEST_F(MessageCacheTest, MessageDataIntegrity) {
  // Test that message data is preserved correctly
  auto original_message = create_test_message(10, "/integrity_test");
  original_message.metadata["test_key"] = "test_value";
  original_message.serialization_format = "cdr";
  
  cache_->put_message(10, original_message);
  
  BagMessage retrieved;
  EXPECT_TRUE(cache_->get_message(10, retrieved));
  
  // Verify all fields are preserved
  EXPECT_EQ(retrieved.frame_index, original_message.frame_index);
  EXPECT_EQ(retrieved.topic_name, original_message.topic_name);
  EXPECT_EQ(retrieved.message_type, original_message.message_type);
  EXPECT_EQ(retrieved.serialization_format, original_message.serialization_format);
  EXPECT_EQ(retrieved.original_timestamp, original_message.original_timestamp);
  EXPECT_EQ(retrieved.virtual_timestamp, original_message.virtual_timestamp);
  EXPECT_EQ(retrieved.metadata.at("test_key"), "test_value");
  
  // Verify serialized data is preserved
  EXPECT_EQ(retrieved.serialized_data->size(), original_message.serialized_data->size());
  EXPECT_EQ(*retrieved.serialized_data, *original_message.serialized_data);
}