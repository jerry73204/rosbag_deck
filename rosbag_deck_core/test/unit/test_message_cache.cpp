#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include "../fixtures/test_data_generator.hpp"
#include <chrono>
#include <thread>

using namespace rosbag_deck_core;
using namespace rosbag_deck_core::test;
using namespace testing;

class MessageCacheTest : public ::testing::Test {
protected:
  void SetUp() override {
    data_generator_ = std::make_unique<TestDataGenerator>();
    cache_ = std::make_unique<MessageCache>();
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

TEST_F(MessageCacheTest, StoreAndRetrieveSingleMessage) {
  auto message = create_test_message(42);
  
  // Store message
  cache_->store_message(42, message);
  
  // Retrieve message
  auto retrieved = cache_->get_message(42);
  ASSERT_TRUE(retrieved.has_value());
  
  EXPECT_EQ(retrieved->frame_index, 42);
  EXPECT_EQ(retrieved->topic_name, "/test");
  EXPECT_EQ(retrieved->message_type, "sensor_msgs/msg/PointCloud2");
}

TEST_F(MessageCacheTest, RetrieveNonExistentMessage) {
  // Try to retrieve message that doesn't exist
  auto retrieved = cache_->get_message(999);
  EXPECT_FALSE(retrieved.has_value());
}

TEST_F(MessageCacheTest, StoreMultipleMessages) {
  // Store multiple messages
  for (size_t i = 0; i < 10; ++i) {
    auto message = create_test_message(i);
    cache_->store_message(i, message);
  }
  
  // Retrieve all messages
  for (size_t i = 0; i < 10; ++i) {
    auto retrieved = cache_->get_message(i);
    ASSERT_TRUE(retrieved.has_value());
    EXPECT_EQ(retrieved->frame_index, i);
  }
}

TEST_F(MessageCacheTest, OverwriteExistingMessage) {
  auto message1 = create_test_message(5, "/topic1");
  auto message2 = create_test_message(5, "/topic2");
  
  // Store first message
  cache_->store_message(5, message1);
  
  // Verify first message
  auto retrieved1 = cache_->get_message(5);
  ASSERT_TRUE(retrieved1.has_value());
  EXPECT_EQ(retrieved1->topic_name, "/topic1");
  
  // Overwrite with second message
  cache_->store_message(5, message2);
  
  // Verify second message overwrote the first
  auto retrieved2 = cache_->get_message(5);
  ASSERT_TRUE(retrieved2.has_value());
  EXPECT_EQ(retrieved2->topic_name, "/topic2");
}

TEST_F(MessageCacheTest, CacheSizeLimit) {
  const size_t cache_limit = 1000; // Assuming default cache size
  
  // Fill cache beyond limit
  for (size_t i = 0; i < cache_limit + 100; ++i) {
    auto message = create_test_message(i);
    cache_->store_message(i, message);
  }
  
  // Verify cache doesn't grow indefinitely (exact behavior depends on eviction policy)
  size_t cached_count = 0;
  for (size_t i = 0; i < cache_limit + 100; ++i) {
    if (cache_->get_message(i).has_value()) {
      cached_count++;
    }
  }
  
  // Should be at or below the cache limit (with some tolerance for implementation details)
  EXPECT_LE(cached_count, cache_limit + 10);
}

TEST_F(MessageCacheTest, ClearCache) {
  // Store some messages
  for (size_t i = 0; i < 5; ++i) {
    auto message = create_test_message(i);
    cache_->store_message(i, message);
  }
  
  // Verify messages are stored
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_TRUE(cache_->get_message(i).has_value());
  }
  
  // Clear cache
  cache_->clear();
  
  // Verify messages are gone
  for (size_t i = 0; i < 5; ++i) {
    EXPECT_FALSE(cache_->get_message(i).has_value());
  }
}

TEST_F(MessageCacheTest, ThreadSafety) {
  const size_t num_threads = 4;
  const size_t messages_per_thread = 100;
  std::vector<std::thread> threads;
  
  // Launch multiple threads storing messages
  for (size_t t = 0; t < num_threads; ++t) {
    threads.emplace_back([this, t, messages_per_thread]() {
      for (size_t i = 0; i < messages_per_thread; ++i) {
        size_t frame_index = t * messages_per_thread + i;
        auto message = create_test_message(frame_index);
        cache_->store_message(frame_index, message);
      }
    });
  }
  
  // Wait for all threads to complete
  for (auto& thread : threads) {
    thread.join();
  }
  
  // Verify all messages were stored correctly
  for (size_t t = 0; t < num_threads; ++t) {
    for (size_t i = 0; i < messages_per_thread; ++i) {
      size_t frame_index = t * messages_per_thread + i;
      auto retrieved = cache_->get_message(frame_index);
      ASSERT_TRUE(retrieved.has_value());
      EXPECT_EQ(retrieved->frame_index, frame_index);
    }
  }
}

TEST_F(MessageCacheTest, GetCacheStats) {
  // Store some messages
  for (size_t i = 0; i < 10; ++i) {
    auto message = create_test_message(i);
    cache_->store_message(i, message);
  }
  
  // Get cache statistics (if available)
  auto stats = cache_->get_cache_stats();
  if (stats.has_value()) {
    EXPECT_GE(stats->cache_size, 10);
    EXPECT_GE(stats->total_size_bytes, 0);
  }
}

TEST_F(MessageCacheTest, PrefetchRange) {
  // Create a range of messages to prefetch
  std::vector<BagMessage> messages;
  for (size_t i = 10; i < 20; ++i) {
    messages.push_back(create_test_message(i));
  }
  
  // Prefetch range
  cache_->prefetch_range(10, 19, messages);
  
  // Verify all messages in range are cached
  for (size_t i = 10; i < 20; ++i) {
    auto retrieved = cache_->get_message(i);
    ASSERT_TRUE(retrieved.has_value());
    EXPECT_EQ(retrieved->frame_index, i);
  }
}

TEST_F(MessageCacheTest, LargeMessageHandling) {
  // Create a large message (1MB of data)
  auto message = create_test_message(0);
  message.serialized_data = std::make_shared<std::vector<uint8_t>>(1024 * 1024, 0xFF);
  
  // Store large message
  cache_->store_message(0, message);
  
  // Retrieve and verify
  auto retrieved = cache_->get_message(0);
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->serialized_data->size(), 1024 * 1024);
}

TEST_F(MessageCacheTest, CacheEvictionPolicy) {
  // This test assumes LRU eviction policy
  const size_t cache_capacity = 100; // Assume small cache for testing
  
  // Fill cache to capacity
  for (size_t i = 0; i < cache_capacity; ++i) {
    auto message = create_test_message(i);
    cache_->store_message(i, message);
  }
  
  // Access early messages to make them recently used
  for (size_t i = 0; i < 10; ++i) {
    cache_->get_message(i);
  }
  
  // Add more messages to trigger eviction
  for (size_t i = cache_capacity; i < cache_capacity + 20; ++i) {
    auto message = create_test_message(i);
    cache_->store_message(i, message);
  }
  
  // Recently accessed messages should still be available
  for (size_t i = 0; i < 10; ++i) {
    EXPECT_TRUE(cache_->get_message(i).has_value());
  }
  
  // Some middle messages should have been evicted
  bool some_evicted = false;
  for (size_t i = 10; i < cache_capacity; ++i) {
    if (!cache_->get_message(i).has_value()) {
      some_evicted = true;
      break;
    }
  }
  EXPECT_TRUE(some_evicted);
}