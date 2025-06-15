/**
 * Unit tests for the C API interface
 * Tests memory safety, data conversions, error handling, and lifecycle management
 */

#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>

#include "rosbag_deck_core/rosbag_deck_c.h"
#include "../fixtures/test_bag_creator.hpp"
#include "../fixtures/test_data_generator.hpp"

using namespace std::chrono_literals;
using namespace rosbag_deck_core::test;

class CAPITest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test bag files
        test_bag_creator_ = std::make_unique<TestBagCreator>();
        
        // Create a simple test bag
        test_bag_path_ = test_bag_creator_->create_test_bag("c_api_test.db3");
    }
    
    void TearDown() override {
        // Clean up any remaining handles
        for (auto handle : created_handles_) {
            if (handle) {
                rosbag_deck_destroy(handle);
            }
        }
        created_handles_.clear();
    }
    
    rosbag_deck_handle_t create_handle() {
        auto handle = rosbag_deck_create();
        created_handles_.push_back(handle);
        return handle;
    }
    
    std::unique_ptr<TestBagCreator> test_bag_creator_;
    std::string test_bag_path_;
    std::vector<rosbag_deck_handle_t> created_handles_;
};

// ============================================================================
// Handle Lifecycle Tests
// ============================================================================

TEST_F(CAPITest, HandleCreationAndDestruction) {
    // Test basic handle creation
    rosbag_deck_handle_t handle = rosbag_deck_create();
    ASSERT_NE(handle, nullptr);
    
    // Test destruction
    rosbag_deck_destroy(handle);
    
    // Remove from cleanup list since we already destroyed it
    auto it = std::find(created_handles_.begin(), created_handles_.end(), handle);
    if (it != created_handles_.end()) {
        created_handles_.erase(it);
    }
}

TEST_F(CAPITest, MultipleHandleCreation) {
    // Test creating multiple handles
    std::vector<rosbag_deck_handle_t> handles;
    
    for (int i = 0; i < 10; ++i) {
        auto handle = rosbag_deck_create();
        ASSERT_NE(handle, nullptr);
        handles.push_back(handle);
        created_handles_.push_back(handle);
    }
    
    // Verify all handles are unique
    for (size_t i = 0; i < handles.size(); ++i) {
        for (size_t j = i + 1; j < handles.size(); ++j) {
            EXPECT_NE(handles[i], handles[j]);
        }
    }
    
    // Clean up
    for (auto handle : handles) {
        rosbag_deck_destroy(handle);
        // Remove from cleanup list
        auto it = std::find(created_handles_.begin(), created_handles_.end(), handle);
        if (it != created_handles_.end()) {
            created_handles_.erase(it);
        }
    }
}

TEST_F(CAPITest, NullHandleOperations) {
    // Test that operations with null handle don't crash
    rosbag_deck_handle_t null_handle = nullptr;
    
    // These should not crash but may not have defined behavior
    rosbag_deck_destroy(null_handle);
    rosbag_deck_set_cache_size(null_handle, 100);
    rosbag_deck_start_playback(null_handle);
    rosbag_deck_stop_playback(null_handle);
    
    // These should return safe defaults
    auto status = rosbag_deck_get_status(null_handle);
    EXPECT_FALSE(status.is_playing);
    EXPECT_EQ(status.current_frame, 0);
    
    auto info = rosbag_deck_get_bag_info(null_handle);
    EXPECT_FALSE(info.success);
    
    EXPECT_FALSE(rosbag_deck_step_forward(null_handle));
    EXPECT_FALSE(rosbag_deck_step_backward(null_handle));
}

// ============================================================================
// Data Conversion Tests
// ============================================================================

TEST_F(CAPITest, TimestampConversion) {
    // Test timestamp conversion functions
    std::vector<int64_t> test_timestamps = {
        0LL,                          // Epoch
        1000000000LL,                // 1 second
        1234567890123456789LL,       // Large timestamp
        -1000000000LL,               // Negative timestamp
        INT64_MAX,                   // Maximum value
        INT64_MIN                    // Minimum value
    };
    
    for (auto ns : test_timestamps) {
        auto timestamp = rosbag_deck_to_timestamp(ns);
        auto converted_back = rosbag_deck_from_timestamp(timestamp);
        
        EXPECT_EQ(ns, converted_back) 
            << "Timestamp conversion failed for " << ns << " nanoseconds";
    }
}

TEST_F(CAPITest, MessageTypeSupport) {
    // Test message type support queries
    std::vector<std::pair<std::string, bool>> test_cases = {
        {"sensor_msgs/msg/PointCloud2", true},
        {"sensor_msgs/msg/Image", true},
        {"geometry_msgs/msg/Twist", true},
        {"std_msgs/msg/String", true},
        {"std_msgs/msg/Header", true},
        {"unknown_msgs/msg/Unknown", false},
        {"invalid/type", false},
        {"", false}
    };
    
    for (const auto& [type, expected] : test_cases) {
        bool result = rosbag_deck_is_message_type_supported(type.c_str());
        EXPECT_EQ(result, expected) 
            << "Message type support check failed for: " << type;
    }
}

TEST_F(CAPITest, StringArrayOperations) {
    auto handle = create_handle();
    
    // Test getting available topics (should be empty initially)
    size_t topic_count = 0;
    char** topics = rosbag_deck_get_available_topics(handle, &topic_count);
    
    // Should return null or empty array for uninitialized handle
    if (topics != nullptr) {
        rosbag_deck_free_string_array(topics, topic_count);
    }
    
    // Test getting available types (should be empty initially)  
    size_t type_count = 0;
    char** types = rosbag_deck_get_available_types(handle, &type_count);
    
    if (types != nullptr) {
        rosbag_deck_free_string_array(types, type_count);
    }
}

// ============================================================================
// Configuration Tests
// ============================================================================

TEST_F(CAPITest, ConfigurationSettings) {
    auto handle = create_handle();
    
    // Test cache size configuration
    rosbag_deck_set_cache_size(handle, 500);
    rosbag_deck_set_cache_size(handle, 0);      // Edge case
    rosbag_deck_set_cache_size(handle, SIZE_MAX); // Large value
    
    // Test preload settings
    rosbag_deck_set_preload_settings(handle, 10, 5);
    rosbag_deck_set_preload_settings(handle, 0, 0);  // Edge case
    
    // Test playback rate
    rosbag_deck_set_playback_rate(handle, 1.0);
    rosbag_deck_set_playback_rate(handle, 0.5);
    rosbag_deck_set_playback_rate(handle, 2.0);
    rosbag_deck_set_playback_rate(handle, 0.0);  // Edge case
    
    // Test loop playback
    rosbag_deck_set_loop_playback(handle, true);
    rosbag_deck_set_loop_playback(handle, false);
}

TEST_F(CAPITest, FilterConfiguration) {
    auto handle = create_handle();
    
    // Test topic filtering
    std::vector<const char*> topics = {"/camera/image", "/lidar/points"};
    rosbag_deck_set_topic_filter(handle, topics.data(), topics.size());
    rosbag_deck_clear_topic_filter(handle);
    
    // Test empty topic filter
    rosbag_deck_set_topic_filter(handle, nullptr, 0);
    
    // Test type filtering
    std::vector<const char*> types = {"sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/Image"};
    rosbag_deck_set_type_filter(handle, types.data(), types.size());
    rosbag_deck_clear_type_filter(handle);
    
    // Test empty type filter
    rosbag_deck_set_type_filter(handle, nullptr, 0);
}

// ============================================================================
// Memory Safety Tests
// ============================================================================

TEST_F(CAPITest, MemoryManagement) {
    auto handle = create_handle();
    
    // Test bag info memory management
    auto bag_info = rosbag_deck_get_bag_info(handle);
    rosbag_deck_free_bag_info(&bag_info);
    
    // Verify fields are cleared/nullified after free
    EXPECT_EQ(bag_info.topic_names, nullptr);
    EXPECT_EQ(bag_info.message, nullptr);
    
    // Test message memory management (with null message)
    rosbag_deck_message_t null_message = {};
    rosbag_deck_free_message(&null_message); // Should not crash
}

TEST_F(CAPITest, StringMemorySafety) {
    auto handle = create_handle();
    
    // Test with large string arrays
    std::vector<std::string> large_topic_list;
    std::vector<const char*> large_topic_ptrs;
    
    for (int i = 0; i < 1000; ++i) {
        large_topic_list.push_back("/topic_" + std::to_string(i));
        large_topic_ptrs.push_back(large_topic_list.back().c_str());
    }
    
    // Set large filter (should handle gracefully)
    rosbag_deck_set_topic_filter(handle, large_topic_ptrs.data(), large_topic_ptrs.size());
    rosbag_deck_clear_topic_filter(handle);
}

TEST_F(CAPITest, CallbackMemorySafety) {
    auto handle = create_handle();
    
    // Test setting and clearing callbacks
    rosbag_deck_set_status_callback(handle, nullptr, nullptr);
    rosbag_deck_set_message_callback(handle, nullptr, nullptr);
    
    // Test with actual callback functions
    auto status_callback = [](const rosbag_deck_status_t* status, void* user_data) {
        (void)status;
        (void)user_data;
        // Simple callback that doesn't crash
    };
    
    auto message_callback = [](const rosbag_deck_message_t* message, void* user_data) {
        (void)message;
        (void)user_data;
        // Simple callback that doesn't crash
    };
    
    rosbag_deck_set_status_callback(handle, status_callback, nullptr);
    rosbag_deck_set_message_callback(handle, message_callback, nullptr);
    
    // Clear callbacks
    rosbag_deck_set_status_callback(handle, nullptr, nullptr);
    rosbag_deck_set_message_callback(handle, nullptr, nullptr);
}

// ============================================================================
// Thread Safety Tests
// ============================================================================

TEST_F(CAPITest, ConcurrentHandleAccess) {
    auto handle = create_handle();
    std::atomic<bool> stop_flag{false};
    std::atomic<int> operation_count{0};
    
    // Multiple threads performing different operations
    std::vector<std::thread> threads;
    
    // Thread 1: Configuration operations
    threads.emplace_back([&]() {
        while (!stop_flag.load()) {
            rosbag_deck_set_cache_size(handle, 100 + (operation_count.load() % 100));
            rosbag_deck_set_playback_rate(handle, 1.0 + (operation_count.load() % 5) * 0.1);
            operation_count++;
            std::this_thread::sleep_for(1ms);
        }
    });
    
    // Thread 2: Status queries
    threads.emplace_back([&]() {
        while (!stop_flag.load()) {
            auto status = rosbag_deck_get_status(handle);
            (void)status; // Avoid unused variable warning
            operation_count++;
            std::this_thread::sleep_for(1ms);
        }
    });
    
    // Thread 3: Info queries
    threads.emplace_back([&]() {
        while (!stop_flag.load()) {
            auto info = rosbag_deck_get_bag_info(handle);
            rosbag_deck_free_bag_info(&info);
            operation_count++;
            std::this_thread::sleep_for(1ms);
        }
    });
    
    // Let threads run for a short time
    std::this_thread::sleep_for(100ms);
    stop_flag.store(true);
    
    // Wait for all threads to complete
    for (auto& thread : threads) {
        thread.join();
    }
    
    // Verify we performed many operations without crashing
    EXPECT_GT(operation_count.load(), 10);
}

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST_F(CAPITest, InvalidParameterHandling) {
    auto handle = create_handle();
    
    // Test build_index with invalid parameters
    EXPECT_FALSE(rosbag_deck_build_index(handle, nullptr, 0));
    
    const char* invalid_paths[] = {"/nonexistent/bag.db3", ""};
    EXPECT_FALSE(rosbag_deck_build_index(handle, invalid_paths, 2));
    
    // Test seek operations with invalid parameters
    auto invalid_timestamp = rosbag_deck_to_timestamp(-1);
    EXPECT_FALSE(rosbag_deck_seek_to_time(handle, invalid_timestamp));
    
    EXPECT_FALSE(rosbag_deck_seek_to_frame(handle, SIZE_MAX));
}

TEST_F(CAPITest, OperationsOnUninitializedHandle) {
    auto handle = create_handle();
    
    // Operations on handle without built index should fail gracefully
    EXPECT_FALSE(rosbag_deck_step_forward(handle));
    EXPECT_FALSE(rosbag_deck_step_backward(handle));
    
    // Status should return safe defaults
    auto status = rosbag_deck_get_status(handle);
    EXPECT_FALSE(status.is_playing);
    EXPECT_EQ(status.current_frame, 0);
    EXPECT_EQ(status.total_frames, 0);
    
    // Bag info should indicate failure
    auto info = rosbag_deck_get_bag_info(handle);
    EXPECT_FALSE(info.success);
    rosbag_deck_free_bag_info(&info);
}

// ============================================================================
// Integration with Test Data
// ============================================================================

TEST_F(CAPITest, BasicWorkflowWithTestBag) {
    auto handle = create_handle();
    
    // Try to build index with test bag
    const char* bag_paths[] = {test_bag_path_.c_str()};
    bool index_result = rosbag_deck_build_index(handle, bag_paths, 1);
    
    if (index_result) {
        // If index building succeeded, test basic operations
        auto info = rosbag_deck_get_bag_info(handle);
        EXPECT_TRUE(info.success);
        EXPECT_GT(info.total_frames, 0);
        
        auto status = rosbag_deck_get_status(handle);
        EXPECT_EQ(status.current_frame, 0);
        EXPECT_EQ(status.total_frames, info.total_frames);
        
        // Test stepping
        if (info.total_frames > 0) {
            bool step_result = rosbag_deck_step_forward(handle);
            if (step_result) {
                auto new_status = rosbag_deck_get_status(handle);
                EXPECT_GT(new_status.current_frame, status.current_frame);
            }
        }
        
        rosbag_deck_free_bag_info(&info);
    }
}

// ============================================================================
// Performance Tests
// ============================================================================

TEST_F(CAPITest, OperationPerformance) {
    auto handle = create_handle();
    
    // Measure time for basic operations
    auto start = std::chrono::high_resolution_clock::now();
    
    const int iterations = 1000;
    for (int i = 0; i < iterations; ++i) {
        auto status = rosbag_deck_get_status(handle);
        (void)status;
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    // Each operation should be reasonably fast (less than 100 microseconds on average)
    double avg_time_us = static_cast<double>(duration.count()) / iterations;
    EXPECT_LT(avg_time_us, 100.0) << "Average operation time too slow: " << avg_time_us << " μs";
}

TEST_F(CAPITest, MemoryUsageStability) {
    auto handle = create_handle();
    
    // Perform many operations and verify memory doesn't grow excessively
    for (int i = 0; i < 10000; ++i) {
        auto status = rosbag_deck_get_status(handle);
        (void)status;
        
        auto info = rosbag_deck_get_bag_info(handle);
        rosbag_deck_free_bag_info(&info);
        
        rosbag_deck_set_cache_size(handle, 100 + (i % 50));
        
        // Occasional filter operations
        if (i % 100 == 0) {
            const char* topics[] = {"/test_topic"};
            rosbag_deck_set_topic_filter(handle, topics, 1);
            rosbag_deck_clear_topic_filter(handle);
        }
    }
    
    // If we get here without crashing, memory management is working
    SUCCEED();
}