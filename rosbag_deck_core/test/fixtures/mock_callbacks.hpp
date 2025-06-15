#pragma once

#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include <gmock/gmock.h>
#include <vector>
#include <mutex>

namespace rosbag_deck_core {
namespace test {

// Mock callback handler for testing
class MockCallbackHandler {
public:
  MOCK_METHOD(void, on_status, (const PlaybackStatus& status));
  MOCK_METHOD(void, on_message, (const BagMessage& message));
  
  // Helper methods for verification
  std::vector<PlaybackStatus> get_status_history() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_history_;
  }
  
  std::vector<BagMessage> get_message_history() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return message_history_;
  }
  
  void clear_history() {
    std::lock_guard<std::mutex> lock(mutex_);
    status_history_.clear();
    message_history_.clear();
  }
  
  // Recording callbacks that also call the mock methods
  void record_status(const PlaybackStatus& status) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status_history_.push_back(status);
    }
    on_status(status);
  }
  
  void record_message(const BagMessage& message) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      message_history_.push_back(message);
    }
    on_message(message);
  }

private:
  mutable std::mutex mutex_;
  std::vector<PlaybackStatus> status_history_;
  std::vector<BagMessage> message_history_;
};

// Mock rosbag2 reader for controlled testing
class MockBagReader {
public:
  MOCK_METHOD(void, open, (const rosbag2_storage::StorageOptions& options));
  MOCK_METHOD(void, close, ());
  MOCK_METHOD(bool, has_next, ());
  MOCK_METHOD(std::shared_ptr<rosbag2_storage::SerializedBagMessage>, read_next, ());
  MOCK_METHOD(const rosbag2_storage::BagMetadata&, get_metadata, ());
  MOCK_METHOD(void, reset_filter, ());
};

} // namespace test
} // namespace rosbag_deck_core