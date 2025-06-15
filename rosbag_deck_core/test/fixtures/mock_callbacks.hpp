#pragma once

#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include <vector>
#include <mutex>

namespace rosbag_deck_core {
namespace test {

// Simple callback handler for testing (no GMock to avoid compatibility issues)
class MockCallbackHandler {
public:
  // Simple recording methods instead of MOCK_METHOD
  void on_status(const PlaybackStatus& status) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_history_.push_back(status);
  }
  
  void on_message(const BagMessage& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    message_history_.push_back(message);
  }
  
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
    on_status(status);
  }
  
  void record_message(const BagMessage& message) {
    on_message(message);
  }

private:
  mutable std::mutex mutex_;
  std::vector<PlaybackStatus> status_history_;
  std::vector<BagMessage> message_history_;
};

} // namespace test
} // namespace rosbag_deck_core