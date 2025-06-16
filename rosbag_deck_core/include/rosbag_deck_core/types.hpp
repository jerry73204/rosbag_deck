#pragma once

#include <chrono>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rosbag_deck_core {

using Timestamp = std::chrono::time_point<std::chrono::steady_clock,
                                          std::chrono::nanoseconds>;
using Duration = std::chrono::nanoseconds;

struct BagMessage {
  Timestamp original_timestamp;
  Timestamp virtual_timestamp;
  std::string topic_name;
  std::string message_type;
  std::shared_ptr<std::vector<uint8_t>> serialized_data;
  size_t frame_index;

  // Additional rosbag2 metadata
  std::string serialization_format;
  std::map<std::string, std::string> metadata;
};

struct IndexEntry {
  size_t frame_index;
  Timestamp timestamp;
  std::string topic_name;
  std::string message_type;
  size_t bag_file_index;
  size_t offset_in_bag;
  std::string serialization_format;
};

struct CacheEntry {
  size_t global_frame_index;
  Timestamp timestamp;
  BagMessage message;
};

enum class RequestType { LOAD_RANGE, SEEK_TO_POSITION, PRELOAD_AHEAD };

struct CacheRequest {
  RequestType type;
  size_t start_frame;
  size_t end_frame;
  Timestamp target_time;
  mutable std::promise<bool> completion;
};

struct PlaybackStatus {
  Timestamp current_time;
  bool is_playing;
  size_t current_frame;
  size_t total_frames;
  size_t timeline_segment;
  Timestamp virtual_time;
};

struct BagInfo {
  bool success;
  std::string message;
  Timestamp start_time;
  Timestamp end_time;
  Duration total_duration;
  size_t total_frames;
  std::vector<std::string> topic_names;
};

// Callback types for async communication
using StatusCallback = std::function<void(const PlaybackStatus &)>;
using MessageCallback = std::function<void(const BagMessage &)>;

} // namespace rosbag_deck_core