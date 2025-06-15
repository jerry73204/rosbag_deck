#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

namespace rosbag_deck_core {

using Timestamp = std::chrono::time_point<std::chrono::steady_clock,
                                          std::chrono::nanoseconds>;
using Duration = std::chrono::nanoseconds;

// Forward declarations
class BagWorker;
class MessageCache;
class IndexManager;
class MessageTypeRegistry;

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

class IndexManager {
public:
  void build_index(const std::vector<std::string> &bag_paths);
  size_t find_frame_by_time(const Timestamp &target_time) const;
  const IndexEntry &get_entry(size_t frame_index) const;
  size_t total_frames() const { return index_.size(); }
  Timestamp start_time() const { return start_time_; }
  Timestamp end_time() const { return end_time_; }
  const std::vector<std::string> &topic_names() const { return topic_names_; }
  const std::vector<std::string> &bag_paths() const { return bag_paths_; }
  const std::map<std::string, std::string> &topic_types() const {
    return topic_types_;
  }

private:
  std::vector<IndexEntry> index_;
  Timestamp start_time_;
  Timestamp end_time_;
  std::vector<std::string> topic_names_;
  std::vector<std::string> bag_paths_;
  std::map<std::string, std::string> topic_types_; // topic_name -> message_type
};

class MessageCache {
public:
  MessageCache(size_t max_size);
  bool get_message(size_t frame_index, BagMessage &message);
  void put_message(size_t frame_index, const BagMessage &message);
  void clear();
  void evict_outside_window(size_t center_frame, size_t window_size);
  size_t size() const { return cache_.size(); }
  bool is_frame_cached(size_t frame_index) const;

private:
  std::map<size_t, CacheEntry> cache_;
  size_t max_size_;
  mutable std::mutex cache_mutex_;
};

class BagWorker {
public:
  BagWorker(const IndexManager &index_manager);
  ~BagWorker();

  void start();
  void stop();
  std::future<bool> request_range(size_t start_frame, size_t end_frame);
  std::future<bool> request_seek(const Timestamp &target_time);
  void set_cache(std::shared_ptr<MessageCache> cache);

private:
  void worker_thread();
  void process_request(const CacheRequest &request);
  BagMessage load_message_at_frame(size_t frame_index);
  void open_bag_readers();

  const IndexManager &index_manager_;
  std::shared_ptr<MessageCache> cache_;
  std::vector<std::unique_ptr<rosbag2_cpp::Reader>> readers_;

  std::thread worker_thread_;
  std::queue<CacheRequest> request_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::atomic<bool> should_stop_;
};

// Message Type Registry for dynamic type support
class MessageTypeRegistry {
public:
  MessageTypeRegistry();
  ~MessageTypeRegistry() = default;

  // Register all message types found in a bag
  void
  register_types_from_bag_metadata(const std::vector<std::string> &bag_paths);

  // Type introspection
  bool is_type_registered(const std::string &message_type) const;
  std::vector<std::string> get_registered_types() const;

  // Dynamic deserialization support
  bool can_deserialize(const std::string &message_type) const;
  std::string get_type_hash(const std::string &message_type) const;

  // Topic filtering
  void set_topic_filter(const std::vector<std::string> &topics);
  void clear_topic_filter();
  bool is_topic_enabled(const std::string &topic_name) const;

  // Message type filtering
  void set_type_filter(const std::vector<std::string> &types);
  void clear_type_filter();
  bool is_type_enabled(const std::string &message_type) const;

private:
  std::map<std::string, std::string> type_registry_; // type -> hash
  std::set<std::string> enabled_topics_;
  std::set<std::string> enabled_types_;
  bool topic_filter_active_;
  bool type_filter_active_;
  mutable std::mutex registry_mutex_;
};

class RosbagDeckCore {
public:
  explicit RosbagDeckCore();
  ~RosbagDeckCore();

  // Configuration
  void set_cache_size(size_t cache_size);
  void set_preload_settings(size_t ahead, size_t behind);
  void set_playback_rate(double rate);
  void set_loop_playback(bool loop);

  // Topic and type filtering
  void set_topic_filter(const std::vector<std::string> &topics);
  void clear_topic_filter();
  void set_type_filter(const std::vector<std::string> &types);
  void clear_type_filter();
  std::vector<std::string> get_available_topics() const;
  std::vector<std::string> get_available_types() const;

  // Core operations
  bool build_index(const std::vector<std::string> &bag_paths);
  void start_playback();
  void stop_playback();
  bool step_forward();
  bool step_backward();
  bool seek_to_time(const Timestamp &target_time);
  bool seek_to_frame(size_t frame_index);

  // Information queries
  BagInfo get_bag_info() const;
  PlaybackStatus get_status() const;

  // Callback registration for async updates
  void set_status_callback(StatusCallback callback);
  void set_message_callback(MessageCallback callback);

  // Utility functions
  static Timestamp to_timestamp(int64_t nanoseconds_since_epoch);
  static int64_t from_timestamp(const Timestamp &timestamp);

  // Message serialization/deserialization utilities
  template <typename MessageT>
  static std::shared_ptr<std::vector<uint8_t>>
  serialize_message(const MessageT &message);

  template <typename MessageT>
  static MessageT
  deserialize_message(const std::vector<uint8_t> &serialized_data);

  static bool is_message_type_supported(const std::string &message_type);

  // Access to internal components
  const MessageTypeRegistry &get_type_registry() const {
    return *type_registry_;
  }
  const IndexManager &get_index_manager() const { return *index_manager_; }

private:
  // Core functionality
  void playback_thread();
  bool cached_publish_frame(size_t frame_index);
  void publish_status();
  Timestamp remap_timestamp(const Timestamp &original_time) const;
  void increment_timeline_segment();
  size_t indexed_find_frame_by_time(const Timestamp &target_time);
  void ensure_cache_window(size_t center_frame);
  void trigger_preload_if_needed();

  // Components
  std::unique_ptr<IndexManager> index_manager_;
  std::shared_ptr<MessageCache> message_cache_;
  std::unique_ptr<BagWorker> bag_worker_;
  std::unique_ptr<MessageTypeRegistry> type_registry_;

  // Timeline management
  Timestamp virtual_start_time_;
  size_t timeline_segment_;

  // Playback state
  std::atomic<bool> is_playing_;
  std::atomic<size_t> current_frame_;
  std::thread playback_thread_;
  std::mutex state_mutex_;

  // Configuration
  double playback_rate_;
  bool loop_playback_;
  size_t cache_size_;
  size_t preload_ahead_;
  size_t preload_behind_;

  // Async communication
  StatusCallback status_callback_;
  MessageCallback message_callback_;
  std::mutex callback_mutex_;

  // Status publishing
  std::thread status_thread_;
  std::atomic<bool> status_thread_running_;
};

} // namespace rosbag_deck_core
