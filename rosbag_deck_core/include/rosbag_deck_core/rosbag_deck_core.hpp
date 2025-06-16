#pragma once

#include "bag_worker.hpp"
#include "index_manager.hpp"
#include "message_cache.hpp"
#include "message_type_registry.hpp"
#include "types.hpp"
#include "utilities.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

namespace rosbag_deck_core {

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

  // Message serialization/deserialization utilities
  template <typename MessageT>
  static std::shared_ptr<std::vector<uint8_t>>
  serialize_message(const MessageT &message);

  template <typename MessageT>
  static MessageT
  deserialize_message(const std::vector<uint8_t> &serialized_data);

  static bool is_message_type_supported(const std::string &message_type);

  // Utility functions
  static Timestamp to_timestamp(int64_t nanoseconds_since_epoch);
  static int64_t from_timestamp(const Timestamp &timestamp);

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
  
  // Index state
  bool index_built_successfully_;
};

} // namespace rosbag_deck_core