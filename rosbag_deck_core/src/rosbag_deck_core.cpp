#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include "rosbag_deck_core/utilities.hpp"
#include <iostream>
#include <rclcpp/serialization.hpp>

namespace rosbag_deck_core {

// RosbagDeckCore implementation
RosbagDeckCore::RosbagDeckCore()
    : timeline_segment_(0), is_playing_(false), current_frame_(0),
      playback_rate_(1.0), loop_playback_(false), cache_size_(1000),
      preload_ahead_(100), preload_behind_(100), status_thread_running_(false),
      index_built_successfully_(false) {
  virtual_start_time_ = std::chrono::steady_clock::now();

  index_manager_ = std::make_unique<IndexManager>();
  message_cache_ = std::make_shared<MessageCache>(cache_size_);
  type_registry_ = std::make_unique<MessageTypeRegistry>();
}

RosbagDeckCore::~RosbagDeckCore() {
  stop_playback();
  status_thread_running_ = false;
  if (status_thread_.joinable()) {
    status_thread_.join();
  }
}

void RosbagDeckCore::set_cache_size(size_t cache_size) {
  cache_size_ = cache_size;
  message_cache_ = std::make_shared<MessageCache>(cache_size_);
  if (bag_worker_) {
    bag_worker_->set_cache(message_cache_);
  }
}

void RosbagDeckCore::set_preload_settings(size_t ahead, size_t behind) {
  preload_ahead_ = ahead;
  preload_behind_ = behind;
}

void RosbagDeckCore::set_playback_rate(double rate) { playback_rate_ = rate; }

void RosbagDeckCore::set_loop_playback(bool loop) { loop_playback_ = loop; }

void RosbagDeckCore::set_topic_filter(const std::vector<std::string> &topics) {
  type_registry_->set_topic_filter(topics);
}

void RosbagDeckCore::clear_topic_filter() {
  type_registry_->clear_topic_filter();
}

void RosbagDeckCore::set_type_filter(const std::vector<std::string> &types) {
  type_registry_->set_type_filter(types);
}

void RosbagDeckCore::clear_type_filter() {
  type_registry_->clear_type_filter();
}

std::vector<std::string> RosbagDeckCore::get_available_topics() const {
  return index_manager_ ? index_manager_->topic_names()
                        : std::vector<std::string>();
}

std::vector<std::string> RosbagDeckCore::get_available_types() const {
  return type_registry_->get_registered_types();
}

bool RosbagDeckCore::build_index(const std::vector<std::string> &bag_paths) {
  try {
    // Register all message types from the bags
    type_registry_->register_types_from_bag_metadata(bag_paths);

    index_manager_->build_index(bag_paths);

    bag_worker_ = std::make_unique<BagWorker>(*index_manager_);
    bag_worker_->set_cache(message_cache_);
    bag_worker_->start();

    // Start status thread
    status_thread_running_ = true;
    status_thread_ = std::thread([this]() {
      while (status_thread_running_) {
        publish_status();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });

    index_built_successfully_ = true;
    return true;
  } catch (const std::exception &e) {
    std::cerr << "Error building index: " << e.what() << std::endl;
    
    // Clear the index manager on failure so get_bag_info returns failure
    index_manager_ = std::make_unique<IndexManager>();
    index_built_successfully_ = false;
    
    return false;
  }
}

void RosbagDeckCore::start_playback() {
  if (is_playing_.load()) {
    return;
  }

  is_playing_.store(true);
  playback_thread_ = std::thread(&RosbagDeckCore::playback_thread, this);
}

void RosbagDeckCore::stop_playback() {
  is_playing_.store(false);
  if (playback_thread_.joinable()) {
    playback_thread_.join();
  }
}

bool RosbagDeckCore::step_forward() {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (current_frame_.load() < index_manager_->total_frames() - 1) {
    current_frame_++;
    return cached_publish_frame(current_frame_.load());
  }
  return false;
}

bool RosbagDeckCore::step_backward() {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (current_frame_.load() > 0) {
    increment_timeline_segment();
    current_frame_--;
    return cached_publish_frame(current_frame_.load());
  }
  return false;
}

bool RosbagDeckCore::seek_to_time(const Timestamp &target_time) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  size_t target_frame = indexed_find_frame_by_time(target_time);

  if (target_frame < index_manager_->total_frames()) {
    if (target_frame < current_frame_.load()) {
      increment_timeline_segment();
    }

    current_frame_.store(target_frame);
    ensure_cache_window(target_frame);
    return cached_publish_frame(current_frame_.load());
  }
  return false;
}

bool RosbagDeckCore::seek_to_frame(size_t frame_index) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (frame_index < index_manager_->total_frames()) {
    if (frame_index < current_frame_.load()) {
      increment_timeline_segment();
    }

    current_frame_.store(frame_index);
    ensure_cache_window(frame_index);
    return cached_publish_frame(current_frame_.load());
  }
  return false;
}

BagInfo RosbagDeckCore::get_bag_info() const {
  BagInfo info;
  if (!index_manager_ || !index_built_successfully_) {
    info.success = false;
    info.message = index_built_successfully_ ? "No bags indexed" : "Failed to build index";
    return info;
  }

  info.success = true;
  info.total_frames = index_manager_->total_frames();
  info.topic_names = index_manager_->topic_names();
  
  if (info.total_frames == 0) {
    info.message = "Empty bag - no messages found";
    info.start_time = Timestamp::min();
    info.end_time = Timestamp::min();
    info.total_duration = Duration::zero();
  } else {
    info.message = "Bag information retrieved successfully";
    info.start_time = index_manager_->start_time();
    info.end_time = index_manager_->end_time();
    info.total_duration =
        std::chrono::duration_cast<Duration>(info.end_time - info.start_time);
  }

  return info;
}

PlaybackStatus RosbagDeckCore::get_status() const {
  PlaybackStatus status;
  status.is_playing = is_playing_.load();
  status.current_frame = current_frame_.load();
  status.total_frames = index_manager_ ? index_manager_->total_frames() : 0;
  status.timeline_segment = timeline_segment_;

  if (current_frame_.load() < index_manager_->total_frames()) {
    const auto &entry = index_manager_->get_entry(current_frame_.load());
    status.current_time = entry.timestamp;
    status.virtual_time = remap_timestamp(entry.timestamp);
  }

  return status;
}

void RosbagDeckCore::set_status_callback(StatusCallback callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  status_callback_ = callback;
}

void RosbagDeckCore::set_message_callback(MessageCallback callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  message_callback_ = callback;
}

// Template specializations
template <typename MessageT>
std::shared_ptr<std::vector<uint8_t>>
RosbagDeckCore::serialize_message(const MessageT &message) {
  return rosbag_deck_core::serialize_message<MessageT>(message);
}

template <typename MessageT>
MessageT RosbagDeckCore::deserialize_message(
    const std::vector<uint8_t> &serialized_data) {
  return rosbag_deck_core::deserialize_message<MessageT>(serialized_data);
}

bool RosbagDeckCore::is_message_type_supported(const std::string &message_type) {
  return rosbag_deck_core::is_message_type_supported(message_type);
}

Timestamp RosbagDeckCore::to_timestamp(int64_t nanoseconds_since_epoch) {
  return rosbag_deck_core::to_timestamp(nanoseconds_since_epoch);
}

int64_t RosbagDeckCore::from_timestamp(const Timestamp &timestamp) {
  return rosbag_deck_core::from_timestamp(timestamp);
}

void RosbagDeckCore::playback_thread() {
  auto last_time = std::chrono::steady_clock::now();

  while (is_playing_.load() &&
         current_frame_.load() < index_manager_->total_frames()) {
    trigger_preload_if_needed();

    auto current_time = std::chrono::steady_clock::now();
    auto elapsed =
        std::chrono::duration_cast<Duration>(current_time - last_time).count();

    if (current_frame_.load() > 0) {
      const auto &current_entry =
          index_manager_->get_entry(current_frame_.load());
      const auto &prev_entry =
          index_manager_->get_entry(current_frame_.load() - 1);
      auto time_diff = std::chrono::duration_cast<Duration>(
                           current_entry.timestamp - prev_entry.timestamp)
                           .count();
      auto expected_interval = static_cast<int64_t>(time_diff / playback_rate_);

      if (elapsed < expected_interval) {
        std::this_thread::sleep_for(Duration(expected_interval - elapsed));
      }
    }

    cached_publish_frame(current_frame_.load());
    current_frame_++;
    last_time = std::chrono::steady_clock::now();

    if (current_frame_.load() >= index_manager_->total_frames() &&
        loop_playback_) {
      increment_timeline_segment();
      current_frame_.store(0);
    }
  }

  is_playing_.store(false);
}

bool RosbagDeckCore::cached_publish_frame(size_t frame_index) {
  if (frame_index >= index_manager_->total_frames()) {
    return false;
  }

  // Get the index entry to check topic and type filters
  const auto &entry = index_manager_->get_entry(frame_index);

  // Skip if topic is filtered out
  if (!type_registry_->is_topic_enabled(entry.topic_name)) {
    return true; // Return true to continue playback, just skip this message
  }

  // Skip if message type is filtered out
  if (!type_registry_->is_type_enabled(entry.message_type)) {
    return true; // Return true to continue playback, just skip this message
  }

  BagMessage message;
  if (!message_cache_->get_message(frame_index, message)) {
    auto future = bag_worker_->request_range(frame_index, frame_index);
    if (future.wait_for(std::chrono::milliseconds(100)) ==
        std::future_status::ready) {
      if (future.get() && message_cache_->get_message(frame_index, message)) {
        // Successfully loaded
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  // Update virtual timestamp
  message.virtual_timestamp = remap_timestamp(message.original_timestamp);

  // Call message callback if set
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (message_callback_) {
      message_callback_(message);
    }
  }

  return true;
}

void RosbagDeckCore::publish_status() {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  if (status_callback_) {
    status_callback_(get_status());
  }
}

Timestamp
RosbagDeckCore::remap_timestamp(const Timestamp &original_time) const {
  auto bag_start = index_manager_->start_time();
  auto offset = original_time - bag_start;
  auto segment_offset = Duration(timeline_segment_ * 1000000000LL);
  return virtual_start_time_ + offset + segment_offset;
}

void RosbagDeckCore::increment_timeline_segment() { timeline_segment_++; }

size_t
RosbagDeckCore::indexed_find_frame_by_time(const Timestamp &target_time) {
  return index_manager_->find_frame_by_time(target_time);
}

void RosbagDeckCore::ensure_cache_window(size_t center_frame) {
  size_t window_size = preload_ahead_ + preload_behind_;
  size_t start_frame =
      (center_frame > preload_behind_) ? center_frame - preload_behind_ : 0;
  size_t end_frame = std::min(center_frame + preload_ahead_,
                              index_manager_->total_frames() - 1);

  message_cache_->evict_outside_window(center_frame, window_size);
  bag_worker_->request_range(start_frame, end_frame);
}

void RosbagDeckCore::trigger_preload_if_needed() {
  size_t current = current_frame_.load();
  size_t total = index_manager_->total_frames();

  if (current + preload_ahead_ / 2 >= total ||
      !message_cache_->is_frame_cached(current + preload_ahead_ / 2)) {
    size_t preload_start = current;
    size_t preload_end = std::min(current + preload_ahead_, total - 1);
    bag_worker_->request_range(preload_start, preload_end);
  }
}

} // namespace rosbag_deck_core