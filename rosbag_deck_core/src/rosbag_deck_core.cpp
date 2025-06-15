#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include <iostream>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/metadata_io.hpp>

namespace rosbag_deck_core {

// Utility functions
Timestamp RosbagDeckCore::to_timestamp(int64_t nanoseconds_since_epoch) {
  return Timestamp(Duration(nanoseconds_since_epoch));
}

int64_t RosbagDeckCore::from_timestamp(const Timestamp &timestamp) {
  return std::chrono::duration_cast<Duration>(timestamp.time_since_epoch())
      .count();
}

// Template specializations for message serialization/deserialization
template <typename MessageT>
std::shared_ptr<std::vector<uint8_t>>
RosbagDeckCore::serialize_message(const MessageT &message) {
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<MessageT> serialization;
  serialization.serialize_message(&message, &serialized_msg);

  auto data = std::make_shared<std::vector<uint8_t>>(
      serialized_msg.get_rcl_serialized_message().buffer,
      serialized_msg.get_rcl_serialized_message().buffer +
          serialized_msg.size());

  return data;
}

template <typename MessageT>
MessageT RosbagDeckCore::deserialize_message(
    const std::vector<uint8_t> &serialized_data) {
  rclcpp::SerializedMessage serialized_msg;
  serialized_msg.reserve(serialized_data.size());

  // Copy data to serialized message
  auto &rcl_msg = serialized_msg.get_rcl_serialized_message();
  rcl_msg.buffer_length = serialized_data.size();
  rcl_msg.buffer_capacity = serialized_data.size();
  rcl_msg.buffer = const_cast<uint8_t *>(serialized_data.data());

  MessageT message;
  rclcpp::Serialization<MessageT> serialization;
  serialization.deserialize_message(&serialized_msg, &message);

  return message;
}

bool RosbagDeckCore::is_message_type_supported(
    const std::string &message_type) {
  // Basic support for common ROS 2 message types
  static const std::set<std::string> supported_types = {
      "sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/Image",
      "sensor_msgs/msg/LaserScan",   "sensor_msgs/msg/Imu",
      "geometry_msgs/msg/Twist",     "geometry_msgs/msg/PoseStamped",
      "nav_msgs/msg/Odometry",       "std_msgs/msg/String",
      "std_msgs/msg/Header"};

  return supported_types.find(message_type) != supported_types.end();
}

// IndexManager implementation
void IndexManager::build_index(const std::vector<std::string> &bag_paths) {
  index_.clear();
  topic_names_.clear();
  bag_paths_ = bag_paths;

  std::cout << "Building index for " << bag_paths.size()
            << " bags using rosbag2" << std::endl;

  std::set<std::string> unique_topics;
  size_t global_frame_index = 0;
  Timestamp earliest_time = Timestamp::max();
  Timestamp latest_time = Timestamp::min();
  topic_types_.clear();

  for (size_t bag_idx = 0; bag_idx < bag_paths.size(); ++bag_idx) {
    try {
      rosbag2_cpp::Reader reader;
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = bag_paths[bag_idx];

      reader.open(storage_options);

      // Get bag metadata
      auto metadata = reader.get_metadata();

      // Collect topic information
      for (const auto &topic_metadata : metadata.topics_with_message_count) {
        unique_topics.insert(topic_metadata.topic_metadata.name);
        topic_types_[topic_metadata.topic_metadata.name] =
            topic_metadata.topic_metadata.type;
      }

      // Read messages to build index
      while (reader.has_next()) {
        auto bag_message = reader.read_next();

        IndexEntry entry;
        entry.frame_index = global_frame_index++;

        // Convert rosbag2 timestamp to our timestamp format
        entry.timestamp =
            Timestamp(std::chrono::nanoseconds(bag_message->time_stamp));

        entry.topic_name = bag_message->topic_name;
        entry.message_type = topic_types_[bag_message->topic_name];
        entry.bag_file_index = bag_idx;
        entry.offset_in_bag = 0; // rosbag2 doesn't expose file offsets directly
        entry.serialization_format =
            "cdr"; // rosbag2 default serialization format

        index_.push_back(entry);

        // Track time bounds
        if (entry.timestamp < earliest_time) {
          earliest_time = entry.timestamp;
        }
        if (entry.timestamp > latest_time) {
          latest_time = entry.timestamp;
        }
      }

      reader.close();

    } catch (const std::exception &e) {
      std::cerr << "Error reading bag " << bag_paths[bag_idx] << ": "
                << e.what() << std::endl;
      continue;
    }
  }

  // Sort index by timestamp
  std::sort(index_.begin(), index_.end(),
            [](const IndexEntry &a, const IndexEntry &b) {
              return a.timestamp < b.timestamp;
            });

  // Reassign frame indices after sorting
  for (size_t i = 0; i < index_.size(); ++i) {
    index_[i].frame_index = i;
  }

  // Store topic names and time bounds
  topic_names_.assign(unique_topics.begin(), unique_topics.end());
  start_time_ = earliest_time;
  end_time_ = latest_time;

  std::cout << "Index built: " << index_.size() << " messages, "
            << topic_names_.size() << " topics" << std::endl;
}

size_t IndexManager::find_frame_by_time(const Timestamp &target_time) const {
  if (index_.empty()) {
    return 0;
  }

  auto it =
      std::lower_bound(index_.begin(), index_.end(), target_time,
                       [](const IndexEntry &entry, const Timestamp &time) {
                         return entry.timestamp < time;
                       });

  if (it == index_.end()) {
    return index_.size() - 1;
  }

  return std::distance(index_.begin(), it);
}

const IndexEntry &IndexManager::get_entry(size_t frame_index) const {
  return index_.at(frame_index);
}

// MessageCache implementation
MessageCache::MessageCache(size_t max_size) : max_size_(max_size) {}

bool MessageCache::get_message(size_t frame_index, BagMessage &message) {
  std::lock_guard<std::mutex> lock(cache_mutex_);
  auto it = cache_.find(frame_index);
  if (it != cache_.end()) {
    message = it->second.message;
    return true;
  }
  return false;
}

void MessageCache::put_message(size_t frame_index, const BagMessage &message) {
  std::lock_guard<std::mutex> lock(cache_mutex_);

  if (cache_.size() >= max_size_) {
    cache_.erase(cache_.begin());
  }

  CacheEntry entry;
  entry.global_frame_index = frame_index;
  entry.timestamp = message.original_timestamp;
  entry.message = message;

  cache_[frame_index] = std::move(entry);
}

void MessageCache::clear() {
  std::lock_guard<std::mutex> lock(cache_mutex_);
  cache_.clear();
}

void MessageCache::evict_outside_window(size_t center_frame,
                                        size_t window_size) {
  std::lock_guard<std::mutex> lock(cache_mutex_);

  size_t half_window = window_size / 2;
  size_t min_frame =
      (center_frame > half_window) ? center_frame - half_window : 0;
  size_t max_frame = center_frame + half_window;

  auto it = cache_.begin();
  while (it != cache_.end()) {
    if (it->first < min_frame || it->first > max_frame) {
      it = cache_.erase(it);
    } else {
      ++it;
    }
  }
}

bool MessageCache::is_frame_cached(size_t frame_index) const {
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return cache_.find(frame_index) != cache_.end();
}

// BagWorker implementation
BagWorker::BagWorker(const IndexManager &index_manager)
    : index_manager_(index_manager), should_stop_(false) {
  std::cout << "BagWorker initialized" << std::endl;
  open_bag_readers();
}

BagWorker::~BagWorker() { stop(); }

void BagWorker::start() {
  should_stop_ = false;
  worker_thread_ = std::thread(&BagWorker::worker_thread, this);
}

void BagWorker::stop() {
  should_stop_ = true;
  queue_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void BagWorker::set_cache(std::shared_ptr<MessageCache> cache) {
  cache_ = cache;
}

std::future<bool> BagWorker::request_range(size_t start_frame,
                                           size_t end_frame) {
  CacheRequest request;
  request.type = RequestType::LOAD_RANGE;
  request.start_frame = start_frame;
  request.end_frame = end_frame;

  auto future = request.completion.get_future();

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    request_queue_.push(std::move(request));
  }
  queue_cv_.notify_one();

  return future;
}

std::future<bool> BagWorker::request_seek(const Timestamp &target_time) {
  CacheRequest request;
  request.type = RequestType::SEEK_TO_POSITION;
  request.target_time = target_time;

  auto future = request.completion.get_future();

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    request_queue_.push(std::move(request));
  }
  queue_cv_.notify_one();

  return future;
}

void BagWorker::worker_thread() {
  while (!should_stop_) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_cv_.wait(lock,
                   [this] { return !request_queue_.empty() || should_stop_; });

    if (should_stop_)
      break;

    CacheRequest request = std::move(request_queue_.front());
    request_queue_.pop();
    lock.unlock();

    process_request(request);
  }
}

void BagWorker::process_request(const CacheRequest &request) {
  try {
    switch (request.type) {
    case RequestType::LOAD_RANGE:
      for (size_t frame = request.start_frame;
           frame <= request.end_frame && frame < index_manager_.total_frames();
           ++frame) {
        if (!cache_->is_frame_cached(frame)) {
          BagMessage message = load_message_at_frame(frame);
          cache_->put_message(frame, message);
        }
      }
      request.completion.set_value(true);
      break;

    case RequestType::SEEK_TO_POSITION: {
      size_t target_frame =
          index_manager_.find_frame_by_time(request.target_time);
      BagMessage message = load_message_at_frame(target_frame);
      cache_->put_message(target_frame, message);
      request.completion.set_value(true);
    } break;

    default:
      request.completion.set_value(false);
    }
  } catch (const std::exception &e) {
    request.completion.set_value(false);
  }
}

void BagWorker::open_bag_readers() {
  readers_.clear();
  const auto &bag_paths = index_manager_.bag_paths();

  for (const auto &bag_path : bag_paths) {
    auto reader = std::make_unique<rosbag2_cpp::Reader>();
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;

    try {
      reader->open(storage_options);
      readers_.push_back(std::move(reader));
    } catch (const std::exception &e) {
      std::cerr << "Failed to open bag " << bag_path << ": " << e.what()
                << std::endl;
    }
  }
}

BagMessage BagWorker::load_message_at_frame(size_t frame_index) {
  const auto &entry = index_manager_.get_entry(frame_index);

  if (entry.bag_file_index >= readers_.size()) {
    throw std::runtime_error("Invalid bag file index");
  }

  auto &reader = readers_[entry.bag_file_index];

  // Reset reader and seek to the desired message
  // Note: rosbag2 doesn't support direct seeking by timestamp easily,
  // so we'll need to iterate through messages. This is a simplified approach.
  reader->reset_filter();

  BagMessage message;
  message.frame_index = frame_index;
  message.original_timestamp = entry.timestamp;
  message.topic_name = entry.topic_name;
  message.message_type = entry.message_type;
  message.serialization_format = entry.serialization_format;

  // For now, create a placeholder implementation
  // In a real implementation, you would seek through the bag to find the exact
  // message
  try {
    while (reader->has_next()) {
      auto bag_message = reader->read_next();

      // Convert timestamp to match our format
      auto converted_time =
          Timestamp(std::chrono::nanoseconds(bag_message->time_stamp));

      if (bag_message->topic_name == entry.topic_name &&
          std::abs(std::chrono::duration_cast<std::chrono::nanoseconds>(
                       converted_time - entry.timestamp)
                       .count()) < 1000) {

        // Extract serialized message data
        message.serialized_data = std::make_shared<std::vector<uint8_t>>(
            bag_message->serialized_data->buffer,
            bag_message->serialized_data->buffer +
                bag_message->serialized_data->buffer_length);

        // Store additional metadata if available
        message.metadata["original_timestamp"] =
            std::to_string(bag_message->time_stamp);
        message.metadata["topic_name"] = bag_message->topic_name;

        break;
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Error loading message at frame " << frame_index << ": "
              << e.what() << std::endl;
    // Fallback to dummy data
    message.serialized_data = std::make_shared<std::vector<uint8_t>>(1024);
  }

  return message;
}

// RosbagDeckCore implementation
RosbagDeckCore::RosbagDeckCore()
    : timeline_segment_(0), is_playing_(false), current_frame_(0),
      playback_rate_(1.0), loop_playback_(false), cache_size_(1000),
      preload_ahead_(100), preload_behind_(100), status_thread_running_(false) {
  virtual_start_time_ = std::chrono::steady_clock::now();

  index_manager_ = std::make_unique<IndexManager>();
  message_cache_ = std::make_shared<MessageCache>(cache_size_);
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

bool RosbagDeckCore::build_index(const std::vector<std::string> &bag_paths) {
  try {
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

    return true;
  } catch (const std::exception &e) {
    std::cerr << "Error building index: " << e.what() << std::endl;
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
  if (!index_manager_ || index_manager_->total_frames() == 0) {
    info.success = false;
    info.message = "No bags indexed";
    return info;
  }

  info.success = true;
  info.message = "Bag information retrieved successfully";
  info.start_time = index_manager_->start_time();
  info.end_time = index_manager_->end_time();
  info.total_duration =
      std::chrono::duration_cast<Duration>(info.end_time - info.start_time);
  info.total_frames = index_manager_->total_frames();
  info.topic_names = index_manager_->topic_names();

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
