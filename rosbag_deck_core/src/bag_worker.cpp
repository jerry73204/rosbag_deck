#include "rosbag_deck_core/bag_worker.hpp"
#include <chrono>
#include <iostream>
#include <rosbag2_storage/storage_options.hpp>

namespace rosbag_deck_core {

BagWorker::BagWorker(const IndexManager &index_manager)
    : index_manager_(index_manager), should_stop_(false) {
  std::cout << "BagWorker initialized" << std::endl;
  open_bag_readers();
}

BagWorker::~BagWorker() { stop(); }

void BagWorker::start() {
  // Stop any existing thread first
  if (worker_thread_.joinable()) {
    should_stop_ = true;
    queue_cv_.notify_all();
    worker_thread_.join();
  }
  
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

} // namespace rosbag_deck_core