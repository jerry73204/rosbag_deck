#include "rosbag_deck_core/message_cache.hpp"
#include <mutex>

namespace rosbag_deck_core {

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

  // Special case: window_size 0 means evict everything
  if (window_size == 0) {
    cache_.clear();
    return;
  }

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

} // namespace rosbag_deck_core