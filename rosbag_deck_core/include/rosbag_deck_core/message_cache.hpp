#pragma once

#include "types.hpp"
#include <map>
#include <mutex>

namespace rosbag_deck_core {

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

} // namespace rosbag_deck_core