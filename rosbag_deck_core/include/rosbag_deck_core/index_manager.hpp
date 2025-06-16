#pragma once

#include "types.hpp"
#include <map>
#include <string>
#include <vector>

namespace rosbag_deck_core {

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

} // namespace rosbag_deck_core