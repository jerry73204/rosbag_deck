#include "rosbag_deck_core/index_manager.hpp"
#include <algorithm>
#include <iostream>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/metadata_io.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <set>

namespace rosbag_deck_core {

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
  
  size_t successful_bags = 0;

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
      successful_bags++;

    } catch (const std::exception &e) {
      std::cerr << "Error reading bag " << bag_paths[bag_idx] << ": "
                << e.what() << std::endl;
      continue;
    }
  }
  
  // If no bags were successfully processed, throw an exception
  if (successful_bags == 0 && !bag_paths.empty()) {
    throw std::runtime_error("Failed to open any of the provided bag files");
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
  if (index_.empty()) {
    // No messages found - set default timestamps
    start_time_ = Timestamp::min();
    end_time_ = Timestamp::min();
  } else {
    start_time_ = earliest_time;
    end_time_ = latest_time;
  }

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

} // namespace rosbag_deck_core