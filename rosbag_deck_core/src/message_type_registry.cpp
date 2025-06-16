#include "rosbag_deck_core/message_type_registry.hpp"
#include <iostream>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

namespace rosbag_deck_core {

MessageTypeRegistry::MessageTypeRegistry()
    : topic_filter_active_(false), type_filter_active_(false) {}

void MessageTypeRegistry::register_types_from_bag_metadata(
    const std::vector<std::string> &bag_paths) {
  std::lock_guard<std::mutex> lock(registry_mutex_);

  for (const auto &bag_path : bag_paths) {
    try {
      rosbag2_cpp::Reader reader;
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = bag_path;
      reader.open(storage_options);

      auto metadata = reader.get_metadata();

      for (const auto &topic_metadata : metadata.topics_with_message_count) {
        type_registry_[topic_metadata.topic_metadata.type] =
            topic_metadata.topic_metadata.type; // TODO: Get actual type hash
      }

      reader.close();
    } catch (const std::exception &e) {
      std::cerr << "Error reading metadata from " << bag_path << ": "
                << e.what() << std::endl;
    }
  }
}

bool MessageTypeRegistry::is_type_registered(
    const std::string &message_type) const {
  std::lock_guard<std::mutex> lock(registry_mutex_);
  return type_registry_.find(message_type) != type_registry_.end();
}

std::vector<std::string> MessageTypeRegistry::get_registered_types() const {
  std::lock_guard<std::mutex> lock(registry_mutex_);
  std::vector<std::string> types;
  for (const auto &[type, hash] : type_registry_) {
    types.push_back(type);
  }
  return types;
}

bool MessageTypeRegistry::can_deserialize(
    const std::string &message_type) const {
  // For now, we support all registered types
  return is_type_registered(message_type);
}

std::string
MessageTypeRegistry::get_type_hash(const std::string &message_type) const {
  std::lock_guard<std::mutex> lock(registry_mutex_);
  auto it = type_registry_.find(message_type);
  return (it != type_registry_.end()) ? it->second : "";
}

void MessageTypeRegistry::set_topic_filter(
    const std::vector<std::string> &topics) {
  std::lock_guard<std::mutex> lock(registry_mutex_);
  enabled_topics_.clear();
  enabled_topics_.insert(topics.begin(), topics.end());
  topic_filter_active_ = true; // Any call to set_topic_filter activates filtering
}

void MessageTypeRegistry::clear_topic_filter() {
  std::lock_guard<std::mutex> lock(registry_mutex_);
  enabled_topics_.clear();
  topic_filter_active_ = false;
}

bool MessageTypeRegistry::is_topic_enabled(
    const std::string &topic_name) const {
  std::lock_guard<std::mutex> lock(registry_mutex_);
  if (!topic_filter_active_) {
    // If no filter is active, check if the topic exists in any registered types
    // Since we don't track topics separately, we'll be permissive and return true
    // This maintains backward compatibility but may need refinement
    return true;
  }
  return enabled_topics_.find(topic_name) != enabled_topics_.end();
}

void MessageTypeRegistry::set_type_filter(
    const std::vector<std::string> &types) {
  std::lock_guard<std::mutex> lock(registry_mutex_);
  enabled_types_.clear();
  enabled_types_.insert(types.begin(), types.end());
  type_filter_active_ = true; // Any call to set_type_filter activates filtering
}

void MessageTypeRegistry::clear_type_filter() {
  std::lock_guard<std::mutex> lock(registry_mutex_);
  enabled_types_.clear();
  type_filter_active_ = false;
}

bool MessageTypeRegistry::is_type_enabled(
    const std::string &message_type) const {
  std::lock_guard<std::mutex> lock(registry_mutex_);
  if (!type_filter_active_) {
    return true; // All types enabled if no filter
  }
  return enabled_types_.find(message_type) != enabled_types_.end();
}

} // namespace rosbag_deck_core