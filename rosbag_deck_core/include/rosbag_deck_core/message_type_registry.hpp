#pragma once

#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace rosbag_deck_core {

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

} // namespace rosbag_deck_core