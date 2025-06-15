#include "rosbag_deck_core/rosbag_deck_c.h"
#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include <cstring>
#include <memory>

using namespace rosbag_deck_core;

// Internal wrapper struct to hold C++ object and callback data
struct RosbagDeckWrapper {
  std::unique_ptr<RosbagDeckCore> core;
  rosbag_deck_status_callback_t status_callback;
  rosbag_deck_message_callback_t message_callback;
  void *status_user_data;
  void *message_user_data;

  // Storage for converted data to ensure memory safety
  std::vector<std::string> topic_names_storage;
  std::vector<const char *> topic_names_ptrs;
  std::string bag_info_message_storage;
  std::string message_topic_storage;
  std::string message_type_storage;
  std::string message_serialization_format_storage;
  std::vector<uint8_t> message_data_storage;
  std::vector<std::string> metadata_keys_storage;
  std::vector<std::string> metadata_values_storage;
};

// Helper function to convert C++ timestamp to C timestamp
rosbag_deck_timestamp_t cpp_to_c_timestamp(const Timestamp &timestamp) {
  rosbag_deck_timestamp_t result;
  result.nanoseconds_since_epoch = RosbagDeckCore::from_timestamp(timestamp);
  return result;
}

// Helper function to convert C timestamp to C++ timestamp
Timestamp c_to_cpp_timestamp(const rosbag_deck_timestamp_t &timestamp) {
  return RosbagDeckCore::to_timestamp(timestamp.nanoseconds_since_epoch);
}

// C++ to C callback adapters
void status_callback_adapter(const PlaybackStatus &status,
                             RosbagDeckWrapper *wrapper) {
  if (wrapper->status_callback) {
    rosbag_deck_status_t c_status;
    c_status.current_time = cpp_to_c_timestamp(status.current_time);
    c_status.is_playing = status.is_playing;
    c_status.current_frame = status.current_frame;
    c_status.total_frames = status.total_frames;
    c_status.timeline_segment = status.timeline_segment;
    c_status.virtual_time = cpp_to_c_timestamp(status.virtual_time);

    wrapper->status_callback(&c_status, wrapper->status_user_data);
  }
}

void message_callback_adapter(const BagMessage &message,
                              RosbagDeckWrapper *wrapper) {
  if (wrapper->message_callback) {
    // Store strings in wrapper to ensure memory safety
    wrapper->message_topic_storage = message.topic_name;
    wrapper->message_type_storage = message.message_type;
    wrapper->message_serialization_format_storage =
        message.serialization_format;
    wrapper->message_data_storage = *message.serialized_data;

    // Store metadata
    wrapper->metadata_keys_storage.clear();
    wrapper->metadata_values_storage.clear();
    for (const auto &[key, value] : message.metadata) {
      wrapper->metadata_keys_storage.push_back(key);
      wrapper->metadata_values_storage.push_back(value);
    }

    rosbag_deck_message_t c_message;
    c_message.original_timestamp =
        cpp_to_c_timestamp(message.original_timestamp);
    c_message.virtual_timestamp = cpp_to_c_timestamp(message.virtual_timestamp);
    c_message.topic_name = wrapper->message_topic_storage.c_str();
    c_message.message_type = wrapper->message_type_storage.c_str();
    c_message.serialized_data = wrapper->message_data_storage.data();
    c_message.serialized_data_size = wrapper->message_data_storage.size();
    c_message.frame_index = message.frame_index;
    c_message.serialization_format =
        wrapper->message_serialization_format_storage.c_str();

    // Convert metadata to C arrays
    std::vector<const char *> keys;
    std::vector<const char *> values;
    for (size_t i = 0; i < wrapper->metadata_keys_storage.size(); ++i) {
      keys.push_back(wrapper->metadata_keys_storage[i].c_str());
      values.push_back(wrapper->metadata_values_storage[i].c_str());
    }
    c_message.metadata_keys = keys.data();
    c_message.metadata_values = values.data();
    c_message.metadata_count = keys.size();

    wrapper->message_callback(&c_message, wrapper->message_user_data);
  }
}

// Core lifecycle functions
rosbag_deck_handle_t rosbag_deck_create(void) {
  try {
    auto wrapper = new RosbagDeckWrapper();
    wrapper->core = std::make_unique<RosbagDeckCore>();
    wrapper->status_callback = nullptr;
    wrapper->message_callback = nullptr;
    wrapper->status_user_data = nullptr;
    wrapper->message_user_data = nullptr;
    return static_cast<rosbag_deck_handle_t>(wrapper);
  } catch (...) {
    return nullptr;
  }
}

void rosbag_deck_destroy(rosbag_deck_handle_t handle) {
  if (handle) {
    auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
    delete wrapper;
  }
}

// Configuration functions
void rosbag_deck_set_cache_size(rosbag_deck_handle_t handle,
                                size_t cache_size) {
  if (handle) {
    auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
    wrapper->core->set_cache_size(cache_size);
  }
}

void rosbag_deck_set_preload_settings(rosbag_deck_handle_t handle, size_t ahead,
                                      size_t behind) {
  if (handle) {
    auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
    wrapper->core->set_preload_settings(ahead, behind);
  }
}

void rosbag_deck_set_playback_rate(rosbag_deck_handle_t handle, double rate) {
  if (handle) {
    auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
    wrapper->core->set_playback_rate(rate);
  }
}

void rosbag_deck_set_loop_playback(rosbag_deck_handle_t handle, bool loop) {
  if (handle) {
    auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
    wrapper->core->set_loop_playback(loop);
  }
}

// Core operations
bool rosbag_deck_build_index(rosbag_deck_handle_t handle,
                             const char **bag_paths, size_t bag_paths_count) {
  if (!handle)
    return false;

  try {
    auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
    std::vector<std::string> paths;
    for (size_t i = 0; i < bag_paths_count; ++i) {
      paths.emplace_back(bag_paths[i]);
    }
    return wrapper->core->build_index(paths);
  } catch (...) {
    return false;
  }
}

void rosbag_deck_start_playback(rosbag_deck_handle_t handle) {
  if (handle) {
    auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
    wrapper->core->start_playback();
  }
}

void rosbag_deck_stop_playback(rosbag_deck_handle_t handle) {
  if (handle) {
    auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
    wrapper->core->stop_playback();
  }
}

bool rosbag_deck_step_forward(rosbag_deck_handle_t handle) {
  if (!handle)
    return false;

  try {
    auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
    return wrapper->core->step_forward();
  } catch (...) {
    return false;
  }
}

bool rosbag_deck_step_backward(rosbag_deck_handle_t handle) {
  if (!handle)
    return false;

  try {
    auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
    return wrapper->core->step_backward();
  } catch (...) {
    return false;
  }
}

bool rosbag_deck_seek_to_time(rosbag_deck_handle_t handle,
                              rosbag_deck_timestamp_t target_time) {
  if (!handle)
    return false;

  try {
    auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
    return wrapper->core->seek_to_time(c_to_cpp_timestamp(target_time));
  } catch (...) {
    return false;
  }
}

bool rosbag_deck_seek_to_frame(rosbag_deck_handle_t handle,
                               size_t frame_index) {
  if (!handle)
    return false;

  try {
    auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
    return wrapper->core->seek_to_frame(frame_index);
  } catch (...) {
    return false;
  }
}

// Information queries
rosbag_deck_bag_info_t rosbag_deck_get_bag_info(rosbag_deck_handle_t handle) {
  rosbag_deck_bag_info_t result = {};

  if (!handle) {
    result.success = false;
    result.message = "Invalid handle";
    return result;
  }

  try {
    auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
    auto bag_info = wrapper->core->get_bag_info();

    // Store message in wrapper for memory safety
    wrapper->bag_info_message_storage = bag_info.message;

    // Store topic names in wrapper for memory safety
    wrapper->topic_names_storage.clear();
    wrapper->topic_names_ptrs.clear();
    for (const auto &topic : bag_info.topic_names) {
      wrapper->topic_names_storage.push_back(topic);
    }
    for (const auto &topic : wrapper->topic_names_storage) {
      wrapper->topic_names_ptrs.push_back(topic.c_str());
    }

    result.success = bag_info.success;
    result.message = wrapper->bag_info_message_storage.c_str();
    result.start_time = cpp_to_c_timestamp(bag_info.start_time);
    result.end_time = cpp_to_c_timestamp(bag_info.end_time);
    result.total_duration_ns = bag_info.total_duration.count();
    result.total_frames = bag_info.total_frames;
    result.topic_names = wrapper->topic_names_ptrs.data();
    result.topic_names_count = wrapper->topic_names_ptrs.size();

    return result;
  } catch (...) {
    result.success = false;
    result.message = "Exception occurred";
    return result;
  }
}

rosbag_deck_status_t rosbag_deck_get_status(rosbag_deck_handle_t handle) {
  rosbag_deck_status_t result = {};

  if (!handle)
    return result;

  try {
    auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
    auto status = wrapper->core->get_status();

    result.current_time = cpp_to_c_timestamp(status.current_time);
    result.is_playing = status.is_playing;
    result.current_frame = status.current_frame;
    result.total_frames = status.total_frames;
    result.timeline_segment = status.timeline_segment;
    result.virtual_time = cpp_to_c_timestamp(status.virtual_time);

    return result;
  } catch (...) {
    return result;
  }
}

// Callback registration
void rosbag_deck_set_status_callback(rosbag_deck_handle_t handle,
                                     rosbag_deck_status_callback_t callback,
                                     void *user_data) {
  if (!handle)
    return;

  auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
  wrapper->status_callback = callback;
  wrapper->status_user_data = user_data;

  if (callback) {
    wrapper->core->set_status_callback([wrapper](const PlaybackStatus &status) {
      status_callback_adapter(status, wrapper);
    });
  } else {
    wrapper->core->set_status_callback(nullptr);
  }
}

void rosbag_deck_set_message_callback(rosbag_deck_handle_t handle,
                                      rosbag_deck_message_callback_t callback,
                                      void *user_data) {
  if (!handle)
    return;

  auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
  wrapper->message_callback = callback;
  wrapper->message_user_data = user_data;

  if (callback) {
    wrapper->core->set_message_callback([wrapper](const BagMessage &message) {
      message_callback_adapter(message, wrapper);
    });
  } else {
    wrapper->core->set_message_callback(nullptr);
  }
}

// Utility functions
rosbag_deck_timestamp_t
rosbag_deck_to_timestamp(int64_t nanoseconds_since_epoch) {
  rosbag_deck_timestamp_t result;
  result.nanoseconds_since_epoch = nanoseconds_since_epoch;
  return result;
}

int64_t rosbag_deck_from_timestamp(rosbag_deck_timestamp_t timestamp) {
  return timestamp.nanoseconds_since_epoch;
}

// Memory management for returned data
void rosbag_deck_free_bag_info(rosbag_deck_bag_info_t *bag_info) {
  // Note: The actual strings are stored in the wrapper object and will be
  // cleaned up when the wrapper is destroyed. This function is provided
  // for API completeness and future extensibility.
  (void)bag_info; // Suppress unused parameter warning
}

void rosbag_deck_free_message(rosbag_deck_message_t *message) {
  // Note: The actual strings and data are stored in the wrapper object and will
  // be cleaned up when the wrapper is destroyed. This function is provided for
  // API completeness and future extensibility.
  (void)message; // Suppress unused parameter warning
}

// Topic and type filtering implementation
void rosbag_deck_set_topic_filter(rosbag_deck_handle_t handle,
                                  const char **topics, size_t topics_count) {
  if (!handle)
    return;

  auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
  std::vector<std::string> topic_list;
  for (size_t i = 0; i < topics_count; ++i) {
    topic_list.push_back(topics[i]);
  }
  wrapper->core->set_topic_filter(topic_list);
}

void rosbag_deck_clear_topic_filter(rosbag_deck_handle_t handle) {
  if (!handle)
    return;

  auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
  wrapper->core->clear_topic_filter();
}

void rosbag_deck_set_type_filter(rosbag_deck_handle_t handle,
                                 const char **types, size_t types_count) {
  if (!handle)
    return;

  auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
  std::vector<std::string> type_list;
  for (size_t i = 0; i < types_count; ++i) {
    type_list.push_back(types[i]);
  }
  wrapper->core->set_type_filter(type_list);
}

void rosbag_deck_clear_type_filter(rosbag_deck_handle_t handle) {
  if (!handle)
    return;

  auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
  wrapper->core->clear_type_filter();
}

char **rosbag_deck_get_available_topics(rosbag_deck_handle_t handle,
                                        size_t *count) {
  if (!handle || !count)
    return nullptr;

  auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
  auto topics = wrapper->core->get_available_topics();

  *count = topics.size();
  if (topics.empty())
    return nullptr;

  char **result = new char *[topics.size()];
  for (size_t i = 0; i < topics.size(); ++i) {
    result[i] = new char[topics[i].length() + 1];
    std::strcpy(result[i], topics[i].c_str());
  }

  return result;
}

char **rosbag_deck_get_available_types(rosbag_deck_handle_t handle,
                                       size_t *count) {
  if (!handle || !count)
    return nullptr;

  auto wrapper = static_cast<RosbagDeckWrapper *>(handle);
  auto types = wrapper->core->get_available_types();

  *count = types.size();
  if (types.empty())
    return nullptr;

  char **result = new char *[types.size()];
  for (size_t i = 0; i < types.size(); ++i) {
    result[i] = new char[types[i].length() + 1];
    std::strcpy(result[i], types[i].c_str());
  }

  return result;
}

void rosbag_deck_free_string_array(char **array, size_t count) {
  if (!array)
    return;

  for (size_t i = 0; i < count; ++i) {
    delete[] array[i];
  }
  delete[] array;
}

bool rosbag_deck_is_message_type_supported(const char *message_type) {
  if (!message_type)
    return false;
  return RosbagDeckCore::is_message_type_supported(message_type);
}
