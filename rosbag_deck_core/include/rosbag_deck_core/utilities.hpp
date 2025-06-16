#pragma once

#include "types.hpp"
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/serialization.hpp>

namespace rosbag_deck_core {

// Utility functions
Timestamp to_timestamp(int64_t nanoseconds_since_epoch);
int64_t from_timestamp(const Timestamp &timestamp);

// Template specializations for message serialization/deserialization
template <typename MessageT>
std::shared_ptr<std::vector<uint8_t>>
serialize_message(const MessageT &message);

template <typename MessageT>
MessageT deserialize_message(const std::vector<uint8_t> &serialized_data);

bool is_message_type_supported(const std::string &message_type);

} // namespace rosbag_deck_core