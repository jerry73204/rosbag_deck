#include "rosbag_deck_node/rosbag_deck_node.hpp"
#include <rclcpp/serialization.hpp>
#include <rcutils/allocator.h>
#include <cstring>

namespace rosbag_deck_node {

RosbagDeckNode::RosbagDeckNode(const rclcpp::NodeOptions &options)
    : Node("rosbag_deck", options) {
  // Declare parameters
  this->declare_parameter<std::vector<std::string>>("bag_paths",
                                                    std::vector<std::string>{});
  this->declare_parameter<double>("playback_rate", 1.0);
  this->declare_parameter<bool>("loop_playback", false);
  this->declare_parameter<int>("cache_size", 1000);
  this->declare_parameter<int>("preload_ahead", 100);
  this->declare_parameter<int>("preload_behind", 100);

  // Get parameters
  auto bag_paths = this->get_parameter("bag_paths").as_string_array();
  auto playback_rate = this->get_parameter("playback_rate").as_double();
  auto loop_playback = this->get_parameter("loop_playback").as_bool();
  auto cache_size = this->get_parameter("cache_size").as_int();
  auto preload_ahead = this->get_parameter("preload_ahead").as_int();
  auto preload_behind = this->get_parameter("preload_behind").as_int();

  // Initialize core library
  core_ = std::make_unique<rosbag_deck_core::RosbagDeckCore>();

  // Configure core library
  core_->set_cache_size(cache_size);
  core_->set_preload_settings(preload_ahead, preload_behind);
  core_->set_playback_rate(playback_rate);
  core_->set_loop_playback(loop_playback);

  // Set up callbacks
  core_->set_status_callback(std::bind(&RosbagDeckNode::on_status_update, this,
                                       std::placeholders::_1));
  core_->set_message_callback(std::bind(&RosbagDeckNode::on_message_update,
                                        this, std::placeholders::_1));

  // Create publishers
  status_publisher_ =
      this->create_publisher<rosbag_deck_interface::msg::PlaybackStatus>(
          "~/status", 10);

  // Create services
  play_pause_service_ = this->create_service<std_srvs::srv::SetBool>(
      "~/play_pause", std::bind(&RosbagDeckNode::play_pause_callback, this,
                                std::placeholders::_1, std::placeholders::_2));

  step_forward_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/step_forward",
      std::bind(&RosbagDeckNode::step_forward_callback, this,
                std::placeholders::_1, std::placeholders::_2));

  step_backward_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/step_backward",
      std::bind(&RosbagDeckNode::step_backward_callback, this,
                std::placeholders::_1, std::placeholders::_2));

  get_bag_info_service_ =
      this->create_service<rosbag_deck_interface::srv::GetBagInfo>(
          "~/get_bag_info",
          std::bind(&RosbagDeckNode::get_bag_info_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  seek_to_time_service_ =
      this->create_service<rosbag_deck_interface::srv::SeekToTime>(
          "~/seek_to_time",
          std::bind(&RosbagDeckNode::seek_to_time_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  // Build index if bag paths provided
  if (!bag_paths.empty()) {
    if (core_->build_index(bag_paths)) {
      auto bag_info = core_->get_bag_info();
      RCLCPP_INFO(this->get_logger(),
                  "Successfully indexed %zu bags with %zu total frames",
                  bag_paths.size(), bag_info.total_frames);

      // Create generic publishers for all topics with their message types
      // Get topic types from the index manager
      const auto& index_topic_types = core_->get_index_manager().topic_types();
      for (const auto &topic_name : bag_info.topic_names) {
        auto type_it = index_topic_types.find(topic_name);
        if (type_it != index_topic_types.end()) {
          publishers_[topic_name] = this->create_generic_publisher(
              topic_name, type_it->second, rclcpp::SystemDefaultsQoS());
          RCLCPP_INFO(this->get_logger(), 
                      "Created publisher for topic: %s [type: %s]",
                      topic_name.c_str(), type_it->second.c_str());
        }
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to build index");
    }
  }
}

RosbagDeckNode::~RosbagDeckNode() {
  if (core_) {
    core_->stop_playback();
  }
}

void RosbagDeckNode::play_pause_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  if (request->data) {
    core_->start_playback();
    response->message = "Playback started";
  } else {
    core_->stop_playback();
    response->message = "Playback paused";
  }
  response->success = true;
}

void RosbagDeckNode::step_forward_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (core_->step_forward()) {
    auto status = core_->get_status();
    response->message =
        "Stepped forward to frame " + std::to_string(status.current_frame);
    response->success = true;
  } else {
    response->message = "Failed to step forward";
    response->success = false;
  }
}

void RosbagDeckNode::step_backward_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (core_->step_backward()) {
    auto status = core_->get_status();
    response->message =
        "Stepped backward to frame " + std::to_string(status.current_frame);
    response->success = true;
  } else {
    response->message = "Failed to step backward";
    response->success = false;
  }
}

void RosbagDeckNode::get_bag_info_callback(
    const std::shared_ptr<
        rosbag_deck_interface::srv::GetBagInfo::Request> /*request*/,
    std::shared_ptr<rosbag_deck_interface::srv::GetBagInfo::Response>
        response) {
  auto bag_info = core_->get_bag_info();

  response->success = bag_info.success;
  response->message = bag_info.message;
  response->start_time = core_to_ros_time(bag_info.start_time);
  response->end_time = core_to_ros_time(bag_info.end_time);

  // Convert duration
  auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         bag_info.total_duration)
                         .count();
  response->total_duration.sec =
      static_cast<int32_t>(duration_ns / 1000000000LL);
  response->total_duration.nanosec =
      static_cast<uint32_t>(duration_ns % 1000000000LL);

  response->total_frames = bag_info.total_frames;
  response->topic_names = bag_info.topic_names;
}

void RosbagDeckNode::seek_to_time_callback(
    const std::shared_ptr<rosbag_deck_interface::srv::SeekToTime::Request>
        request,
    std::shared_ptr<rosbag_deck_interface::srv::SeekToTime::Response>
        response) {
  auto target_time = ros_to_core_time(rclcpp::Time(request->target_time));

  if (core_->seek_to_time(target_time)) {
    auto status = core_->get_status();
    response->success = true;
    response->message =
        "Seeked to frame " + std::to_string(status.current_frame);
    response->actual_frame = status.current_frame;
    response->actual_time = core_to_ros_time(status.current_time);
  } else {
    response->success = false;
    response->message = "Failed to seek to target time";
    auto status = core_->get_status();
    response->actual_frame = status.current_frame;
    response->actual_time = core_to_ros_time(status.current_time);
  }
}

void RosbagDeckNode::on_status_update(
    const rosbag_deck_core::PlaybackStatus &status) {
  auto ros_status = rosbag_deck_interface::msg::PlaybackStatus();
  ros_status.stamp = this->now();
  ros_status.is_playing = status.is_playing;
  ros_status.current_frame = status.current_frame;
  ros_status.timeline_segment = status.timeline_segment;
  ros_status.current_time = core_to_ros_time(status.current_time);
  ros_status.virtual_time = core_to_ros_time(status.virtual_time);

  status_publisher_->publish(ros_status);
}

void RosbagDeckNode::on_message_update(
    const rosbag_deck_core::BagMessage &message) {
  auto it = publishers_.find(message.topic_name);
  if (it != publishers_.end() && message.serialized_data) {
    // Publish the raw serialized message using generic publisher
    rclcpp::SerializedMessage serialized_msg;
    serialized_msg.reserve(message.serialized_data->size());
    
    // Copy the serialized data
    auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
    rcl_msg.buffer_length = message.serialized_data->size();
    rcl_msg.buffer_capacity = message.serialized_data->size();
    rcl_msg.buffer = reinterpret_cast<uint8_t*>(
        rcutils_get_default_allocator().allocate(
            rcl_msg.buffer_length, rcutils_get_default_allocator().state));
    
    if (rcl_msg.buffer != nullptr) {
      std::memcpy(rcl_msg.buffer, message.serialized_data->data(), 
                  message.serialized_data->size());
      
      // Update timestamp in the serialized message if possible
      // Note: This is a simplified approach. In a real implementation,
      // you might need to deserialize, update header, and re-serialize
      
      it->second->publish(serialized_msg);
    } else {
      RCLCPP_ERROR(this->get_logger(), 
                   "Failed to allocate memory for serialized message");
    }
  }
}

rclcpp::Time
RosbagDeckNode::core_to_ros_time(const rosbag_deck_core::Timestamp &timestamp) {
  auto ns_since_epoch =
      rosbag_deck_core::RosbagDeckCore::from_timestamp(timestamp);
  return rclcpp::Time(ns_since_epoch);
}

rosbag_deck_core::Timestamp
RosbagDeckNode::ros_to_core_time(const rclcpp::Time &ros_time) {
  return rosbag_deck_core::RosbagDeckCore::to_timestamp(ros_time.nanoseconds());
}

} // namespace rosbag_deck_node
