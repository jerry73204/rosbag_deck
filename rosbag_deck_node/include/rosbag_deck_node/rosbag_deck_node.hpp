#pragma once

#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include "rosbag_deck_interface/msg/playback_status.hpp"
#include "rosbag_deck_interface/srv/get_bag_info.hpp"
#include "rosbag_deck_interface/srv/seek_to_time.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace rosbag_deck_node {

class RosbagDeckNode : public rclcpp::Node {
public:
  explicit RosbagDeckNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~RosbagDeckNode();

private:
  // Service callbacks
  void play_pause_callback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void step_forward_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void step_backward_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void get_bag_info_callback(
      const std::shared_ptr<rosbag_deck_interface::srv::GetBagInfo::Request>
          request,
      std::shared_ptr<rosbag_deck_interface::srv::GetBagInfo::Response>
          response);

  void seek_to_time_callback(
      const std::shared_ptr<rosbag_deck_interface::srv::SeekToTime::Request>
          request,
      std::shared_ptr<rosbag_deck_interface::srv::SeekToTime::Response>
          response);

  // Core library callbacks
  void on_status_update(const rosbag_deck_core::PlaybackStatus &status);
  void on_message_update(const rosbag_deck_core::BagMessage &message);

  // Utility functions
  rclcpp::Time core_to_ros_time(const rosbag_deck_core::Timestamp &timestamp);
  rosbag_deck_core::Timestamp ros_to_core_time(const rclcpp::Time &ros_time);

  // Core library instance
  std::unique_ptr<rosbag_deck_core::RosbagDeckCore> core_;

  // ROS publishers
  std::unordered_map<
      std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
      publishers_;
  rclcpp::Publisher<rosbag_deck_interface::msg::PlaybackStatus>::SharedPtr
      status_publisher_;

  // ROS services
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr play_pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr step_forward_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr step_backward_service_;
  rclcpp::Service<rosbag_deck_interface::srv::GetBagInfo>::SharedPtr
      get_bag_info_service_;
  rclcpp::Service<rosbag_deck_interface::srv::SeekToTime>::SharedPtr
      seek_to_time_service_;
};

} // namespace rosbag_deck_node
