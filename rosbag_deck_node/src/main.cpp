#include "rosbag_deck_node/rosbag_deck_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rosbag_deck_node::RosbagDeckNode>();

  RCLCPP_INFO(node->get_logger(), "RosBag Deck node started");
  RCLCPP_INFO(node->get_logger(), "Available services:");
  RCLCPP_INFO(node->get_logger(),
              "  ~/play_pause (std_srvs/SetBool) - Start/stop playback");
  RCLCPP_INFO(node->get_logger(),
              "  ~/step_forward (std_srvs/Trigger) - Step forward one frame");
  RCLCPP_INFO(node->get_logger(),
              "  ~/step_backward (std_srvs/Trigger) - Step backward one frame");
  RCLCPP_INFO(
      node->get_logger(),
      "  ~/get_bag_info (rosbag_deck_interface/GetBagInfo) - Get bag metadata");
  RCLCPP_INFO(node->get_logger(),
              "  ~/seek_to_time (rosbag_deck_interface/SeekToTime) - Seek to "
              "specific timestamp");
  RCLCPP_INFO(node->get_logger(), "Available topics:");
  RCLCPP_INFO(node->get_logger(),
              "  ~/status (rosbag_deck_interface/PlaybackStatus) - Current "
              "playback status");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
