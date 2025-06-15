#include "rosbag_deck_core/rosbag_deck_core.hpp"
#include <iostream>
#include <string>
#include <vector>

using namespace rosbag_deck_core;

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <bag_file_path> [topic_filter1] [topic_filter2] ..."
              << std::endl;
    return 1;
  }

  // Create core instance
  RosbagDeckCore deck;

  // Configure playback
  deck.set_cache_size(500);
  deck.set_playback_rate(1.0);

  // Build index
  std::vector<std::string> bag_paths = {argv[1]};
  if (!deck.build_index(bag_paths)) {
    std::cerr << "Failed to build index" << std::endl;
    return 1;
  }

  // Get available topics and types
  auto topics = deck.get_available_topics();
  auto types = deck.get_available_types();

  std::cout << "\nAvailable topics:" << std::endl;
  for (const auto &topic : topics) {
    std::cout << "  - " << topic << std::endl;
  }

  std::cout << "\nAvailable message types:" << std::endl;
  for (const auto &type : types) {
    std::cout << "  - " << type << std::endl;
  }

  // Apply topic filter if provided
  if (argc > 2) {
    std::vector<std::string> topic_filter;
    for (int i = 2; i < argc; ++i) {
      topic_filter.push_back(argv[i]);
    }
    std::cout << "\nApplying topic filter:" << std::endl;
    for (const auto &topic : topic_filter) {
      std::cout << "  - " << topic << std::endl;
    }
    deck.set_topic_filter(topic_filter);
  }

  // Set up message callback to demonstrate dynamic message handling
  int message_count = 0;
  deck.set_message_callback([&message_count](const BagMessage &msg) {
    std::cout << "\nMessage " << ++message_count << ":" << std::endl;
    std::cout << "  Topic: " << msg.topic_name << std::endl;
    std::cout << "  Type: " << msg.message_type << std::endl;
    std::cout << "  Size: " << msg.serialized_data->size() << " bytes"
              << std::endl;
    std::cout << "  Format: " << msg.serialization_format << std::endl;

    // Show metadata if available
    if (!msg.metadata.empty()) {
      std::cout << "  Metadata:" << std::endl;
      for (const auto &[key, value] : msg.metadata) {
        std::cout << "    " << key << ": " << value << std::endl;
      }
    }

    // Stop after 10 messages for demo
    if (message_count >= 10) {
      std::cout << "\n[Demo limit reached - stopping playback]" << std::endl;
      // Note: Can't stop from callback directly, would need external control
    }
  });

  // Get bag info
  auto info = deck.get_bag_info();
  std::cout << "\nBag info:" << std::endl;
  std::cout << "  Total frames: " << info.total_frames << std::endl;
  std::cout << "  Duration: " << info.total_duration.count() / 1e9 << " seconds"
            << std::endl;

  // Play first 10 messages
  std::cout << "\nPlaying first 10 messages..." << std::endl;
  for (size_t i = 0; i < 10 && i < info.total_frames; ++i) {
    if (!deck.step_forward()) {
      std::cerr << "Failed to step forward at frame " << i << std::endl;
      break;
    }
  }

  std::cout << "\nDynamic message type support demonstration complete!"
            << std::endl;

  return 0;
}
