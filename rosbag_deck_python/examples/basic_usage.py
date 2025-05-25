#!/usr/bin/env python3
"""
Basic usage example for RosbagDeck Python API
"""

import sys
import time
from rosbag_deck import RosbagDeck, Status, Message


def on_status_update(status: Status):
    """Status callback function"""
    print(
        f"Status: Playing={status.is_playing}, "
        f"Frame={status.current_frame}/{status.total_frames}, "
        f"Time={status.current_time} ns"
    )


def on_message_update(message: Message):
    """Message callback function"""
    print(
        f"Message: Topic={message.topic_name}, "
        f"Type={message.message_type}, "
        f"Size={len(message.serialized_data)} bytes, "
        f"Frame={message.frame_index}"
    )


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <bag_file1> [bag_file2] [...]")
        return 1

    bag_files = sys.argv[1:]

    try:
        # Create RosbagDeck with context manager for automatic cleanup
        with RosbagDeck() as deck:
            # Configure settings
            deck.set_cache_size(100)
            deck.set_preload_settings(ahead=10, behind=10)
            deck.set_playback_rate(1.0)
            deck.set_loop_playback(False)

            # Register callbacks
            deck.set_status_callback(on_status_update)
            deck.set_message_callback(on_message_update)

            # Build index from bag files
            print(f"Building index from {len(bag_files)} bag files...")
            deck.build_index(bag_files)

            # Get bag info
            bag_info = deck.get_bag_info()
            if bag_info.success:
                print("\nBag Info:")
                print(f"  Total frames: {bag_info.total_frames}")
                print(f"  Duration: {bag_info.total_duration_ns / 1e9:.2f} seconds")
                print(f"  Topics: {len(bag_info.topic_names)}")
                for topic in bag_info.topic_names:
                    print(f"    - {topic}")
            else:
                print(f"Failed to get bag info: {bag_info.message}")
                return 1

            # Demonstrate manual stepping
            print("\nManual stepping through first 5 frames:")
            for i in range(5):
                if not deck.step_forward():
                    print("Failed to step forward")
                    break
                time.sleep(0.1)

            # Demonstrate seeking
            print("\nSeeking to frame 10:")
            if not deck.seek_to_frame(10):
                print("Failed to seek to frame 10")

            # Demonstrate time-based seeking
            seek_time = bag_info.start_time + int(1e9)  # +1 second
            print(f"\nSeeking to 1 second after start:")
            if not deck.seek_to_time(seek_time):
                print("Failed to seek to time")

            # Start playback for 3 seconds
            print("\nStarting playback for 3 seconds:")
            deck.start_playback()
            time.sleep(3)
            deck.stop_playback()

            print("\nDemo complete")

    except Exception as e:
        print(f"Error: {e}")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
