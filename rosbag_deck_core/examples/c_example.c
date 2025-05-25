#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "rosbag_deck_core/rosbag_deck_c.h"

// Status callback function
void on_status_update(const rosbag_deck_status_t* status, void* user_data) {
    printf("Status: Playing=%d, Frame=%zu/%zu, Time=%ld ns\n",
           status->is_playing,
           status->current_frame,
           status->total_frames,
           status->current_time.nanoseconds_since_epoch);
}

// Message callback function
void on_message_update(const rosbag_deck_message_t* message, void* user_data) {
    printf("Message: Topic=%s, Type=%s, Size=%zu bytes, Frame=%zu\n",
           message->topic_name,
           message->message_type,
           message->serialized_data_size,
           message->frame_index);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printf("Usage: %s <bag_file1> [bag_file2] [...]\n", argv[0]);
        return 1;
    }

    // Create rosbag deck instance
    rosbag_deck_handle_t deck = rosbag_deck_create();
    if (!deck) {
        printf("Failed to create rosbag deck\n");
        return 1;
    }

    // Configure settings
    rosbag_deck_set_cache_size(deck, 100);
    rosbag_deck_set_preload_settings(deck, 10, 10);
    rosbag_deck_set_playback_rate(deck, 1.0);
    rosbag_deck_set_loop_playback(deck, false);

    // Register callbacks
    rosbag_deck_set_status_callback(deck, on_status_update, NULL);
    rosbag_deck_set_message_callback(deck, on_message_update, NULL);

    // Build index from bag files
    const char** bag_paths = (const char**)&argv[1];
    size_t bag_count = argc - 1;
    
    printf("Building index from %zu bag files...\n", bag_count);
    if (!rosbag_deck_build_index(deck, bag_paths, bag_count)) {
        printf("Failed to build index\n");
        rosbag_deck_destroy(deck);
        return 1;
    }

    // Get bag info
    rosbag_deck_bag_info_t bag_info = rosbag_deck_get_bag_info(deck);
    if (bag_info.success) {
        printf("Bag Info:\n");
        printf("  Total frames: %zu\n", bag_info.total_frames);
        printf("  Duration: %ld ns\n", bag_info.total_duration_ns);
        printf("  Topics: %zu\n", bag_info.topic_names_count);
        for (size_t i = 0; i < bag_info.topic_names_count; ++i) {
            printf("    - %s\n", bag_info.topic_names[i]);
        }
    } else {
        printf("Failed to get bag info: %s\n", bag_info.message);
        rosbag_deck_destroy(deck);
        return 1;
    }

    // Demonstrate manual stepping
    printf("\nManual stepping through first 5 frames:\n");
    for (int i = 0; i < 5; ++i) {
        if (!rosbag_deck_step_forward(deck)) {
            printf("Failed to step forward\n");
            break;
        }
        usleep(100000); // 100ms delay
    }

    // Demonstrate seeking
    printf("\nSeeking to frame 10:\n");
    if (!rosbag_deck_seek_to_frame(deck, 10)) {
        printf("Failed to seek to frame 10\n");
    }

    // Demonstrate time-based seeking
    rosbag_deck_timestamp_t seek_time = rosbag_deck_to_timestamp(bag_info.start_time.nanoseconds_since_epoch + 1000000000); // +1 second
    printf("\nSeeking to 1 second after start:\n");
    if (!rosbag_deck_seek_to_time(deck, seek_time)) {
        printf("Failed to seek to time\n");
    }

    // Start playback for 3 seconds
    printf("\nStarting playback for 3 seconds:\n");
    rosbag_deck_start_playback(deck);
    sleep(3);
    rosbag_deck_stop_playback(deck);

    printf("\nDemo complete\n");

    // Clean up
    rosbag_deck_destroy(deck);
    return 0;
}