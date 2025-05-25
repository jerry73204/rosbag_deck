#ifndef ROSBAG_DECK_C_H
#define ROSBAG_DECK_C_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle types
typedef void* rosbag_deck_handle_t;

// C-compatible structs
typedef struct {
    int64_t nanoseconds_since_epoch;
} rosbag_deck_timestamp_t;

typedef struct {
    rosbag_deck_timestamp_t original_timestamp;
    rosbag_deck_timestamp_t virtual_timestamp;
    const char* topic_name;
    const char* message_type;
    const uint8_t* serialized_data;
    size_t serialized_data_size;
    size_t frame_index;
} rosbag_deck_message_t;

typedef struct {
    rosbag_deck_timestamp_t current_time;
    bool is_playing;
    size_t current_frame;
    size_t total_frames;
    size_t timeline_segment;
    rosbag_deck_timestamp_t virtual_time;
} rosbag_deck_status_t;

typedef struct {
    bool success;
    const char* message;
    rosbag_deck_timestamp_t start_time;
    rosbag_deck_timestamp_t end_time;
    int64_t total_duration_ns;
    size_t total_frames;
    const char** topic_names;
    size_t topic_names_count;
} rosbag_deck_bag_info_t;

// Callback function types
typedef void (*rosbag_deck_status_callback_t)(const rosbag_deck_status_t* status, void* user_data);
typedef void (*rosbag_deck_message_callback_t)(const rosbag_deck_message_t* message, void* user_data);

// Core lifecycle functions
rosbag_deck_handle_t rosbag_deck_create(void);
void rosbag_deck_destroy(rosbag_deck_handle_t handle);

// Configuration functions
void rosbag_deck_set_cache_size(rosbag_deck_handle_t handle, size_t cache_size);
void rosbag_deck_set_preload_settings(rosbag_deck_handle_t handle, size_t ahead, size_t behind);
void rosbag_deck_set_playback_rate(rosbag_deck_handle_t handle, double rate);
void rosbag_deck_set_loop_playback(rosbag_deck_handle_t handle, bool loop);

// Core operations
bool rosbag_deck_build_index(rosbag_deck_handle_t handle, const char** bag_paths, size_t bag_paths_count);
void rosbag_deck_start_playback(rosbag_deck_handle_t handle);
void rosbag_deck_stop_playback(rosbag_deck_handle_t handle);
bool rosbag_deck_step_forward(rosbag_deck_handle_t handle);
bool rosbag_deck_step_backward(rosbag_deck_handle_t handle);
bool rosbag_deck_seek_to_time(rosbag_deck_handle_t handle, rosbag_deck_timestamp_t target_time);
bool rosbag_deck_seek_to_frame(rosbag_deck_handle_t handle, size_t frame_index);

// Information queries
rosbag_deck_bag_info_t rosbag_deck_get_bag_info(rosbag_deck_handle_t handle);
rosbag_deck_status_t rosbag_deck_get_status(rosbag_deck_handle_t handle);

// Callback registration
void rosbag_deck_set_status_callback(rosbag_deck_handle_t handle, 
                                   rosbag_deck_status_callback_t callback, 
                                   void* user_data);
void rosbag_deck_set_message_callback(rosbag_deck_handle_t handle, 
                                    rosbag_deck_message_callback_t callback, 
                                    void* user_data);

// Utility functions
rosbag_deck_timestamp_t rosbag_deck_to_timestamp(int64_t nanoseconds_since_epoch);
int64_t rosbag_deck_from_timestamp(rosbag_deck_timestamp_t timestamp);

// Memory management for returned data
void rosbag_deck_free_bag_info(rosbag_deck_bag_info_t* bag_info);
void rosbag_deck_free_message(rosbag_deck_message_t* message);

#ifdef __cplusplus
}
#endif

#endif // ROSBAG_DECK_C_H