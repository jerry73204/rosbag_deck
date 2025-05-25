# rosbag_deck_core

Core C++ library for the RosBag Deck system, providing efficient bag file reading and caching functionality.

## Overview

This library implements the core functionality for reading and managing ROS bag files with a focus on memory efficiency and performance. It provides:

- Lightweight indexing of bag file contents
- LRU cache for message management
- Background I/O operations for non-blocking access
- Virtual timeline system for handling rewind operations

## Architecture Components

### IndexManager
- Builds lightweight metadata index on startup
- Provides O(log n) timestamp → frame lookups
- Tracks bag file positions for efficient seeking
- Memory usage: ~100 bytes per message

### MessageCache (LRU)
- Configurable cache size (default: 1000 messages)
- Window-based eviction around current playback position
- Thread-safe operations with mutex protection
- Intelligent preloading based on playback direction

### BagWorker
- Background thread for bag I/O operations
- Request queue with futures for async operation
- Handles seeking and message loading
- Maintains open bag readers for efficiency

### Virtual Timeline System
- Maintains monotonic timestamps during rewind operations
- Timeline segments prevent downstream node breakage
- Frame IDs include segment info: `lidar1_seg_0`, `lidar1_seg_1`

## C++ API

### Headers

```cpp
#include <rosbag_deck_core/rosbag_deck_core.hpp>
```

### Basic Usage

```cpp
// Create bag reader
auto reader = std::make_unique<rosbag_deck::BagReader>();

// Configure cache
reader->setCacheSize(2000);
reader->setPreloadAhead(200);
reader->setPreloadBehind(100);

// Load bag files
std::vector<std::string> bag_paths = {
    "/path/to/bag1.db3",
    "/path/to/bag2.db3"
};
reader->loadBags(bag_paths);

// Play/pause control
reader->play();
reader->pause();

// Frame navigation
reader->stepForward();
reader->stepBackward();

// Seek to time
rclcpp::Time target_time(1234567890, 500000000);
reader->seekToTime(target_time);

// Get current status
auto status = reader->getStatus();
```

## C API

For interfacing with other languages:

```c
#include <rosbag_deck_core/rosbag_deck_c.h>

// Create reader
void* reader = rosbag_deck_create_reader();

// Load bags
const char* paths[] = {"/path/to/bag1.db3", "/path/to/bag2.db3"};
rosbag_deck_load_bags(reader, paths, 2);

// Control playback
rosbag_deck_play(reader);
rosbag_deck_pause(reader);
rosbag_deck_step_forward(reader);

// Cleanup
rosbag_deck_destroy_reader(reader);
```

## Performance Characteristics

### Memory Usage
- **Index**: ~100 bytes per message (timestamps + metadata)
- **Cache**: ~1MB per cached message (configurable limit)
- **Total**: For 1M messages with 1000 message cache ≈ 100MB + 1GB = ~1.1GB

### Performance Optimizations
- **Spatial Locality**: Cache consecutive frame ranges
- **Temporal Locality**: Preload based on playback direction
- **Binary Search**: O(log n) timestamp lookups
- **Background I/O**: Non-blocking message loading
- **Reader Reuse**: Keep bag readers open for efficiency

## Building

This package is built as part of the RosBag Deck system:

```bash
cd /path/to/rosbag_deck
colcon build --packages-select rosbag_deck_core
```

## Dependencies

- ROS 2 (Humble or later)
- rosbag2_cpp
- rclcpp (for time types)
- sensor_msgs (for PointCloud2)

## Examples

See the `examples/` directory for:
- `c_example.c` - Demonstrates C API usage

## Implementation Notes

### Thread Safety
- All public methods are thread-safe
- Cache operations are protected by mutex
- Background worker uses lock-free queue for requests

### Current Limitations
- Sequential seeking reopens bags for random access
- Topic types hardcoded for sensor_msgs/PointCloud2
- Single-threaded cache operations

### Future Improvements
- Indexed seeking using bag file indices
- Smart reader positioning
- Dynamic topic type detection
- Adaptive cache sizing