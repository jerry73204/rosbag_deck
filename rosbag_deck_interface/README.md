# rosbag_deck_interface

ROS 2 interface definitions for the RosBag Deck system.

## Overview

This package contains the message and service definitions used by the RosBag Deck system for communication between nodes and client applications.

## Message Definitions

### PlaybackStatus.msg

Real-time playback status published at 10Hz on the `~/status` topic.

```
# Current playback state
bool is_playing
bool is_paused

# Timing information
builtin_interfaces/Time current_time
builtin_interfaces/Time start_time
builtin_interfaces/Time end_time

# Frame information
int64 current_frame
int64 total_frames

# Playback settings
float64 playback_rate
bool loop_enabled

# Cache statistics
int32 cache_size
int32 messages_cached
int32 cache_hits
int32 cache_misses

# Virtual timeline info
int32 timeline_segment
```

## Service Definitions

### GetBagInfo.srv

Query metadata about loaded bag files.

**Request:**
```
# Empty request
```

**Response:**
```
# Bag file information
string[] bag_paths
string[] topics
builtin_interfaces/Time start_time
builtin_interfaces/Time end_time
int64 total_messages
int64 total_frames
```

### SeekToTime.srv

Seek to a specific timestamp in the bag files.

**Request:**
```
# Target timestamp to seek to
builtin_interfaces/Time target_time
```

**Response:**
```
# Whether seek was successful
bool success
string message

# Actual time seeked to (may differ if target is out of bounds)
builtin_interfaces/Time actual_time
int64 frame_number
```

## Usage in Other Packages

### C++ Include

```cpp
#include <rosbag_deck_interface/msg/playback_status.hpp>
#include <rosbag_deck_interface/srv/get_bag_info.hpp>
#include <rosbag_deck_interface/srv/seek_to_time.hpp>
```

### Python Import

```python
from rosbag_deck_interface.msg import PlaybackStatus
from rosbag_deck_interface.srv import GetBagInfo, SeekToTime
```

### Rust Usage

This package automatically generates Rust bindings that can be used in Rust nodes:

```rust
use rosbag_deck_interface::msg::PlaybackStatus;
use rosbag_deck_interface::srv::{GetBagInfo, SeekToTime};
```

## Building

This package is built as part of the RosBag Deck system:

```bash
cd /path/to/rosbag_deck
colcon build --packages-select rosbag_deck_interface
```

## Interface Design Notes

### Virtual Timeline Segment

The `timeline_segment` field in PlaybackStatus is incremented each time the player rewinds to maintain monotonic timestamps for downstream nodes. This prevents issues with nodes that expect strictly increasing timestamps.

### Frame-based Navigation

The system uses frame numbers for precise navigation, where each frame corresponds to a synchronized set of messages across topics at a specific timestamp.

### Cache Statistics

The cache statistics in PlaybackStatus provide insight into the streaming system's performance, helping users tune cache parameters for optimal performance.