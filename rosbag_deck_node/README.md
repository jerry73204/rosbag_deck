# rosbag_deck_node

ROS 2 node implementation for the RosBag Deck interactive bag player.

## Overview

This package provides the ROS 2 node that exposes the RosBag Deck functionality through ROS topics and services. It wraps the `rosbag_deck_core` library and provides the interface for controlling bag playback.

## Node: rosbag_deck_node

### Published Topics

- `~/status` ([rosbag_deck_interface/PlaybackStatus](../rosbag_deck_interface/README.md#playbackstatusmsg)) - Playback status at 10Hz
- Bag topics are republished with their original names and types

### Services

- `~/play_pause` (std_srvs/SetBool) - Start/stop playback
- `~/step_forward` (std_srvs/Trigger) - Step forward one frame
- `~/step_backward` (std_srvs/Trigger) - Step backward one frame
- `~/seek_to_time` ([rosbag_deck_interface/SeekToTime](../rosbag_deck_interface/README.md#seektotimesrv)) - Seek to specific time
- `~/get_bag_info` ([rosbag_deck_interface/GetBagInfo](../rosbag_deck_interface/README.md#getbaginfosrv)) - Get bag metadata

### Parameters

```yaml
# Required
bag_paths: []                    # List of bag file paths to load

# Optional
playback_rate: 1.0              # Playback speed multiplier (default: 1.0)
loop_playback: false            # Enable loop playback (default: false)
cache_size: 1000                # Max messages in cache (default: 1000)
preload_ahead: 100              # Messages to preload ahead (default: 100)
preload_behind: 100             # Messages to keep behind (default: 100)
```

## Usage Examples

### Basic Usage

```bash
ros2 run rosbag_deck_node rosbag_deck_node \
  --ros-args \
  -p bag_paths:="['/path/to/bag1.db3', '/path/to/bag2.db3']"
```

### With Custom Parameters

```bash
ros2 run rosbag_deck_node rosbag_deck_node \
  --ros-args \
  -p bag_paths:="['/data/lidar1.db3', '/data/lidar2.db3']" \
  -p cache_size:=2000 \
  -p preload_ahead:=200 \
  -p playback_rate:=0.5
```

### Using a Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbag_deck_node',
            executable='rosbag_deck_node',
            name='rosbag_deck',
            parameters=[{
                'bag_paths': ['/path/to/bag1.db3', '/path/to/bag2.db3'],
                'cache_size': 2000,
                'preload_ahead': 200,
                'playback_rate': 1.0,
                'loop_playback': False
            }],
            output='screen'
        )
    ])
```

## Service Call Examples

### Start Playback
```bash
ros2 service call /rosbag_deck/play_pause std_srvs/srv/SetBool "data: true"
```

### Pause Playback
```bash
ros2 service call /rosbag_deck/play_pause std_srvs/srv/SetBool "data: false"
```

### Step Forward
```bash
ros2 service call /rosbag_deck/step_forward std_srvs/srv/Trigger
```

### Step Backward
```bash
ros2 service call /rosbag_deck/step_backward std_srvs/srv/Trigger
```

### Seek to Time
```bash
ros2 service call /rosbag_deck/seek_to_time rosbag_deck_interface/srv/SeekToTime \
  "{target_time: {sec: 1234567890, nanosec: 500000000}}"
```

### Get Bag Information
```bash
ros2 service call /rosbag_deck/get_bag_info rosbag_deck_interface/srv/GetBagInfo
```

## Monitoring Status

```bash
# Watch playback status
ros2 topic echo /rosbag_deck/status

# Check status update rate
ros2 topic hz /rosbag_deck/status
```

## Implementation Details

### Architecture

The node consists of:
- **Main Node Class**: Handles ROS 2 interface and parameter management
- **Service Handlers**: Process service requests and forward to core library
- **Status Publisher**: Publishes status at 10Hz using a timer
- **Message Publisher**: Republishes bag messages maintaining original timestamps

### Threading Model

- Main thread: ROS 2 executor and service handling
- Timer thread: Status publishing at 10Hz
- BagWorker thread: Background I/O (from core library)

### Virtual Timeline Handling

When rewinding, the node maintains a virtual timeline to ensure monotonic timestamps:
- Increments timeline segment on each rewind
- Modifies frame IDs to include segment: `lidar1` → `lidar1_seg_0`
- Adjusts timestamps to maintain forward progression

## Building

```bash
cd /path/to/rosbag_deck
colcon build --packages-select rosbag_deck_node
```

## Dependencies

- rosbag_deck_core
- rosbag_deck_interface
- rclcpp
- std_srvs
- sensor_msgs (for PointCloud2 republishing)

## Troubleshooting

### Node doesn't start
- Check that bag_paths parameter is provided
- Verify bag files exist and are readable
- Check for error messages about missing dependencies

### Playback is choppy
- Increase cache_size parameter
- Increase preload_ahead parameter
- Check CPU usage and system resources

### Status not updating
- Verify the node is running: `ros2 node list`
- Check status topic: `ros2 topic list | grep status`
- Look for errors in node output