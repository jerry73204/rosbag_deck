# RosBag Deck

A fast, interactive ROS 2 bag player written in Rust. Play, inspect, and edit rosbag2 files with tape deck-style controls, a terminal UI, and Python bindings.

## Features

- **Headless or interactive TUI** playback with real-time pacing
- **ROS 2 topic publishing** with automatic QoS matching
- **Loop modes**: restart (original timestamps) or monotonic (ever-increasing timestamps)
- **Topic filtering** by name, regex, or exclusion pattern
- **Bag editing**: slice, merge, filter, and transform timestamps
- **Python bindings** via PyO3 for scripted access
- **Streaming architecture** that handles arbitrarily large bags without loading them into memory

## Requirements

- ROS 2 Humble or later (sourced)
- Rust stable toolchain
- [just](https://github.com/casey/just) command runner

## Building

```bash
just setup    # Install colcon-cargo-ros2 (first time only)
just build    # Build all packages
```

## Quick Start

```bash
# Inspect a bag
rosbag_deck info /path/to/bag

# Play a bag (publishes to ROS 2 topics)
rosbag_deck play /path/to/bag

# Interactive TUI with topic filtering
rosbag_deck play /path/to/bag --tui --topics /camera/image /imu

# Loop with monotonically increasing timestamps
rosbag_deck play /path/to/bag --loop monotonic

# Extract 60 seconds of camera data into a new bag
rosbag_deck edit /path/to/bag -o output.db --topics /camera/image --start 0 --end 60
```

## Packages

| Package                                              | Description                                                 |
|------------------------------------------------------|-------------------------------------------------------------|
| [`rosbag_deck`](packages/rosbag_deck/)               | CLI and TUI — play, info, edit subcommands                  |
| [`rosbag_deck_python`](packages/rosbag_deck_python/) | Python bindings via PyO3                                    |
| [`rosbag_deck_core`](packages/rosbag_deck_core/)     | Core Rust library: timeline, caching, streaming, publishing |
| [`rosbag_deck_ffi`](packages/rosbag_deck_ffi/)       | C/C++ FFI to rosbag2_cpp, rclcpp, rcutils                   |

## Architecture

```
rosbag_deck_ffi          (C/C++ FFI to rosbag2_cpp, rclcpp, rcutils)
    |
rosbag_deck_core         (Rust core: timeline, caching, streaming, publishing)
    |            \
rosbag_deck     rosbag_deck_python
(CLI + TUI)     (PyO3 bindings)
```

All packages are built as ROS 2 `ament_cargo` packages via colcon.
