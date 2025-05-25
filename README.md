# RosBag Deck

Interactive ROS 2 bag player with tape deck-style controls and streaming architecture for large bag files.

## Overview

RosBag Deck provides an interactive interface for playing, pausing, and navigating through ROS bag files with frame-level precision. It features a memory-efficient streaming architecture that can handle arbitrarily large bag files without loading everything into memory.

## Package Structure

The RosBag Deck system consists of five packages:

- **[rosbag_deck_interface](./rosbag_deck_interface/)** - ROS 2 message and service definitions
- **[rosbag_deck_core](./rosbag_deck_core/)** - Core C++ library with bag reading and caching functionality
- **[rosbag_deck_node](./rosbag_deck_node/)** - ROS 2 node implementation
- **[rosbag_deck_python](./rosbag_deck_python/)** - Python bindings for the core library
- **[rosbag_deck_tui](./rosbag_deck_tui/)** - Terminal User Interface for interactive control

## Quick Start

### Building

```bash
cd /path/to/rosbag_deck
colcon build
source install/setup.bash
```

### Running the Node

```bash
ros2 run rosbag_deck_node rosbag_deck_node \
  --ros-args \
  -p bag_paths:="['/path/to/bag1.db3', '/path/to/bag2.db3']"
```

### Using the TUI

```bash
ros2 run rosbag_deck_tui rosbag_deck_tui
```

## Key Features

- **Interactive Controls**: Play, pause, step forward/backward, seek to time
- **Memory Efficient**: Streaming architecture with LRU cache
- **Multi-bag Support**: Load and synchronize multiple bag files
- **Python Bindings**: Access core functionality from Python
- **Terminal UI**: Interactive control interface

## Documentation

For detailed information about each package:

- [Core Library Documentation](./rosbag_deck_core/README.md)
- [Interface Definitions](./rosbag_deck_interface/README.md)
- [ROS 2 Node Documentation](./rosbag_deck_node/README.md)
- [Python Bindings Documentation](./rosbag_deck_python/README.md)
- [Terminal UI Documentation](./rosbag_deck_tui/README.md)

## License

This project is part of the LCTK (LiDAR and Camera Toolkit) and follows the same license terms.