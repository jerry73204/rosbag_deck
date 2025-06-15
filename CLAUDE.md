# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RosBag Deck is an interactive ROS 2 bag player with tape deck-style controls. It provides frame-level precision playback and handles large bag files efficiently through a streaming architecture.

## Build Commands

### Full Build (All Packages)
```bash
colcon build
```

### Individual Package Builds
```bash
# Core C++ library
colcon build --packages-select rosbag_deck_core

# ROS 2 node
colcon build --packages-select rosbag_deck_node

# Python bindings
colcon build --packages-select rosbag_deck_python

# Terminal UI
colcon build --packages-select rosbag_deck_tui
```

### Python Development
```bash
# Install Python dependencies (from workspace root)
rye sync

# Build Python packages with Rye
cd rosbag_deck_python && rye build
cd rosbag_deck_tui && rye build
```

## Linting

### C++ Packages
Linting is integrated into the CMake build process via `ament_lint_auto`. It runs automatically during build.

### Python Packages
```bash
# Run from individual Python package directories
rye lint
```

## Testing

No test suites are currently implemented. When adding tests:
- C++ tests should use gtest with ament_cmake_gtest
- Python tests should use pytest

## Architecture

### Package Dependencies
```
rosbag_deck_interface (msgs/srvs)
    ↓
rosbag_deck_core (C++ library)
    ↓         ↓
rosbag_deck_node  rosbag_deck_python (via CFFI)
    ↓                    ↓
    └─── rosbag_deck_tui ───┘
```

### Key Design Patterns

1. **Streaming Architecture**: The core library streams messages from disk rather than loading entire bags into memory. This is critical for handling large bag files.

2. **Virtual Timeline System**: Handles rewind operations by creating timeline segments. Frame IDs are modified to include segment information (e.g., `base_link` becomes `base_link_segment_1`).

3. **Thread Safety**: All core components use mutex protection. The BagWorker runs on a separate thread for I/O operations.

4. **C API for FFI**: `rosbag_deck_core` exposes a C API (`rosbag_deck_api.h`) for Python bindings via CFFI.

### Important Files

- Core API: `rosbag_deck_core/include/rosbag_deck_core/rosbag_deck_api.h`
- Python bindings: `rosbag_deck_python/src/rosbag_deck_python/deck.py`
- Node implementation: `rosbag_deck_node/src/rosbag_deck_node.cpp`
- TUI implementation: `rosbag_deck_tui/src/rosbag_deck_tui/app.py`

## Development Notes

### Current Limitations
- Hardcoded to work with `sensor_msgs/PointCloud2` messages only
- No CI/CD pipeline configured
- No automated tests

### When Modifying Core Library
1. Update the C API in `rosbag_deck_api.h` if adding new functionality
2. Regenerate Python bindings: `cd rosbag_deck_python && python build_cffi.py`
3. Update both the node and Python wrapper to expose new features

### ROS 2 Integration
- The project uses ROS 2 Humble or later
- All ROS communication goes through `rosbag_deck_interface` definitions
- Status is published at 10Hz by default
- Services use standard ROS 2 request/response patterns

### Python Package Management
- Uses Rye workspace at the root level
- Individual `pyproject.toml` files for each Python package
- CFFI bindings are built during colcon build, not Rye build

### Environment Setup
- Rye is used to manage the project. Run `rye sync` to setup the virtual environment.

### Build Helpers
- Run `build.sh` to re-build the whole Colcon project.

### Code Formatting
- Use clang-format to format C/C++ code. Be aware that it might break the code.