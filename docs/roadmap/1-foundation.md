# Phase 1: Foundation

Set up the Rust workspace, build system, and ROS 2 FFI layer.

## Workspace & Build System

- [x] Create Cargo workspace with all crate skeletons
- [x] Set up colcon-cargo-ros2 integration (package.xml with `ament_cargo`)
- [x] Configure nextest for integration tests
- [x] Replace Rye with UV for Python package management
- [x] Set up rust-toolchain.toml (stable)

## ROS 2 FFI (`rosbag-deck-ffi`)

- [ ] Create bindgen-based bindings to rosbag2_cpp (SequentialReader, StorageOptions)
- [ ] Wrap rosbag2_storage for storage plugin discovery
- [ ] Expose safe Rust API for opening and reading bag files
- [ ] Support sqlite3 and mcap storage backends
- [ ] Handle message metadata (topic name, type, serialization format)

## Criteria

- `cargo build` succeeds for all workspace members
- `colcon build` succeeds via ament_cargo
- FFI crate can open a real ROS 2 bag file and iterate messages
