# Phase 1: Foundation

Set up the Rust workspace, build system, and ROS 2 FFI layer.

## 1.1 Workspace & Build System

- [x] Create Cargo workspace with all crate skeletons
- [x] Set up colcon-cargo-ros2 integration (package.xml with `ament_cargo`)
- [x] Configure nextest for integration tests
- [x] Replace Rye with UV for Python package management
- [x] Set up rust-toolchain.toml (stable)

## 1.2 Bag Reader Trait (`rosbag-deck`)

Define the abstraction that the core library uses to read bags, decoupled from any specific backend.

- [x] `BagReader` trait: read metadata, iterate messages, seek by timestamp
- [x] `BagMetadata` struct: topics, message counts, duration, start/end time, storage format
- [x] `TopicInfo` struct: topic name, message type, serialization format, message count
- [x] `RawMessage` struct: timestamp, topic, serialized data (bytes)

## 1.3 ROS 2 FFI Backend (`rosbag-deck-ffi`)

Implement `BagReader` via FFI to rosbag2, for reading bags produced by ROS 2.

- [x] C wrapper around rosbag2_cpp (`Reader`, `StorageOptions`, `BagMetadata`)
- [x] C wrapper around rosbag2_storage (`TopicMetadata`, `SerializedBagMessage`)
- [x] Safe Rust wrapper implementing `BagReader` trait (`Rosbag2Reader`)
- [x] Storage backend selection (sqlite3, mcap) via `StorageOptions`
- [x] Build script (`build.rs`) to find rosbag2 headers/libs via `AMENT_PREFIX_PATH`

## Criteria

- [x] `cargo build` succeeds for all workspace members
- [x] `colcon build` succeeds via ament_cargo
- [ ] FFI crate can open a real ROS 2 bag file and iterate messages
- [x] Core library depends only on `BagReader` trait, not on FFI crate directly
