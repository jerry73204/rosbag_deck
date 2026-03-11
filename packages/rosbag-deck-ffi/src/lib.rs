// TODO: FFI bindings to ROS 2 libraries
//
// This crate provides Rust wrappers around:
// - rosbag2_cpp: Bag file reading (SequentialReader, StorageOptions)
// - rosbag2_storage: Storage plugin interface
//
// Strategy: Use bindgen to generate bindings from rosbag2 C++ headers,
// or manually define the subset of the API we need.
