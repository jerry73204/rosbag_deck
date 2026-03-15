# Phase 8: ROS 2 Topic Publishing

Publish bag messages to live ROS 2 topics during playback, so downstream nodes (rviz2, SLAM, sensor fusion) can consume them as if recorded data were live. Publishing is on by default — the same as `ros2 bag play`.

See [design/ros2-publishing.md](../design/ros2-publishing.md) for detailed design.

## 8.1 FFI: Publisher API

Expose rclcpp's `GenericPublisher` and node lifecycle via the C wrapper.

- [ ] Add `Rosbag2Node` opaque handle (wraps `rclcpp::Node` with single-threaded executor)
- [ ] `rosbag2_node_create(node_name)` / `rosbag2_node_destroy()`
- [ ] `rosbag2_node_create_publisher(node, topic, type_name, qos_depth, reliable)` — returns publisher handle
- [ ] `rosbag2_node_publish(publisher, data, data_len)` — publish serialized CDR bytes
- [ ] `rosbag2_node_destroy_publisher(publisher)`
- [ ] `rosbag2_node_spin_some(node)` — non-blocking spin for discovery
- [ ] Add `rclcpp` to link libraries in `build.rs`
- [ ] Safe Rust wrappers: `RosNode`, `RosPublisher` in `rosbag_deck_ffi`

## 8.2 Core: Publisher Manager

A component that owns the ROS 2 node and per-topic publishers, called by the Deck.

- [ ] `PublisherManager` struct in `rosbag_deck_core` (behind `ros2` feature flag)
- [ ] `new(node_name)` — creates the underlying ROS node
- [ ] `ensure_publisher(topic, type_name)` — lazily creates publishers on first message
- [ ] `publish(msg: &RawMessage)` — looks up publisher, publishes serialized data
- [ ] `spin()` — calls `spin_some` for node discovery / graph updates
- [ ] `shutdown()` — tears down publishers and node
- [ ] `recreate_publisher(topic)` — destroy + recreate with new QoS (for live QoS changes)
- [ ] Publisher trait abstraction so Deck doesn't depend on FFI directly (testable with mock)

## 8.3 Deck Integration

Wire the publisher into the Deck's message delivery path. Publishing is enabled by default.

- [ ] Add `PublisherManager` field to `Deck` (created during `open()`)
- [ ] Publish each message in `next_message()`, `try_next_message()`, `step_forward()` after pacing
- [ ] Publish the patched message (after `maybe_patch_stamp`) so Monotonic timestamps propagate
- [ ] Call `spin()` periodically (e.g., in `try_next_message` before returning None)
- [ ] Topic filtering: only publish messages that pass the registry filter
- [ ] `Deck::disable_publishing()` / `Deck::enable_publishing()` for runtime toggle

## 8.4 CLI: `--no-publish` Flag

Publishing is on by default. `--no-publish` disables it.

- [ ] Add `--no-publish` flag to `play` subcommand (publishing on by default)
- [ ] Default node name: `rosbag_deck` (configurable via `--node-name`)
- [ ] `--qos-depth <N>` for publisher queue depth (default: 10)
- [ ] Print "Publishing to N topics" on startup (unless `--no-publish`)
- [ ] Headless mode: `play --no-tui` publishes by default, useful for CI/testing pipelines

## 8.5 TUI: Publish Toggle

Publishing is on by default in TUI mode.

- [ ] `p` key toggles publishing on/off during playback
- [ ] Header shows "Pub" indicator when publishing is active (shown by default)
- [ ] Status message on toggle: "Publishing disabled" / "Publishing enabled (N topics)"

## 8.6 QoS Configuration

rclcpp does not support changing QoS on an existing publisher — the publisher must be destroyed and recreated. `PublisherManager::recreate_publisher()` handles this transparently.

- [ ] Default QoS: `sensor_msgs/msg/*` → `BEST_EFFORT`, all others → `RELIABLE`; `KEEP_LAST(10)`, `VOLATILE`
- [ ] `--qos-profile sensor|default|reliable` global presets at startup
- [ ] Read original QoS from bag metadata if available (rosbag2 stores offered QoS)
- [ ] Per-topic QoS override via `--qos-override <topic>=<profile>`
- [ ] TUI: `Q` key cycles QoS preset for all publishers (destroy + recreate each)
- [ ] TUI: per-topic QoS change in topic panel (stretch goal)

## 8.7 Clock Publishing

- [ ] Publish `/clock` topic (`rosgraph_msgs/msg/Clock`) with current playback time
- [ ] Enable with `--clock` flag (mirrors `ros2 bag play --clock`)
- [ ] Respect playback speed (clock advances at `speed` rate)
- [ ] Pause/stop halts clock publishing

## Criteria

- [ ] `ros2 topic list` shows bag topics while `rosbag_deck play` is running
- [ ] `ros2 topic echo /some_topic` receives messages at correct rate
- [ ] rviz2 can visualize PointCloud2/Image topics from bag playback
- [ ] Pausing playback pauses publishing; resume continues
- [ ] Seeking re-publishes from the new position
- [ ] Topic filter applies to publishing (filtered topics not published)
- [ ] Monotonic loop mode: published timestamps increase across loop boundary
- [ ] `--clock` publishes correct simulated time
- [ ] QoS preset change via TUI recreates publishers correctly
- [ ] `--no-publish` disables all publishing
- [ ] `just quality` passes
