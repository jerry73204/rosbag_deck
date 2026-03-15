# Phase 8: ROS 2 Topic Publishing

Publish bag messages to live ROS 2 topics during playback, so downstream nodes (rviz2, SLAM, sensor fusion) can consume them as if recorded data were live. Publishing is on by default — the same as `ros2 bag play`.

See [design/ros2-publishing.md](../design/ros2-publishing.md) for detailed design.

## 8.1 FFI: Publisher API

Expose rclcpp's `GenericPublisher` and node lifecycle via the C wrapper.

- [x] Add `Rosbag2Node` opaque handle (wraps `rclcpp::Node` with single-threaded executor)
- [x] `rosbag2_node_create(node_name)` / `rosbag2_node_destroy()`
- [x] `rosbag2_node_create_publisher(node, topic, type_name, qos_depth, reliable)` — returns publisher handle
- [x] `rosbag2_node_publish(publisher, data, data_len)` — publish serialized CDR bytes
- [x] `rosbag2_node_destroy_publisher(publisher)`
- [x] `rosbag2_node_spin_some(node)` — non-blocking spin for discovery
- [x] Add `rclcpp` to link libraries in `build.rs`
- [x] Safe Rust wrappers: `RosNode`, `RosPublisher` in `rosbag_deck_ffi`

## 8.2 Core: Publisher Manager

A component that owns the ROS 2 node and per-topic publishers, called by the Deck.

- [x] `PublisherManager` struct in `rosbag_deck_core`
- [x] `new(backend, qos_depth, qos_preset)` — creates manager with backend
- [x] `ensure_and_publish(topic, type_name, data)` — lazily creates publishers on first message
- [x] `spin_some()` — calls backend `spin_some` for node discovery / graph updates
- [x] `shutdown()` — tears down publishers and node
- [x] `set_qos_preset()` — clears all publishers for lazy recreation with new QoS
- [x] Publisher trait abstraction (`TopicPublisher` / `PublisherBackend`) so Deck doesn't depend on FFI directly
- [x] `RosPublisherBackend` in `rosbag_deck_ffi` implements `PublisherBackend`

## 8.3 Deck Integration

Wire the publisher into the Deck's message delivery path. Publishing is enabled by default.

- [x] Add optional `PublisherManager` field to `Deck`
- [x] `Deck::enable_publishing(manager)` / `Deck::disable_publishing()`
- [x] Publish each message in `next_message()`, `try_next_message()`, `step_forward()` after pacing
- [x] Publish the patched message (after `maybe_patch_stamp`) so Monotonic timestamps propagate
- [x] Call `spin_some()` periodically in `next_message` and `try_next_message`
- [x] Topic filtering: only publish messages that pass the registry filter
- [x] `Deck::is_publishing()`, `publisher_manager()`, `publisher_manager_mut()` accessors

## 8.4 CLI: `--no-publish` Flag

Publishing is on by default. `--no-publish` disables it.

- [x] Add `--no-publish` flag to `play` subcommand (publishing on by default)
- [x] Default node name: `rosbag_deck` (configurable via `--node-name`)
- [x] `--qos-depth <N>` for publisher queue depth (default: 10)
- [x] `--qos-profile auto|sensor|reliable` for QoS preset selection
- [x] Print "Publishing enabled (node: ..., QoS: ...)" on startup (unless `--no-publish`)
- [x] Headless mode: `play --no-tui` publishes by default

## 8.5 TUI: Publish Toggle

Publishing is on by default in TUI mode.

- [x] `p` key toggles publishing on/off during playback
- [x] Header shows "Pub" indicator when publishing is active (shown by default)
- [x] Status message on toggle: "Publishing disabled" / "Publishing enabled (N topics)"

## 8.6 QoS Configuration

rclcpp does not support changing QoS on an existing publisher — the publisher must be destroyed and recreated. `PublisherManager::set_qos_preset()` clears all publishers for lazy recreation.

- [x] Default QoS: `sensor_msgs/msg/*` → `BEST_EFFORT`, all others → `RELIABLE`; `KEEP_LAST(10)`, `VOLATILE`
- [x] `--qos-profile auto|sensor|reliable` global presets at startup
- [ ] Read original QoS from bag metadata if available (rosbag2 stores offered QoS)
- [ ] Per-topic QoS override via `--qos-override <topic>=<profile>`
- [x] TUI: `Q` key cycles QoS preset for all publishers (destroy + recreate each)
- [ ] TUI: per-topic QoS change in topic panel (stretch goal)

## 8.7 Clock Publishing

- [ ] Publish `/clock` topic (`rosgraph_msgs/msg/Clock`) with current playback time
- [ ] Enable with `--clock` flag (mirrors `ros2 bag play --clock`)
- [ ] Respect playback speed (clock advances at `speed` rate)
- [ ] Pause/stop halts clock publishing

## Criteria

- [x] `ros2 topic list` shows bag topics while `rosbag_deck play` is running
- [x] `ros2 topic echo /some_topic` receives messages at correct rate
- [ ] rviz2 can visualize PointCloud2/Image topics from bag playback
- [x] Pausing playback pauses publishing; resume continues
- [x] Seeking re-publishes from the new position
- [x] Topic filter applies to publishing (filtered topics not published)
- [x] Monotonic loop mode: published timestamps increase across loop boundary
- [ ] `--clock` publishes correct simulated time
- [x] QoS preset change via TUI recreates publishers correctly
- [x] `--no-publish` disables all publishing
- [x] `just quality` passes
