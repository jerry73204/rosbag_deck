# Design: ROS 2 Topic Publishing

## Motivation

RosBag Deck can play bag files with frame-level precision, but currently only displays messages in the TUI or prints them headlessly. Downstream tools like rviz2, SLAM nodes, and sensor fusion pipelines expect data on live ROS 2 topics. Without publishing, the player is a viewer but not a drop-in replacement for `ros2 bag play`.

Publishing is **on by default** — the same behavior as `ros2 bag play`. Use `--no-publish` to disable.

## C++ API Availability

| Feature             | rclcpp API                 | Notes                                                          |
|---------------------|----------------------------|----------------------------------------------------------------|
| Create node         | `rclcpp::Node`             | Manages lifecycle, discovery, executors                        |
| Generic publisher   | `rclcpp::GenericPublisher` | Publishes serialized bytes without compile-time type knowledge |
| Spin (non-blocking) | `executor.spin_some()`     | Needed for DDS discovery and graph updates                     |
| QoS profiles        | `rclcpp::QoS`              | Configurable reliability, durability, depth                    |
| Clock publishing    | Publish to `/clock`        | `rosgraph_msgs/msg/Clock`, used with `use_sim_time`            |

Key finding: `rclcpp::GenericPublisher` (available since Humble) accepts raw serialized data (`rcl_serialized_message_t`), which is exactly what we have in `RawMessage.data`. No deserialization or type-specific code is needed.

## FFI Design

### C API Surface

```c
/* Opaque handles */
typedef struct Rosbag2Node Rosbag2Node;
typedef struct Rosbag2Publisher Rosbag2Publisher;

/* Node lifecycle */
Rosbag2Node *rosbag2_node_create(const char *node_name);
void rosbag2_node_destroy(Rosbag2Node *node);
void rosbag2_node_spin_some(Rosbag2Node *node);

/* Publisher lifecycle */
Rosbag2Publisher *rosbag2_node_create_publisher(
    Rosbag2Node *node,
    const char *topic,
    const char *type_name,
    size_t qos_depth,
    bool reliable);          /* true=RELIABLE, false=BEST_EFFORT */
void rosbag2_node_destroy_publisher(Rosbag2Publisher *pub);

/* Publishing */
int rosbag2_node_publish(
    Rosbag2Publisher *pub,
    const uint8_t *data,
    size_t data_len);
```

### C++ Implementation

```cpp
struct Rosbag2Node {
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::executors::SingleThreadedExecutor executor;
};

struct Rosbag2Publisher {
    rclcpp::GenericPublisher::SharedPtr publisher;
};
```

The node owns a `SingleThreadedExecutor`. `spin_some()` does a non-blocking spin for DDS discovery. Publishers are created via `node->create_generic_publisher(topic, type_name, qos)`.

Publishing uses `rcl_serialized_message_t` to wrap our existing CDR bytes without copying:

```cpp
int rosbag2_node_publish(Rosbag2Publisher *pub,
                          const uint8_t *data, size_t data_len) {
    auto msg = rclcpp::SerializedMessage(data_len);
    memcpy(msg.get_rcl_serialized_message().buffer, data, data_len);
    msg.get_rcl_serialized_message().buffer_length = data_len;
    pub->publisher->publish(msg);
    return 0;
}
```

### Rust Wrappers

```rust
pub struct RosNode { handle: *mut sys::Rosbag2Node }
pub struct RosPublisher { handle: *mut sys::Rosbag2Publisher }

impl RosNode {
    pub fn new(name: &str) -> Result<Self>;
    pub fn create_publisher(&self, topic: &str, type_name: &str, qos_depth: usize, reliable: bool) -> Result<RosPublisher>;
    pub fn spin_some(&self);
}

impl RosPublisher {
    pub fn publish(&self, data: &[u8]) -> Result<()>;
}
```

## Core Architecture

### PublisherManager

Lives in `rosbag_deck_core`, gated behind a `ros2` feature flag. Uses a trait to abstract the FFI layer:

```rust
pub trait TopicPublisher: Send {
    fn publish(&self, data: &[u8]) -> Result<()>;
}

pub trait PublisherBackend: Send {
    fn create_publisher(&mut self, topic: &str, type_name: &str, qos_depth: usize, reliable: bool) -> Result<Box<dyn TopicPublisher>>;
    fn spin_some(&self);
    fn shutdown(&mut self);
}

pub struct PublisherManager {
    backend: Box<dyn PublisherBackend>,
    publishers: HashMap<String, Box<dyn TopicPublisher>>,
    qos_config: QosConfig,
}
```

This trait-based design keeps `rosbag_deck_core` testable without ROS 2 — tests can use a mock backend that records published messages.

### Live QoS Changes

rclcpp does not support changing QoS on an existing publisher. To change QoS at runtime, `PublisherManager` destroys the old publisher and creates a new one:

```rust
impl PublisherManager {
    /// Recreate all publishers with a new QoS config.
    /// Called when the user changes the global QoS preset (e.g., TUI `Q` key).
    pub fn set_qos_config(&mut self, config: QosConfig) {
        self.qos_config = config;
        // Clear all publishers — they'll be lazily recreated on next message
        // with the new QoS settings.
        self.publishers.clear();
    }
}
```

Clearing the HashMap drops all publisher handles (calling `rosbag2_node_destroy_publisher` via `Drop`). The next message on each topic triggers lazy recreation with the new QoS. There may be a brief gap where subscribers see the publisher disappear and reappear — this is inherent to how DDS handles QoS changes and matches `ros2 bag play` behavior (which doesn't support live QoS changes at all).

### Message Flow

```
Deck::try_next_message()
  → find_next_cached_message()
  → topic filter check
  → real-time pacing
  → maybe_patch_stamp()          // CDR + timestamp shift
  → maybe_publish()              // NEW: publish to ROS 2
  → return TimedMessage
```

`maybe_publish()` publishes by default. It is a no-op only when publishing has been explicitly disabled via `--no-publish` or the TUI `p` toggle (publisher_manager is None).

### Lazy Publisher Creation

Publishers are created on first message for each topic, not upfront. This avoids creating publishers for topics that are filtered out or never appear:

```rust
fn maybe_publish(&mut self, msg: &RawMessage) {
    let pm = match &mut self.publisher_manager { Some(pm) => pm, None => return };
    let type_name = match self.registry.topic_info(&msg.topic) {
        Some(info) => &info.type_name,
        None => return,
    };
    pm.ensure_and_publish(&msg.topic, type_name, &msg.data);
}
```

## QoS Strategy

Default QoS for bag playback matches `ros2 bag play` behavior:

| Setting | Default | Rationale |
|---------|---------|-----------|
| Reliability | `BEST_EFFORT` for sensor data, `RELIABLE` otherwise | Sensor topics (images, pointclouds) are high-bandwidth; dropping is acceptable |
| Durability | `VOLATILE` | No need to replay old messages to late joiners |
| History | `KEEP_LAST(10)` | Small queue; playback pacing prevents buildup |

Sensor data detection heuristic: topics with type `sensor_msgs/msg/*` use `BEST_EFFORT`. All others use `RELIABLE`. This matches the typical QoS profile used by ROS 2 drivers.

### QoS Presets

```rust
pub enum QosPreset {
    /// sensor_msgs/* → BEST_EFFORT, others → RELIABLE (default)
    Auto,
    /// All topics BEST_EFFORT
    Sensor,
    /// All topics RELIABLE
    Reliable,
}
```

Set at startup via `--qos-profile auto|sensor|reliable`. Default is `auto`.

### Runtime QoS Changes

`ros2 bag play` does not support changing QoS during playback. RosBag Deck goes further: the TUI `Q` key cycles the global QoS preset. Under the hood, this calls `PublisherManager::set_qos_config()`, which clears all publishers so they are lazily recreated with the new QoS on the next message. Subscribers will briefly see the publisher disappear and reappear — this is inherent to DDS and acceptable for an interactive debugging tool.

Per-topic QoS overrides are supported via `--qos-override <topic>=<preset>` at startup. Live per-topic changes in the TUI topic panel are a stretch goal.

## Clock Publishing

When `--clock` is enabled, publish `rosgraph_msgs/msg/Clock` to `/clock` at ~100 Hz (or message rate, whichever is lower). Downstream nodes using `use_sim_time:=true` will synchronize to bag time.

Clock messages contain the current virtual timeline time, adjusted for speed and loop iteration (Monotonic mode).

## Threading Considerations

Publishing happens on the **main thread** (same thread as `next_message` / `try_next_message`), not on a worker thread. This is safe because:

1. `GenericPublisher::publish()` is thread-safe in rclcpp
2. The Deck's message delivery is already single-threaded
3. No new threads needed — `spin_some()` is called inline

For the TUI, `spin_some()` is called in the tick loop alongside `try_next_message()`, keeping DDS discovery alive without blocking the UI.

## Implementation Order

1. **FFI layer first** — get `GenericPublisher` working via C wrapper
2. **Core traits** — `TopicPublisher` / `PublisherBackend` with mock for testing
3. **PublisherManager** — lazy creation, HashMap lookup, QoS config
4. **Deck integration** — `maybe_publish()` in delivery path, publishing on by default
5. **CLI flags** — `--no-publish`, `--qos-profile`, `--node-name`
6. **TUI** — `p` toggle, `Q` QoS cycle, header indicator
7. **Clock** — `/clock` topic publishing

## Comparison with `ros2 bag play`

| Feature               | `ros2 bag play`                | `rosbag_deck play`                |
|-----------------------|--------------------------------|-----------------------------------|
| Basic publishing      | Yes (default)                  | Yes (default)                     |
| Disable publishing    | N/A                            | `--no-publish`                    |
| Pause/resume          | Yes                            | Yes (existing)                    |
| Seek                  | Limited                        | Yes (frame-level)                 |
| Step forward/backward | No                             | Yes (existing)                    |
| Speed control         | `--rate`                       | Yes (existing, dynamic)           |
| Topic filtering       | `--topics`                     | Yes (existing, regex + TUI panel) |
| Loop modes            | `--loop`                       | Restart + Monotonic               |
| TUI                   | No                             | Yes                               |
| Clock publishing      | `--clock`                      | `--clock`                         |
| QoS override          | `--qos-profile-overrides-path` | `--qos-profile` + `--qos-override` |
| Live QoS change       | No                             | Yes (TUI `Q` key, recreates publishers) |
