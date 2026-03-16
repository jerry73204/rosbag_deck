# rosbag_deck

CLI and interactive TUI for playing, inspecting, and editing ROS 2 bag files.

## Commands

### `rosbag_deck info` — Inspect a bag

```bash
rosbag_deck info /path/to/bag
```

Prints storage format, duration, message count, and a table of topics with types and per-topic counts.

### `rosbag_deck play` — Play a bag

```bash
# Headless playback (publishes to ROS 2 topics by default)
rosbag_deck play /path/to/bag

# Interactive TUI
rosbag_deck play /path/to/bag --tui

# 2x speed, only specific topics
rosbag_deck play /path/to/bag --rate 2.0 --topics /camera/image /imu

# Loop with monotonically increasing timestamps
rosbag_deck play /path/to/bag --loop monotonic

# Disable publishing (offline inspection)
rosbag_deck play /path/to/bag --tui --no-publish
```

**Options:**

| Option                    | Description                                                               |
|---------------------------|---------------------------------------------------------------------------|
| `--tui`                   | Interactive terminal UI                                                   |
| `-r, --rate <N>`          | Speed multiplier (default: 1.0)                                           |
| `-t, --topics <T>...`     | Topic whitelist (space or comma-separated)                                |
| `--regex <PATTERN>`       | Include topics matching regex                                             |
| `-e, --exclude <PATTERN>` | Exclude topics matching regex                                             |
| `-l, --loop [MODE]`       | `off` (default), `restart`, or `monotonic`. Bare `--loop` means `restart` |
| `--no-publish`            | Disable ROS 2 topic publishing                                            |
| `--node-name <NAME>`      | ROS 2 node name (default: `rosbag_deck`)                                  |
| `--qos-depth <N>`         | Publisher queue depth (default: 10)                                       |
| `--qos-profile <P>`       | `auto` (default), `sensor`, or `reliable`                                 |
| `--storage <ID>`          | Force storage backend (`sqlite3`, `mcap`)                                 |

### `rosbag_deck edit` — Edit bags

Slice, merge, filter, and transform bag files.

```bash
# Extract 60 seconds of camera data
rosbag_deck edit /path/to/bag -o output.db --topics /camera/image --start 0 --end 60

# Merge two bags
rosbag_deck edit bag1 bag2 -o merged.db

# Shift all timestamps 5 seconds earlier
rosbag_deck edit /path/to/bag -o shifted.db --time-offset -5.0

# Scale timestamps to half speed
rosbag_deck edit /path/to/bag -o slowed.db --time-scale 0.5

# Dry run to preview what would be written
rosbag_deck edit /path/to/bag --dry-run --regex "/imu.*" --start 10 --end 30
```

**Options:**

| Option                    | Description                                   |
|---------------------------|-----------------------------------------------|
| `-o, --output <PATH>`     | Output bag path (required unless `--dry-run`) |
| `--format <FMT>`          | Output format (default: `sqlite3`)            |
| `-t, --topics <T>...`     | Topic whitelist                               |
| `--regex <PATTERN>`       | Include topics matching regex                 |
| `-e, --exclude <PATTERN>` | Exclude topics matching regex                 |
| `--start <SECS>`          | Start time (seconds from bag start)           |
| `--end <SECS>`            | End time (seconds from bag start)             |
| `--duration <SECS>`       | Max duration from start                       |
| `--time-offset <SECS>`    | Shift all timestamps by N seconds             |
| `--time-scale <FACTOR>`   | Scale timestamps relative to first message    |
| `--dry-run`               | Preview without writing                       |
| `-v, --verbose`           | Print per-message progress                    |

## TUI Keybindings

### Playback

| Key           | Action                                  |
|---------------|-----------------------------------------|
| `Space`       | Play / pause                            |
| `s`           | Stop                                    |
| `Left` / `h`  | Step backward                           |
| `Right` / `l` | Step forward                            |
| `+` / `=`     | Speed up                                |
| `-` / `_`     | Speed down                              |
| `Home`        | Seek to start                           |
| `End`         | Seek to end                             |
| `g`           | Seek to time (enter seconds or `MM:SS`) |

### Modes

| Key | Action                                                     |
|-----|------------------------------------------------------------|
| `o` | Cycle loop mode: Off &rarr; Restart &rarr; Monotonic       |
| `p` | Toggle publishing                                          |
| `Q` | Cycle QoS preset: auto &rarr; sensor &rarr; reliable       |

### Topics

| Key | Action                     |
|-----|----------------------------|
| `t` | Open topic selection panel |

In the topic panel: `j`/`k` to navigate, `Space` to toggle, `a` select all, `n` deselect all, `/` to search, `Enter` to apply, `Esc` to cancel.

### General

| Key            | Action |
|----------------|--------|
| `q` / `Ctrl+C` | Quit   |

## Loop Modes

**Off** (default) — Play once and stop.

**Restart** (`--loop restart`) — Loop playback with original timestamps. When the bag ends, seek back to the start and continue.

**Monotonic** (`--loop monotonic`) — Loop playback with ever-increasing timestamps. On each iteration, `Header.stamp` inside CDR-serialized messages is shifted forward by `iteration * bag_duration`, so timestamps never jump backward. This is useful for consumers that expect monotonic time, such as SLAM pipelines, sensor fusion, or transform buffers.

Only message types with `std_msgs/msg/Header` as their first field are patched (detected automatically via ROS 2 type introspection). Message-level `timestamp_ns` is also shifted.

## QoS Presets

| Preset     | Reliability                | Use case                                                     |
|------------|----------------------------|--------------------------------------------------------------|
| `auto`     | Inferred from message type | Default, works for most bags                                 |
| `sensor`   | Best-effort                | High-frequency sensor streams (camera, lidar, IMU)           |
| `reliable` | Reliable                   | Control commands, diagnostics, low-frequency critical topics |

## Topic Filtering

Three filter mechanisms can be combined:

1. **Whitelist** (`--topics`): Only include listed topics
2. **Regex include** (`--regex`): Add topics matching a pattern
3. **Exclude** (`--exclude`): Remove topics matching a pattern

Resolution order: start with whitelist (or all topics if only regex/exclude given), add regex matches, then remove exclude matches.

```bash
# Only lidar and IMU
rosbag_deck play bag --topics /lidar/points /imu/data

# All camera topics except diagnostics
rosbag_deck play bag --regex "^/camera/.*" --exclude "/camera/diagnostic"

# Everything except debug topics
rosbag_deck play bag --exclude "^/debug"
```
