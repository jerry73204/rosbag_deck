# rosbag_deck_python

Python bindings for RosBag Deck via PyO3. Provides programmatic access to bag playback, seeking, stepping, and metadata inspection.

## Installation

Requires ROS 2 sourced and Rust toolchain installed.

```bash
cd packages/rosbag_deck_python && maturin develop
```

## Usage

```python
from rosbag_deck import Deck

# Iterate over all messages
with Deck("/path/to/bag") as deck:
    for msg in deck:
        print(f"{msg.topic}: {len(msg.data)} bytes @ {msg.timestamp_ns}")

# Playback with speed control
deck = Deck("/path/to/bag")
deck.set_speed(2.0)
deck.play()
while msg := deck.next_message():
    process(msg)

# Inspect metadata
info = deck.info()
print(f"Duration: {info.duration_secs:.1f}s, {info.message_count} messages")
for t in info.topics:
    print(f"  {t.name} [{t.type_name}]: {t.message_count} msgs")

# Seek and step
deck.seek_to_ratio(0.5)       # Jump to midpoint
msg = deck.step_forward()     # Single message
deck.seek_to_time(timestamp)  # Absolute nanosecond timestamp
```

## API

### `Deck(path, storage="")`

Open a bag file. `storage` can be `""` (auto-detect), `"sqlite3"`, or `"mcap"`.

Supports context manager (`with` statement) and iteration (`for msg in deck`).

### Transport Control

| Method               | Description       |
|----------------------|-------------------|
| `play()`             | Start playback    |
| `pause()`            | Pause playback    |
| `stop()`             | Stop playback     |
| `toggle_play_pause()`| Toggle play/pause |

### Playback Settings

| Method                  | Description                             |
|-------------------------|-----------------------------------------|
| `set_speed(f)`          | Set playback speed multiplier           |
| `set_mode(m)`           | `"realtime"` or `"best_effort"`         |
| `set_looping(bool)`     | Enable/disable looping                  |
| `set_topic_filter(list)`| Filter topics (or `None` for all)       |

### Navigation

| Method              | Description                    |
|---------------------|--------------------------------|
| `next_message()`    | Next message during playback   |
| `step_forward()`    | Step one message forward       |
| `step_backward()`   | Step one message backward      |
| `seek_to_time(ns)`  | Seek to nanosecond timestamp   |
| `seek_to_ratio(f)`  | Seek to 0.0&ndash;1.0 position |

### Metadata

| Method     | Returns                                          |
|------------|--------------------------------------------------|
| `info()`   | `BagInfo` — topics, message count, duration, etc |
| `status()` | `PlaybackStatus` — state, cursor, speed, looping |

### Data Types

**`BagInfo`** — `topics: list[TopicInfo]`, `message_count: int`, `duration_secs: float`, `start_time_ns: int`, `end_time_ns: int`, `storage_identifier: str`

**`TopicInfo`** — `name: str`, `type_name: str`, `serialization_format: str`, `message_count: int`

**`Message`** — `timestamp_ns: int`, `topic: str`, `data: bytes`

**`PlaybackStatus`** — `state: str`, `cursor_ns: int`, `speed: float`, `looping: bool`, `segment_id: int`
