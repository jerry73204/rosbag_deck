# rosbag_deck_python

Python bindings for the RosBag Deck core library, providing a Pythonic interface for interactive bag file playback.

## Overview

This package provides Python bindings for the `rosbag_deck_core` C++ library using CFFI (C Foreign Function Interface). It offers a high-level, type-safe Python API for controlling ROS bag playback with tape deck-style controls.

## Features

- **Pythonic Interface**: Context managers, type hints, and dataclasses
- **Streaming Architecture**: Handle large bag files without loading into memory
- **Interactive Controls**: Play, pause, step, seek operations
- **Async Callbacks**: Real-time status and message callbacks
- **Automatic Resource Management**: Proper cleanup with context managers
- **Type Safety**: Full type hints and runtime validation

## Installation

### Building with Colcon (Recommended)

```bash
cd /path/to/rosbag_deck
colcon build --packages-select rosbag_deck_python
source install/setup.bash
```

### Development Installation

```bash
cd /path/to/rosbag_deck/rosbag_deck_python
pip install -e .
```

### Manual CFFI Build (if needed)

```bash
cd /path/to/rosbag_deck/rosbag_deck_python
python ffi_builder.py
```

## Quick Start

```python
from rosbag_deck import RosbagDeck

# Create deck and load bags
with RosbagDeck() as deck:
    deck.build_index(["/path/to/bag1.db3", "/path/to/bag2.db3"])
    
    # Get bag information
    info = deck.get_bag_info()
    print(f"Total frames: {info.total_frames}")
    print(f"Duration: {info.total_duration_ns / 1e9:.2f} seconds")
    
    # Step through frames
    for i in range(10):
        if deck.step_forward():
            status = deck.get_status()
            print(f"Frame {status.current_frame}/{status.total_frames}")
```

## API Reference

### RosbagDeck Class

The main interface for controlling bag playback.

#### Constructor

```python
deck = RosbagDeck(library_path: Optional[str] = None)
```

- `library_path`: Path to `librosbag_deck_core.so` (auto-detected if None)

#### Context Manager

```python
with RosbagDeck() as deck:
    # Automatic initialization and cleanup
    deck.build_index(["bag.db3"])
```

#### Index Building

```python
deck.build_index(bag_paths: List[str]) -> None
```

Build index from bag files. Must be called before any playback operations.

#### Playback Control

```python
# Continuous playback
deck.start_playback() -> None
deck.stop_playback() -> None

# Frame-by-frame control
deck.step_forward() -> bool
deck.step_backward() -> bool

# Seeking
deck.seek_to_time(nanoseconds: int) -> bool
deck.seek_to_frame(frame_index: int) -> bool
```

#### Configuration

```python
deck.set_cache_size(size: int) -> None
deck.set_preload_settings(ahead: int, behind: int) -> None
deck.set_playback_rate(rate: float) -> None
deck.set_loop_playback(loop: bool) -> None
```

#### Information Queries

```python
info = deck.get_bag_info() -> BagInfo
status = deck.get_status() -> Status
```

#### Callbacks

```python
# Status callback (called at ~10Hz during playback)
def on_status(status: Status) -> None:
    print(f"Frame {status.current_frame}")

deck.set_status_callback(on_status)

# Message callback (called for each published message)
def on_message(message: Message) -> None:
    print(f"Topic: {message.topic_name}")

deck.set_message_callback(on_message)
```

### Data Classes

#### BagInfo

```python
@dataclass
class BagInfo:
    success: bool              # Whether info retrieval succeeded
    message: str              # Error message if failed
    start_time: int           # Start time in nanoseconds
    end_time: int             # End time in nanoseconds
    total_duration_ns: int    # Total duration in nanoseconds
    total_frames: int         # Total number of frames
    topic_names: List[str]    # List of topics in bags
```

#### Status

```python
@dataclass
class Status:
    current_time: int         # Current bag time in nanoseconds
    is_playing: bool          # Whether playback is active
    current_frame: int        # Current frame index
    total_frames: int         # Total number of frames
    timeline_segment: int     # Virtual timeline segment
    virtual_time: int         # Virtual time (monotonic)
```

#### Message

```python
@dataclass
class Message:
    original_timestamp: int   # Original message timestamp
    virtual_timestamp: int    # Virtual timestamp (monotonic)
    topic_name: str          # Topic name
    message_type: str        # Message type (e.g., "sensor_msgs/PointCloud2")
    serialized_data: bytes   # Serialized message data
    frame_index: int         # Frame index
```

## Examples

### Basic Playback Control

```python
from rosbag_deck import RosbagDeck
import time

with RosbagDeck() as deck:
    # Load bags
    deck.build_index(["/data/recording1.db3", "/data/recording2.db3"])
    
    # Configure
    deck.set_cache_size(200)
    deck.set_playback_rate(2.0)  # 2x speed
    
    # Play for 10 seconds
    deck.start_playback()
    time.sleep(10)
    deck.stop_playback()
    
    # Get final status
    status = deck.get_status()
    print(f"Stopped at frame {status.current_frame}")
```

### Interactive Frame Selection

```python
from rosbag_deck import RosbagDeck

with RosbagDeck() as deck:
    deck.build_index(["calibration_data.db3"])
    
    # Find specific frames for calibration
    target_frames = []
    
    while len(target_frames) < 5:
        print(f"\nFrame {deck.get_status().current_frame}")
        cmd = input("Mark this frame? (y/n/q): ")
        
        if cmd == 'y':
            target_frames.append(deck.get_status().current_frame)
            print(f"Marked frame {target_frames[-1]}")
        elif cmd == 'q':
            break
            
        if not deck.step_forward():
            print("Reached end of bag")
            break
    
    print(f"Selected frames: {target_frames}")
```

### Message Processing

```python
from rosbag_deck import RosbagDeck, Message
import struct

def process_pointcloud(message: Message):
    if message.message_type != "sensor_msgs/PointCloud2":
        return
    
    # Process serialized data
    # (In practice, use ROS deserialization libraries)
    print(f"PointCloud2 from {message.topic_name}: {len(message.serialized_data)} bytes")

with RosbagDeck() as deck:
    deck.set_message_callback(process_pointcloud)
    deck.build_index(["lidar_data.db3"])
    
    # Process first 100 frames
    for _ in range(100):
        if not deck.step_forward():
            break
```

### Status Monitoring

```python
from rosbag_deck import RosbagDeck, Status
import threading
import time

status_lock = threading.Lock()
current_status = None

def status_callback(status: Status):
    global current_status
    with status_lock:
        current_status = status

with RosbagDeck() as deck:
    deck.set_status_callback(status_callback)
    deck.build_index(["long_recording.db3"])
    deck.start_playback()
    
    # Monitor progress
    while True:
        time.sleep(1)
        with status_lock:
            if current_status:
                progress = current_status.current_frame / current_status.total_frames * 100
                print(f"Progress: {progress:.1f}%", end='\r')
                
                if not current_status.is_playing:
                    break
```

## Exception Handling

```python
from rosbag_deck import (
    RosbagDeck,
    RosbagDeckError,
    RosbagDeckInitError,
    RosbagDeckIndexError
)

try:
    with RosbagDeck() as deck:
        deck.build_index(["missing.db3"])
except RosbagDeckInitError as e:
    print(f"Failed to initialize: {e}")
except RosbagDeckIndexError as e:
    print(f"Failed to build index: {e}")
except RosbagDeckError as e:
    print(f"General error: {e}")
```

## Implementation Details

### CFFI Integration

The package uses CFFI for C++ bindings:
- Automatic binding generation from C headers
- Type-safe interface with runtime validation
- Efficient data transfer with minimal overhead

### Memory Management

- Automatic cleanup via context managers
- Safe handling of C++ allocated memory
- Callback lifetime management

### Thread Safety

- Callbacks are invoked from C++ threads
- GIL handling for Python callbacks
- Thread-safe status updates

## Dependencies

- Python 3.8+
- CFFI >= 1.0.0
- rosbag_deck_core (C++ library)

## Building from Source

The CFFI extension is built automatically during package installation. For manual building:

```bash
# Build CFFI extension
python ffi_builder.py

# Build and test
python ffi_builder.py test
```

## Troubleshooting

### Library Not Found

If the C++ library isn't found, specify the path explicitly:

```python
deck = RosbagDeck(library_path="/path/to/librosbag_deck_core.so")
```

### CFFI Build Errors

Ensure the C++ library is built first:

```bash
colcon build --packages-select rosbag_deck_core
```

### Callback Issues

- Callbacks should be lightweight to avoid blocking
- Use thread-safe operations in callbacks
- Avoid modifying deck state from callbacks