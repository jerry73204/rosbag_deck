# RosbagDeck Python API

A Python wrapper for the RosbagDeck C++ library providing tape deck style rosbag playback with streaming architecture for large bag files.

## Features

- **Streaming Architecture**: Efficiently handle large rosbag files without loading everything into memory
- **Tape Deck Controls**: Play, pause, step forward/backward, seek to time/frame
- **Configurable Caching**: Adjustable cache size and preloading settings
- **Async Callbacks**: Real-time status and message callbacks
- **Context Manager Support**: Automatic resource cleanup
- **Type Safety**: Full type hints and dataclass-based API

## Installation

### Automatic Installation (Recommended)

```bash
# Build with colcon (automatically builds CFFI extension)
colcon build

# Or install with pip (automatically builds CFFI extension)
pip install -e .
```

### Manual Installation (if automatic fails)

```bash
cd /path/to/rosbag_deck_python

# Install the package
pip install -e .

# Manually build the CFFI extension (if automatic build failed)
python ffi_builder.py
```

### Requirements

- Python 3.8 or later
- CFFI 1.0.0 or later (automatically installed)
- The `librosbag_deck_core.so` library (built from the C++ core)

### Build Process

The Python API uses CFFI to automatically generate bindings from the C header file. This provides several advantages:

1. **Automatic Generation**: No manual maintenance of FFI bindings
2. **Type Safety**: Better type checking and validation
3. **Performance**: More efficient than manual ctypes bindings
4. **Future-proof**: Automatically picks up changes to the C API

The build process is now **automatic**:
1. `colcon build` or `pip install -e .` automatically builds the CFFI extension
2. If the automatic build fails (e.g., missing C library), you can run `python ffi_builder.py` manually
3. The extension is ready to use immediately

### Manual Commands

```bash
# Build CFFI extension manually
python ffi_builder.py

# Test CFFI extension (build + test)
python ffi_builder.py test
```

## Quick Start

```python
from rosbag_deck import RosbagDeck

# Create deck and load bags
with RosbagDeck() as deck:
    deck.build_index(["bag1.bag", "bag2.bag"])
    
    # Get bag information
    info = deck.get_bag_info()
    print(f"Total frames: {info.total_frames}")
    print(f"Topics: {info.topic_names}")
    
    # Configure playback
    deck.set_cache_size(100)
    deck.set_playback_rate(2.0)  # 2x speed
    
    # Manual control
    deck.step_forward()
    deck.seek_to_frame(100)
    
    # Continuous playback
    deck.start_playback()
    time.sleep(5)
    deck.stop_playback()
```

## API Reference

### RosbagDeck Class

#### Initialization

```python
deck = RosbagDeck(library_path=None)
```

- `library_path`: Optional path to `librosbag_deck_core.so`. If None, searches common locations.

#### Configuration Methods

```python
deck.set_cache_size(size: int)
deck.set_preload_settings(ahead: int, behind: int)
deck.set_playback_rate(rate: float)
deck.set_loop_playback(loop: bool)
```

#### Core Operations

```python
deck.build_index(bag_paths: List[str])  # Build index from bag files
deck.start_playback()                   # Start continuous playback
deck.stop_playback()                    # Stop continuous playback
deck.step_forward() -> bool             # Step forward one frame
deck.step_backward() -> bool            # Step backward one frame
deck.seek_to_time(nanoseconds: int) -> bool      # Seek to specific time
deck.seek_to_frame(frame_index: int) -> bool     # Seek to specific frame
```

#### Information Queries

```python
info = deck.get_bag_info()     # Returns BagInfo object
status = deck.get_status()     # Returns Status object
```

#### Callback Registration

```python
def on_status(status: Status):
    print(f"Frame {status.current_frame}/{status.total_frames}")

def on_message(message: Message):
    print(f"Topic: {message.topic_name}, Size: {len(message.serialized_data)}")

deck.set_status_callback(on_status)
deck.set_message_callback(on_message)
```

### Data Classes

#### BagInfo

```python
@dataclass
class BagInfo:
    success: bool
    message: str
    start_time: int          # nanoseconds since epoch
    end_time: int            # nanoseconds since epoch
    total_duration_ns: int
    total_frames: int
    topic_names: List[str]
```

#### Status

```python
@dataclass
class Status:
    current_time: int        # nanoseconds since epoch
    is_playing: bool
    current_frame: int
    total_frames: int
    timeline_segment: int
    virtual_time: int        # nanoseconds since epoch
```

#### Message

```python
@dataclass
class Message:
    original_timestamp: int  # nanoseconds since epoch
    virtual_timestamp: int   # nanoseconds since epoch
    topic_name: str
    message_type: str
    serialized_data: bytes
    frame_index: int
```

## Examples

### Basic Usage

```python
from rosbag_deck import RosbagDeck

with RosbagDeck() as deck:
    deck.build_index(["data.bag"])
    
    # Get bag info
    info = deck.get_bag_info()
    print(f"Duration: {info.total_duration_ns / 1e9:.2f} seconds")
    
    # Step through frames manually
    for i in range(10):
        if not deck.step_forward():
            break
```

### With Callbacks

```python
from rosbag_deck import RosbagDeck, Status, Message

def on_status(status: Status):
    progress = status.current_frame / status.total_frames * 100
    print(f"Progress: {progress:.1f}%")

def on_message(message: Message):
    if message.topic_name == "/camera/image":
        print(f"Camera image: {len(message.serialized_data)} bytes")

with RosbagDeck() as deck:
    deck.set_status_callback(on_status)
    deck.set_message_callback(on_message)
    
    deck.build_index(["sensor_data.bag"])
    deck.start_playback()
    
    # Let it run for 10 seconds
    time.sleep(10)
    deck.stop_playback()
```

### Interactive Playback

```python
from rosbag_deck import RosbagDeck

with RosbagDeck() as deck:
    deck.build_index(["recording.bag"])
    deck.set_cache_size(200)
    
    while True:
        cmd = input("Command (f=forward, b=backward, s=seek, p=play, q=quit): ")
        
        if cmd == 'f':
            deck.step_forward()
        elif cmd == 'b':
            deck.step_backward()
        elif cmd == 's':
            frame = int(input("Frame number: "))
            deck.seek_to_frame(frame)
        elif cmd == 'p':
            deck.start_playback()
            input("Press Enter to stop...")
            deck.stop_playback()
        elif cmd == 'q':
            break
        
        status = deck.get_status()
        print(f"Current frame: {status.current_frame}/{status.total_frames}")
```

## Error Handling

The library defines custom exceptions:

- `RosbagDeckError`: Base exception
- `RosbagDeckInitError`: Initialization failed
- `RosbagDeckIndexError`: Index building failed

```python
from rosbag_deck import RosbagDeck, RosbagDeckInitError, RosbagDeckIndexError

try:
    with RosbagDeck() as deck:
        deck.build_index(["nonexistent.bag"])
except RosbagDeckInitError as e:
    print(f"Failed to initialize: {e}")
except RosbagDeckIndexError as e:
    print(f"Failed to build index: {e}")
```

## Library Path Configuration

The Python API automatically searches for `librosbag_deck_core.so` in common locations:

1. `/usr/local/lib/librosbag_deck_core.so`
2. `/usr/lib/librosbag_deck_core.so`
3. `../install/rosbag_deck_core/lib/librosbag_deck_core.so` (relative to package)
4. Current directory

You can also specify a custom path:

```python
deck = RosbagDeck(library_path="/custom/path/librosbag_deck_core.so")
```

## Architecture

The Python API is built on top of the C FFI interface, providing:

- **Memory Safety**: Automatic cleanup and proper lifetime management
- **Type Safety**: Full type hints and validation
- **Performance**: Direct C library calls with minimal overhead
- **Pythonic Interface**: Context managers, properties, and dataclasses

## License

This project is licensed under the MIT License.