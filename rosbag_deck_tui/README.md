# rosbag_deck_tui

Terminal User Interface for the RosBag Deck system, providing interactive control over ROS 2 bag playback.

## Overview

This package provides a terminal-based user interface for controlling the RosBag Deck node. It offers an intuitive, keyboard-driven interface for playing, pausing, and navigating through ROS bag files.

## Features

- **Interactive Controls**: Keyboard shortcuts for all playback operations
- **Real-time Status Display**: Live updates of playback status
- **Progress Visualization**: Progress bar and frame counter
- **Service Integration**: Communicates with rosbag_deck_node via ROS 2 services
- **Responsive UI**: Built with Python's curses library

## Installation

### Building with Colcon

```bash
cd /path/to/rosbag_deck
colcon build --packages-select rosbag_deck_tui
source install/setup.bash
```

### Development Installation

```bash
cd /path/to/rosbag_deck/rosbag_deck_tui
pip install -e .
```

## Usage

### Prerequisites

Ensure the RosBag Deck node is running:

```bash
ros2 run rosbag_deck_node rosbag_deck_node \
  --ros-args \
  -p bag_paths:="['/path/to/bag1.db3', '/path/to/bag2.db3']"
```

### Running the TUI

```bash
ros2 run rosbag_deck_tui rosbag_deck_tui
```

### Command-line Options

```bash
# Specify custom node name
ros2 run rosbag_deck_tui rosbag_deck_tui --node-name /my_rosbag_deck

# Run with ROS arguments
ros2 run rosbag_deck_tui rosbag_deck_tui --ros-args -r __ns:=/custom_namespace
```

## Keyboard Controls

| Key | Action | Description |
|-----|--------|-------------|
| `Space` | Play/Pause | Toggle playback |
| `→` / `l` | Step Forward | Advance one frame |
| `←` / `h` | Step Backward | Go back one frame |
| `↑` / `k` | Seek Forward | Jump forward 10 frames |
| `↓` / `j` | Seek Backward | Jump backward 10 frames |
| `Home` / `g` | Go to Start | Jump to first frame |
| `End` / `G` | Go to End | Jump to last frame |
| `s` | Seek to Time | Enter specific timestamp |
| `f` | Seek to Frame | Enter specific frame number |
| `+` / `=` | Increase Rate | Speed up playback |
| `-` / `_` | Decrease Rate | Slow down playback |
| `1` | Normal Speed | Reset to 1x playback rate |
| `i` | Info | Display bag information |
| `r` | Refresh | Refresh display |
| `q` / `Esc` | Quit | Exit the TUI |
| `?` / `h` | Help | Show help screen |

## User Interface Layout

```
┌─────────────────────────────────────────────────────────┐
│                   RosBag Deck TUI                       │
├─────────────────────────────────────────────────────────┤
│ Status: Playing                  Rate: 1.0x             │
│ Frame: 1234/5678                Time: 00:12:34          │
│                                                         │
│ Progress: [████████████░░░░░░░░░░░░░░░░] 45%          │
│                                                         │
│ Topics: /lidar1, /lidar2, /camera/image                │
│ Cache: 950/1000 (Hits: 85%)                           │
│ Timeline Segment: 0                                     │
├─────────────────────────────────────────────────────────┤
│ Controls: Space=Play/Pause ←→=Step ↑↓=Seek q=Quit ?=Help│
└─────────────────────────────────────────────────────────┘
```

## Implementation Details

### Architecture

The TUI consists of several components:

- **Player Class**: Manages ROS 2 service clients and state
- **TUI Class**: Handles terminal interface and user input
- **Main Loop**: Coordinates updates and user interaction

### ROS 2 Integration

The TUI communicates with the rosbag_deck_node using:

- Service clients for control operations
- Topic subscription for status updates
- Asynchronous service calls to prevent UI blocking

### Terminal Handling

- Uses Python's curses library for terminal control
- Handles terminal resize events
- Provides clean shutdown on exit

## Code Structure

### Main Components

```python
# player.py - ROS 2 interface
class RosbagDeckPlayer:
    def __init__(self, node_name: str = '/rosbag_deck'):
        # Initialize ROS 2 clients
    
    def play(self) -> bool:
        # Call play/pause service
    
    def step_forward(self) -> bool:
        # Call step forward service
    
    # ... other control methods

# main.py - Terminal UI
class RosbagDeckTUI:
    def __init__(self, player: RosbagDeckPlayer):
        # Initialize curses
    
    def run(self):
        # Main event loop
    
    def handle_input(self, key: int):
        # Process keyboard input
    
    def update_display(self):
        # Refresh screen content
```

## Customization

### Adding New Controls

To add new keyboard shortcuts:

1. Add key mapping in `handle_input()` method
2. Implement corresponding action
3. Update help screen
4. Document in README

### Modifying Display

The display layout can be customized by modifying the `update_display()` method in the TUI class.

## Dependencies

- Python 3.8+
- rclpy (ROS 2 Python client library)
- curses (standard library)
- rosbag_deck_interface

## Troubleshooting

### TUI doesn't start

- Ensure rosbag_deck_node is running
- Check node namespace matches
- Verify terminal supports curses

### Display issues

- Try resizing terminal window
- Ensure terminal supports colors
- Check TERM environment variable

### Service timeouts

- Verify network connectivity
- Check ROS 2 domain ID
- Increase service timeout in code

## Future Enhancements

- Mouse support for click-based control
- Configurable color schemes
- Multi-bag status display
- Waveform visualization for sensor data
- Recording controls integration

## Examples

### Custom Node Name

```python
# Connect to a specific node
player = RosbagDeckPlayer(node_name='/my_namespace/rosbag_deck')
tui = RosbagDeckTUI(player)
tui.run()
```

### Programmatic Control

```python
# Use the player without TUI
from rosbag_deck_tui.player import RosbagDeckPlayer

player = RosbagDeckPlayer()
player.play()
time.sleep(5)
player.pause()
status = player.get_status()
print(f"Current frame: {status.current_frame}")
```