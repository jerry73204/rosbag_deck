# Phase 3: CLI & TUI

Build the command-line interface and terminal UI.

## 3.1 CLI (`rosbag-deck-cli`)

- [x] `info <bag_path>` — Show bag metadata (topics, message counts, duration)
- [x] `play <bag_path>` — Play a bag file (launches TUI by default)
- [x] `--no-tui` flag for headless playback (prints messages to stdout)
- [x] `--topics` flag for topic filtering (comma-separated)
- [x] `--rate` flag for playback speed
- [x] `--loop` flag for loop playback
- [x] `--storage` flag for storage backend selection

## 3.2 TUI (ratatui)

- [x] Tape deck-style visual layout (header, transport bar, message log, help bar)
- [x] Playback controls: play/pause (Space), stop (s), step forward/backward (←→/hl)
- [x] Real-time status display (current time, duration, speed, state)
- [x] Timeline progress bar with playback position
- [x] Scrollable message log viewer (last 200 messages)
- [x] Seek dialog (g — enter seconds or MM:SS)
- [x] Loop toggle (o)
- [x] Speed control (±)
- [x] Home/End to jump to start/end
- [x] Keyboard shortcuts for all controls
- [x] TUI as optional feature (`--features tui`, enabled by default)

## Criteria

- [x] `rosbag-deck info <bag>` prints topic/type/count/duration summary
- [x] `rosbag-deck play <bag>` opens TUI with working controls
- [x] `rosbag-deck play <bag> --no-tui` runs headless playback
- [x] All keyboard shortcuts functional
- [x] `just quality` passes (clippy clean, all tests green)
- [x] `just build` succeeds (colcon + cargo)
