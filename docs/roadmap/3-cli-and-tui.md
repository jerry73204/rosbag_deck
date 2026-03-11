# Phase 3: CLI & TUI

Build the command-line interface and terminal UI.

## 3.1 CLI (`rosbag-deck-cli`)

- [ ] `play <bag_path>` — Play a bag file (launches TUI by default)
- [ ] `info <bag_path>` — Show bag metadata (topics, message counts, duration)
- [ ] `--no-tui` flag for headless playback
- [ ] `--topics` flag for topic filtering
- [ ] `--rate` flag for playback speed

## 3.2 TUI (ratatui)

- [ ] Tape deck-style visual layout
- [ ] Playback controls: play, pause, stop, step forward/backward
- [ ] Real-time status display (current time, frame, topic stats)
- [ ] Scrollable message log viewer
- [ ] Seek dialog (jump to timestamp)
- [ ] File browser dialog
- [ ] Loop toggle
- [ ] Keyboard shortcuts for all controls

## Criteria

- `rosbag-deck play <bag>` opens TUI with working controls
- `rosbag-deck info <bag>` prints topic/type/count/duration summary
- All keyboard shortcuts functional
- Responsive UI even with large bag files (streaming architecture)
