# Phase 4: Python Bindings

Thin PyO3 wrapper around the Rust core library.

## PyO3 Module (`rosbag-deck-python`)

- [ ] Expose `Deck` class with playback controls (play, pause, stop, step, seek)
- [ ] Expose `BagInfo` dataclass (topics, message counts, duration, storage format)
- [ ] Expose `PlaybackStatus` dataclass (state, current time, frame index)
- [ ] Callback registration for status updates and message events
- [ ] Context manager support (`with Deck(path) as deck:`)

## Build Integration

- [ ] Build via maturin (PyO3 extension module)
- [ ] Integrate with colcon-cargo-ros2 for ament installation
- [ ] Publish to PyPI (optional)

## Criteria

- `import rosbag_deck` works after `pip install` or colcon build
- Python API covers all core playback operations
- Callback latency < 100 microseconds
- No memory leaks across FFI boundary
