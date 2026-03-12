# Phase 4: Python Bindings

Thin PyO3 wrapper around the Rust core library.

## 4.1 PyO3 Module (`rosbag-deck-python`)

### Data classes (`#[pyclass(frozen, get_all)]`)
- [x] `TopicInfo` — name, type_name, serialization_format, message_count
- [x] `BagInfo` — topics, message_count, duration_secs, start_time_ns, end_time_ns, storage_identifier
- [x] `Message` — timestamp_ns, topic, data (bytes)
- [x] `PlaybackStatus` — state, cursor_ns, speed, looping, segment_id

### Deck class (`#[pyclass(unsendable)]`)
- [x] `Deck(path, storage="")` — open bag via FFI, auto-detect storage backend
- [x] `info()` → `BagInfo`
- [x] `status()` → `PlaybackStatus`
- [x] `play()`, `pause()`, `stop()`, `toggle_play_pause()`
- [x] `seek_to_time(timestamp_ns)`, `seek_to_ratio(ratio)`
- [x] `step_forward()`, `step_backward()` → `Optional[Message]`
- [x] `next_message()` → `Optional[Message]`
- [x] `set_speed(speed)`, `set_mode(mode)`, `set_looping(looping)`
- [x] `set_topic_filter(topics=None)`

### Python protocols
- [x] Context manager (`__enter__` / `__exit__` — stop on exit)
- [x] Iterator protocol (`__iter__` / `__next__` — yields messages)
- [x] `__repr__` — `Deck(messages=20, duration=4.5s, state=Stopped)`

### Module-level
- [x] `__version__` from `CARGO_PKG_VERSION`

## 4.2 Build Integration

### colcon build (ament_python + pre-built cdylib)

The Python package uses `ament_python` build type with a custom `setup.py` that copies the pre-built cdylib from the cargo target directory. This avoids needing maturin or setuptools-rust at colcon build time.

Build flow:
1. `ament_cargo` builds the `rosbag-deck` crate (and its cdylib dependency `rosbag-deck-python`) via cargo workspace
2. `ament_python` runs `setup.py install` for `rosbag_deck_python`, which:
   - Copies the pre-built `librosbag_deck.so` → `rosbag_deck.cpython-*.so`
   - Installs `__init__.py` + `.so` to `lib/python3.10/site-packages/rosbag_deck/`
   - Creates PYTHONPATH hook so `source install/setup.bash` makes it importable
   - Installs ament index markers and `package.xml`

- [x] `just build` compiles Rust + installs Python package
- [x] `source install/setup.bash && python3 -c "import rosbag_deck"` works
- [ ] Publish to PyPI via maturin (optional, separate from colcon)

## Criteria

- [x] `cargo clippy` clean, `cargo +nightly fmt --check` clean
- [x] All 34 unit tests pass
- [x] `import rosbag_deck` works after `just build && source install/setup.bash`
- [x] All 5 Python classes importable (Deck, BagInfo, TopicInfo, Message, PlaybackStatus)
- [ ] Python API covers all core playback operations (manual smoke test with real bag)
- [ ] No memory leaks across FFI boundary
