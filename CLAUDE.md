# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RosBag Deck is an interactive ROS 2 bag player with tape deck-style controls. It provides frame-level precision playback and handles large bag files efficiently through a streaming architecture.

The project is written in Rust with Python bindings via PyO3.

## Build Commands

### Rust (Cargo)
```bash
# Build all workspace members
cargo build

# Build a specific package
cargo build -p rosbag-deck
cargo build -p rosbag-deck-cli
cargo build -p rosbag-deck-python
cargo build -p rosbag-deck-ffi
```

### Colcon (ROS 2 integration via colcon-cargo-ros2)
```bash
# Full build
colcon build

# Individual package
colcon build --packages-select rosbag_deck
```

### Python
```bash
# Install Python dependencies
uv sync

# Build Python bindings (via maturin)
cd packages/rosbag-deck-python && maturin develop
```

## Linting

```bash
# Rust
cargo clippy --workspace
cargo fmt --check

# Python
uv run ruff check packages/rosbag-deck-python/
```

## Testing

```bash
# Rust tests (nextest)
cargo nextest run

# Python tests
uv run pytest packages/rosbag-deck-python/ -v

# Full suite via justfile
just test
```

## Architecture

### Package Dependencies
```
rosbag-deck-ffi (FFI to ROS 2: rosbag2, etc.)
    ↓
rosbag-deck (Rust core library)
    ↓              ↓
rosbag-deck-cli   rosbag-deck-python (PyO3)
    (CLI + TUI)

rosbag-deck-tests (integration tests)
```

### Packages

| Package | Path | Description |
|---------|------|-------------|
| `rosbag-deck` | `packages/rosbag-deck/` | Core library: indexing, caching, streaming, virtual timeline |
| `rosbag-deck-cli` | `packages/rosbag-deck-cli/` | CLI + optional ratatui TUI |
| `rosbag-deck-python` | `packages/rosbag-deck-python/` | PyO3 Python bindings |
| `rosbag-deck-ffi` | `packages/rosbag-deck-ffi/` | FFI bindings to ROS 2 C/C++ libraries |
| `rosbag-deck-tests` | `packages/testing/rosbag-deck-tests/` | Integration tests (nextest) |

### Key Design Patterns

1. **Streaming Architecture**: Stream messages from disk rather than loading entire bags into memory.
2. **Virtual Timeline System**: Handle rewind operations by creating timeline segments.
3. **Thread Safety**: All core components use safe Rust concurrency primitives.
4. **FFI for ROS 2**: Reuse rosbag2 via FFI rather than reimplementing bag format parsing.

### ROS 2 Integration

- Packages declare `<build_type>ament_cargo</build_type>` in package.xml
- Built via colcon-cargo-ros2 plugin
- ROS 2 Humble or later

### Python Package Management

- Uses UV (not pip/rye) for workspace management
- Python bindings built with maturin (PyO3)
- Root `pyproject.toml` defines UV workspace

### Build Helpers (justfile)

```bash
just build    # Build with dev-release profile (release + debug info/assertions)
just check    # clippy + fmt check
just test     # cargo nextest run
just format   # cargo fmt
just clean    # cargo clean + remove colcon dirs
```

The default Cargo profile is `dev-release` (inherits `release` with debug, debug-assertions, overflow-checks enabled).

## Development Principles

- If a feature or work is not done yet, always leave TODO comments. Don't generate dummy values or any kind of silent error.
- Use `thiserror` for library error types, `anyhow` for application error handling.
- Prefer channels over shared mutable state for thread communication.
