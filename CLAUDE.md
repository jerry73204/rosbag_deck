# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RosBag Deck is an interactive ROS 2 bag player with tape deck-style controls. It provides frame-level precision playback and handles large bag files efficiently through a streaming architecture.

The project is written in Rust with Python bindings via PyO3.

## Justfile Recipes

```bash
just setup    # Install colcon-cargo-ros2
just build    # colcon build with dev-release profile
just check    # clippy (deny warnings) + nightly fmt check
just test     # cargo nextest run
just quality  # check + test (run before finishing implementation work)
just format   # cargo +nightly fmt
just clean    # cargo clean + remove colcon dirs (build/, install/, log/)
```

The default Cargo profile is `dev-release` (inherits `release` with `debug`, `debug-assertions`, `overflow-checks` enabled). Override with `just --set profile release`.

## Running Builds, Checks, and Tests

Always use justfile recipes instead of raw cargo/colcon/nextest commands. This ensures consistent profiles, flags, and tool configurations. Only use direct commands (e.g., `cargo clippy -p ...`, `cargo nextest run -p ...`) when you need to target a specific package or pass custom flags not covered by the recipes.

```bash
just build    # Build everything (colcon + cargo, dev-release profile)
just check    # Clippy + nightly fmt check
just test     # Run all Rust tests via nextest
just quality  # check + test (run before finishing work)
just format   # Auto-format with nightly rustfmt
```

### Python
```bash
uv sync
cd packages/rosbag_deck_python && maturin develop
uv run ruff check packages/rosbag_deck_python/
uv run pytest packages/rosbag_deck_python/ -v
```

## Architecture

### Package Dependencies
```
rosbag_deck_ffi (FFI to ROS 2: rosbag2, etc.)
    ↓
rosbag_deck_core (Rust core library)
    ↓              ↓
rosbag_deck       rosbag_deck_python (PyO3)
    (CLI + TUI)

rosbag_deck_tests (integration tests)
```

### Packages

| Package | Path | Description |
|---------|------|-------------|
| `rosbag_deck_core` | `packages/rosbag_deck_core/` | Core library: timeline, caching, streaming, publishing, CDR stamp patching |
| `rosbag_deck` | `packages/rosbag_deck/` | CLI (play, info, edit subcommands) + optional ratatui TUI |
| `rosbag_deck_python` | `packages/rosbag_deck_python/` | PyO3 Python bindings |
| `rosbag_deck_ffi` | `packages/rosbag_deck_ffi/` | FFI bindings to ROS 2 C/C++ libraries |
| `rosbag_deck_tests` | `packages/testing/rosbag_deck_tests/` | Integration tests (nextest) |

### Key Design Patterns

1. **Streaming Architecture**: Stream messages from disk rather than loading entire bags into memory.
2. **Virtual Timeline System**: `VirtualTimeline` maps wall-clock time to bag time for pacing. Supports `resync()` after blocking operations to prevent message bursts.
3. **Thread Safety**: All core components use safe Rust concurrency primitives.
4. **FFI for ROS 2**: Reuse rosbag2 via FFI rather than reimplementing bag format parsing.
5. **Tracing/Logging**: Uses `tracing` crate throughout. TUI mode captures logs via a channel-based `tracing::Layer` subscriber (`tui/log_subscriber.rs`). ROS 2 C++ rcutils logs are routed through a custom handler into `tracing` (`install_ros_log_handler`). Non-TUI modes use `tracing-subscriber` with `EnvFilter` (default level: `info`, override with `RUST_LOG`).
6. **CDR Stamp Patching**: Monotonic loop mode patches `Header.stamp` directly in serialized CDR data (`stamp.rs`) without deserialization, using the encapsulation header to detect endianness.

### ROS 2 Integration

- Packages declare `<build_type>ament_cargo</build_type>` in package.xml
- Built via colcon-cargo-ros2 plugin (`just setup` to install)
- ROS 2 Humble or later
- `.envrc` sources `/opt/ros/humble/setup.bash` and `install/setup.bash` automatically

### Releasing / .deb Packaging

- `.github/workflows/release-deb.yml` builds a `ros-humble-rosbag-deck` .deb on `vX.Y.Z` tag push
- Uses `cargo-deb` with metadata in `packages/rosbag_deck/Cargo.toml` (`[package.metadata.deb]`)
- Ament integration files (index marker, environment hooks, local_setup scripts) are in `packages/rosbag_deck/deb/ament/` and are included as cargo-deb assets
- The .deb installs the binary to `/opt/ros/humble/bin/` with full ament package structure so `source /opt/ros/humble/setup.bash` adds it to PATH
- Bloom/ROS buildfarm is **not supported** for Rust packages — see [ros2-rust#310](https://github.com/ros2-rust/ros2_rust/issues/310)

### Python Package Management

- Uses UV (not pip/rye) for workspace management
- Python bindings built with maturin (PyO3)
- Root `pyproject.toml` defines UV workspace

## Quality Gate

Before finishing any implementation work, always run:

```bash
just quality
```

This runs clippy (with `-D warnings`) and all tests via nextest. All warnings and test failures must be resolved before considering the work complete.

## Temporary Files

Place all temporary/scratch files and scripts in `tmp/` (git-ignored). Do not create temporary files in the project root or other directories.

## Development Principles

- If a feature or work is not done yet, always leave TODO comments. Don't generate dummy values or any kind of silent error.
- Use `thiserror` for library error types, `anyhow` for application error handling.
- Prefer channels over shared mutable state for thread communication.
- Use `cargo +nightly fmt` for formatting (nightly rustfmt).

## Testing

### Pacing test script

`tmp/test_pacing.sh` runs the player alongside a ROS 2 Python subscriber to measure wall-clock timing gaps. The subscriber auto-matches the publisher's QoS and records per-message timing to CSV. Usage:

```bash
./tmp/test_pacing.sh [BAG_PATH] [TOPIC]
```

Requires: ROS 2 running, the dev-release binary built (`just build`).
