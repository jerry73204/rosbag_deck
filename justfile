profile := "dev-release"

default:
    @just --list

build:
    colcon build --cargo-args --profile {{ profile }}

check:
    cargo clippy --workspace --profile {{ profile }} -- -D warnings
    cargo +nightly fmt --check

test:
    cargo nextest run --profile {{ profile }}

format:
    cargo +nightly fmt

setup:
    pip install colcon-cargo-ros2

generate-bindings:
    cargo build --profile {{ profile }} -p rosbag-deck-ffi --features generate-bindings

clean:
    cargo clean
    rm -rf build install log
