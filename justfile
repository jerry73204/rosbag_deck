profile := "dev-release"

default:
    @just --list

build:
    colcon build --cargo-args --profile {{ profile }}

check:
    cargo clippy --workspace --profile {{ profile }} -- -D warnings
    cargo +nightly fmt --check

test:
    cargo nextest run --cargo-profile {{ profile }}

format:
    cargo +nightly fmt

setup:
    pip install colcon-cargo-ros2

generate-bindings:
    cargo build --profile {{ profile }} -p rosbag_deck_ffi --features generate-bindings

run *args:
    cargo run --profile {{ profile }} -p rosbag_deck -- {{ args }}

bench:
    cargo bench -p rosbag_deck_core --bench playback

quality: check test

clean:
    cargo clean
    rm -rf build install log
