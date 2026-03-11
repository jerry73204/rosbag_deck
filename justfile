profile := "dev-release"

default:
    @just --list

build:
    cargo build --profile {{ profile }}

check:
    cargo clippy --workspace --profile {{ profile }} -- -D warnings
    cargo +nightly fmt --check

test:
    cargo nextest run --profile {{ profile }}

format:
    cargo +nightly fmt

clean:
    cargo clean
    rm -rf build install log
