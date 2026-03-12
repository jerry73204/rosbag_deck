# Phase 5: Testing & Quality

Comprehensive test coverage, test infrastructure, and CI/CD.

## Test Data

Downloaded via `./scripts/download-test-bags.sh` into `test_bags/` (gitignored).

### Small bags (~3 MB total, `./scripts/download-test-bags.sh small`)

| Bag                                 | Format             | Duration | Messages | Topics                                       | Use case                                 |
|-------------------------------------|--------------------|----------|----------|----------------------------------------------|------------------------------------------|
| `rosbag2/talker`                    | sqlite3            | 4.5s     | 20       | 3 (`/topic`, `/rosout`, `/parameter_events`) | Basic playback, multi-topic              |
| `rosbag2/cdr_test`                  | sqlite3            | 151ms    | 7        | 2 (BasicTypes, Arrays)                       | CDR message types                        |
| `rosbag2/test_bag_for_seek_sqlite3` | sqlite3            | 400ms    | 5        | 1 (`topic1`)                                 | Seek operations                          |
| `rosbag2/test_bag_for_seek_mcap`    | mcap               | 400ms    | 5        | 1 (`topic1`)                                 | Seek with MCAP backend                   |
| `rosbag2/empty_bag`                 | sqlite3            | 0        | 0        | 2 (empty)                                    | Edge case: empty bag                     |
| `rosbag2/wbag`                      | sqlite3 (5 splits) | ~0       | 6,074    | 8 (`/AAA`..`/HHH`)                           | Multi-file split bag, high message count |
| `rosbag2/convert_a`                 | sqlite3            | 100ms    | 150      | 2 (Empty, BasicTypes)                        | Multi-type                               |
| `mcap/demo.mcap`                    | mcap               | —        | —        | —                                            | MCAP format basics                       |
| `mcap/compressed.mcap`              | mcap               | —        | —        | —                                            | MCAP compression                         |
| `mcap/uncompressed.mcap`            | mcap               | —        | —        | —                                            | MCAP uncompressed (2 MB)                 |
| `mcap/conformance/*`                | mcap               | —        | —        | —                                            | MCAP spec conformance                    |

### Large bags (~460 MB total, `./scripts/download-test-bags.sh large`)

| Bag                   | Format    | Duration | Messages | Topics                           | Use case                                   |
|-----------------------|-----------|----------|----------|----------------------------------|--------------------------------------------|
| `zenodo_14209720`     | sqlite3   | 33.9s    | 4,330    | 14 (LiDAR, perception, tracking) | Real-world Autoware perception, 643 MB db3 |
| `zenodo_7780490/faro` | image+PLY | —        | —        | —                                | Visual SLAM reference data                 |

## Test Infrastructure

### Nextest configuration (`.config/nextest.toml`)

- [x] Default profile: 30s slow-timeout, JUnit XML output
- [x] `ci` profile: `fail-fast = true`
- [x] Test groups for serialization:
  - `bag-io`: Serialize tests that open real bag files (prevent I/O contention)

### Test crate (`rosbag_deck_tests`)

- [x] `rosbag_deck_ffi` dev-dependency (needed to open real bags)
- [x] Test data path helper: `fn test_bags_dir() -> PathBuf` (reads `ROSBAG_DECK_TEST_BAGS` env var, falls back to `<workspace>/test_bags/`)
- [x] Skip macros: `skip_if_no_bags!()` and `skip_if_no_large_bags!()` for graceful skipping
- [x] Large test gating via `ROSBAG_DECK_LARGE_TESTS=1` env var

### Test organization

```
packages/testing/rosbag_deck_tests/
├── Cargo.toml
└── tests/
    ├── integration.rs         # mod declarations
    └── integration/
        ├── helpers.rs         # test utilities, macros
        ├── bag_open.rs        # open bags, read metadata
        ├── bag_playback.rs    # play, seek, step, filter
        ├── bag_seek.rs        # seek operations with real bags
        ├── bag_large.rs       # large bag tests (gated)
        ├── cli.rs             # CLI subcommand tests (assert_cmd)
        └── edge_cases.rs      # empty bags, split bags, error handling
```

## 5.1 Unit Tests (per-crate `#[cfg(test)]`, MockBagReader)

Already implemented in Phase 2. No real bags needed.

- [x] Index manager: milestones, visited index, eviction, multi-bag routing
- [x] Message cache: insert/get, memory budget, window eviction, concurrent access
- [x] Bag worker: seek, prefetch, end-of-bag, shutdown, error propagation
- [x] Virtual timeline: real-time delay, speed, best-effort, pause/resume, segments, looping
- [x] Message type registry: build, filter, accept/reject

## 5.2 Integration Tests (real bag files, nextest)

Tests that open real ROS 2 bags via FFI. Skipped when `test_bags/` is not present.

### Bag open & metadata
- [x] Open `talker` bag, verify metadata matches (3 topics, 20 messages, ~4.5s duration)
- [x] Open `cdr_test` bag, verify metadata (2 topics, 7 messages)
- [x] Open `empty_bag`, verify 0 messages
- [x] Open `test_bag_for_seek_sqlite3`, verify 5 messages on `topic1`
- [x] Open `wbag` (multi-file split), verify 6,074 messages across 8 topics

### Playback
- [x] Play `talker` in best-effort mode, collect all messages, verify count and timestamp order
- [x] Play `convert_a`, verify messages from both topics are interleaved by timestamp
- [x] Play `wbag`, verify all 6,074 messages delivered in order
- [x] Topic filter: play `talker` with filter on `/topic` only, verify no `/rosout` messages

### Seek
- [x] Seek to midpoint of `talker`, verify first message is at or after seek target
- [x] Seek to start, verify first message timestamp matches bag start
- [x] Seek to end, verify no messages (or end-of-bag)
- [x] Seek on `test_bag_for_seek_sqlite3`, verify correct positioning

### Large bag (gated behind `ROSBAG_DECK_LARGE_TESTS=1`)
- [x] Open `zenodo_14209720` (643 MB), verify metadata loads instantly (no full scan)
- [x] Seek to midpoint of large bag, verify response time < 1s
- [x] Play 1000 messages from large bag, verify no OOM and timestamp order

### CLI
- [x] `rosbag_deck info <talker>` prints correct topic/count/duration table
- [x] `rosbag_deck play <talker> --no-tui` outputs messages to stdout
- [x] `rosbag_deck play <talker> --no-tui --topics /topic` filters correctly
- [x] `rosbag_deck info <nonexistent>` exits with error

## 5.3 Performance Benchmarks

Criterion benchmarks in `packages/rosbag_deck_core/benches/playback.rs`. Run with `just bench`.

- [x] `deck_open` — Open time across message counts (100, 1K, 10K)
- [x] `sequential_playback` — Throughput across topic counts, message counts, data sizes
- [x] `seek` — Seek latency across message counts (1K, 10K)
- [x] `step_forward` — Single-step latency
- [x] `filtered_playback` — Playback with 7/8 topics filtered out

## 5.4 CI/CD

GitHub Actions workflow in `.github/workflows/ci.yml`.

- [x] `check` job: clippy + nightly fmt check
- [x] `test` job: download small test bags, run nextest
- [x] `build` job: colcon build
- [x] `large-tests` job: runs on push to main only, downloads all bags, `ROSBAG_DECK_LARGE_TESTS=1`
- [ ] Coverage reporting (cargo-llvm-cov) — TODO: add when needed
- [x] Platform: Linux (Ubuntu 22.04 with ROS 2 Humble)

## Criteria

- [x] All unit tests pass (37 tests, MockBagReader)
- [x] All integration tests pass with small test bags (35 tests)
- [x] Large bag tests pass when enabled (4 tests)
- [x] CLI tests pass via assert_cmd
- [x] `just quality` clean (clippy + nextest)
- [x] No clippy warnings
- [x] Performance benchmarks run via `just bench`
