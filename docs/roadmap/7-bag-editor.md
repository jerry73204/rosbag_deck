# Phase 7: Bag Editor Tool

Add a `rosbag_deck edit` subcommand for slicing, merging, filtering, and transforming bag files.

See [design/bag-editor.md](../design/bag-editor.md) for detailed design.

## 7.1 FFI: Writer API

Expose rosbag2's `Writer` via the C wrapper.

- [x] Add `Rosbag2Writer` struct to C wrapper (open, create_topic, write, close)
- [x] Add safe Rust wrappers in `rosbag_deck_ffi`
- [x] Define `BagWriter` trait in `rosbag_deck_core`
- [x] Implement `Rosbag2Writer` backed by FFI
- [x] Support storage format selection (sqlite3, mcap)
- [ ] Support compression options (none, zstd, lz4)

## 7.2 Core: Edit Pipeline

Streaming pipeline that reads, filters, transforms, and writes messages.

- [x] `EditPipeline` struct with builder pattern for configuration
- [x] Time range filtering: `--start` / `--end` / `--duration`
- [x] Topic filtering: reuse `StorageFilter` + registry from Phase 6
- [x] `EditStats` return type with message counts and timing
- [x] Single-bag passthrough (simplest case)

## 7.3 Multi-Bag Merge

- [x] Priority queue (`BinaryHeap`) merge of N readers by timestamp
- [x] Bounded memory: one message per reader in the heap
- [x] Deduplication of topic metadata across inputs
- [x] Concatenation order respects timestamps (not input order)

## 7.4 Timestamp Transform

- [x] `--time-offset`: shift all timestamps by ±N seconds
- [x] `--time-scale`: scale timestamps relative to first message
- [x] Transforms applied after filtering, before writing
- [ ] Validates that output timestamps remain monotonic (warn if scale inverts order)

## 7.5 CLI: `edit` Subcommand

- [x] Argument parsing with clap (see design doc for full flag list)
- [x] Progress output: message counter, throughput (msgs/sec)
- [x] `--dry-run`: run pipeline without writing, print summary
- [x] `--verbose`: per-message output
- [ ] Error handling: partial write cleanup on failure

## 7.6 Output Options

- [x] `--format sqlite3|mcap` — output storage format
- [ ] `--compress none|zstd|lz4` — compression
- [ ] `--max-size` — split output at N bytes

## Criteria

- [x] Single-bag slice: `edit input/ -o output/ --start 10 --end 40` produces correct output
- [x] Multi-bag merge: timestamps globally sorted in output
- [x] Topic filter: only selected topics in output bag
- [x] Timestamp offset: all output timestamps shifted by exact amount
- [x] Timestamp scale: inter-message intervals scaled correctly
- [x] Dry run matches actual run counts
- [ ] Large bag (>1GB) completes without excessive memory usage
- [x] `just quality` passes
