# Phase 7: Bag Editor Tool

Add a `rosbag_deck edit` subcommand for slicing, merging, filtering, and transforming bag files.

See [design/bag-editor.md](../design/bag-editor.md) for detailed design.

## 7.1 FFI: Writer API

Expose rosbag2's `Writer` via the C wrapper.

- [ ] Add `Rosbag2Writer` struct to C wrapper (open, create_topic, write, close)
- [ ] Add safe Rust wrappers in `rosbag_deck_ffi`
- [ ] Define `BagWriter` trait in `rosbag_deck_core`
- [ ] Implement `Rosbag2Writer` backed by FFI
- [ ] Support storage format selection (sqlite3, mcap)
- [ ] Support compression options (none, zstd, lz4)

## 7.2 Core: Edit Pipeline

Streaming pipeline that reads, filters, transforms, and writes messages.

- [ ] `EditPipeline` struct with builder pattern for configuration
- [ ] Time range filtering: `--start` / `--end` / `--duration`
- [ ] Topic filtering: reuse `StorageFilter` + registry from Phase 6
- [ ] `EditStats` return type with message counts and timing
- [ ] Single-bag passthrough (simplest case)

## 7.3 Multi-Bag Merge

- [ ] Priority queue (`BinaryHeap`) merge of N readers by timestamp
- [ ] Bounded memory: one message per reader in the heap
- [ ] Deduplication of topic metadata across inputs
- [ ] Concatenation order respects timestamps (not input order)

## 7.4 Timestamp Transform

- [ ] `--time-offset`: shift all timestamps by ±N seconds
- [ ] `--time-scale`: scale timestamps relative to first message
- [ ] Transforms applied after filtering, before writing
- [ ] Validates that output timestamps remain monotonic (warn if scale inverts order)

## 7.5 CLI: `edit` Subcommand

- [ ] Argument parsing with clap (see design doc for full flag list)
- [ ] Progress output: message counter, throughput (msgs/sec)
- [ ] `--dry-run`: run pipeline without writing, print summary
- [ ] `--verbose`: per-message output
- [ ] Error handling: partial write cleanup on failure

## 7.6 Output Options

- [ ] `--format sqlite3|mcap` — output storage format
- [ ] `--compress none|zstd|lz4` — compression
- [ ] `--max-size` — split output at N bytes

## Criteria

- [ ] Single-bag slice: `edit input/ -o output/ --start 10 --end 40` produces correct output
- [ ] Multi-bag merge: timestamps globally sorted in output
- [ ] Topic filter: only selected topics in output bag
- [ ] Timestamp offset: all output timestamps shifted by exact amount
- [ ] Timestamp scale: inter-message intervals scaled correctly
- [ ] Dry run matches actual run counts
- [ ] Large bag (>1GB) completes without excessive memory usage
- [ ] `just quality` passes
