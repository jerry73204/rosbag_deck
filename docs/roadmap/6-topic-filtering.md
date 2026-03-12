# Phase 6: Topic Filtering Enhancements

Upgrade topic filtering from basic CLI whitelist to full interactive selection with storage-level optimization.

See [design/topic-filtering.md](../design/topic-filtering.md) for detailed design.

## 6.1 FFI: Storage-Level Filtering

Expose rosbag2's native `StorageFilter` to skip messages at the SQLite query level.

- [x] Add `rosbag2_reader_set_filter()` and `rosbag2_reader_reset_filter()` to C wrapper
- [x] Add safe Rust wrappers in `rosbag_deck_ffi`
- [x] Extend `BagReader` trait with `set_filter()` / `reset_filter()` methods
- [x] Add `SetFilter` command to `BagWorker` command channel
- [x] Worker applies `StorageFilter` to reader on command

## 6.2 CLI: Regex and Exclude Patterns

Match `ros2 bag play` / `ros2 bag record` CLI conventions.

- [x] `--topics` accepts space-separated values (in addition to comma-separated)
- [x] `--regex` flag: include topics matching a regex pattern
- [x] `--exclude` flag: exclude topics by name or regex
- [x] Resolve patterns against bag metadata at open time into concrete `HashSet<String>`

## 6.3 Core: Two-Level Filter Pipeline

- [x] On filter change: push `StorageFilter` to workers (I/O efficiency) and update registry (delivery correctness)
- [x] Handle dynamic filter changes without re-seeking (registry acts as final check)
- [x] Filter resolution: combine `--topics`, `--regex`, `--exclude` into a single topic set

## 6.4 TUI: Interactive Topic Selection Panel

- [x] `t` key opens topic selection overlay
- [x] List all topics with message counts from metadata
- [x] Checkbox per topic reflecting current filter state
- [x] `Space` to toggle, `a` select all, `n` deselect all
- [x] `/` to search/filter the topic list
- [x] `Enter` to apply, `Esc` to cancel
- [x] Applies filter via `Deck::set_topic_filter()` + worker `SetFilter`

## Criteria

- [x] Storage-level filtering reduces I/O (measurable: fewer `read_next()` calls)
- [x] `--regex` and `--exclude` resolve correctly against bag topic list
- [x] TUI topic panel is responsive during playback
- [x] Filter changes take effect within one prefetch cycle
- [x] `just quality` passes
