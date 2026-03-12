# Phase 6: Topic Filtering Enhancements

Upgrade topic filtering from basic CLI whitelist to full interactive selection with storage-level optimization.

See [design/topic-filtering.md](../design/topic-filtering.md) for detailed design.

## 6.1 FFI: Storage-Level Filtering

Expose rosbag2's native `StorageFilter` to skip messages at the SQLite query level.

- [ ] Add `rosbag2_reader_set_filter()` and `rosbag2_reader_reset_filter()` to C wrapper
- [ ] Add safe Rust wrappers in `rosbag_deck_ffi`
- [ ] Extend `BagReader` trait with `set_filter()` / `reset_filter()` methods
- [ ] Add `SetFilter` command to `BagWorker` command channel
- [ ] Worker applies `StorageFilter` to reader on command

## 6.2 CLI: Regex and Exclude Patterns

Match `ros2 bag play` / `ros2 bag record` CLI conventions.

- [ ] `--topics` accepts space-separated values (in addition to comma-separated)
- [ ] `--regex` flag: include topics matching a regex pattern
- [ ] `--exclude` flag: exclude topics by name or regex
- [ ] Resolve patterns against bag metadata at open time into concrete `HashSet<String>`

## 6.3 Core: Two-Level Filter Pipeline

- [ ] On filter change: push `StorageFilter` to workers (I/O efficiency) and update registry (delivery correctness)
- [ ] Handle dynamic filter changes without re-seeking (registry acts as final check)
- [ ] Filter resolution: combine `--topics`, `--regex`, `--exclude` into a single topic set

## 6.4 TUI: Interactive Topic Selection Panel

- [ ] `t` key opens topic selection overlay
- [ ] List all topics with message counts from metadata
- [ ] Checkbox per topic reflecting current filter state
- [ ] `Space` to toggle, `a` select all, `n` deselect all
- [ ] `/` to search/filter the topic list
- [ ] `Enter` to apply, `Esc` to cancel
- [ ] Applies filter via `Deck::set_topic_filter()` + worker `SetFilter`

## Criteria

- [ ] Storage-level filtering reduces I/O (measurable: fewer `read_next()` calls)
- [ ] `--regex` and `--exclude` resolve correctly against bag topic list
- [ ] TUI topic panel is responsive during playback
- [ ] Filter changes take effect within one prefetch cycle
- [ ] `just quality` passes
