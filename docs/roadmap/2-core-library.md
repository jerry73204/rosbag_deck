# Phase 2: Core Library

Build the playback engine on top of the `BagReader` trait from Phase 1. Leverages native backend seeking (sqlite3 B-tree index, MCAP chunk index) instead of scanning every message at open time.

## 2.1 Index Manager (Two-Tier Sparse Index)

Lazy, memory-bounded index that avoids full-bag iteration at open time.

### Tier 1: Milestones (permanent, O(1) to build)
- [x] Build milestone entries from `BagMetadata` time ranges (arithmetic, no I/O)
- [x] Divide each bag's `[start_time, end_time]` into N uniform intervals (default 1000)
- [x] Store `{ timestamp_ns, bag_id, enriched }` per milestone
- [x] Enrich milestones lazily when nearby messages are actually read
- [x] Multi-bag routing: `bags_containing(timestamp_ns)` from milestone time ranges

### Tier 2: Visited Index (bounded, evictable)
- [x] Record every message that flows through playback into `BTreeMap<i64, VisitedEntry>`
- [x] Configurable entry limit (default 50K entries)
- [x] Batch eviction: drop farthest 25% from cursor when limit reached
- [x] Exact timestamp lookup for previously-visited regions

## 2.2 Message Cache

Thread-safe message cache bounded by memory budget.

- [x] `BTreeMap<i64, RawMessage>` keyed by timestamp, protected by `Mutex`
- [x] Configurable memory budget (default 256 MB), tracked via `data.len()` sum
- [x] Window-based eviction: drop messages furthest from cursor
- [x] Full cache clear on distant seek (jump beyond prefetch window)
- [x] `get()`, `insert()`, `clear()` API

## 2.3 Bag Worker

One background I/O thread per bag file. Owns a `BagReader`, fills shared cache and index.

- [x] Command channel: `Seek { timestamp_ns }`, `Prefetch { count }`, `Shutdown`
- [x] Event channel: `Ready`, `SeekComplete`, `EndOfBag`, `Error`
- [x] On seek: delegate to `reader.seek()` (backend's native O(log n)), then prefetch forward
- [x] On prefetch: read next N messages, insert into cache + visited index
- [x] Graceful shutdown and error propagation

## 2.4 Virtual Timeline

Maps wall-clock time to bag time. Pure logic, no I/O.

- [x] Real-time mode: `bag_time = bag_anchor + (now - wall_anchor) * speed`
- [x] Best-effort mode: return messages immediately, no timing
- [x] `delay_until(msg_timestamp_ns) -> Option<Duration>` for playback pacing
- [x] Segment IDs: increment on seek/step/rewind, discard stale prefetch results
- [x] Playback speed control (`set_speed(f64)`)
- [x] Pause/resume with timeline anchor adjustment
- [x] Loop playback support

## 2.5 Message Type Registry

Topic-to-type map with optional filtering. Built from merged `BagMetadata`.

- [x] Build from `BagMetadata::topics` at open time
- [x] Topic filter: `set_filter(Option<HashSet<String>>)`
- [x] `is_accepted(topic)` check for playback loop
- [x] `topic_info(name)` lookup

## 2.6 Deck (Public API Facade)

Unified API that owns all components above.

- [x] `open(readers: Vec<Box<dyn BagReader>>, config) -> Result<Self>` — build milestones, spawn workers, initial prefetch
- [x] `play()`, `pause()`, `stop()`, `toggle_play_pause()`
- [x] `seek_to_time(ns)`, `seek_to_ratio(0.0..1.0)`
- [x] `step_forward()`, `step_backward()`
- [x] `set_speed(f64)`, `set_mode(RealTime|BestEffort)`, `set_looping(bool)`
- [x] `set_topic_filter(Option<HashSet<String>>)`
- [x] `next_message() -> Result<Option<TimedMessage>>` — core playback tick
- [x] Backward playback via seek-back-and-reverse-iterate strategy

## Criteria

- [x] `Deck::open()` completes in O(1) (metadata only, no full-bag scan)
- [x] Seeking delegates to backend's native index (O(log n))
- [x] Streaming playback with play/pause/stop/step controls
- [x] Seek creates new timeline segments correctly
- [x] Cache hit rate > 90% for sequential playback
- [x] Index and cache stay within configured memory bounds
- [x] Multi-bag playback merges messages by timestamp
- [x] `just quality` passes (clippy clean, all tests green)

## Tests

All tests use a `MockBagReader` that generates synthetic messages in-memory, so no ROS 2 installation or real bag files are needed.

### 2.1 Index Manager

- `test_milestones_from_single_bag` — milestones span the full time range, count matches config
- `test_milestones_from_multi_bag` — overlapping bags produce milestones for each, `bags_containing()` returns correct bag IDs
- `test_visited_record_and_lookup` — record entries, look up by exact timestamp
- `test_visited_eviction` — fill to limit, verify batch eviction drops farthest entries from cursor
- `test_visited_eviction_preserves_cursor_window` — entries near cursor survive eviction
- `test_bags_containing_non_overlapping` — bags with disjoint time ranges route correctly
- `test_bags_containing_overlapping` — overlapping bags both returned for timestamps in the overlap

### 2.2 Message Cache

- `test_insert_and_get` — insert message, retrieve by timestamp
- `test_memory_budget_eviction` — insert messages exceeding budget, verify oldest/farthest evicted and total bytes under limit
- `test_window_eviction_keeps_cursor_neighborhood` — messages near cursor survive, distant ones evicted
- `test_distant_seek_clears_cache` — after a large jump, cache is empty
- `test_concurrent_access` — spawn reader and writer threads, verify no panics or data corruption

### 2.3 Bag Worker

- `test_seek_and_prefetch` — send Seek, verify messages appear in cache and visited index
- `test_prefetch_count` — send Prefetch { count: N }, verify exactly N messages loaded
- `test_end_of_bag_event` — exhaust the mock reader, verify EndOfBag event received
- `test_shutdown` — send Shutdown, verify thread joins cleanly
- `test_error_propagation` — mock reader returns error, verify Error event with message

### 2.4 Virtual Timeline

- `test_realtime_delay` — at 1x speed, delay_until matches wall-clock difference between messages
- `test_speed_2x` — at 2x speed, delays are halved
- `test_best_effort_no_delay` — best-effort mode always returns `None` from delay_until
- `test_pause_resume_adjusts_anchor` — pause, wait, resume: next delay is unaffected by pause duration
- `test_segment_id_increments_on_seek` — each seek/step bumps segment_id
- `test_loop_wraps_to_start` — when bag_time exceeds end, loop mode wraps to start

### 2.5 Message Type Registry

- `test_build_from_metadata` — all topics from metadata are present
- `test_filter_accepts_matching` — set filter, matching topics accepted
- `test_filter_rejects_non_matching` — non-matching topics rejected
- `test_no_filter_accepts_all` — with no filter, all topics accepted

### 2.6 Deck (Integration)

- `test_open_builds_milestones` — open with mock reader, verify milestones exist and no messages were iterated
- `test_sequential_playback` — play in best-effort mode, collect all messages, verify timestamp order
- `test_seek_and_play` — seek to midpoint, play forward, verify first message is at or after seek target
- `test_step_forward` — step one message at a time, verify cursor advances
- `test_topic_filter` — set filter to one topic, verify only that topic's messages returned
- `test_multi_bag_merge_order` — open two mock bags with interleaved timestamps, verify merged output is globally sorted
- `test_play_pause_resume` — play, pause, verify no messages during pause, resume and verify messages continue
