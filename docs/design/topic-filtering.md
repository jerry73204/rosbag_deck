# Design: Topic Filtering Enhancements

## Status Quo

Topic filtering exists but is limited:

- **CLI**: `--topics /a,/b` whitelist via comma-separated flag
- **Core**: `Deck::set_topic_filter(Option<HashSet<String>>)` — applied at message delivery time in `next_message()` / `try_next_message()`
- **TUI**: No interactive topic selection; filter is set once at startup from CLI args
- **FFI**: rosbag2's `StorageFilter` (native topic filter on the reader) is **not exposed** — filtering happens in Rust after messages are read from disk

## Goals

1. Interactive topic selection in the TUI
2. CLI flags matching `ros2 bag play` conventions
3. Push filtering down to the storage layer for I/O efficiency
4. Regex and exclude patterns

## Design

### 1. FFI: Expose `StorageFilter`

rosbag2 natively supports topic filtering at the storage layer via `Reader::set_filter(StorageFilter)` and `Reader::reset_filter()`. This avoids reading unwanted messages from SQLite entirely.

Add to the C wrapper (`rosbag2_wrapper.h`):

```c
int rosbag2_reader_set_filter(Rosbag2Reader *reader,
                              const char **topics, size_t topic_count);
int rosbag2_reader_reset_filter(Rosbag2Reader *reader);
```

Add corresponding Rust safe wrappers in `rosbag_deck_ffi`.

### 2. Core: Two-Level Filtering

Filtering operates at two levels:

| Level | Where | What | Why |
|-------|-------|------|-----|
| Storage | `BagReader::set_filter()` | Skip messages in SQLite query | Reduces I/O and cache pressure |
| Registry | `MessageTypeRegistry::is_accepted()` | Final check at delivery | Handles dynamic filter changes without re-seeking |

When the filter changes:
1. Send new `StorageFilter` to each `BagWorker` (new `SetFilter` command)
2. Worker applies it to the reader and prefetches
3. Registry filter updates immediately for the delivery check

Add to `BagReader` trait:

```rust
fn set_filter(&mut self, topics: &[String]) -> Result<()>;
fn reset_filter(&mut self) -> Result<()>;
```

### 3. CLI: Match `ros2 bag play` Conventions

Current:
```
--topics /a,/b          # comma-separated whitelist
```

Add:
```
--topics /a /b /c       # space-separated (ros2 bag play style)
--regex 'camera.*'      # include by regex (like ros2 bag record -e)
--exclude '/rosout'     # exclude by name or regex (like ros2 bag record -x)
```

The `--topics` flag should accept both comma-separated and space-separated values for compatibility. Internally, regex and exclude patterns are resolved to a concrete topic set at open time using the bag's metadata topic list.

### 4. TUI: Interactive Topic Panel

Add a topic selection overlay triggered by `t` key:

```
┌─ Topics ────────────────────────────────┐
│ [x] /camera/image_raw     (1200 msgs)  │
│ [x] /camera/camera_info   (1200 msgs)  │
│ [ ] /rosout                 (50 msgs)  │
│ [x] /imu/data             (6000 msgs)  │
│ [ ] /parameter_events        (0 msgs)  │
│                                         │
│ ↑↓ navigate  Space toggle  Enter apply  │
│ a all  n none  / filter                 │
└─────────────────────────────────────────┘
```

Behavior:
- Opens as a modal overlay (doesn't stop playback)
- Pre-populated from `Deck::metadata().topics` with message counts
- Checkboxes reflect current filter state
- `Space` toggles individual topics
- `a` selects all, `n` deselects all
- `/` opens a search/filter bar for the topic list itself
- `Enter` applies the selection and closes the panel
- `Esc` cancels changes

On apply, calls `Deck::set_topic_filter()` with the selected set (or `None` if all selected).

### 5. Filter Resolution Pipeline

```
CLI flags (--topics, --regex, --exclude)
        ↓
  resolve against bag metadata topic list
        ↓
  concrete HashSet<String>
        ↓
  ┌─────────────┬──────────────────┐
  │ StorageFilter│ Registry filter  │
  │ (push to FFI)│ (Deck layer)    │
  └─────────────┴──────────────────┘
```

For TUI dynamic changes, only the Registry filter updates immediately. The StorageFilter update goes through the worker command channel and takes effect on the next prefetch cycle.

## Implementation Order

1. FFI: `set_filter` / `reset_filter` in C wrapper + Rust bindings
2. Core: `BagReader` trait extension, `SetFilter` worker command
3. CLI: `--regex` and `--exclude` flags, space-separated `--topics`
4. TUI: Topic selection panel with keybinding

## Not In Scope

- Per-topic QoS overrides (ros2 bag play feature, not needed for playback)
- Topic remapping (separate feature)
