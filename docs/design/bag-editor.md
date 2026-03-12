# Design: Rosbag Editor Tool

## Motivation

`ros2 bag` provides `convert` but it's limited — no time slicing, no timestamp manipulation, no multi-bag merge with reordering. A dedicated editor tool fills this gap.

## C++ API Availability

| Feature | rosbag2 Native | Notes |
|---------|---------------|-------|
| Read messages | Yes | `Reader::read_next()` |
| Write messages | Yes | `Writer::write()`, `Writer::create_topic()` |
| Topic filtering (read) | Yes | `Reader::set_filter(StorageFilter)` |
| Bag merge/rewrite | Yes | `rosbag2_transport::bag_rewrite()` — multi-input, multi-output |
| Time range selection | Partial | `seek()` for start bound; end bound is manual |
| Timestamp shifting/scaling | No | Must modify timestamps in Rust before writing |
| Compression control | Yes | `StorageOptions::compression_*` on writer |
| Storage format conversion | Yes | Read sqlite3, write mcap (or vice versa) |

Key finding: `bag_rewrite()` from `rosbag2_transport` handles merge + format conversion + topic filtering in one pass. However, it doesn't support timestamp manipulation, so we need our own pipeline for that.

## CLI Design

New subcommand: `rosbag_deck edit`

```
rosbag_deck edit [OPTIONS] <INPUT>... -o <OUTPUT>

Arguments:
  <INPUT>...              One or more input bag paths

Options:
  -o, --output <PATH>     Output bag path (required)
  --format <FORMAT>       Output storage format: sqlite3, mcap [default: sqlite3]
  --topics <TOPICS>       Topic whitelist (space or comma separated)
  --exclude <PATTERN>     Exclude topics by name or regex
  --regex <PATTERN>       Include topics matching regex
  --start <TIME>          Start time (seconds from bag start, or absolute ns)
  --end <TIME>            End time (seconds from bag start, or absolute ns)
  --duration <SECONDS>    Max duration from start
  --time-offset <SECONDS> Shift all timestamps by ±N seconds
  --time-scale <FACTOR>   Scale timestamps relative to first message
  --compress <FORMAT>     Compression: none, zstd, lz4 [default: none]
  --max-size <BYTES>      Split output at N bytes
  --dry-run               Show what would be done without writing
  --verbose               Print per-message progress
```

### Example Workflows

```bash
# Extract 30 seconds starting at 10s, only camera topics
rosbag_deck edit input/ -o output/ --start 10 --end 40 --topics /camera/image_raw

# Concatenate two bags into one
rosbag_deck edit bag1/ bag2/ -o merged/

# Shift timestamps by -5 seconds (align to external reference)
rosbag_deck edit input/ -o shifted/ --time-offset -5.0

# Convert sqlite3 to mcap with compression
rosbag_deck edit input/ -o output/ --format mcap --compress zstd

# Slow down timestamps to 0.5x (double the time between messages)
rosbag_deck edit input/ -o output/ --time-scale 0.5

# Dry run to see message counts per topic and time range
rosbag_deck edit input/ -o output/ --topics /imu --start 0 --end 60 --dry-run
```

## Architecture

### FFI Extensions

Add to `rosbag2_wrapper`:

```c
// Writer API
Rosbag2Writer *rosbag2_writer_open(const char *uri, const char *storage_id);
int rosbag2_writer_create_topic(Rosbag2Writer *w,
                                const char *name, const char *type,
                                const char *serialization_format);
int rosbag2_writer_write(Rosbag2Writer *w,
                         const char *topic, int64_t timestamp_ns,
                         const uint8_t *data, size_t data_len);
void rosbag2_writer_close(Rosbag2Writer *w);
```

### Core: `BagWriter` Trait

```rust
pub trait BagWriter: Send {
    fn create_topic(&mut self, topic: &TopicInfo) -> Result<()>;
    fn write(&mut self, msg: &RawMessage) -> Result<()>;
    fn close(self: Box<Self>) -> Result<()>;
}
```

FFI implementation: `Rosbag2Writer` wraps the C API.

### Core: Edit Pipeline

The edit operation is a streaming pipeline:

```
Input Bags (1..N)
    │
    ▼
┌─────────────┐
│ Open readers │  BagReader per input
│ Merge by ts  │  Priority queue on timestamp
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Time filter  │  Skip messages outside [start, end]
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ Topic filter │  StorageFilter at reader + registry check
└──────┬──────┘
       │
       ▼
┌──────────────┐
│ Timestamp    │  Apply offset and/or scale
│ transform    │  new_ts = (old_ts - anchor) * scale + anchor + offset
└──────┬───────┘
       │
       ▼
┌─────────────┐
│ Writer       │  BagWriter outputs to destination
└─────────────┘
```

Implementation as a struct:

```rust
pub struct EditPipeline {
    readers: Vec<Box<dyn BagReader>>,
    writer: Box<dyn BagWriter>,
    topic_filter: Option<HashSet<String>>,
    time_range: Option<(i64, i64)>,       // (start_ns, end_ns)
    time_offset_ns: i64,
    time_scale: f64,
    time_anchor_ns: i64,                  // reference point for scaling
}

impl EditPipeline {
    pub fn run(&mut self) -> Result<EditStats> { ... }
}

pub struct EditStats {
    pub messages_read: u64,
    pub messages_written: u64,
    pub messages_filtered: u64,
    pub topics_written: HashSet<String>,
    pub output_time_range: (i64, i64),
    pub elapsed: Duration,
}
```

### Multi-Bag Merge

When multiple inputs are provided, messages are merged in timestamp order using a `BinaryHeap`:

```rust
struct MergeEntry {
    timestamp_ns: i64,
    bag_index: usize,
    message: RawMessage,
}
```

Each reader is advanced one message at a time. The heap always contains at most one message per reader. This keeps memory bounded regardless of bag size.

### Timestamp Transform

Two operations, applied in order:

1. **Scale**: Stretch or compress time relative to an anchor (first message timestamp)
   ```
   new_ts = anchor + (old_ts - anchor) * scale
   ```
   - `scale = 2.0` → double speed (halve intervals)
   - `scale = 0.5` → half speed (double intervals)

2. **Offset**: Shift all timestamps by a fixed amount
   ```
   new_ts = new_ts + offset_ns
   ```
   - Positive = shift forward, negative = shift backward

### Dry Run

With `--dry-run`, the pipeline runs the same merge/filter/time logic but skips the writer. Prints a summary:

```
Input:  2 bags, 15,432 messages, 12 topics
Filter: 3 topics selected, time range 10.0s - 40.0s
Output: 4,218 messages would be written
        Time range: 10.000s - 39.997s (29.997s duration)
        Topics:
          /camera/image_raw      1,200 messages
          /camera/camera_info    1,200 messages
          /imu/data              1,818 messages
```

## Implementation Order

1. **FFI Writer API** — C wrapper + Rust `BagWriter` trait + `Rosbag2Writer`
2. **EditPipeline** — Core streaming pipeline in `rosbag_deck_core`
3. **CLI `edit` subcommand** — Argument parsing, pipeline construction, progress output
4. **Multi-bag merge** — Priority queue merge
5. **Timestamp transform** — Offset and scale
6. **Dry run** — Summary without writing
7. **Compression / format options** — Pass through to writer StorageOptions

## Not In Scope (Future Work)

- Message content modification (e.g., rewriting frame_id fields)
- Message type conversion / re-serialization
- Interactive TUI editor mode (select time range visually)
- Splitting one bag into multiple outputs by topic
