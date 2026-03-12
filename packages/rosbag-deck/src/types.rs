use std::time::Duration;

/// Metadata about a single topic in a bag.
#[derive(Debug, Clone)]
pub struct TopicInfo {
    /// Topic name (e.g., "/camera/image_raw").
    pub name: String,
    /// Message type (e.g., "sensor_msgs/msg/Image").
    pub type_name: String,
    /// Serialization format (e.g., "cdr").
    pub serialization_format: String,
    /// Number of messages on this topic.
    pub message_count: u64,
}

/// Metadata about an entire bag (or set of bags).
#[derive(Debug, Clone)]
pub struct BagMetadata {
    /// Per-topic information.
    pub topics: Vec<TopicInfo>,
    /// Total number of messages across all topics.
    pub message_count: u64,
    /// Total duration of the bag.
    pub duration: Duration,
    /// Start time as nanoseconds since Unix epoch.
    pub start_time_ns: i64,
    /// End time as nanoseconds since Unix epoch.
    pub end_time_ns: i64,
    /// Storage backend identifier (e.g., "sqlite3", "mcap").
    pub storage_identifier: String,
}

/// A single serialized message read from a bag.
#[derive(Debug, Clone)]
pub struct RawMessage {
    /// Timestamp as nanoseconds since Unix epoch.
    pub timestamp_ns: i64,
    /// Topic this message was published on.
    pub topic: String,
    /// CDR-serialized message data.
    pub data: Vec<u8>,
}

/// A message returned from the playback engine with playback metadata.
#[derive(Debug, Clone)]
pub struct TimedMessage {
    /// The underlying raw message.
    pub message: RawMessage,
    /// Timeline segment ID at the time this message was produced.
    pub segment_id: u64,
}

/// Playback mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaybackMode {
    /// Messages are paced according to their recorded timestamps.
    RealTime,
    /// Messages are delivered as fast as possible.
    BestEffort,
}

/// Playback state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaybackState {
    Stopped,
    Playing,
    Paused,
}

/// Configuration for the `Deck` playback engine.
#[derive(Debug, Clone)]
pub struct DeckConfig {
    /// Maximum number of entries in the visited index (default 50,000).
    pub visited_index_limit: usize,
    /// Number of milestone intervals per bag (default 1000).
    pub milestone_count: usize,
    /// Maximum bytes for the message cache (default 256 MB).
    pub cache_max_bytes: usize,
    /// Number of messages to prefetch ahead (default 1000).
    pub prefetch_ahead: usize,
}

impl Default for DeckConfig {
    fn default() -> Self {
        Self {
            visited_index_limit: 50_000,
            milestone_count: 1000,
            cache_max_bytes: 256 * 1024 * 1024,
            prefetch_ahead: 1000,
        }
    }
}
