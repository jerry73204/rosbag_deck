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
