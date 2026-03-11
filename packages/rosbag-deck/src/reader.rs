use crate::{BagMetadata, RawMessage, Result};

/// Trait for reading messages from a bag file.
///
/// Implementations provide access to bag metadata, sequential message iteration,
/// and seeking by timestamp. Each concrete reader (e.g., rosbag2 FFI, native mcap)
/// implements this trait.
///
/// Constructors live on the concrete types, not on the trait.
pub trait BagReader: Send {
    /// Returns metadata about the bag (topics, duration, message counts, etc.).
    fn metadata(&self) -> &BagMetadata;

    /// Returns `true` if there are more messages to read.
    fn has_next(&self) -> bool;

    /// Reads the next serialized message, or `None` if the reader is exhausted.
    fn read_next(&mut self) -> Result<Option<RawMessage>>;

    /// Seeks to the given timestamp (nanoseconds since Unix epoch).
    ///
    /// After seeking, `read_next` returns the first message at or after `timestamp_ns`.
    fn seek(&mut self, timestamp_ns: i64) -> Result<()>;
}
