use crate::{types::TopicInfo, RawMessage, Result};

/// Trait for writing messages to a bag file.
///
/// Implementations handle topic creation, message serialization, and output
/// finalization. Each concrete writer (e.g., rosbag2 FFI) implements this trait.
pub trait BagWriter: Send {
    /// Create a topic in the output bag. Must be called before writing
    /// messages on that topic.
    fn create_topic(&mut self, topic: &TopicInfo) -> Result<()>;

    /// Write a serialized message to the output bag.
    fn write(&mut self, msg: &RawMessage) -> Result<()>;

    /// Flush and close the writer. Consumes the boxed writer.
    fn close(self: Box<Self>) -> Result<()>;
}
