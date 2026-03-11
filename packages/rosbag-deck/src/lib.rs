pub mod error;
pub mod reader;
pub mod types;

pub use error::{Error, Result};
pub use reader::BagReader;
pub use types::{BagMetadata, RawMessage, TopicInfo};
