/// Errors from the rosbag-deck core library.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("failed to open bag: {0}")]
    Open(String),

    #[error("failed to read message: {0}")]
    Read(String),

    #[error("seek failed: {0}")]
    Seek(String),

    #[error("FFI error: {0}")]
    Ffi(String),
}

pub type Result<T> = std::result::Result<T, Error>;
