mod sys;

use std::{
    ffi::{CStr, CString},
    path::Path,
    ptr,
    time::Duration,
};

use rosbag_deck_core::{
    publisher::{PublisherBackend, TopicPublisher},
    reader::BagReader,
    types::{BagMetadata, RawMessage, TopicInfo},
    writer::BagWriter,
    Error, Result,
};

/// Returns the last error string from the C wrapper, or a fallback.
fn last_error() -> String {
    unsafe {
        let p = sys::rosbag2_last_error();
        if p.is_null() {
            "unknown error".to_string()
        } else {
            CStr::from_ptr(p).to_string_lossy().into_owned()
        }
    }
}

/// BagReader implementation backed by rosbag2_cpp via C FFI.
pub struct Rosbag2Reader {
    handle: *mut sys::Rosbag2Reader,
    metadata: BagMetadata,
}

// The C++ reader is single-threaded but we guarantee exclusive access via &mut self.
unsafe impl Send for Rosbag2Reader {}

impl Rosbag2Reader {
    /// Open a bag file.
    ///
    /// `storage_id` can be `""` to auto-detect, or `"sqlite3"` / `"mcap"`.
    pub fn open(path: &Path, storage_id: &str) -> Result<Self> {
        let uri = CString::new(
            path.to_str()
                .ok_or_else(|| Error::Open("path contains invalid UTF-8".into()))?,
        )
        .map_err(|e| Error::Open(format!("path contains null byte: {e}")))?;

        let sid = CString::new(storage_id)
            .map_err(|e| Error::Open(format!("storage_id contains null byte: {e}")))?;

        let handle = unsafe { sys::rosbag2_reader_open(uri.as_ptr(), sid.as_ptr()) };
        if handle.is_null() {
            return Err(Error::Open(last_error()));
        }

        let metadata = read_metadata(handle)?;
        Ok(Self { handle, metadata })
    }
}

impl Drop for Rosbag2Reader {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe { sys::rosbag2_reader_close(self.handle) };
            self.handle = ptr::null_mut();
        }
    }
}

impl BagReader for Rosbag2Reader {
    fn metadata(&self) -> &BagMetadata {
        &self.metadata
    }

    fn has_next(&self) -> bool {
        unsafe { sys::rosbag2_reader_has_next(self.handle) }
    }

    fn read_next(&mut self) -> Result<Option<RawMessage>> {
        let mut msg = sys::Rosbag2Message {
            timestamp_ns: 0,
            topic: ptr::null_mut(),
            data: ptr::null_mut(),
            data_len: 0,
        };

        let rc = unsafe { sys::rosbag2_reader_read_next(self.handle, &mut msg) };
        if rc != 0 {
            // -1 can mean end-of-bag or error; distinguish by has_next.
            if !self.has_next() {
                return Ok(None);
            }
            return Err(Error::Read(last_error()));
        }

        let result = unsafe {
            let topic = CStr::from_ptr(msg.topic).to_string_lossy().into_owned();
            let data = std::slice::from_raw_parts(msg.data, msg.data_len).to_vec();
            RawMessage {
                timestamp_ns: msg.timestamp_ns,
                topic,
                data,
            }
        };

        unsafe { sys::rosbag2_message_free(&mut msg) };
        Ok(Some(result))
    }

    fn seek(&mut self, timestamp_ns: i64) -> Result<()> {
        let rc = unsafe { sys::rosbag2_reader_seek(self.handle, timestamp_ns) };
        if rc != 0 {
            return Err(Error::Seek(last_error()));
        }
        Ok(())
    }

    fn set_filter(&mut self, topics: &[String]) -> Result<()> {
        if topics.is_empty() {
            return self.reset_filter();
        }
        let c_strings: Vec<CString> = topics
            .iter()
            .map(|t| CString::new(t.as_str()).unwrap())
            .collect();
        let ptrs: Vec<*const core::ffi::c_char> = c_strings.iter().map(|s| s.as_ptr()).collect();
        let rc = unsafe { sys::rosbag2_reader_set_filter(self.handle, ptrs.as_ptr(), ptrs.len()) };
        if rc != 0 {
            return Err(Error::Ffi(last_error()));
        }
        Ok(())
    }

    fn reset_filter(&mut self) -> Result<()> {
        let rc = unsafe { sys::rosbag2_reader_reset_filter(self.handle) };
        if rc != 0 {
            return Err(Error::Ffi(last_error()));
        }
        Ok(())
    }
}

/// BagWriter implementation backed by rosbag2_cpp via C FFI.
pub struct Rosbag2Writer {
    handle: *mut sys::Rosbag2Writer,
}

// The C++ writer is single-threaded but we guarantee exclusive access via &mut self.
unsafe impl Send for Rosbag2Writer {}

impl Rosbag2Writer {
    /// Open a bag file for writing.
    ///
    /// `storage_id` selects the storage backend: `"sqlite3"` or `"mcap"`.
    pub fn open(path: &Path, storage_id: &str) -> Result<Self> {
        let uri = CString::new(
            path.to_str()
                .ok_or_else(|| Error::Open("path contains invalid UTF-8".into()))?,
        )
        .map_err(|e| Error::Open(format!("path contains null byte: {e}")))?;

        let sid = CString::new(storage_id)
            .map_err(|e| Error::Open(format!("storage_id contains null byte: {e}")))?;

        let handle = unsafe { sys::rosbag2_writer_open(uri.as_ptr(), sid.as_ptr()) };
        if handle.is_null() {
            return Err(Error::Open(last_error()));
        }

        Ok(Self { handle })
    }
}

impl Drop for Rosbag2Writer {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe { sys::rosbag2_writer_close(self.handle) };
            self.handle = ptr::null_mut();
        }
    }
}

impl BagWriter for Rosbag2Writer {
    fn create_topic(&mut self, topic: &TopicInfo) -> Result<()> {
        let name = CString::new(topic.name.as_str())
            .map_err(|e| Error::Ffi(format!("topic name null byte: {e}")))?;
        let type_name = CString::new(topic.type_name.as_str())
            .map_err(|e| Error::Ffi(format!("type name null byte: {e}")))?;
        let ser_format = CString::new(topic.serialization_format.as_str())
            .map_err(|e| Error::Ffi(format!("serialization format null byte: {e}")))?;

        let rc = unsafe {
            sys::rosbag2_writer_create_topic(
                self.handle,
                name.as_ptr(),
                type_name.as_ptr(),
                ser_format.as_ptr(),
            )
        };
        if rc != 0 {
            return Err(Error::Ffi(last_error()));
        }
        Ok(())
    }

    fn write(&mut self, msg: &RawMessage) -> Result<()> {
        let topic = CString::new(msg.topic.as_str())
            .map_err(|e| Error::Ffi(format!("topic name null byte: {e}")))?;

        let rc = unsafe {
            sys::rosbag2_writer_write(
                self.handle,
                topic.as_ptr(),
                msg.timestamp_ns,
                msg.data.as_ptr(),
                msg.data.len(),
            )
        };
        if rc != 0 {
            return Err(Error::Ffi(last_error()));
        }
        Ok(())
    }

    fn close(mut self: Box<Self>) -> Result<()> {
        if !self.handle.is_null() {
            unsafe { sys::rosbag2_writer_close(self.handle) };
            self.handle = ptr::null_mut();
        }
        Ok(())
    }
}

// --- ROS 2 Node / Publisher API ---

/// A ROS 2 node that can create generic publishers.
pub struct RosNode {
    handle: *mut sys::Rosbag2Node,
}

// The C++ node is accessed only via our safe wrappers.
unsafe impl Send for RosNode {}

impl RosNode {
    /// Create a new ROS 2 node. Initializes rclcpp if needed.
    pub fn new(name: &str) -> Result<Self> {
        let c_name =
            CString::new(name).map_err(|e| Error::Ffi(format!("node name null byte: {e}")))?;
        let handle = unsafe { sys::rosbag2_node_create(c_name.as_ptr()) };
        if handle.is_null() {
            return Err(Error::Ffi(last_error()));
        }
        Ok(Self { handle })
    }

    /// Create a generic publisher for serialized CDR data.
    pub fn create_publisher(
        &self,
        topic: &str,
        type_name: &str,
        qos_depth: usize,
        reliable: bool,
    ) -> Result<RosPublisher> {
        let c_topic =
            CString::new(topic).map_err(|e| Error::Ffi(format!("topic null byte: {e}")))?;
        let c_type =
            CString::new(type_name).map_err(|e| Error::Ffi(format!("type name null byte: {e}")))?;
        let handle = unsafe {
            sys::rosbag2_node_create_publisher(
                self.handle,
                c_topic.as_ptr(),
                c_type.as_ptr(),
                qos_depth,
                reliable,
            )
        };
        if handle.is_null() {
            return Err(Error::Ffi(last_error()));
        }
        Ok(RosPublisher { handle })
    }

    /// Non-blocking spin for DDS discovery and graph updates.
    pub fn spin_some(&self) {
        unsafe { sys::rosbag2_node_spin_some(self.handle) };
    }
}

impl Drop for RosNode {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe { sys::rosbag2_node_destroy(self.handle) };
            self.handle = ptr::null_mut();
        }
    }
}

/// A generic publisher that publishes serialized CDR data.
pub struct RosPublisher {
    handle: *mut sys::Rosbag2Publisher,
}

// Publisher handles are accessed only via safe wrappers.
unsafe impl Send for RosPublisher {}

impl RosPublisher {
    /// Publish serialized CDR data.
    pub fn publish(&self, data: &[u8]) -> Result<()> {
        let rc = unsafe { sys::rosbag2_node_publish(self.handle, data.as_ptr(), data.len()) };
        if rc != 0 {
            return Err(Error::Ffi(last_error()));
        }
        Ok(())
    }
}

impl Drop for RosPublisher {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe { sys::rosbag2_node_destroy_publisher(self.handle) };
            self.handle = ptr::null_mut();
        }
    }
}

// --- PublisherBackend implementation ---

/// FFI-backed publisher backend using rclcpp.
pub struct RosPublisherBackend {
    node: RosNode,
}

impl RosPublisherBackend {
    /// Create a new backend with a ROS 2 node.
    pub fn new(node_name: &str) -> Result<Self> {
        let node = RosNode::new(node_name)?;
        Ok(Self { node })
    }
}

/// Wraps `RosPublisher` to implement `TopicPublisher`.
struct FfiTopicPublisher {
    inner: RosPublisher,
}

impl TopicPublisher for FfiTopicPublisher {
    fn publish(&self, data: &[u8]) -> Result<()> {
        self.inner.publish(data)
    }
}

impl PublisherBackend for RosPublisherBackend {
    fn create_publisher(
        &mut self,
        topic: &str,
        type_name: &str,
        qos_depth: usize,
        reliable: bool,
    ) -> Result<Box<dyn TopicPublisher>> {
        let pub_handle = self
            .node
            .create_publisher(topic, type_name, qos_depth, reliable)?;
        Ok(Box::new(FfiTopicPublisher { inner: pub_handle }))
    }

    fn spin_some(&self) {
        self.node.spin_some();
    }

    fn shutdown(&mut self) {
        // Node cleanup happens in Drop.
    }
}

/// Check whether a message type has `std_msgs/msg/Header` as its first field.
///
/// Uses ROS 2 runtime type introspection via `dlopen`/`dlsym`.
pub fn type_has_header_first(type_name: &str) -> Result<bool> {
    let c_name =
        CString::new(type_name).map_err(|e| Error::Ffi(format!("type name null byte: {e}")))?;

    let rc = unsafe { sys::rosbag2_type_has_header_first(c_name.as_ptr()) };
    match rc {
        1 => Ok(true),
        0 => Ok(false),
        _ => Err(Error::Ffi(last_error())),
    }
}

/// Read metadata from an open reader handle.
fn read_metadata(handle: *mut sys::Rosbag2Reader) -> Result<BagMetadata> {
    let mut raw = sys::Rosbag2Metadata {
        topics: ptr::null_mut(),
        topic_count: 0,
        message_count: 0,
        duration_ns: 0,
        start_time_ns: 0,
        end_time_ns: 0,
        storage_identifier: ptr::null_mut(),
    };

    let rc = unsafe { sys::rosbag2_reader_get_metadata(handle, &mut raw) };
    if rc != 0 {
        return Err(Error::Ffi(last_error()));
    }

    let metadata = unsafe {
        let topics: Vec<TopicInfo> = (0..raw.topic_count)
            .map(|i| {
                let t = &*raw.topics.add(i);
                TopicInfo {
                    name: CStr::from_ptr(t.name).to_string_lossy().into_owned(),
                    type_name: CStr::from_ptr(t.type_name).to_string_lossy().into_owned(),
                    serialization_format: CStr::from_ptr(t.serialization_format)
                        .to_string_lossy()
                        .into_owned(),
                    message_count: t.message_count,
                }
            })
            .collect();

        let storage_identifier = CStr::from_ptr(raw.storage_identifier)
            .to_string_lossy()
            .into_owned();

        let duration_ns = raw.duration_ns.max(0) as u64;

        BagMetadata {
            topics,
            message_count: raw.message_count,
            duration: Duration::from_nanos(duration_ns),
            start_time_ns: raw.start_time_ns,
            end_time_ns: raw.end_time_ns,
            storage_identifier,
        }
    };

    unsafe { sys::rosbag2_metadata_free(&mut raw) };
    Ok(metadata)
}
