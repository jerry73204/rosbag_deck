mod sys;

use std::{
    ffi::{CStr, CString},
    path::Path,
    ptr,
    time::Duration,
};

use rosbag_deck_core::{
    reader::BagReader,
    types::{BagMetadata, RawMessage, TopicInfo},
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
