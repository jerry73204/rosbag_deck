use std::{collections::HashSet, path::PathBuf};

use pyo3::{exceptions::PyRuntimeError, prelude::*};

use rosbag_deck_core::reader::BagReader;

// ── Helper ──────────────────────────────────────────────────────────────────

fn to_py_err(e: impl std::fmt::Display) -> PyErr {
    PyRuntimeError::new_err(e.to_string())
}

// ── Data classes ────────────────────────────────────────────────────────────

/// Information about a single topic.
#[pyclass(frozen, get_all)]
#[derive(Clone)]
struct TopicInfo {
    name: String,
    type_name: String,
    serialization_format: String,
    message_count: u64,
}

/// Bag metadata.
#[pyclass(frozen, get_all)]
#[derive(Clone)]
struct BagInfo {
    topics: Vec<TopicInfo>,
    message_count: u64,
    duration_secs: f64,
    start_time_ns: i64,
    end_time_ns: i64,
    storage_identifier: String,
}

/// A single message from the bag.
#[pyclass(frozen, get_all)]
#[derive(Clone)]
struct Message {
    timestamp_ns: i64,
    topic: String,
    data: Vec<u8>,
}

/// Playback status snapshot.
#[pyclass(frozen, get_all)]
#[derive(Clone)]
struct PlaybackStatus {
    state: String,
    cursor_ns: i64,
    speed: f64,
    looping: bool,
    segment_id: u64,
}

// ── Deck class ──────────────────────────────────────────────────────────────

/// Interactive ROS 2 bag player.
///
/// Usage:
///     deck = Deck("/path/to/bag")
///     deck.play()
///     while msg := deck.next_message():
///         print(msg.topic, len(msg.data))
///
/// Or as a context manager:
///     with Deck("/path/to/bag") as deck:
///         for msg in deck:
///             print(msg.topic)
#[pyclass(unsendable)]
struct Deck {
    inner: rosbag_deck_core::Deck,
}

#[pymethods]
impl Deck {
    /// Open a bag file.
    ///
    /// Args:
    ///     path: Path to the bag directory or file.
    ///     storage: Storage backend ("" for auto-detect, "sqlite3", "mcap").
    #[new]
    #[pyo3(signature = (path, storage=""))]
    fn new(path: &str, storage: &str) -> PyResult<Self> {
        let reader = rosbag_deck_ffi::Rosbag2Reader::open(&PathBuf::from(path), storage)
            .map_err(to_py_err)?;
        let readers: Vec<Box<dyn BagReader>> = vec![Box::new(reader)];
        let inner = rosbag_deck_core::Deck::open(readers, rosbag_deck_core::DeckConfig::default())
            .map_err(to_py_err)?;
        Ok(Self { inner })
    }

    /// Get bag metadata.
    fn info(&self) -> BagInfo {
        let meta = self.inner.metadata();
        BagInfo {
            topics: meta
                .topics
                .iter()
                .map(|t| TopicInfo {
                    name: t.name.clone(),
                    type_name: t.type_name.clone(),
                    serialization_format: t.serialization_format.clone(),
                    message_count: t.message_count,
                })
                .collect(),
            message_count: meta.message_count,
            duration_secs: meta.duration.as_secs_f64(),
            start_time_ns: meta.start_time_ns,
            end_time_ns: meta.end_time_ns,
            storage_identifier: meta.storage_identifier.clone(),
        }
    }

    /// Get current playback status.
    fn status(&self) -> PlaybackStatus {
        PlaybackStatus {
            state: format!("{:?}", self.inner.state()),
            cursor_ns: self.inner.cursor_ns(),
            speed: self.inner.speed(),
            looping: self.inner.looping(),
            segment_id: self.inner.segment_id(),
        }
    }

    /// Start playback.
    fn play(&mut self) {
        self.inner.play();
    }

    /// Pause playback.
    fn pause(&mut self) {
        self.inner.pause();
    }

    /// Stop playback.
    fn stop(&mut self) {
        self.inner.stop();
    }

    /// Toggle play/pause.
    fn toggle_play_pause(&mut self) {
        self.inner.toggle_play_pause();
    }

    /// Seek to a timestamp (nanoseconds since epoch).
    fn seek_to_time(&mut self, timestamp_ns: i64) {
        self.inner.seek_to_time(timestamp_ns);
    }

    /// Seek to a position ratio (0.0 = start, 1.0 = end).
    fn seek_to_ratio(&mut self, ratio: f64) {
        self.inner.seek_to_ratio(ratio);
    }

    /// Step one message forward. Returns the message or None.
    fn step_forward(&mut self) -> PyResult<Option<Message>> {
        match self.inner.step_forward().map_err(to_py_err)? {
            Some(timed) => Ok(Some(Message {
                timestamp_ns: timed.message.timestamp_ns,
                topic: timed.message.topic,
                data: timed.message.data,
            })),
            None => Ok(None),
        }
    }

    /// Step one message backward. Returns the message or None.
    fn step_backward(&mut self) -> PyResult<Option<Message>> {
        match self.inner.step_backward().map_err(to_py_err)? {
            Some(timed) => Ok(Some(Message {
                timestamp_ns: timed.message.timestamp_ns,
                topic: timed.message.topic,
                data: timed.message.data,
            })),
            None => Ok(None),
        }
    }

    /// Get the next message during playback. Returns None when paused/stopped/end.
    fn next_message(&mut self) -> PyResult<Option<Message>> {
        match self.inner.next_message().map_err(to_py_err)? {
            Some(timed) => Ok(Some(Message {
                timestamp_ns: timed.message.timestamp_ns,
                topic: timed.message.topic,
                data: timed.message.data,
            })),
            None => Ok(None),
        }
    }

    /// Set playback speed (e.g., 0.5, 1.0, 2.0).
    fn set_speed(&mut self, speed: f64) {
        self.inner.set_speed(speed);
    }

    /// Set playback mode: "realtime" or "best_effort".
    fn set_mode(&mut self, mode: &str) -> PyResult<()> {
        let m = match mode {
            "realtime" => rosbag_deck_core::PlaybackMode::RealTime,
            "best_effort" => rosbag_deck_core::PlaybackMode::BestEffort,
            _ => {
                return Err(PyRuntimeError::new_err(format!(
                    "unknown mode: {mode} (use 'realtime' or 'best_effort')"
                )));
            }
        };
        self.inner.set_mode(m);
        Ok(())
    }

    /// Enable or disable loop playback.
    fn set_looping(&mut self, looping: bool) {
        self.inner.set_looping(looping);
    }

    /// Set topic filter. Pass None to accept all topics.
    #[pyo3(signature = (topics=None))]
    fn set_topic_filter(&mut self, topics: Option<Vec<String>>) {
        self.inner
            .set_topic_filter(topics.map(|v| v.into_iter().collect::<HashSet<_>>()));
    }

    // -- Python protocols --

    fn __enter__(slf: Py<Self>) -> Py<Self> {
        slf
    }

    #[pyo3(signature = (_exc_type=None, _exc_val=None, _exc_tb=None))]
    fn __exit__(
        &mut self,
        _exc_type: Option<&Bound<'_, PyAny>>,
        _exc_val: Option<&Bound<'_, PyAny>>,
        _exc_tb: Option<&Bound<'_, PyAny>>,
    ) {
        self.inner.stop();
    }

    fn __iter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
        slf
    }

    fn __next__(&mut self) -> PyResult<Option<Message>> {
        self.next_message()
    }

    fn __repr__(&self) -> String {
        let meta = self.inner.metadata();
        format!(
            "Deck(messages={}, duration={:.1}s, state={:?})",
            meta.message_count,
            meta.duration.as_secs_f64(),
            self.inner.state(),
        )
    }
}

// ── Module ──────────────────────────────────────────────────────────────────

#[pymodule]
fn rosbag_deck(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add_class::<Deck>()?;
    m.add_class::<BagInfo>()?;
    m.add_class::<TopicInfo>()?;
    m.add_class::<Message>()?;
    m.add_class::<PlaybackStatus>()?;
    Ok(())
}
