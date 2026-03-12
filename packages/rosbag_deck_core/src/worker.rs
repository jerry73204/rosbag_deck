use std::{
    sync::{
        mpsc::{self, Receiver, Sender},
        Arc,
    },
    thread::{self, JoinHandle},
};

use crate::{
    cache::MessageCache,
    index::{IndexManager, VisitedEntry},
    reader::BagReader,
    Error,
};

/// Commands sent from the control thread to a bag worker.
pub enum WorkerCommand {
    /// Seek the reader to a timestamp, then prefetch forward.
    Seek { timestamp_ns: i64 },
    /// Read the next `count` messages into the cache.
    Prefetch { count: usize },
    /// Apply a topic filter at the storage level. Empty vec clears filter.
    SetFilter { topics: Vec<String> },
    /// Shut down the worker thread.
    Shutdown,
}

/// Events sent from a bag worker back to the control thread.
pub enum WorkerEvent {
    /// Seek completed for this bag.
    SeekComplete { bag_id: u16 },
    /// Prefetch batch loaded.
    Ready { bag_id: u16 },
    /// No more messages in this bag.
    EndOfBag { bag_id: u16 },
    /// An error occurred.
    Error { bag_id: u16, message: String },
}

/// Handle to a running bag worker thread.
pub struct BagWorkerHandle {
    pub bag_id: u16,
    cmd_tx: Sender<WorkerCommand>,
    pub event_rx: Receiver<WorkerEvent>,
    join_handle: Option<JoinHandle<()>>,
}

impl BagWorkerHandle {
    /// Spawn a new worker thread for the given bag reader.
    pub fn spawn(
        bag_id: u16,
        reader: Box<dyn BagReader>,
        index: Arc<IndexManager>,
        cache: Arc<MessageCache>,
    ) -> Self {
        let (cmd_tx, cmd_rx) = mpsc::channel();
        let (event_tx, event_rx) = mpsc::channel();

        let join_handle = thread::spawn(move || {
            worker_loop(bag_id, reader, index, cache, cmd_rx, event_tx);
        });

        Self {
            bag_id,
            cmd_tx,
            event_rx,
            join_handle: Some(join_handle),
        }
    }

    /// Send a command to the worker.
    pub fn send(&self, cmd: WorkerCommand) -> Result<(), Error> {
        self.cmd_tx.send(cmd).map_err(|_| Error::Worker {
            bag_id: self.bag_id,
            message: "worker channel closed".to_string(),
        })
    }

    /// Try to receive a pending event (non-blocking).
    pub fn try_recv(&self) -> Option<WorkerEvent> {
        self.event_rx.try_recv().ok()
    }

    /// Block until the next event.
    pub fn recv(&self) -> Result<WorkerEvent, Error> {
        self.event_rx.recv().map_err(|_| Error::Worker {
            bag_id: self.bag_id,
            message: "worker event channel closed".to_string(),
        })
    }

    /// Shut down the worker and wait for the thread to finish.
    pub fn shutdown(mut self) {
        let _ = self.cmd_tx.send(WorkerCommand::Shutdown);
        if let Some(handle) = self.join_handle.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for BagWorkerHandle {
    fn drop(&mut self) {
        let _ = self.cmd_tx.send(WorkerCommand::Shutdown);
        if let Some(handle) = self.join_handle.take() {
            let _ = handle.join();
        }
    }
}

fn worker_loop(
    bag_id: u16,
    mut reader: Box<dyn BagReader>,
    index: Arc<IndexManager>,
    cache: Arc<MessageCache>,
    cmd_rx: Receiver<WorkerCommand>,
    event_tx: Sender<WorkerEvent>,
) {
    while let Ok(cmd) = cmd_rx.recv() {
        match cmd {
            WorkerCommand::Seek { timestamp_ns } => {
                if let Err(e) = reader.seek(timestamp_ns) {
                    let _ = event_tx.send(WorkerEvent::Error {
                        bag_id,
                        message: e.to_string(),
                    });
                    continue;
                }
                // After seeking, prefetch ahead.
                let prefetch_count = cache.prefetch_ahead();
                read_into_cache(
                    bag_id,
                    &mut reader,
                    &index,
                    &cache,
                    prefetch_count,
                    &event_tx,
                );
                let _ = event_tx.send(WorkerEvent::SeekComplete { bag_id });
            }
            WorkerCommand::Prefetch { count } => {
                read_into_cache(bag_id, &mut reader, &index, &cache, count, &event_tx);
            }
            WorkerCommand::SetFilter { topics } => {
                let result = if topics.is_empty() {
                    reader.reset_filter()
                } else {
                    reader.set_filter(&topics)
                };
                if let Err(e) = result {
                    let _ = event_tx.send(WorkerEvent::Error {
                        bag_id,
                        message: format!("set_filter failed: {e}"),
                    });
                }
            }
            WorkerCommand::Shutdown => break,
        }
    }
}

fn read_into_cache(
    bag_id: u16,
    reader: &mut Box<dyn BagReader>,
    index: &Arc<IndexManager>,
    cache: &Arc<MessageCache>,
    count: usize,
    event_tx: &Sender<WorkerEvent>,
) {
    let mut loaded = 0;
    for _ in 0..count {
        if !reader.has_next() {
            let _ = event_tx.send(WorkerEvent::EndOfBag { bag_id });
            return;
        }
        match reader.read_next() {
            Ok(Some(msg)) => {
                let cursor_ns = msg.timestamp_ns;
                index.record(
                    VisitedEntry {
                        timestamp_ns: msg.timestamp_ns,
                        topic: msg.topic.clone(),
                        bag_id,
                        data_size: msg.data.len() as u32,
                    },
                    cursor_ns,
                );
                cache.insert(msg, cursor_ns);
                loaded += 1;
            }
            Ok(None) => {
                let _ = event_tx.send(WorkerEvent::EndOfBag { bag_id });
                return;
            }
            Err(e) => {
                let _ = event_tx.send(WorkerEvent::Error {
                    bag_id,
                    message: e.to_string(),
                });
                return;
            }
        }
    }
    if loaded > 0 {
        let _ = event_tx.send(WorkerEvent::Ready { bag_id });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        cache::CacheConfig,
        types::{BagMetadata, RawMessage},
    };
    use std::time::Duration;

    /// A mock BagReader that produces synthetic messages.
    struct MockBagReader {
        metadata: BagMetadata,
        messages: Vec<RawMessage>,
        position: usize,
        error_on_next: bool,
    }

    impl MockBagReader {
        fn new(count: usize, start_ns: i64, step_ns: i64) -> Self {
            let messages: Vec<RawMessage> = (0..count)
                .map(|i| RawMessage {
                    timestamp_ns: start_ns + i as i64 * step_ns,
                    topic: "/test".to_string(),
                    data: vec![i as u8; 64],
                })
                .collect();
            let end_ns = messages.last().map(|m| m.timestamp_ns).unwrap_or(start_ns);
            Self {
                metadata: BagMetadata {
                    topics: vec![],
                    message_count: count as u64,
                    duration: Duration::from_nanos((end_ns - start_ns) as u64),
                    start_time_ns: start_ns,
                    end_time_ns: end_ns,
                    storage_identifier: "mock".to_string(),
                },
                messages,
                position: 0,
                error_on_next: false,
            }
        }

        fn with_error(mut self) -> Self {
            self.error_on_next = true;
            self
        }
    }

    impl BagReader for MockBagReader {
        fn metadata(&self) -> &BagMetadata {
            &self.metadata
        }

        fn has_next(&self) -> bool {
            self.position < self.messages.len()
        }

        fn read_next(&mut self) -> crate::Result<Option<RawMessage>> {
            if self.error_on_next {
                self.error_on_next = false;
                return Err(Error::Read("mock error".to_string()));
            }
            if self.position < self.messages.len() {
                let msg = self.messages[self.position].clone();
                self.position += 1;
                Ok(Some(msg))
            } else {
                Ok(None)
            }
        }

        fn seek(&mut self, timestamp_ns: i64) -> crate::Result<()> {
            self.position = self
                .messages
                .partition_point(|m| m.timestamp_ns < timestamp_ns);
            Ok(())
        }
    }

    fn setup(reader: MockBagReader) -> (BagWorkerHandle, Arc<IndexManager>, Arc<MessageCache>) {
        let meta = reader.metadata().clone();
        let index = Arc::new(IndexManager::new(&[meta], 10, 1000));
        let cache = Arc::new(MessageCache::new(CacheConfig {
            max_bytes: 1_000_000,
            prefetch_ahead: 10,
        }));
        let handle =
            BagWorkerHandle::spawn(0, Box::new(reader), Arc::clone(&index), Arc::clone(&cache));
        (handle, index, cache)
    }

    #[test]
    fn test_seek_and_prefetch() {
        let reader = MockBagReader::new(100, 1000, 100);
        let (handle, index, cache) = setup(reader);

        handle
            .send(WorkerCommand::Seek { timestamp_ns: 5000 })
            .unwrap();

        // Wait for events.
        let mut got_ready = false;
        let mut got_seek_complete = false;
        for _ in 0..10 {
            match handle.recv().unwrap() {
                WorkerEvent::Ready { .. } => got_ready = true,
                WorkerEvent::SeekComplete { .. } => {
                    got_seek_complete = true;
                    break;
                }
                _ => {}
            }
        }

        assert!(got_ready || got_seek_complete);
        assert!(cache.get(5000).is_some());
        assert!(index.lookup(5000).is_some());

        handle.shutdown();
    }

    #[test]
    fn test_prefetch_count() {
        let reader = MockBagReader::new(100, 0, 100);
        let (handle, _index, cache) = setup(reader);

        handle.send(WorkerCommand::Prefetch { count: 5 }).unwrap();

        match handle.recv().unwrap() {
            WorkerEvent::Ready { .. } => {}
            other => panic!("expected Ready, got {:?}", std::mem::discriminant(&other)),
        }

        assert_eq!(cache.len(), 5);

        handle.shutdown();
    }

    #[test]
    fn test_end_of_bag_event() {
        let reader = MockBagReader::new(3, 0, 100);
        let (handle, _index, _cache) = setup(reader);

        // Request more messages than exist.
        handle.send(WorkerCommand::Prefetch { count: 100 }).unwrap();

        let mut got_end = false;
        for _ in 0..5 {
            if let Ok(event) = handle.event_rx.try_recv() {
                if matches!(event, WorkerEvent::EndOfBag { .. }) {
                    got_end = true;
                    break;
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }

        assert!(got_end);

        handle.shutdown();
    }

    #[test]
    fn test_shutdown() {
        let reader = MockBagReader::new(10, 0, 100);
        let (handle, _index, _cache) = setup(reader);
        // Just verify shutdown doesn't panic or hang.
        handle.shutdown();
    }

    #[test]
    fn test_error_propagation() {
        let reader = MockBagReader::new(10, 0, 100).with_error();
        let (handle, _index, _cache) = setup(reader);

        handle.send(WorkerCommand::Prefetch { count: 5 }).unwrap();

        let mut got_error = false;
        for _ in 0..5 {
            if let Ok(event) = handle.event_rx.try_recv() {
                if matches!(event, WorkerEvent::Error { .. }) {
                    got_error = true;
                    break;
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }

        assert!(got_error);

        handle.shutdown();
    }
}
