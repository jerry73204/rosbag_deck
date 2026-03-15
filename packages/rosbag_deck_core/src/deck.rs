use std::{collections::HashSet, sync::Arc, time::Duration};

use crate::{
    cache::{CacheConfig, MessageCache},
    index::IndexManager,
    reader::BagReader,
    registry::MessageTypeRegistry,
    stamp,
    timeline::VirtualTimeline,
    types::{
        BagMetadata, DeckConfig, LoopMode, PlaybackMode, PlaybackState, TimedMessage, TopicInfo,
    },
    worker::{BagWorkerHandle, WorkerCommand, WorkerEvent},
    Error, Result,
};

/// Unified playback engine for ROS 2 bag files.
///
/// `Deck` owns all playback components: the sparse index, message cache,
/// I/O workers, virtual timeline, and topic registry. It provides a
/// tape-deck-style API for controlling playback.
pub struct Deck {
    index: Arc<IndexManager>,
    cache: Arc<MessageCache>,
    workers: Vec<BagWorkerHandle>,
    timeline: VirtualTimeline,
    registry: MessageTypeRegistry,
    /// Current cursor timestamp.
    cursor_ns: i64,
    /// Sub-index within messages at cursor_ns (for duplicate timestamps).
    cursor_sub: u32,
    metadata: BagMetadata,
}

impl Deck {
    /// Open one or more bag files for playback.
    ///
    /// Builds milestones from metadata (O(1), no message iteration),
    /// spawns one I/O worker per bag, and triggers initial prefetch.
    pub fn open(readers: Vec<Box<dyn BagReader>>, config: DeckConfig) -> Result<Self> {
        if readers.is_empty() {
            return Err(Error::NoBags);
        }

        let all_metadata: Vec<BagMetadata> = readers.iter().map(|r| r.metadata().clone()).collect();

        let merged = merge_metadata(&all_metadata);
        let index = Arc::new(IndexManager::new(
            &all_metadata,
            config.milestone_count,
            config.visited_index_limit,
        ));
        let cache = Arc::new(MessageCache::new(CacheConfig {
            max_bytes: config.cache_max_bytes,
            prefetch_ahead: config.prefetch_ahead,
        }));

        let registry = MessageTypeRegistry::new(&all_metadata);

        let mut workers = Vec::with_capacity(readers.len());
        for (bag_id, reader) in readers.into_iter().enumerate() {
            let handle = BagWorkerHandle::spawn(
                bag_id as u16,
                reader,
                Arc::clone(&index),
                Arc::clone(&cache),
            );
            workers.push(handle);
        }

        let timeline = VirtualTimeline::new(merged.start_time_ns, merged.end_time_ns);
        let cursor_ns = merged.start_time_ns;

        let mut deck = Self {
            index,
            cache,
            workers,
            timeline,
            registry,
            cursor_ns,
            cursor_sub: 0,
            metadata: merged,
        };

        // Initial prefetch from all bags: send commands and wait for completion.
        for worker in &deck.workers {
            let _ = worker.send(WorkerCommand::Prefetch {
                count: config.prefetch_ahead,
            });
        }
        // Block until all workers report back.
        for worker in &deck.workers {
            match worker.event_rx.recv_timeout(Duration::from_secs(10)) {
                Ok(
                    WorkerEvent::Ready { .. }
                    | WorkerEvent::EndOfBag { .. }
                    | WorkerEvent::SeekComplete { .. },
                ) => {}
                Ok(WorkerEvent::Error { bag_id, message }) => {
                    tracing::warn!(bag_id, %message, "worker error during initial prefetch");
                }
                Err(_) => {
                    tracing::warn!("timeout waiting for initial prefetch");
                }
            }
        }
        deck.drain_events_nonblocking();

        // If the cache has messages before cursor_ns (metadata/data mismatch),
        // adjust cursor and metadata to match actual data.
        if let Some((first_ts, _)) = deck.cache.first_key() {
            if first_ts < deck.cursor_ns {
                deck.cursor_ns = first_ts;
                deck.cursor_sub = 0;
                deck.metadata.start_time_ns = first_ts;
            }
        }

        Ok(deck)
    }

    // -- Playback controls --

    pub fn play(&mut self) {
        self.timeline.play();
    }

    pub fn pause(&mut self) {
        self.timeline.pause();
    }

    pub fn stop(&mut self) {
        self.timeline.stop();
    }

    pub fn toggle_play_pause(&mut self) {
        self.timeline.toggle_play_pause();
    }

    // -- Seeking --

    /// Seek to a specific bag timestamp (nanoseconds).
    pub fn seek_to_time(&mut self, timestamp_ns: i64) {
        self.timeline.seek(timestamp_ns);
        self.cursor_ns = timestamp_ns;
        self.cursor_sub = 0;
        self.cache.clear();

        // Drain stale events before sending new commands.
        self.drain_events_nonblocking();

        // Don't bother seeking workers if we're past the end.
        if timestamp_ns > self.metadata.end_time_ns {
            return;
        }

        let bags = self.index.bags_containing(timestamp_ns);
        for worker in &self.workers {
            if bags.contains(&worker.bag_id) {
                let _ = worker.send(WorkerCommand::Seek { timestamp_ns });
            }
        }

        // Block until at least one worker completes its seek.
        self.drain_until_seek_complete(&bags);
    }

    /// Seek to a ratio (0.0 = start, 1.0 = end).
    pub fn seek_to_ratio(&mut self, ratio: f64) {
        let ratio = ratio.clamp(0.0, 1.0);
        if ratio >= 1.0 {
            // Past the end — no more messages.
            self.seek_to_time(self.metadata.end_time_ns + 1);
        } else {
            let range = self.metadata.end_time_ns - self.metadata.start_time_ns;
            let ts = self.metadata.start_time_ns + (range as f64 * ratio) as i64;
            self.seek_to_time(ts);
        }
    }

    /// Step one message forward.
    pub fn step_forward(&mut self) -> Result<Option<TimedMessage>> {
        self.timeline.advance_segment();
        self.ensure_cache_near_cursor();

        if let Some(mut msg) = self.cache.get_at(self.cursor_ns, self.cursor_sub) {
            self.maybe_patch_stamp(&mut msg);
            let timed = TimedMessage {
                message: msg,
                segment_id: self.timeline.segment_id(),
            };
            self.advance_cursor();
            return Ok(Some(timed));
        }

        // Try next message after cursor.
        if let Some((next_ts, next_sub)) =
            self.cache.next_key_after(self.cursor_ns, self.cursor_sub)
        {
            self.cursor_ns = next_ts;
            self.cursor_sub = next_sub;
            if let Some(mut msg) = self.cache.get_at(next_ts, next_sub) {
                self.maybe_patch_stamp(&mut msg);
                let timed = TimedMessage {
                    message: msg,
                    segment_id: self.timeline.segment_id(),
                };
                self.advance_cursor();
                return Ok(Some(timed));
            }
        }

        Ok(None)
    }

    /// Step one message backward.
    pub fn step_backward(&mut self) -> Result<Option<TimedMessage>> {
        self.timeline.advance_segment();

        if let Some((prev_ts, prev_sub)) =
            self.cache.prev_key_before(self.cursor_ns, self.cursor_sub)
        {
            self.cursor_ns = prev_ts;
            self.cursor_sub = prev_sub;
            if let Some(msg) = self.cache.get_at(prev_ts, prev_sub) {
                return Ok(Some(TimedMessage {
                    message: msg,
                    segment_id: self.timeline.segment_id(),
                }));
            }
        }

        Ok(None)
    }

    // -- Configuration --

    pub fn set_speed(&mut self, speed: f64) {
        self.timeline.set_speed(speed);
    }

    pub fn set_mode(&mut self, mode: PlaybackMode) {
        self.timeline.set_mode(mode);
    }

    pub fn set_loop_mode(&mut self, mode: LoopMode) {
        self.timeline.set_loop_mode(mode);
    }

    pub fn set_looping(&mut self, looping: bool) {
        self.timeline.set_looping(looping);
    }

    pub fn loop_mode(&self) -> LoopMode {
        self.timeline.loop_mode()
    }

    pub fn loop_iteration(&self) -> u64 {
        self.timeline.loop_iteration()
    }

    pub fn set_topic_filter(&mut self, filter: Option<HashSet<String>>) {
        // Push storage-level filter to workers for I/O efficiency.
        let worker_topics: Vec<String> = match &filter {
            Some(set) => set.iter().cloned().collect(),
            None => vec![], // empty = clear filter
        };
        for worker in &self.workers {
            let _ = worker.send(WorkerCommand::SetFilter {
                topics: worker_topics.clone(),
            });
        }
        // Update registry for immediate delivery-side filtering.
        self.registry.set_filter(filter);
    }

    /// Returns an iterator over all known topic names.
    pub fn topic_names(&self) -> Vec<String> {
        self.registry.topic_names().map(|s| s.to_string()).collect()
    }

    /// Returns the current topic filter (None = all topics accepted).
    pub fn topic_filter(&self) -> Option<&HashSet<String>> {
        self.registry.current_filter()
    }

    // -- Playback tick --

    /// The core playback method. Returns the next message according to the
    /// current timeline and playback state.
    ///
    /// In real-time mode, this blocks until it's time to deliver the message.
    /// In best-effort mode, messages are returned immediately.
    /// Returns `None` when paused, stopped, or at end of bag.
    pub fn next_message(&mut self) -> Result<Option<TimedMessage>> {
        loop {
            match self.timeline.state() {
                PlaybackState::Stopped | PlaybackState::Paused => return Ok(None),
                PlaybackState::Playing => {}
            }

            let msg = self.find_next_cached_message();

            let msg = match msg {
                Some(msg) => msg,
                None => {
                    // Check for end of bag.
                    if let Some(wrapped_ts) = self.timeline.check_end(self.cursor_ns) {
                        // Looping — seek workers back to start and refill cache.
                        self.cursor_ns = wrapped_ts;
                        self.cursor_sub = 0;
                        self.cache.clear();
                        self.seek_workers(wrapped_ts);
                        continue;
                    }
                    return Ok(None);
                }
            };

            // Topic filter.
            if !self.registry.is_accepted(&msg.topic) {
                self.advance_cursor();
                continue;
            }

            // Real-time pacing — block until it's time.
            if let Some(delay) = self.timeline.delay_until(msg.timestamp_ns) {
                std::thread::sleep(delay);
            }

            let mut msg = msg;
            self.maybe_patch_stamp(&mut msg);

            let timed = TimedMessage {
                message: msg,
                segment_id: self.timeline.segment_id(),
            };

            self.advance_cursor();

            return Ok(Some(timed));
        }
    }

    /// Non-blocking variant of [`next_message`].
    ///
    /// Returns `Ok(Some(msg))` if a message is due for delivery right now.
    /// Returns `Ok(None)` if no message is ready yet (waiting for real-time
    /// pacing, paused, stopped, or at end of bag). Call this repeatedly from
    /// an event loop without blocking the thread.
    pub fn try_next_message(&mut self) -> Result<Option<TimedMessage>> {
        loop {
            match self.timeline.state() {
                PlaybackState::Stopped | PlaybackState::Paused => return Ok(None),
                PlaybackState::Playing => {}
            }

            let msg = self.find_next_cached_message();

            let msg = match msg {
                Some(msg) => msg,
                None => {
                    if let Some(wrapped_ts) = self.timeline.check_end(self.cursor_ns) {
                        self.cursor_ns = wrapped_ts;
                        self.cursor_sub = 0;
                        self.cache.clear();
                        self.seek_workers(wrapped_ts);
                        continue;
                    }
                    return Ok(None);
                }
            };

            // Topic filter.
            if !self.registry.is_accepted(&msg.topic) {
                self.advance_cursor();
                continue;
            }

            // Non-blocking: if message isn't due yet, return None.
            if self.timeline.delay_until(msg.timestamp_ns).is_some() {
                return Ok(None);
            }

            let mut msg = msg;
            self.maybe_patch_stamp(&mut msg);

            let timed = TimedMessage {
                message: msg,
                segment_id: self.timeline.segment_id(),
            };

            self.advance_cursor();

            return Ok(Some(timed));
        }
    }

    // -- Accessors --

    pub fn metadata(&self) -> &BagMetadata {
        &self.metadata
    }

    pub fn cursor_ns(&self) -> i64 {
        self.cursor_ns
    }

    pub fn state(&self) -> PlaybackState {
        self.timeline.state()
    }

    pub fn segment_id(&self) -> u64 {
        self.timeline.segment_id()
    }

    pub fn speed(&self) -> f64 {
        self.timeline.speed()
    }

    pub fn looping(&self) -> bool {
        self.timeline.looping()
    }

    pub fn mode(&self) -> PlaybackMode {
        self.timeline.mode()
    }

    // -- Internal helpers --

    /// Find the next message at or after the cursor in the cache.
    /// Updates cursor to the found position. Returns `None` if cache is empty
    /// near the cursor.
    fn find_next_cached_message(&mut self) -> Option<crate::types::RawMessage> {
        self.ensure_cache_near_cursor();

        if let Some(msg) = self.cache.get_at(self.cursor_ns, self.cursor_sub) {
            return Some(msg);
        }

        if let Some((ts, sub)) = self.cache.next_key_after(self.cursor_ns, self.cursor_sub) {
            self.cursor_ns = ts;
            self.cursor_sub = sub;
            return self.cache.get_at(ts, sub);
        }

        None
    }

    fn advance_cursor(&mut self) {
        if let Some((next_ts, next_sub)) =
            self.cache.next_key_after(self.cursor_ns, self.cursor_sub)
        {
            self.cursor_ns = next_ts;
            self.cursor_sub = next_sub;
            self.maybe_prefetch();
        } else {
            // Cache exhausted — try a blocking prefetch before giving up.
            self.prefetch_all_blocking();
            if let Some((next_ts, next_sub)) =
                self.cache.next_key_after(self.cursor_ns, self.cursor_sub)
            {
                self.cursor_ns = next_ts;
                self.cursor_sub = next_sub;
            } else {
                // Truly at end — no more data from any worker.
                self.cursor_ns = self.metadata.end_time_ns + 1;
                self.cursor_sub = 0;
            }
        }
    }

    fn maybe_prefetch(&self) {
        // Simple heuristic: if cache has fewer entries than prefetch_ahead, request more.
        let ahead = self.cache.prefetch_ahead();
        if self.cache.len() < ahead {
            self.prefetch_all(ahead);
        }
    }

    fn prefetch_all(&self, count: usize) {
        for worker in &self.workers {
            let _ = worker.send(WorkerCommand::Prefetch { count });
        }
        // Drain ready events (non-blocking).
        self.drain_events_nonblocking();
    }

    /// Send Prefetch to all workers and block until each responds.
    fn prefetch_all_blocking(&self) {
        let count = self.cache.prefetch_ahead();
        for worker in &self.workers {
            let _ = worker.send(WorkerCommand::Prefetch { count });
        }
        for worker in &self.workers {
            match worker.event_rx.recv_timeout(Duration::from_secs(5)) {
                Ok(
                    WorkerEvent::Ready { .. }
                    | WorkerEvent::EndOfBag { .. }
                    | WorkerEvent::SeekComplete { .. },
                ) => {}
                Ok(WorkerEvent::Error { bag_id, message }) => {
                    tracing::warn!(bag_id, %message, "worker error during prefetch");
                }
                Err(_) => {
                    tracing::warn!("timeout waiting for prefetch");
                }
            }
        }
        self.drain_events_nonblocking();
    }

    fn ensure_cache_near_cursor(&self) {
        if self.cache.get_at(self.cursor_ns, self.cursor_sub).is_none()
            && self
                .cache
                .next_key_after(self.cursor_ns, self.cursor_sub)
                .is_none()
        {
            // Cache miss — request data from workers.
            let mut bags = self.index.bags_containing(self.cursor_ns);
            if bags.is_empty() {
                // Metadata time ranges may not cover actual data — ask all workers.
                bags = self.index.all_bag_ids();
            }
            for worker in &self.workers {
                if bags.contains(&worker.bag_id) {
                    let _ = worker.send(WorkerCommand::Prefetch {
                        count: self.cache.prefetch_ahead(),
                    });
                }
            }
            self.drain_until_seek_complete(&bags);
        }
    }

    fn drain_until_seek_complete(&self, _bags: &[u16]) {
        // Wait for at least one seek complete or ready event per worker.
        for worker in &self.workers {
            match worker.event_rx.recv_timeout(Duration::from_secs(5)) {
                Ok(
                    WorkerEvent::SeekComplete { .. }
                    | WorkerEvent::Ready { .. }
                    | WorkerEvent::EndOfBag { .. },
                ) => {}
                Ok(WorkerEvent::Error { bag_id, message }) => {
                    tracing::warn!(bag_id, %message, "worker error during seek");
                }
                Err(_) => {
                    tracing::warn!("timeout waiting for worker seek complete");
                }
            }
        }
        // Drain any remaining events from this seek.
        self.drain_events_nonblocking();
    }

    /// Seek all workers to a timestamp and wait for completion.
    fn seek_workers(&self, timestamp_ns: i64) {
        self.drain_events_nonblocking();
        let bags = self.index.bags_containing(timestamp_ns);
        for worker in &self.workers {
            if bags.is_empty() || bags.contains(&worker.bag_id) {
                let _ = worker.send(WorkerCommand::Seek { timestamp_ns });
            }
        }
        self.drain_until_seek_complete(&bags);
    }

    fn drain_events_nonblocking(&self) {
        for worker in &self.workers {
            while let Some(event) = worker.try_recv() {
                if let WorkerEvent::Error { bag_id, message } = event {
                    tracing::warn!(bag_id, %message, "worker error");
                }
            }
        }
    }

    /// Populate the registry's header cache using a checker function.
    ///
    /// Call this after `open()` with an FFI checker like:
    /// `deck.populate_header_cache(|t| ffi::type_has_header_first(t).unwrap_or(false))`
    pub fn populate_header_cache(&mut self, checker: impl Fn(&str) -> bool) {
        self.registry.populate_header_cache(checker);
    }

    /// Patch CDR Header.stamp and message timestamp if in Monotonic loop mode.
    fn maybe_patch_stamp(&self, msg: &mut crate::types::RawMessage) {
        let iteration = self.timeline.loop_iteration();
        if self.timeline.loop_mode() != LoopMode::Monotonic || iteration == 0 {
            return;
        }

        let offset_ns = iteration as i64 * self.timeline.bag_duration_ns();

        if self.registry.has_header_first(&msg.topic) {
            stamp::patch_header_stamp(&mut msg.data, offset_ns);
        }

        msg.timestamp_ns += offset_ns;
    }
}

/// Merge metadata from multiple bags into a single view.
fn merge_metadata(all: &[BagMetadata]) -> BagMetadata {
    let mut topics: Vec<TopicInfo> = Vec::new();
    let mut seen_topics = HashSet::new();
    let mut total_messages = 0u64;
    let mut start_ns = i64::MAX;
    let mut end_ns = i64::MIN;

    for meta in all {
        total_messages += meta.message_count;
        start_ns = start_ns.min(meta.start_time_ns);
        end_ns = end_ns.max(meta.end_time_ns);

        for topic in &meta.topics {
            if seen_topics.insert(topic.name.clone()) {
                topics.push(topic.clone());
            }
        }
    }

    BagMetadata {
        topics,
        message_count: total_messages,
        duration: Duration::from_nanos((end_ns - start_ns).max(0) as u64),
        start_time_ns: start_ns,
        end_time_ns: end_ns,
        storage_identifier: all
            .first()
            .map(|m| m.storage_identifier.clone())
            .unwrap_or_default(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::RawMessage;

    /// Mock BagReader for Deck integration tests.
    struct MockBagReader {
        metadata: BagMetadata,
        messages: Vec<RawMessage>,
        position: usize,
    }

    impl MockBagReader {
        fn new(topics: &[&str], msgs_per_topic: usize, start_ns: i64, step_ns: i64) -> Self {
            let mut messages = Vec::new();
            for i in 0..(topics.len() * msgs_per_topic) {
                let topic_idx = i % topics.len();
                messages.push(RawMessage {
                    timestamp_ns: start_ns + i as i64 * step_ns,
                    topic: topics[topic_idx].to_string(),
                    data: vec![(i & 0xff) as u8; 32],
                });
            }
            messages.sort_by_key(|m| m.timestamp_ns);

            let end_ns = messages.last().map(|m| m.timestamp_ns).unwrap_or(start_ns);
            let topic_infos: Vec<TopicInfo> = topics
                .iter()
                .map(|name| TopicInfo {
                    name: name.to_string(),
                    type_name: "std_msgs/msg/String".to_string(),
                    serialization_format: "cdr".to_string(),
                    message_count: msgs_per_topic as u64,
                })
                .collect();

            Self {
                metadata: BagMetadata {
                    topics: topic_infos,
                    message_count: messages.len() as u64,
                    duration: Duration::from_nanos((end_ns - start_ns).max(0) as u64),
                    start_time_ns: start_ns,
                    end_time_ns: end_ns,
                    storage_identifier: "mock".to_string(),
                },
                messages,
                position: 0,
            }
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

    fn default_config() -> DeckConfig {
        DeckConfig {
            visited_index_limit: 10_000,
            milestone_count: 100,
            cache_max_bytes: 10 * 1024 * 1024,
            prefetch_ahead: 100,
        }
    }

    #[test]
    fn test_open_builds_milestones() {
        let reader = MockBagReader::new(&["/test"], 50, 1_000, 1_000);
        let deck = Deck::open(vec![Box::new(reader)], default_config()).unwrap();

        // Milestones should exist, derived from metadata.
        let milestones = deck.index.milestones();
        assert!(!milestones.is_empty());
        assert_eq!(deck.state(), PlaybackState::Stopped);
    }

    #[test]
    fn test_sequential_playback() {
        let reader = MockBagReader::new(&["/test"], 20, 1_000, 1_000);
        let mut deck = Deck::open(vec![Box::new(reader)], default_config()).unwrap();

        deck.set_mode(PlaybackMode::BestEffort);
        deck.play();

        let mut timestamps = Vec::new();
        while let Ok(Some(timed)) = deck.next_message() {
            timestamps.push(timed.message.timestamp_ns);
            if timestamps.len() >= 20 {
                break;
            }
        }

        assert!(!timestamps.is_empty());
        // Verify timestamp order.
        for w in timestamps.windows(2) {
            assert!(w[0] <= w[1], "timestamps not in order: {} > {}", w[0], w[1]);
        }
    }

    #[test]
    fn test_seek_and_play() {
        let reader = MockBagReader::new(&["/test"], 50, 1_000, 1_000);
        let mut deck = Deck::open(vec![Box::new(reader)], default_config()).unwrap();

        deck.set_mode(PlaybackMode::BestEffort);
        deck.play();

        // Seek to midpoint.
        let mid = deck.metadata().start_time_ns
            + (deck.metadata().end_time_ns - deck.metadata().start_time_ns) / 2;
        deck.seek_to_time(mid);

        if let Ok(Some(timed)) = deck.next_message() {
            assert!(
                timed.message.timestamp_ns >= mid,
                "first message after seek ({}) should be >= seek target ({})",
                timed.message.timestamp_ns,
                mid
            );
        }
    }

    #[test]
    fn test_step_forward() {
        let reader = MockBagReader::new(&["/test"], 10, 1_000, 1_000);
        let mut deck = Deck::open(vec![Box::new(reader)], default_config()).unwrap();

        let mut timestamps = Vec::new();
        for _ in 0..5 {
            if let Ok(Some(timed)) = deck.step_forward() {
                timestamps.push(timed.message.timestamp_ns);
            }
        }

        assert!(timestamps.len() >= 2);
        for w in timestamps.windows(2) {
            assert!(w[0] < w[1]);
        }
    }

    #[test]
    fn test_topic_filter() {
        let reader = MockBagReader::new(&["/camera", "/imu"], 10, 1_000, 500);
        let mut deck = Deck::open(vec![Box::new(reader)], default_config()).unwrap();

        deck.set_topic_filter(Some(HashSet::from(["/camera".to_string()])));
        deck.set_mode(PlaybackMode::BestEffort);
        deck.play();

        let mut collected = Vec::new();
        while let Ok(Some(timed)) = deck.next_message() {
            collected.push(timed.message.topic.clone());
            if collected.len() >= 10 {
                break;
            }
        }

        assert!(!collected.is_empty());
        assert!(
            collected.iter().all(|t| t == "/camera"),
            "expected only /camera, got: {:?}",
            collected
        );
    }

    #[test]
    fn test_multi_bag_merge_order() {
        // Two bags with interleaved timestamps.
        let reader_a = MockBagReader::new(&["/a"], 10, 1_000, 2_000); // 1000, 3000, 5000, ...
        let reader_b = MockBagReader::new(&["/b"], 10, 2_000, 2_000); // 2000, 4000, 6000, ...

        let mut deck = Deck::open(
            vec![Box::new(reader_a), Box::new(reader_b)],
            default_config(),
        )
        .unwrap();

        deck.set_mode(PlaybackMode::BestEffort);
        deck.play();

        let mut timestamps = Vec::new();
        while let Ok(Some(timed)) = deck.next_message() {
            timestamps.push(timed.message.timestamp_ns);
            if timestamps.len() >= 20 {
                break;
            }
        }

        assert!(timestamps.len() >= 2);
        for w in timestamps.windows(2) {
            assert!(
                w[0] <= w[1],
                "multi-bag merge not in order: {} > {}",
                w[0],
                w[1]
            );
        }
    }

    #[test]
    fn test_play_pause_resume() {
        let reader = MockBagReader::new(&["/test"], 20, 1_000, 1_000);
        let mut deck = Deck::open(vec![Box::new(reader)], default_config()).unwrap();

        deck.set_mode(PlaybackMode::BestEffort);

        // Paused — should return None.
        deck.pause();
        assert!(deck.next_message().unwrap().is_none());

        // Play — should return messages.
        deck.play();
        let msg = deck.next_message().unwrap();
        assert!(msg.is_some());

        // Pause again — should return None.
        deck.pause();
        assert!(deck.next_message().unwrap().is_none());

        // Resume — should continue.
        deck.play();
        let msg = deck.next_message().unwrap();
        assert!(msg.is_some());
    }

    #[test]
    fn test_duplicate_timestamp_playback() {
        // Create a reader where multiple messages share timestamps.
        let mut messages = Vec::new();
        for i in 0..10 {
            // 2 messages per timestamp
            messages.push(RawMessage {
                timestamp_ns: 1000 + (i / 2) * 1000,
                topic: format!("/topic_{}", i % 2),
                data: vec![i as u8; 32],
            });
        }
        messages.sort_by_key(|m| m.timestamp_ns);

        let end_ns = messages.last().unwrap().timestamp_ns;
        let reader = MockBagReader {
            metadata: BagMetadata {
                topics: vec![
                    TopicInfo {
                        name: "/topic_0".to_string(),
                        type_name: "std_msgs/msg/String".to_string(),
                        serialization_format: "cdr".to_string(),
                        message_count: 5,
                    },
                    TopicInfo {
                        name: "/topic_1".to_string(),
                        type_name: "std_msgs/msg/String".to_string(),
                        serialization_format: "cdr".to_string(),
                        message_count: 5,
                    },
                ],
                message_count: 10,
                duration: Duration::from_nanos((end_ns - 1000) as u64),
                start_time_ns: 1000,
                end_time_ns: end_ns,
                storage_identifier: "mock".to_string(),
            },
            messages,
            position: 0,
        };

        let mut deck = Deck::open(vec![Box::new(reader)], default_config()).unwrap();
        deck.set_mode(PlaybackMode::BestEffort);
        deck.play();

        let mut count = 0;
        while let Ok(Some(_)) = deck.next_message() {
            count += 1;
        }
        assert_eq!(
            count, 10,
            "should play all 10 messages including duplicates"
        );
    }
}
