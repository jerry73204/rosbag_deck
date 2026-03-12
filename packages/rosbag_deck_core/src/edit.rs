use std::{
    cmp::Reverse,
    collections::{BinaryHeap, HashSet},
    time::{Duration, Instant},
};

use crate::{
    reader::BagReader,
    types::{RawMessage, TopicInfo},
    writer::BagWriter,
    Result,
};

/// Statistics returned after an edit pipeline run.
#[derive(Debug, Clone)]
pub struct EditStats {
    /// Total messages read from input bags.
    pub messages_read: u64,
    /// Messages actually written to output.
    pub messages_written: u64,
    /// Messages skipped by filters.
    pub messages_filtered: u64,
    /// Topics present in the output.
    pub topics_written: HashSet<String>,
    /// Time range of output messages (start_ns, end_ns). Zero if no messages written.
    pub output_time_range: (i64, i64),
    /// Wall-clock time elapsed.
    pub elapsed: Duration,
}

/// Configuration for the edit pipeline.
pub struct EditConfig {
    /// Topic whitelist. None = all topics accepted.
    pub topic_filter: Option<HashSet<String>>,
    /// Time range filter: (start_ns, end_ns). None = no time filtering.
    pub time_range: Option<(i64, i64)>,
    /// Timestamp offset in nanoseconds, applied after scaling.
    pub time_offset_ns: i64,
    /// Timestamp scale factor (1.0 = no change).
    pub time_scale: f64,
    /// If true, skip writing — just compute stats.
    pub dry_run: bool,
    /// If true, print per-message progress.
    pub verbose: bool,
}

impl Default for EditConfig {
    fn default() -> Self {
        Self {
            topic_filter: None,
            time_range: None,
            time_offset_ns: 0,
            time_scale: 1.0,
            dry_run: false,
            verbose: false,
        }
    }
}

/// A streaming edit pipeline that reads from N bags, filters, transforms
/// timestamps, and writes to an output bag.
pub struct EditPipeline {
    readers: Vec<Box<dyn BagReader>>,
    writer: Option<Box<dyn BagWriter>>,
    config: EditConfig,
}

/// Entry in the merge heap — wraps a message with its reader index.
struct MergeEntry {
    timestamp_ns: i64,
    reader_index: usize,
    message: RawMessage,
}

impl PartialEq for MergeEntry {
    fn eq(&self, other: &Self) -> bool {
        self.timestamp_ns == other.timestamp_ns
    }
}

impl Eq for MergeEntry {}

impl PartialOrd for MergeEntry {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for MergeEntry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // We use Reverse in the heap, so natural ordering gives us min-heap.
        self.timestamp_ns.cmp(&other.timestamp_ns)
    }
}

impl EditPipeline {
    /// Create a new edit pipeline.
    ///
    /// `writer` may be `None` for dry-run mode.
    pub fn new(
        readers: Vec<Box<dyn BagReader>>,
        writer: Option<Box<dyn BagWriter>>,
        config: EditConfig,
    ) -> Self {
        Self {
            readers,
            writer,
            config,
        }
    }

    /// Run the pipeline to completion. Returns statistics.
    pub fn run(mut self) -> Result<EditStats> {
        let start = Instant::now();

        // Collect and deduplicate topics across all readers.
        let all_topics = self.collect_topics();

        // Create topics in the writer.
        if let Some(ref mut writer) = self.writer {
            for topic in &all_topics {
                if is_topic_accepted(&self.config.topic_filter, &topic.name) {
                    writer.create_topic(topic)?;
                }
            }
        }

        // Determine time anchor for scaling (first message's original timestamp).
        // We'll set this when we see the first message.
        let mut time_anchor_ns: Option<i64> = None;

        let mut stats = EditStats {
            messages_read: 0,
            messages_written: 0,
            messages_filtered: 0,
            topics_written: HashSet::new(),
            output_time_range: (0, 0),
            elapsed: Duration::ZERO,
        };

        // Initialize the merge heap with one message per reader.
        let mut heap: BinaryHeap<Reverse<MergeEntry>> = BinaryHeap::new();
        for (i, reader) in self.readers.iter_mut().enumerate() {
            if let Some(entry) = read_next_entry(reader.as_mut(), i)? {
                heap.push(Reverse(entry));
            }
        }

        // Process messages in timestamp order.
        while let Some(Reverse(entry)) = heap.pop() {
            stats.messages_read += 1;

            // Refill the heap from the same reader.
            if let Some(next) = read_next_entry(
                self.readers[entry.reader_index].as_mut(),
                entry.reader_index,
            )? {
                heap.push(Reverse(next));
            }

            // Topic filter.
            if !is_topic_accepted(&self.config.topic_filter, &entry.message.topic) {
                stats.messages_filtered += 1;
                continue;
            }

            // Time range filter (on original timestamps).
            if let Some((start_ns, end_ns)) = self.config.time_range {
                if entry.message.timestamp_ns < start_ns {
                    stats.messages_filtered += 1;
                    continue;
                }
                if entry.message.timestamp_ns > end_ns {
                    stats.messages_filtered += 1;
                    continue;
                }
            }

            // Set time anchor from first accepted message.
            let anchor = *time_anchor_ns.get_or_insert(entry.message.timestamp_ns);

            // Apply timestamp transform.
            let new_ts = self.transform_timestamp(entry.message.timestamp_ns, anchor);

            let output_msg = RawMessage {
                timestamp_ns: new_ts,
                topic: entry.message.topic.clone(),
                data: entry.message.data,
            };

            if self.config.verbose {
                let ts_sec = new_ts as f64 / 1e9;
                eprintln!(
                    "  {:.6}  {:<40}  [{} bytes]",
                    ts_sec,
                    output_msg.topic,
                    output_msg.data.len(),
                );
            }

            // Update stats.
            stats.topics_written.insert(output_msg.topic.clone());
            if stats.messages_written == 0 {
                stats.output_time_range.0 = new_ts;
            }
            stats.output_time_range.1 = new_ts;
            stats.messages_written += 1;

            // Write (unless dry run).
            if let Some(ref mut writer) = self.writer {
                writer.write(&output_msg)?;
            }

            // Progress indicator every 10000 messages.
            if stats.messages_read % 10000 == 0 {
                eprint!(
                    "\r  {} messages read, {} written...",
                    stats.messages_read, stats.messages_written
                );
            }
        }

        if stats.messages_read >= 10000 {
            eprintln!(); // Clear progress line.
        }

        // Close the writer.
        if let Some(writer) = self.writer {
            writer.close()?;
        }

        stats.elapsed = start.elapsed();
        Ok(stats)
    }

    fn transform_timestamp(&self, original_ns: i64, anchor_ns: i64) -> i64 {
        let scaled = if (self.config.time_scale - 1.0).abs() < f64::EPSILON {
            original_ns
        } else {
            anchor_ns + ((original_ns - anchor_ns) as f64 * self.config.time_scale) as i64
        };
        scaled + self.config.time_offset_ns
    }

    fn collect_topics(&self) -> Vec<TopicInfo> {
        let mut seen = HashSet::new();
        let mut topics = Vec::new();
        for reader in &self.readers {
            for topic in &reader.metadata().topics {
                if seen.insert(topic.name.clone()) {
                    topics.push(topic.clone());
                }
            }
        }
        topics
    }
}

fn is_topic_accepted(filter: &Option<HashSet<String>>, topic: &str) -> bool {
    match filter {
        None => true,
        Some(allowed) => allowed.contains(topic),
    }
}

/// Read the next message from a reader, wrapping it in a MergeEntry.
fn read_next_entry(reader: &mut dyn BagReader, reader_index: usize) -> Result<Option<MergeEntry>> {
    match reader.read_next()? {
        Some(msg) => Ok(Some(MergeEntry {
            timestamp_ns: msg.timestamp_ns,
            reader_index,
            message: msg,
        })),
        None => Ok(None),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::BagMetadata;

    struct MockReader {
        metadata: BagMetadata,
        messages: Vec<RawMessage>,
        position: usize,
    }

    impl MockReader {
        fn new(topic: &str, timestamps: &[i64]) -> Self {
            let messages: Vec<RawMessage> = timestamps
                .iter()
                .map(|&ts| RawMessage {
                    timestamp_ns: ts,
                    topic: topic.to_string(),
                    data: vec![0u8; 16],
                })
                .collect();
            let start = timestamps.first().copied().unwrap_or(0);
            let end = timestamps.last().copied().unwrap_or(0);
            Self {
                metadata: BagMetadata {
                    topics: vec![TopicInfo {
                        name: topic.to_string(),
                        type_name: "std_msgs/msg/String".to_string(),
                        serialization_format: "cdr".to_string(),
                        message_count: messages.len() as u64,
                    }],
                    message_count: messages.len() as u64,
                    duration: Duration::from_nanos((end - start).max(0) as u64),
                    start_time_ns: start,
                    end_time_ns: end,
                    storage_identifier: "mock".to_string(),
                },
                messages,
                position: 0,
            }
        }
    }

    impl BagReader for MockReader {
        fn metadata(&self) -> &BagMetadata {
            &self.metadata
        }
        fn has_next(&self) -> bool {
            self.position < self.messages.len()
        }
        fn read_next(&mut self) -> Result<Option<RawMessage>> {
            if self.position < self.messages.len() {
                let msg = self.messages[self.position].clone();
                self.position += 1;
                Ok(Some(msg))
            } else {
                Ok(None)
            }
        }
        fn seek(&mut self, timestamp_ns: i64) -> Result<()> {
            self.position = self
                .messages
                .partition_point(|m| m.timestamp_ns < timestamp_ns);
            Ok(())
        }
    }

    /// A mock writer that collects messages in memory.
    struct MockWriter {
        topics: Vec<TopicInfo>,
        messages: Vec<RawMessage>,
        closed: bool,
    }

    impl MockWriter {
        fn new() -> Self {
            Self {
                topics: Vec::new(),
                messages: Vec::new(),
                closed: false,
            }
        }
    }

    impl BagWriter for MockWriter {
        fn create_topic(&mut self, topic: &TopicInfo) -> Result<()> {
            self.topics.push(topic.clone());
            Ok(())
        }
        fn write(&mut self, msg: &RawMessage) -> Result<()> {
            self.messages.push(msg.clone());
            Ok(())
        }
        fn close(mut self: Box<Self>) -> Result<()> {
            self.closed = true;
            Ok(())
        }
    }

    #[test]
    fn test_single_bag_passthrough() {
        let reader = MockReader::new("/test", &[1000, 2000, 3000, 4000, 5000]);
        let writer = MockWriter::new();

        let pipeline = EditPipeline::new(
            vec![Box::new(reader)],
            Some(Box::new(writer)),
            EditConfig::default(),
        );

        let stats = pipeline.run().unwrap();
        assert_eq!(stats.messages_read, 5);
        assert_eq!(stats.messages_written, 5);
        assert_eq!(stats.messages_filtered, 0);
    }

    #[test]
    fn test_topic_filter() {
        let reader_a = MockReader::new("/camera", &[1000, 3000, 5000]);
        let reader_b = MockReader::new("/imu", &[2000, 4000, 6000]);

        let pipeline = EditPipeline::new(
            vec![Box::new(reader_a), Box::new(reader_b)],
            None,
            EditConfig {
                topic_filter: Some(HashSet::from(["/camera".to_string()])),
                dry_run: true,
                ..Default::default()
            },
        );

        let stats = pipeline.run().unwrap();
        assert_eq!(stats.messages_written, 3);
        assert_eq!(stats.messages_filtered, 3);
        assert!(stats.topics_written.contains("/camera"));
        assert!(!stats.topics_written.contains("/imu"));
    }

    #[test]
    fn test_time_range_filter() {
        let reader = MockReader::new("/test", &[1000, 2000, 3000, 4000, 5000]);

        let pipeline = EditPipeline::new(
            vec![Box::new(reader)],
            None,
            EditConfig {
                time_range: Some((2000, 4000)),
                dry_run: true,
                ..Default::default()
            },
        );

        let stats = pipeline.run().unwrap();
        assert_eq!(stats.messages_written, 3); // 2000, 3000, 4000
        assert_eq!(stats.messages_filtered, 2); // 1000, 5000
    }

    #[test]
    fn test_multi_bag_merge_order() {
        let reader_a = MockReader::new("/a", &[1000, 3000, 5000]);
        let reader_b = MockReader::new("/b", &[2000, 4000, 6000]);
        let writer = MockWriter::new();

        // We need to capture writer output, so use a shared mechanism.
        // Since MockWriter is consumed, use dry_run + verify stats instead.
        let pipeline = EditPipeline::new(
            vec![Box::new(reader_a), Box::new(reader_b)],
            Some(Box::new(writer)),
            EditConfig::default(),
        );

        let stats = pipeline.run().unwrap();
        assert_eq!(stats.messages_written, 6);
        // Output time range should span 1000..6000.
        assert_eq!(stats.output_time_range, (1000, 6000));
    }

    #[test]
    fn test_timestamp_offset() {
        let reader = MockReader::new("/test", &[1000, 2000, 3000]);

        let pipeline = EditPipeline::new(
            vec![Box::new(reader)],
            None,
            EditConfig {
                time_offset_ns: 5000,
                dry_run: true,
                ..Default::default()
            },
        );

        let stats = pipeline.run().unwrap();
        assert_eq!(stats.messages_written, 3);
        // First message: 1000 + 5000 = 6000
        // Last message: 3000 + 5000 = 8000
        assert_eq!(stats.output_time_range, (6000, 8000));
    }

    #[test]
    fn test_timestamp_scale() {
        let reader = MockReader::new("/test", &[1000, 2000, 3000]);

        let pipeline = EditPipeline::new(
            vec![Box::new(reader)],
            None,
            EditConfig {
                time_scale: 2.0,
                dry_run: true,
                ..Default::default()
            },
        );

        let stats = pipeline.run().unwrap();
        assert_eq!(stats.messages_written, 3);
        // anchor = 1000
        // msg1: 1000 + (1000-1000)*2 = 1000
        // msg2: 1000 + (2000-1000)*2 = 3000
        // msg3: 1000 + (3000-1000)*2 = 5000
        assert_eq!(stats.output_time_range, (1000, 5000));
    }

    #[test]
    fn test_dry_run_no_writer() {
        let reader = MockReader::new("/test", &[1000, 2000, 3000]);

        let pipeline = EditPipeline::new(
            vec![Box::new(reader)],
            None,
            EditConfig {
                dry_run: true,
                ..Default::default()
            },
        );

        let stats = pipeline.run().unwrap();
        assert_eq!(stats.messages_written, 3);
    }

    #[test]
    fn test_empty_input() {
        let reader = MockReader::new("/test", &[]);

        let pipeline = EditPipeline::new(
            vec![Box::new(reader)],
            None,
            EditConfig {
                dry_run: true,
                ..Default::default()
            },
        );

        let stats = pipeline.run().unwrap();
        assert_eq!(stats.messages_read, 0);
        assert_eq!(stats.messages_written, 0);
    }

    #[test]
    fn test_combined_scale_and_offset() {
        let reader = MockReader::new("/test", &[1000, 2000, 3000]);

        let pipeline = EditPipeline::new(
            vec![Box::new(reader)],
            None,
            EditConfig {
                time_scale: 0.5,
                time_offset_ns: -500,
                dry_run: true,
                ..Default::default()
            },
        );

        let stats = pipeline.run().unwrap();
        // anchor = 1000
        // msg1: 1000 + (1000-1000)*0.5 + (-500) = 500
        // msg2: 1000 + (2000-1000)*0.5 + (-500) = 1000
        // msg3: 1000 + (3000-1000)*0.5 + (-500) = 1500
        assert_eq!(stats.output_time_range, (500, 1500));
    }
}
