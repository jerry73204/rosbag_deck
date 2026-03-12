//! Benchmarks for the core playback engine using MockBagReader.
//!
//! Run with: cargo bench -p rosbag_deck_core

use std::time::Duration;

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use rosbag_deck_core::{
    reader::BagReader, BagMetadata, Deck, DeckConfig, PlaybackMode, RawMessage, TopicInfo,
};

/// Mock BagReader that generates synthetic messages.
struct MockBagReader {
    metadata: BagMetadata,
    messages: Vec<RawMessage>,
    position: usize,
}

impl MockBagReader {
    fn new(num_topics: usize, msgs_per_topic: usize, data_size: usize) -> Self {
        let topics: Vec<String> = (0..num_topics).map(|i| format!("/topic_{i}")).collect();
        let total = num_topics * msgs_per_topic;
        let start_ns: i64 = 1_000_000_000;
        let step_ns: i64 = 1_000_000; // 1ms between messages

        let mut messages = Vec::with_capacity(total);
        for i in 0..total {
            let topic_idx = i % num_topics;
            messages.push(RawMessage {
                timestamp_ns: start_ns + i as i64 * step_ns,
                topic: topics[topic_idx].clone(),
                data: vec![0u8; data_size],
            });
        }
        messages.sort_by_key(|m| m.timestamp_ns);

        let end_ns = messages.last().map(|m| m.timestamp_ns).unwrap_or(start_ns);
        let topic_infos: Vec<TopicInfo> = topics
            .iter()
            .map(|name| TopicInfo {
                name: name.clone(),
                type_name: "std_msgs/msg/String".to_string(),
                serialization_format: "cdr".to_string(),
                message_count: msgs_per_topic as u64,
            })
            .collect();

        Self {
            metadata: BagMetadata {
                topics: topic_infos,
                message_count: total as u64,
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

    fn read_next(&mut self) -> rosbag_deck_core::Result<Option<RawMessage>> {
        if self.position < self.messages.len() {
            let msg = self.messages[self.position].clone();
            self.position += 1;
            Ok(Some(msg))
        } else {
            Ok(None)
        }
    }

    fn seek(&mut self, timestamp_ns: i64) -> rosbag_deck_core::Result<()> {
        self.position = self
            .messages
            .partition_point(|m| m.timestamp_ns < timestamp_ns);
        Ok(())
    }
}

fn config() -> DeckConfig {
    DeckConfig {
        visited_index_limit: 50_000,
        milestone_count: 1000,
        cache_max_bytes: 256 * 1024 * 1024,
        prefetch_ahead: 5000,
    }
}

fn bench_open(c: &mut Criterion) {
    let mut group = c.benchmark_group("deck_open");

    for msg_count in [100, 1_000, 10_000] {
        group.bench_with_input(
            BenchmarkId::from_parameter(msg_count),
            &msg_count,
            |b, &n| {
                b.iter(|| {
                    let reader = MockBagReader::new(4, n / 4, 64);
                    let deck = Deck::open(vec![Box::new(reader)], config()).unwrap();
                    black_box(deck);
                });
            },
        );
    }
    group.finish();
}

fn bench_sequential_playback(c: &mut Criterion) {
    let mut group = c.benchmark_group("sequential_playback");

    for (label, num_topics, msgs_per_topic, data_size) in [
        ("1topic_1k_64B", 1, 1000, 64),
        ("4topics_1k_64B", 4, 250, 64),
        ("1topic_10k_64B", 1, 10_000, 64),
        ("1topic_1k_4KB", 1, 1000, 4096),
    ] {
        group.bench_function(label, |b| {
            b.iter(|| {
                let reader = MockBagReader::new(num_topics, msgs_per_topic, data_size);
                let mut deck = Deck::open(vec![Box::new(reader)], config()).unwrap();
                deck.set_mode(PlaybackMode::BestEffort);
                deck.play();

                let mut count = 0u64;
                while let Ok(Some(_)) = deck.next_message() {
                    count += 1;
                }
                black_box(count);
            });
        });
    }
    group.finish();
}

fn bench_seek(c: &mut Criterion) {
    let mut group = c.benchmark_group("seek");

    for msg_count in [1_000, 10_000] {
        group.bench_with_input(
            BenchmarkId::from_parameter(msg_count),
            &msg_count,
            |b, &n| {
                let reader = MockBagReader::new(1, n, 64);
                let mut deck = Deck::open(vec![Box::new(reader)], config()).unwrap();
                deck.set_mode(PlaybackMode::BestEffort);
                let meta = deck.metadata().clone();
                let mid = meta.start_time_ns + (meta.end_time_ns - meta.start_time_ns) / 2;

                b.iter(|| {
                    deck.seek_to_time(mid);
                    let msg = deck.step_forward().unwrap();
                    black_box(msg);
                });
            },
        );
    }
    group.finish();
}

fn bench_step_forward(c: &mut Criterion) {
    let reader = MockBagReader::new(4, 2500, 64);
    let mut deck = Deck::open(vec![Box::new(reader)], config()).unwrap();

    c.bench_function("step_forward", |b| {
        b.iter(|| {
            let msg = deck.step_forward().unwrap();
            if msg.is_none() {
                // Reset to start when exhausted
                deck.seek_to_ratio(0.0);
            }
            black_box(msg);
        });
    });
}

fn bench_topic_filter_playback(c: &mut Criterion) {
    let mut group = c.benchmark_group("filtered_playback");

    // 8 topics, filter to 1 — 7/8 messages are skipped
    group.bench_function("8topics_filter_1", |b| {
        b.iter(|| {
            let reader = MockBagReader::new(8, 500, 64);
            let mut deck = Deck::open(vec![Box::new(reader)], config()).unwrap();
            deck.set_mode(PlaybackMode::BestEffort);
            deck.set_topic_filter(Some(["/topic_0".to_string()].into_iter().collect()));
            deck.play();

            let mut count = 0u64;
            while let Ok(Some(_)) = deck.next_message() {
                count += 1;
            }
            black_box(count);
        });
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_open,
    bench_sequential_playback,
    bench_seek,
    bench_step_forward,
    bench_topic_filter_playback,
);
criterion_main!(benches);
