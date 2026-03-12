//! Large bag tests, gated behind ROSBAG_DECK_LARGE_TESTS=1.

use std::time::Instant;

use super::helpers::*;

#[test]
fn large_bag_open_is_fast() {
    skip_if_no_large_bags!();
    let bags = test_bags_dir();
    let path = bag_path!(bags, "zenodo_14209720/rosbag2_2024_06_18-17_11_08");

    let start = Instant::now();
    let reader = open_bag(&path);
    let meta = reader.metadata();
    let elapsed = start.elapsed();

    assert!(meta.message_count > 0, "expected messages in large bag");
    assert!(
        elapsed.as_millis() < 500,
        "Deck::open should be O(1) for metadata, took {}ms",
        elapsed.as_millis()
    );
}

#[test]
fn large_bag_seek_latency() {
    skip_if_no_large_bags!();
    let bags = test_bags_dir();
    let path = bag_path!(bags, "zenodo_14209720/rosbag2_2024_06_18-17_11_08");

    let mut deck = open_deck(&path);
    let meta = deck.metadata().clone();

    let midpoint = meta.start_time_ns + (meta.end_time_ns - meta.start_time_ns) / 2;

    let start = Instant::now();
    deck.seek_to_time(midpoint);
    let msg = deck.step_forward().unwrap();
    let elapsed = start.elapsed();

    assert!(msg.is_some(), "expected message after seek");
    assert!(
        msg.as_ref().unwrap().message.timestamp_ns >= midpoint,
        "message should be at or after seek target"
    );
    assert!(
        elapsed.as_secs() < 1,
        "seek + first message should be < 1s, took {}ms",
        elapsed.as_millis()
    );
}

#[test]
fn large_bag_play_1000_messages() {
    skip_if_no_large_bags!();
    let bags = test_bags_dir();
    let path = bag_path!(bags, "zenodo_14209720/rosbag2_2024_06_18-17_11_08");

    let mut deck = open_deck(&path);
    deck.play();

    let mut timestamps = Vec::with_capacity(1000);
    let start = Instant::now();
    while let Ok(Some(timed)) = deck.next_message() {
        timestamps.push(timed.message.timestamp_ns);
        if timestamps.len() >= 1000 {
            break;
        }
    }
    let elapsed = start.elapsed();

    assert_eq!(timestamps.len(), 1000, "expected 1000 messages");

    // Verify timestamp ordering
    for w in timestamps.windows(2) {
        assert!(w[0] <= w[1], "timestamps not in order: {} > {}", w[0], w[1]);
    }

    let throughput = 1000.0 / elapsed.as_secs_f64();
    eprintln!(
        "large bag playback: 1000 messages in {:.1}ms ({:.0} msgs/sec)",
        elapsed.as_secs_f64() * 1000.0,
        throughput
    );
}

#[test]
fn large_bag_metadata() {
    skip_if_no_large_bags!();
    let bags = test_bags_dir();
    let path = bag_path!(bags, "zenodo_14209720/rosbag2_2024_06_18-17_11_08");

    let deck = open_deck(&path);
    let meta = deck.metadata();

    assert_eq!(meta.message_count, 4330);
    assert_eq!(meta.topics.len(), 14);
    assert_eq!(meta.storage_identifier, "sqlite3");

    // Duration should be ~33.9s
    let dur = meta.duration.as_secs_f64();
    assert!(dur > 30.0 && dur < 40.0, "unexpected duration: {dur}s");
}
