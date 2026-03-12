//! Tests for seek operations with real bags.

use super::helpers::*;

#[test]
fn seek_to_midpoint_talker() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));
    let meta = deck.metadata().clone();

    let midpoint = meta.start_time_ns + (meta.end_time_ns - meta.start_time_ns) / 2;
    deck.seek_to_time(midpoint);

    // First message after seek should be at or after the target
    let msg = deck.step_forward().unwrap().expect("no message after seek");
    assert!(
        msg.message.timestamp_ns >= midpoint,
        "message timestamp {} < seek target {}",
        msg.message.timestamp_ns,
        midpoint,
    );
}

#[test]
fn seek_to_start_talker() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));
    let meta = deck.metadata().clone();

    deck.seek_to_time(meta.start_time_ns);
    let msg = deck.step_forward().unwrap().expect("no message after seek to start");

    // Should be the first message
    assert_eq!(msg.message.timestamp_ns, meta.start_time_ns);
}

#[test]
fn seek_to_end_talker() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));
    let meta = deck.metadata().clone();

    // Seek past the end
    deck.seek_to_time(meta.end_time_ns + 1);

    // Should get no message (end of bag)
    let msg = deck.step_forward().unwrap();
    assert!(msg.is_none(), "expected None after seeking past end");
}

#[test]
fn seek_ratio_start() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));
    let meta = deck.metadata().clone();

    deck.seek_to_ratio(0.0);
    let msg = deck.step_forward().unwrap().expect("no message at ratio 0.0");
    assert_eq!(msg.message.timestamp_ns, meta.start_time_ns);
}

#[test]
fn seek_ratio_end() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));

    deck.seek_to_ratio(1.0);
    let msg = deck.step_forward().unwrap();
    // At or past end, no more messages
    assert!(msg.is_none(), "expected None at ratio 1.0");
}

#[test]
fn seek_on_seek_bag_sqlite3() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/test_bag_for_seek_sqlite3"));
    let meta = deck.metadata().clone();

    // Seek to the middle (timestamps: 1.0s, 1.1s, 1.2s, 1.3s, 1.4s)
    let mid = meta.start_time_ns + 200_000_000; // 1.2s
    deck.seek_to_time(mid);

    let msg = deck.step_forward().unwrap().expect("no message after seek");
    assert!(
        msg.message.timestamp_ns >= mid,
        "seek to {} got message at {}",
        mid,
        msg.message.timestamp_ns,
    );
}

#[test]
fn seek_and_play_remaining() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));
    let meta = deck.metadata().clone();

    // Seek to midpoint
    let midpoint = meta.start_time_ns + (meta.end_time_ns - meta.start_time_ns) / 2;
    deck.seek_to_time(midpoint);

    // Play remaining messages
    let msgs = drain_messages(&mut deck);

    // Should get fewer than 20 messages (seeked past some)
    assert!(
        msgs.len() < 20,
        "expected fewer than 20 messages after seek, got {}",
        msgs.len()
    );
    assert!(!msgs.is_empty(), "expected at least some messages after seek");

    // All timestamps should be at or after midpoint
    for m in &msgs {
        assert!(
            m.message.timestamp_ns >= midpoint,
            "message {} before seek target {}",
            m.message.timestamp_ns,
            midpoint,
        );
    }
}
