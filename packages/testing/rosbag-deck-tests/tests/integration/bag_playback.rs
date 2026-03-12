//! Tests for playback: play, step, filter, ordering.

use std::collections::HashSet;

use super::helpers::*;

#[test]
fn play_talker_best_effort_all_messages() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));
    let msgs = drain_messages(&mut deck);

    assert_eq!(msgs.len(), 20, "expected 20 messages from talker bag");
}

#[test]
fn play_talker_timestamps_monotonic() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));
    let msgs = drain_messages(&mut deck);

    for window in msgs.windows(2) {
        assert!(
            window[1].message.timestamp_ns >= window[0].message.timestamp_ns,
            "timestamps not monotonic: {} < {}",
            window[1].message.timestamp_ns,
            window[0].message.timestamp_ns,
        );
    }
}

#[test]
fn play_convert_a_interleaved() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/convert_a"));
    let msgs = drain_messages(&mut deck);

    assert_eq!(msgs.len(), 150);

    // Both topics should appear
    let topics: HashSet<&str> = msgs.iter().map(|m| m.message.topic.as_str()).collect();
    assert!(topics.contains("a_empty"), "missing a_empty topic");
    assert!(
        topics.contains("b_basictypes"),
        "missing b_basictypes topic"
    );

    // Timestamps should be monotonically non-decreasing
    for window in msgs.windows(2) {
        assert!(
            window[1].message.timestamp_ns >= window[0].message.timestamp_ns,
            "timestamps not monotonic in convert_a"
        );
    }
}

#[test]
fn play_cdr_test_all_messages() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/cdr_test"));
    let msgs = drain_messages(&mut deck);

    assert_eq!(msgs.len(), 7);

    // Messages should have non-empty data
    for m in &msgs {
        assert!(!m.message.data.is_empty(), "message has empty data");
    }
}

#[test]
fn play_wbag_all_messages() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/wbag"));
    let msgs = drain_messages(&mut deck);

    assert_eq!(msgs.len(), 6074, "expected 6074 messages from wbag");

    // Timestamps should be non-decreasing
    for window in msgs.windows(2) {
        assert!(
            window[1].message.timestamp_ns >= window[0].message.timestamp_ns,
            "timestamps not monotonic in wbag"
        );
    }
}

#[test]
fn topic_filter_talker() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));

    // Filter to /topic only
    deck.set_topic_filter(Some(HashSet::from(["/topic".to_string()])));
    let msgs = drain_messages(&mut deck);

    assert_eq!(msgs.len(), 10, "expected 10 /topic messages");
    for m in &msgs {
        assert_eq!(m.message.topic, "/topic");
    }
}

#[test]
fn topic_filter_then_clear() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));

    // Filter to /rosout only
    deck.set_topic_filter(Some(HashSet::from(["/rosout".to_string()])));
    let msgs = drain_messages(&mut deck);
    assert_eq!(msgs.len(), 10);
    for m in &msgs {
        assert_eq!(m.message.topic, "/rosout");
    }

    // Clear filter, seek to start, play again
    deck.set_topic_filter(None);
    deck.seek_to_ratio(0.0);
    let msgs = drain_messages(&mut deck);
    assert_eq!(msgs.len(), 20, "all messages after clearing filter");
}

#[test]
fn step_forward_talker() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));

    // Step through all messages one by one
    let mut count = 0;
    let mut prev_ts = i64::MIN;
    while let Ok(Some(m)) = deck.step_forward() {
        assert!(
            m.message.timestamp_ns >= prev_ts,
            "step_forward not monotonic"
        );
        prev_ts = m.message.timestamp_ns;
        count += 1;
    }
    assert_eq!(count, 20, "expected 20 steps through talker");
}

#[test]
fn play_pause_resume() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));

    // Play a few messages
    deck.play();
    let m1 = deck.next_message().unwrap();
    assert!(m1.is_some());

    // Pause
    deck.pause();
    assert_eq!(deck.state(), rosbag_deck::PlaybackState::Paused);

    // Resume
    deck.play();
    assert_eq!(deck.state(), rosbag_deck::PlaybackState::Playing);

    // Should be able to get more messages
    let m2 = deck.next_message().unwrap();
    assert!(m2.is_some());
}
