//! Edge case tests: empty bags, error handling.

use super::helpers::*;
use rosbag_deck::{Deck, DeckConfig, PlaybackMode};

#[test]
fn empty_bag_playback() {
    let bags = test_bags_dir();
    let path = bag_path!(bags, "rosbag2/empty_bag");
    let mut deck = open_deck(&path);

    deck.play();
    let msg = deck.next_message().unwrap();
    assert!(msg.is_none(), "expected no messages from empty bag");
}

#[test]
fn empty_bag_step_forward() {
    let bags = test_bags_dir();
    let path = bag_path!(bags, "rosbag2/empty_bag");
    let mut deck = open_deck(&path);

    let msg = deck.step_forward().unwrap();
    assert!(msg.is_none(), "expected no messages from empty bag step");
}

#[test]
fn open_no_readers() {
    let result = Deck::open(vec![], DeckConfig::default());
    assert!(result.is_err(), "opening with no readers should fail");
}

#[test]
fn double_play_is_idempotent() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));

    deck.play();
    deck.play(); // should not panic or double-start

    let msg = deck.next_message().unwrap();
    assert!(msg.is_some());
}

#[test]
fn stop_then_play_restarts() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));

    // Play some messages
    deck.play();
    let _ = deck.next_message().unwrap();
    let _ = deck.next_message().unwrap();

    // Stop
    deck.stop();
    assert_eq!(deck.state(), rosbag_deck::PlaybackState::Stopped);

    // Play again — should be able to get messages
    let msgs = drain_messages(&mut deck);
    assert!(!msgs.is_empty(), "should be able to play after stop");
}

#[test]
fn speed_changes() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));

    deck.set_speed(2.0);
    assert!((deck.speed() - 2.0).abs() < f64::EPSILON);

    deck.set_speed(0.5);
    assert!((deck.speed() - 0.5).abs() < f64::EPSILON);
}

#[test]
fn looping_flag() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));

    assert!(!deck.looping());
    deck.set_looping(true);
    assert!(deck.looping());
    deck.set_looping(false);
    assert!(!deck.looping());
}

#[test]
fn mode_switching() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/talker"));

    assert_eq!(deck.mode(), PlaybackMode::BestEffort); // set by open_deck helper
    deck.set_mode(PlaybackMode::RealTime);
    assert_eq!(deck.mode(), PlaybackMode::RealTime);
    deck.set_mode(PlaybackMode::BestEffort);
    assert_eq!(deck.mode(), PlaybackMode::BestEffort);
}

#[test]
fn wbag_topic_counts_match_metadata() {
    let bags = test_bags_dir();
    let mut deck = open_deck(&bag_path!(bags, "rosbag2/wbag"));
    let meta = deck.metadata().clone();

    let msgs = drain_messages(&mut deck);

    // Count messages per topic
    let mut counts = std::collections::HashMap::new();
    for m in &msgs {
        *counts.entry(m.message.topic.as_str()).or_insert(0u64) += 1;
    }

    // Verify against metadata
    for topic_info in &meta.topics {
        let actual = counts.get(topic_info.name.as_str()).copied().unwrap_or(0);
        assert_eq!(
            actual, topic_info.message_count,
            "topic {} count mismatch: expected {}, got {}",
            topic_info.name, topic_info.message_count, actual
        );
    }
}
