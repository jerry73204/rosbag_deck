//! Tests for opening bags and reading metadata.

use super::helpers::*;

#[test]
fn open_talker_metadata() {
    let bags = test_bags_dir();
    let path = bag_path!(bags, "rosbag2/talker");
    let reader = open_bag(&path);
    let meta = reader.metadata();

    assert_eq!(meta.message_count, 20);
    assert_eq!(meta.topics.len(), 3);
    assert_eq!(meta.storage_identifier, "sqlite3");
    assert_eq!(meta.start_time_ns, 1585866235112411371);

    // Duration should be ~4.53s
    let dur = meta.duration.as_secs_f64();
    assert!(dur > 4.0 && dur < 5.0, "unexpected duration: {dur}");

    // Check individual topics
    let topic = meta.topics.iter().find(|t| t.name == "/topic").unwrap();
    assert_eq!(topic.type_name, "std_msgs/msg/String");
    assert_eq!(topic.message_count, 10);

    let rosout = meta.topics.iter().find(|t| t.name == "/rosout").unwrap();
    assert_eq!(rosout.type_name, "rcl_interfaces/msg/Log");
    assert_eq!(rosout.message_count, 10);

    let param = meta
        .topics
        .iter()
        .find(|t| t.name == "/parameter_events")
        .unwrap();
    assert_eq!(param.message_count, 0);
}

#[test]
fn open_cdr_test_metadata() {
    let bags = test_bags_dir();
    let path = bag_path!(bags, "rosbag2/cdr_test");
    let reader = open_bag(&path);
    let meta = reader.metadata();

    assert_eq!(meta.message_count, 7);
    assert_eq!(meta.topics.len(), 2);
    assert_eq!(meta.storage_identifier, "sqlite3");

    let basic = meta
        .topics
        .iter()
        .find(|t| t.name == "/test_topic")
        .unwrap();
    assert_eq!(basic.message_count, 3);

    let arrays = meta
        .topics
        .iter()
        .find(|t| t.name == "/array_topic")
        .unwrap();
    assert_eq!(arrays.message_count, 4);
}

#[test]
fn open_empty_bag_metadata() {
    let bags = test_bags_dir();
    let path = bag_path!(bags, "rosbag2/empty_bag");
    let reader = open_bag(&path);
    let meta = reader.metadata();

    assert_eq!(meta.message_count, 0);
    assert_eq!(meta.duration.as_nanos(), 0);
    assert_eq!(meta.topics.len(), 2);
}

#[test]
fn open_seek_sqlite3_metadata() {
    let bags = test_bags_dir();
    let path = bag_path!(bags, "rosbag2/test_bag_for_seek_sqlite3");
    let reader = open_bag(&path);
    let meta = reader.metadata();

    assert_eq!(meta.message_count, 5);
    assert_eq!(meta.topics.len(), 1);
    assert_eq!(meta.topics[0].name, "topic1");
    assert_eq!(meta.start_time_ns, 1000000000);
}

#[test]
fn open_wbag_metadata() {
    let bags = test_bags_dir();
    let path = bag_path!(bags, "rosbag2/wbag");
    let reader = open_bag(&path);
    let meta = reader.metadata();

    assert_eq!(meta.message_count, 6074);
    assert_eq!(meta.topics.len(), 8);
    assert_eq!(meta.storage_identifier, "sqlite3");

    // Verify all 8 topics are present
    let topic_names: Vec<&str> = meta.topics.iter().map(|t| t.name.as_str()).collect();
    for name in ["AAA", "BBB", "CCC", "DDD", "EEE", "FFF", "GGG", "HHH"] {
        assert!(topic_names.contains(&name), "missing topic {name} in wbag");
    }
}

#[test]
fn open_deck_from_talker() {
    let bags = test_bags_dir();
    let path = bag_path!(bags, "rosbag2/talker");
    let deck = open_deck(&path);
    let meta = deck.metadata();

    assert_eq!(meta.message_count, 20);
    assert_eq!(deck.state(), rosbag_deck::PlaybackState::Stopped);
}
