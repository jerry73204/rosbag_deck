//! Tests for the edit pipeline using real bag files.
//!
//! These tests exercise: single-bag passthrough, topic filtering, time range
//! slicing, multi-bag merge, timestamp offset/scale, and dry-run mode.
//!
//! Output bags are written to temporary directories and re-opened to verify
//! correctness.

use std::collections::HashSet;

use rosbag_deck_core::{EditConfig, EditPipeline};
use rosbag_deck_ffi::Rosbag2Writer;

use super::helpers::*;

/// Open a writer targeting a temp directory with sqlite3 format.
fn open_writer(dir: &std::path::Path) -> Box<dyn rosbag_deck_core::writer::BagWriter> {
    Box::new(Rosbag2Writer::open(dir, "sqlite3").expect("failed to open writer"))
}

/// Read all messages from a bag, returning them in order.
fn read_all_messages(path: &std::path::Path) -> Vec<rosbag_deck_core::RawMessage> {
    let mut reader = open_bag(path);
    let mut msgs = Vec::new();
    while let Ok(Some(msg)) = reader.read_next() {
        msgs.push(msg);
    }
    msgs
}

// ---------------------------------------------------------------------------
// Single-bag passthrough
// ---------------------------------------------------------------------------

#[test]
fn edit_passthrough_talker() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("output_bag");

    let reader = open_bag(&input);
    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig::default(),
    );

    let stats = pipeline.run().unwrap();
    assert_eq!(stats.messages_read, 20);
    assert_eq!(stats.messages_written, 20);
    assert_eq!(stats.messages_filtered, 0);

    // Re-open the output and verify.
    let output_msgs = read_all_messages(&out_path);
    assert_eq!(output_msgs.len(), 20);

    // Timestamps should be monotonic.
    for w in output_msgs.windows(2) {
        assert!(
            w[1].timestamp_ns >= w[0].timestamp_ns,
            "output timestamps not monotonic: {} < {}",
            w[1].timestamp_ns,
            w[0].timestamp_ns,
        );
    }
}

#[test]
fn edit_passthrough_preserves_topics() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("output_bag");

    let reader = open_bag(&input);
    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig::default(),
    );
    let stats = pipeline.run().unwrap();

    // Output should have the same topics as input.
    let out_reader = open_bag(&out_path);
    let out_meta = out_reader.metadata();

    // talker has 3 topics: /topic, /rosout, /parameter_events (0 messages)
    // Only topics with messages are written.
    assert_eq!(stats.topics_written.len(), 2);
    assert!(stats.topics_written.contains("/topic"));
    assert!(stats.topics_written.contains("/rosout"));

    // Output metadata should list both topics.
    let out_topic_names: HashSet<&str> = out_meta.topics.iter().map(|t| t.name.as_str()).collect();
    assert!(out_topic_names.contains("/topic"));
    assert!(out_topic_names.contains("/rosout"));
}

#[test]
fn edit_passthrough_preserves_data() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("output_bag");

    let reader = open_bag(&input);
    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig::default(),
    );
    pipeline.run().unwrap();

    // Read both input and output.
    let input_msgs = read_all_messages(&bag_path!(bags, "rosbag2/talker"));
    let output_msgs = read_all_messages(&out_path);

    assert_eq!(input_msgs.len(), output_msgs.len());
    for (inp, out) in input_msgs.iter().zip(output_msgs.iter()) {
        assert_eq!(inp.timestamp_ns, out.timestamp_ns, "timestamp mismatch");
        assert_eq!(inp.topic, out.topic, "topic mismatch");
        assert_eq!(inp.data, out.data, "data mismatch for {}", inp.topic);
    }
}

// ---------------------------------------------------------------------------
// Topic filtering
// ---------------------------------------------------------------------------

#[test]
fn edit_topic_filter_single_topic() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("filtered");

    let reader = open_bag(&input);
    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig {
            topic_filter: Some(HashSet::from(["/topic".to_string()])),
            ..Default::default()
        },
    );

    let stats = pipeline.run().unwrap();
    assert_eq!(stats.messages_written, 10, "expected 10 /topic messages");
    assert_eq!(stats.topics_written.len(), 1);
    assert!(stats.topics_written.contains("/topic"));

    // Verify output contains only /topic.
    let output_msgs = read_all_messages(&out_path);
    assert_eq!(output_msgs.len(), 10);
    for msg in &output_msgs {
        assert_eq!(msg.topic, "/topic", "unexpected topic: {}", msg.topic);
    }
}

#[test]
fn edit_topic_filter_excludes_all() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("empty_output");

    let reader = open_bag(&input);
    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig {
            topic_filter: Some(HashSet::from(["/nonexistent_topic".to_string()])),
            ..Default::default()
        },
    );

    let stats = pipeline.run().unwrap();
    assert_eq!(stats.messages_written, 0);
    assert_eq!(stats.messages_filtered, 20);
}

#[test]
fn edit_topic_filter_convert_a() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/convert_a");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("filtered_a");

    let reader = open_bag(&input);
    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig {
            topic_filter: Some(HashSet::from(["a_empty".to_string()])),
            ..Default::default()
        },
    );

    let stats = pipeline.run().unwrap();
    assert!(stats.messages_written > 0);
    assert!(stats.topics_written.contains("a_empty"));
    assert!(!stats.topics_written.contains("b_basictypes"));

    let output_msgs = read_all_messages(&out_path);
    for msg in &output_msgs {
        assert_eq!(msg.topic, "a_empty");
    }
}

// ---------------------------------------------------------------------------
// Time range slicing
// ---------------------------------------------------------------------------

#[test]
fn edit_time_range_slice_talker() {
    let bags = test_bags_dir();
    let input_path = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("sliced");

    let reader = open_bag(&input_path);
    let start_ns = reader.metadata().start_time_ns;
    let end_ns = reader.metadata().end_time_ns;
    let mid_ns = start_ns + (end_ns - start_ns) / 2;

    let reader = open_bag(&input_path);
    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig {
            time_range: Some((mid_ns, end_ns)),
            ..Default::default()
        },
    );

    let stats = pipeline.run().unwrap();
    assert!(
        stats.messages_written < 20,
        "slicing should produce fewer than 20 messages, got {}",
        stats.messages_written
    );
    assert!(
        stats.messages_written > 0,
        "slicing should produce at least some messages"
    );
    assert!(stats.messages_filtered > 0);

    // All output messages should be >= mid_ns.
    let output_msgs = read_all_messages(&out_path);
    for msg in &output_msgs {
        assert!(
            msg.timestamp_ns >= mid_ns,
            "output message timestamp {} is before slice start {}",
            msg.timestamp_ns,
            mid_ns,
        );
    }
}

#[test]
fn edit_time_range_empty_window() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("empty_slice");

    let reader = open_bag(&input);
    // Use a time range far in the future — no messages should match.
    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig {
            time_range: Some((i64::MAX - 1000, i64::MAX)),
            ..Default::default()
        },
    );

    let stats = pipeline.run().unwrap();
    assert_eq!(stats.messages_written, 0);
    assert_eq!(stats.messages_filtered, 20);
}

// ---------------------------------------------------------------------------
// Multi-bag merge
// ---------------------------------------------------------------------------

#[test]
fn edit_merge_two_bags() {
    let bags = test_bags_dir();
    let input_a = bag_path!(bags, "rosbag2/talker");
    let input_b = bag_path!(bags, "rosbag2/cdr_test");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("merged");

    let reader_a = open_bag(&input_a);
    let reader_b = open_bag(&input_b);

    let count_a = reader_a.metadata().message_count;
    let count_b = reader_b.metadata().message_count;

    let pipeline = EditPipeline::new(
        vec![reader_a, reader_b],
        Some(open_writer(&out_path)),
        EditConfig::default(),
    );

    let stats = pipeline.run().unwrap();
    assert_eq!(
        stats.messages_written,
        count_a + count_b,
        "merged output should contain all messages from both bags"
    );

    // Verify merged output is in timestamp order.
    let output_msgs = read_all_messages(&out_path);
    assert_eq!(output_msgs.len(), (count_a + count_b) as usize);
    for w in output_msgs.windows(2) {
        assert!(
            w[1].timestamp_ns >= w[0].timestamp_ns,
            "merged output not in order: {} < {}",
            w[1].timestamp_ns,
            w[0].timestamp_ns,
        );
    }
}

#[test]
fn edit_merge_deduplicates_topics() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("merged_self");

    // Merge talker with itself — topics should be deduplicated.
    let reader_a = open_bag(&input);
    let reader_b = open_bag(&input);

    let pipeline = EditPipeline::new(
        vec![reader_a, reader_b],
        Some(open_writer(&out_path)),
        EditConfig::default(),
    );

    let stats = pipeline.run().unwrap();
    assert_eq!(stats.messages_written, 40, "2x talker = 40 messages");

    // Output metadata should still have the same number of unique topics.
    let out_reader = open_bag(&out_path);
    let out_meta = out_reader.metadata();
    let out_topic_names: HashSet<&str> = out_meta.topics.iter().map(|t| t.name.as_str()).collect();
    assert!(out_topic_names.contains("/topic"));
    assert!(out_topic_names.contains("/rosout"));
}

// ---------------------------------------------------------------------------
// Timestamp offset
// ---------------------------------------------------------------------------

#[test]
fn edit_timestamp_offset_positive() {
    let bags = test_bags_dir();
    let input_path = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("offset_pos");

    let offset_ns: i64 = 5_000_000_000; // +5 seconds

    let reader = open_bag(&input_path);
    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig {
            time_offset_ns: offset_ns,
            ..Default::default()
        },
    );
    pipeline.run().unwrap();

    // Compare input and output timestamps.
    let input_msgs = read_all_messages(&input_path);
    let output_msgs = read_all_messages(&out_path);
    assert_eq!(input_msgs.len(), output_msgs.len());

    for (inp, out) in input_msgs.iter().zip(output_msgs.iter()) {
        assert_eq!(
            out.timestamp_ns,
            inp.timestamp_ns + offset_ns,
            "timestamp not shifted correctly for topic {}",
            inp.topic,
        );
    }
}

#[test]
fn edit_timestamp_offset_negative() {
    let bags = test_bags_dir();
    let input_path = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("offset_neg");

    let offset_ns: i64 = -1_000_000_000; // -1 second

    let reader = open_bag(&input_path);
    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig {
            time_offset_ns: offset_ns,
            ..Default::default()
        },
    );
    pipeline.run().unwrap();

    let input_msgs = read_all_messages(&input_path);
    let output_msgs = read_all_messages(&out_path);
    assert_eq!(input_msgs.len(), output_msgs.len());

    for (inp, out) in input_msgs.iter().zip(output_msgs.iter()) {
        assert_eq!(
            out.timestamp_ns,
            inp.timestamp_ns + offset_ns,
            "negative offset not applied correctly",
        );
    }
}

// ---------------------------------------------------------------------------
// Timestamp scale
// ---------------------------------------------------------------------------

#[test]
fn edit_timestamp_scale_double() {
    let bags = test_bags_dir();
    let input_path = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("scaled_2x");

    let reader = open_bag(&input_path);
    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig {
            time_scale: 2.0,
            ..Default::default()
        },
    );
    pipeline.run().unwrap();

    let input_msgs = read_all_messages(&input_path);
    let output_msgs = read_all_messages(&out_path);
    assert_eq!(input_msgs.len(), output_msgs.len());

    // First message timestamp should be unchanged (it's the anchor).
    let anchor = input_msgs[0].timestamp_ns;
    assert_eq!(output_msgs[0].timestamp_ns, anchor);

    // Subsequent intervals should be doubled.
    for (inp, out) in input_msgs.iter().zip(output_msgs.iter()).skip(1) {
        let expected = anchor + (inp.timestamp_ns - anchor) * 2;
        assert_eq!(
            out.timestamp_ns, expected,
            "scaled timestamp mismatch: input {} -> expected {}, got {}",
            inp.timestamp_ns, expected, out.timestamp_ns,
        );
    }

    // Output should still be monotonic.
    for w in output_msgs.windows(2) {
        assert!(w[1].timestamp_ns >= w[0].timestamp_ns);
    }
}

#[test]
fn edit_timestamp_scale_half() {
    let bags = test_bags_dir();
    let input_path = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("scaled_half");

    let reader = open_bag(&input_path);
    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig {
            time_scale: 0.5,
            ..Default::default()
        },
    );
    pipeline.run().unwrap();

    let input_msgs = read_all_messages(&input_path);
    let output_msgs = read_all_messages(&out_path);
    let anchor = input_msgs[0].timestamp_ns;

    // Intervals should be halved.
    for (inp, out) in input_msgs.iter().zip(output_msgs.iter()).skip(1) {
        let expected = anchor + ((inp.timestamp_ns - anchor) as f64 * 0.5) as i64;
        assert_eq!(out.timestamp_ns, expected);
    }
}

// ---------------------------------------------------------------------------
// Combined: scale + offset
// ---------------------------------------------------------------------------

#[test]
fn edit_scale_and_offset_combined() {
    let bags = test_bags_dir();
    let input_path = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("combined");

    let offset_ns: i64 = 2_000_000_000; // +2s
    let scale = 0.5;

    let reader = open_bag(&input_path);
    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig {
            time_offset_ns: offset_ns,
            time_scale: scale,
            ..Default::default()
        },
    );
    pipeline.run().unwrap();

    let input_msgs = read_all_messages(&input_path);
    let output_msgs = read_all_messages(&out_path);
    let anchor = input_msgs[0].timestamp_ns;

    // transform: anchor + (ts - anchor) * scale + offset
    for (inp, out) in input_msgs.iter().zip(output_msgs.iter()) {
        let expected = anchor + ((inp.timestamp_ns - anchor) as f64 * scale) as i64 + offset_ns;
        assert_eq!(
            out.timestamp_ns, expected,
            "combined transform mismatch for {}",
            inp.topic,
        );
    }
}

// ---------------------------------------------------------------------------
// Dry run
// ---------------------------------------------------------------------------

#[test]
fn edit_dry_run_matches_real_run() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");

    // Dry run (no writer).
    let reader = open_bag(&input);
    let dry_pipeline = EditPipeline::new(
        vec![reader],
        None,
        EditConfig {
            topic_filter: Some(HashSet::from(["/topic".to_string()])),
            dry_run: true,
            ..Default::default()
        },
    );
    let dry_stats = dry_pipeline.run().unwrap();

    // Real run.
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("real");
    let reader = open_bag(&input);
    let real_pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig {
            topic_filter: Some(HashSet::from(["/topic".to_string()])),
            ..Default::default()
        },
    );
    let real_stats = real_pipeline.run().unwrap();

    assert_eq!(dry_stats.messages_read, real_stats.messages_read);
    assert_eq!(dry_stats.messages_written, real_stats.messages_written);
    assert_eq!(dry_stats.messages_filtered, real_stats.messages_filtered);
    assert_eq!(dry_stats.topics_written, real_stats.topics_written);
    assert_eq!(dry_stats.output_time_range, real_stats.output_time_range);
}

// ---------------------------------------------------------------------------
// Wbag: larger bag with 8 topics, 6074 messages
// ---------------------------------------------------------------------------

#[test]
fn edit_passthrough_wbag() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/wbag");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("wbag_out");

    let reader = open_bag(&input);
    let expected_count = reader.metadata().message_count;

    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig::default(),
    );

    let stats = pipeline.run().unwrap();
    assert_eq!(stats.messages_written, expected_count);

    // Verify output is readable and correct count.
    let out_reader = open_bag(&out_path);
    assert_eq!(out_reader.metadata().message_count, expected_count);
}

#[test]
fn edit_wbag_topic_filter_single() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/wbag");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("wbag_filtered");

    let reader = open_bag(&input);
    let aaa_count = reader
        .metadata()
        .topics
        .iter()
        .find(|t| t.name == "AAA")
        .map(|t| t.message_count)
        .unwrap_or(0);

    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig {
            topic_filter: Some(HashSet::from(["AAA".to_string()])),
            ..Default::default()
        },
    );

    let stats = pipeline.run().unwrap();
    assert_eq!(stats.messages_written, aaa_count);

    let output_msgs = read_all_messages(&out_path);
    for msg in &output_msgs {
        assert_eq!(msg.topic, "AAA");
    }
}

#[test]
fn edit_wbag_time_slice() {
    let bags = test_bags_dir();
    let input_path = bag_path!(bags, "rosbag2/wbag");

    let reader = open_bag(&input_path);
    let start_ns = reader.metadata().start_time_ns;
    let end_ns = reader.metadata().end_time_ns;
    // Slice the first quarter.
    let quarter_ns = start_ns + (end_ns - start_ns) / 4;

    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("wbag_sliced");

    let reader = open_bag(&input_path);
    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig {
            time_range: Some((start_ns, quarter_ns)),
            ..Default::default()
        },
    );

    let stats = pipeline.run().unwrap();
    assert!(stats.messages_written > 0);
    assert!(stats.messages_written < 6074);

    // All output messages should be within the time range.
    let output_msgs = read_all_messages(&out_path);
    for msg in &output_msgs {
        assert!(
            msg.timestamp_ns >= start_ns && msg.timestamp_ns <= quarter_ns,
            "message at {} outside time range [{}, {}]",
            msg.timestamp_ns,
            start_ns,
            quarter_ns,
        );
    }
}

// ---------------------------------------------------------------------------
// Output bag is playable by Deck
// ---------------------------------------------------------------------------

#[test]
fn edit_output_is_playable() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("playable");

    let reader = open_bag(&input);
    let pipeline = EditPipeline::new(
        vec![reader],
        Some(open_writer(&out_path)),
        EditConfig {
            topic_filter: Some(HashSet::from(["/topic".to_string()])),
            ..Default::default()
        },
    );
    pipeline.run().unwrap();

    // Open with Deck and drain all messages.
    let mut deck = open_deck(&out_path);
    let msgs = drain_messages(&mut deck);
    assert_eq!(msgs.len(), 10, "Deck should play all 10 /topic messages");
    for m in &msgs {
        assert_eq!(m.message.topic, "/topic");
    }
}

// ---------------------------------------------------------------------------
// CLI: `edit` subcommand integration
// ---------------------------------------------------------------------------

#[test]
fn cli_edit_dry_run() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");

    cli()
        .arg("edit")
        .arg(input.to_str().unwrap())
        .arg("--dry-run")
        .timeout(std::time::Duration::from_secs(10))
        .assert()
        .success()
        .stderr(predicates::str::contains("20 read"))
        .stderr(predicates::str::contains("20 written"));
}

#[test]
fn cli_edit_with_topic_filter() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");

    cli()
        .arg("edit")
        .arg(input.to_str().unwrap())
        .arg("--topics")
        .arg("/topic")
        .arg("--dry-run")
        .timeout(std::time::Duration::from_secs(10))
        .assert()
        .success()
        .stderr(predicates::str::contains("10 written"));
}

#[test]
fn cli_edit_with_exclude() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");

    cli()
        .arg("edit")
        .arg(input.to_str().unwrap())
        .arg("--exclude")
        .arg("/rosout")
        .arg("--dry-run")
        .timeout(std::time::Duration::from_secs(10))
        .assert()
        .success()
        .stderr(predicates::str::contains("10 written"));
}

#[test]
fn cli_edit_with_time_range() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");

    // Slice to first 2 seconds.
    cli()
        .arg("edit")
        .arg(input.to_str().unwrap())
        .arg("--start")
        .arg("0")
        .arg("--duration")
        .arg("2")
        .arg("--dry-run")
        .timeout(std::time::Duration::from_secs(10))
        .assert()
        .success()
        .stderr(predicates::str::contains("written"));
}

#[test]
fn cli_edit_output_to_file() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("cli_output");

    cli()
        .arg("edit")
        .arg(input.to_str().unwrap())
        .arg("-o")
        .arg(out_path.to_str().unwrap())
        .timeout(std::time::Duration::from_secs(10))
        .assert()
        .success()
        .stderr(predicates::str::contains("20 written"));

    // Verify the output bag exists and is readable.
    let out_reader = open_bag(&out_path);
    assert_eq!(out_reader.metadata().message_count, 20);
}

#[test]
fn cli_edit_requires_output_or_dry_run() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");

    cli()
        .arg("edit")
        .arg(input.to_str().unwrap())
        .timeout(std::time::Duration::from_secs(10))
        .assert()
        .failure()
        .stderr(predicates::str::contains("--output is required"));
}

#[test]
fn cli_edit_with_time_offset() {
    let bags = test_bags_dir();
    let input = bag_path!(bags, "rosbag2/talker");
    let tmp = tempfile::tempdir().unwrap();
    let out_path = tmp.path().join("offset_output");

    cli()
        .arg("edit")
        .arg(input.to_str().unwrap())
        .arg("-o")
        .arg(out_path.to_str().unwrap())
        .arg("--time-offset")
        .arg("5.0")
        .timeout(std::time::Duration::from_secs(10))
        .assert()
        .success();

    // Check that timestamps are shifted.
    let input_msgs = read_all_messages(&bag_path!(bags, "rosbag2/talker"));
    let output_msgs = read_all_messages(&out_path);
    let offset_ns: i64 = 5_000_000_000;

    assert_eq!(input_msgs.len(), output_msgs.len());
    for (inp, out) in input_msgs.iter().zip(output_msgs.iter()) {
        assert_eq!(out.timestamp_ns, inp.timestamp_ns + offset_ns);
    }
}

// ---------------------------------------------------------------------------
// Helpers used by CLI tests
// ---------------------------------------------------------------------------

/// Find the rosbag_deck binary (same as cli.rs).
fn cli_binary() -> std::path::PathBuf {
    let mut dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    for _ in 0..5 {
        let target = dir.join("target");
        if target.is_dir() {
            for profile in ["dev-release", "release", "debug"] {
                let bin = target.join(profile).join("rosbag_deck");
                if bin.exists() {
                    return bin;
                }
            }
        }
        if !dir.pop() {
            break;
        }
    }
    panic!("rosbag_deck binary not found. Build first: cargo build --profile dev-release -p rosbag_deck");
}

fn cli() -> assert_cmd::Command {
    assert_cmd::Command::from_std(std::process::Command::new(cli_binary()))
}
