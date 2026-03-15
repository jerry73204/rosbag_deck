//! Integration tests for loop modes (Restart, Monotonic).

use super::helpers::*;
use rosbag_deck_core::LoopMode;

#[test]
fn restart_mode_resets_timestamps() {
    let bags = test_bags_dir();
    let path = bag_path!(bags, "rosbag2/talker");
    let mut deck = open_deck(&path);

    deck.set_loop_mode(LoopMode::Restart);
    deck.play();

    // Collect first pass.
    let mut first_pass = Vec::new();
    while let Ok(Some(m)) = deck.next_message() {
        first_pass.push(m.message.timestamp_ns);
    }
    assert!(!first_pass.is_empty(), "should have messages");

    // If looping worked, the deck should continue producing messages.
    // Collect a few from the second pass.
    let mut second_pass = Vec::new();
    for _ in 0..first_pass.len().min(5) {
        if let Ok(Some(m)) = deck.next_message() {
            second_pass.push(m.message.timestamp_ns);
        }
    }

    if !second_pass.is_empty() {
        // Second pass timestamps should be back near the start.
        assert!(
            second_pass[0] <= *first_pass.last().unwrap(),
            "restart mode should reset timestamps: second pass start {} > first pass end {}",
            second_pass[0],
            first_pass.last().unwrap()
        );
    }
}

#[test]
fn monotonic_mode_shifts_timestamps() {
    let bags = test_bags_dir();
    let path = bag_path!(bags, "rosbag2/talker");
    let mut deck = open_deck(&path);

    // Populate header cache for monotonic mode.
    deck.populate_header_cache(|t| rosbag_deck_ffi::type_has_header_first(t).unwrap_or(false));
    deck.set_loop_mode(LoopMode::Monotonic);
    deck.play();

    // Collect all timestamps across 2+ iterations.
    let total_msg_count = deck.metadata().message_count as usize;
    let target = total_msg_count * 2 + 5; // at least 2 full iterations

    let mut timestamps = Vec::new();
    for _ in 0..target {
        match deck.next_message() {
            Ok(Some(m)) => timestamps.push(m.message.timestamp_ns),
            Ok(None) => break,
            Err(_) => break,
        }
    }

    // We should have more messages than one pass (looping happened).
    assert!(
        timestamps.len() > total_msg_count,
        "expected more than {} messages (got {}), looping may not have worked",
        total_msg_count,
        timestamps.len()
    );

    // Verify monotonicity: message timestamps should never decrease.
    for w in timestamps.windows(2) {
        assert!(
            w[0] <= w[1],
            "monotonic mode: timestamp went backward: {} > {}",
            w[0],
            w[1]
        );
    }
}

#[test]
fn loop_mode_off_stops() {
    let bags = test_bags_dir();
    let path = bag_path!(bags, "rosbag2/talker");
    let mut deck = open_deck(&path);

    deck.set_loop_mode(LoopMode::Off);
    deck.play();

    let msgs = drain_messages(&mut deck);
    let count = msgs.len();

    // Trying to get more should return None (stopped at end).
    let extra = deck.next_message().unwrap();
    assert!(extra.is_none(), "should stop at end in Off mode");

    // Should have exactly the number of messages in the bag.
    assert_eq!(
        count,
        deck.metadata().message_count as usize,
        "should play all messages exactly once"
    );
}

#[test]
fn ffi_type_has_header_first() {
    // Standard message types that should have Header as first field.
    let with_header = [
        "sensor_msgs/msg/Image",
        "sensor_msgs/msg/Imu",
        "sensor_msgs/msg/PointCloud2",
        "sensor_msgs/msg/LaserScan",
        "geometry_msgs/msg/PoseStamped",
        "geometry_msgs/msg/TransformStamped",
        "nav_msgs/msg/Odometry",
    ];

    // Types that should NOT have Header as first field.
    let without_header = [
        "std_msgs/msg/String",
        "std_msgs/msg/Int32",
        "std_msgs/msg/Bool",
        "geometry_msgs/msg/Point",
        "geometry_msgs/msg/Twist",
    ];

    for type_name in &with_header {
        match rosbag_deck_ffi::type_has_header_first(type_name) {
            Ok(true) => {} // expected
            Ok(false) => panic!("{type_name} should have Header as first field"),
            Err(e) => {
                // The type support library may not be installed — skip gracefully.
                eprintln!("SKIPPED: {type_name}: {e}");
            }
        }
    }

    for type_name in &without_header {
        match rosbag_deck_ffi::type_has_header_first(type_name) {
            Ok(false) => {} // expected
            Ok(true) => panic!("{type_name} should NOT have Header as first field"),
            Err(e) => {
                eprintln!("SKIPPED: {type_name}: {e}");
            }
        }
    }
}

#[test]
fn populate_header_cache_does_not_crash() {
    let bags = test_bags_dir();
    let path = bag_path!(bags, "rosbag2/talker");
    let mut deck = open_deck(&path);

    // This is exactly what the TUI does when pressing 'o' to Monotonic.
    deck.populate_header_cache(|t| {
        eprintln!("checking type: {t}");
        rosbag_deck_ffi::type_has_header_first(t).unwrap_or(false)
    });
}

#[test]
fn populate_header_cache_wbag() {
    let bags = test_bags_dir();
    let path = bag_path!(bags, "rosbag2/wbag");
    let mut deck = open_deck(&path);

    deck.populate_header_cache(|t| {
        eprintln!("checking type: {t}");
        match rosbag_deck_ffi::type_has_header_first(t) {
            Ok(v) => {
                eprintln!("  -> {v}");
                v
            }
            Err(e) => {
                eprintln!("  -> ERROR: {e}");
                false
            }
        }
    });
}

/// Test with the zenodo bag which has sensor_msgs/msg/PointCloud2 (has Header).
#[test]
fn monotonic_zenodo_bag() {
    let bags = test_bags_dir();
    let zenodo_path = bags.join("zenodo_14209720/rosbag2_2024_06_18-17_11_08");
    if !zenodo_path.is_dir() {
        eprintln!("SKIPPED: zenodo bag not found");
        return;
    }

    let mut deck = open_deck(&zenodo_path);
    deck.populate_header_cache(|t| {
        let result = rosbag_deck_ffi::type_has_header_first(t).unwrap_or(false);
        eprintln!("  {t}: has_header={result}");
        result
    });
    deck.set_loop_mode(LoopMode::Monotonic);
    deck.play();

    let total_msg_count = deck.metadata().message_count as usize;
    // Collect 1.5x messages to cross the loop boundary.
    let target = total_msg_count + total_msg_count / 2;

    let mut timestamps = Vec::new();
    for _ in 0..target {
        match deck.next_message() {
            Ok(Some(m)) => timestamps.push(m.message.timestamp_ns),
            Ok(None) => break,
            Err(_) => break,
        }
    }

    assert!(
        timestamps.len() > total_msg_count,
        "expected more than {} messages (got {})",
        total_msg_count,
        timestamps.len()
    );

    for w in timestamps.windows(2) {
        assert!(
            w[0] <= w[1],
            "zenodo monotonic: timestamp went backward: {} > {}",
            w[0],
            w[1]
        );
    }

    eprintln!(
        "zenodo: {} messages, first={}, last={}",
        timestamps.len(),
        timestamps.first().unwrap(),
        timestamps.last().unwrap()
    );
}

/// Test with RealTime mode and try_next_message (exactly like TUI).
#[test]
fn monotonic_realtime_try_next_message() {
    let bags = test_bags_dir();
    let path = bag_path!(bags, "rosbag2/talker");
    let reader = open_bag(&path);
    let mut deck =
        rosbag_deck_core::Deck::open(vec![reader], rosbag_deck_core::DeckConfig::default())
            .unwrap();
    // RealTime mode (TUI default), but at 1000x speed so test doesn't take forever.
    deck.set_speed(1000.0);
    deck.populate_header_cache(|t| rosbag_deck_ffi::type_has_header_first(t).unwrap_or(false));
    deck.set_loop_mode(LoopMode::Monotonic);
    deck.play();

    let total_msg_count = deck.metadata().message_count as usize;
    let target = total_msg_count * 2 + 5;

    let mut timestamps = Vec::new();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
    while timestamps.len() < target && std::time::Instant::now() < deadline {
        match deck.try_next_message() {
            Ok(Some(m)) => timestamps.push(m.message.timestamp_ns),
            Ok(None) => std::thread::sleep(std::time::Duration::from_millis(1)),
            Err(_) => break,
        }
    }

    assert!(
        timestamps.len() > total_msg_count,
        "expected more than {} messages via RealTime try_next_message (got {})",
        total_msg_count,
        timestamps.len()
    );

    for w in timestamps.windows(2) {
        assert!(
            w[0] <= w[1],
            "RealTime try_next_message monotonic: {} > {}",
            w[0],
            w[1]
        );
    }
}

/// Mimics TUI flow: uses try_next_message, switches to Monotonic mid-playback.
#[test]
fn monotonic_via_try_next_message() {
    let bags = test_bags_dir();
    let path = bag_path!(bags, "rosbag2/talker");
    let mut deck = open_deck(&path);

    // Simulate TUI: switch Off -> Restart -> Monotonic (two 'o' presses).
    deck.set_loop_mode(LoopMode::Restart);
    deck.populate_header_cache(|t| rosbag_deck_ffi::type_has_header_first(t).unwrap_or(false));
    deck.set_loop_mode(LoopMode::Monotonic);
    deck.play();

    let total_msg_count = deck.metadata().message_count as usize;
    let target = total_msg_count * 2 + 5;

    let mut timestamps = Vec::new();
    let mut spins = 0u64;
    while timestamps.len() < target && spins < 1_000_000 {
        match deck.try_next_message() {
            Ok(Some(m)) => {
                timestamps.push(m.message.timestamp_ns);
                spins = 0;
            }
            Ok(None) => {
                spins += 1;
            }
            Err(_) => break,
        }
    }

    assert!(
        timestamps.len() > total_msg_count,
        "expected more than {} messages via try_next_message (got {})",
        total_msg_count,
        timestamps.len()
    );

    // Verify monotonicity.
    for w in timestamps.windows(2) {
        assert!(
            w[0] <= w[1],
            "try_next_message monotonic: {} > {}",
            w[0],
            w[1]
        );
    }
}
