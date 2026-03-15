//! CLI integration tests using assert_cmd.

use std::{path::PathBuf, process::Command};

use super::helpers::test_bags_dir;

/// Find the rosbag_deck binary in the cargo target directory.
fn cli_binary() -> PathBuf {
    // Walk up from CARGO_MANIFEST_DIR to find the workspace root with target/.
    let mut dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    for _ in 0..5 {
        let target = dir.join("target");
        if target.is_dir() {
            // Check common profile directories.
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
    panic!(
        "rosbag_deck binary not found in target/. Build it first:\n\
         \n\
         \tcargo build --profile dev-release -p rosbag_deck\n"
    );
}

fn cli() -> assert_cmd::Command {
    assert_cmd::Command::from_std(Command::new(cli_binary()))
}

#[test]
fn info_talker() {
    let bags = test_bags_dir();
    let path = bags.join("rosbag2/talker");

    cli()
        .arg("info")
        .arg(path.to_str().unwrap())
        .assert()
        .success()
        .stdout(predicates::str::contains("/topic"))
        .stdout(predicates::str::contains("std_msgs/msg/String"))
        .stdout(predicates::str::contains("20"));
}

#[test]
fn info_nonexistent_path() {
    cli()
        .arg("info")
        .arg("/nonexistent/path/to/bag")
        .assert()
        .failure();
}

#[test]
fn play_no_tui_talker() {
    let bags = test_bags_dir();
    let path = bags.join("rosbag2/talker");

    let output = cli()
        .arg("play")
        .arg(path.to_str().unwrap())
        .timeout(std::time::Duration::from_secs(10))
        .assert()
        .success();

    // Should print at least some messages
    let stdout = String::from_utf8_lossy(&output.get_output().stdout);
    assert!(!stdout.is_empty(), "expected output from headless playback");
}

#[test]
fn play_no_tui_with_topic_filter() {
    let bags = test_bags_dir();
    let path = bags.join("rosbag2/talker");

    let output = cli()
        .arg("play")
        .arg(path.to_str().unwrap())
        .arg("--topics")
        .arg("/topic")
        .timeout(std::time::Duration::from_secs(10))
        .assert()
        .success();

    let stdout = String::from_utf8_lossy(&output.get_output().stdout);
    // Should only contain /topic messages, not /rosout
    assert!(stdout.contains("/topic"), "expected /topic in output");
    assert!(
        !stdout.contains("/rosout"),
        "/rosout should be filtered out"
    );
}
