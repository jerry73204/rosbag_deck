// Integration test entry point
//
// Tests are organized as separate modules below.
// Run with: cargo nextest run
//
// Prerequisites: download test bags first:
//   ./scripts/download-test-bags.sh small

#[macro_use]
#[path = "integration/helpers.rs"]
mod helpers;

#[path = "integration/bag_open.rs"]
mod bag_open;

#[path = "integration/bag_playback.rs"]
mod bag_playback;

#[path = "integration/bag_seek.rs"]
mod bag_seek;

#[path = "integration/cli.rs"]
mod cli;

#[path = "integration/edge_cases.rs"]
mod edge_cases;

#[path = "integration/bag_edit.rs"]
mod bag_edit;

#[path = "integration/bag_large.rs"]
mod bag_large;
