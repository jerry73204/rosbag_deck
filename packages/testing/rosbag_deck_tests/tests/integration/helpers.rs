//! Shared test helpers and fixtures.

use std::path::{Path, PathBuf};

use rosbag_deck_core::{reader::BagReader, Deck, DeckConfig, PlaybackMode};

/// Try to find the test_bags/ directory. Returns `None` if not present.
pub fn try_test_bags_dir() -> Option<PathBuf> {
    if let Ok(dir) = std::env::var("ROSBAG_DECK_TEST_BAGS") {
        let p = PathBuf::from(dir);
        if p.is_dir() {
            return Some(p);
        }
    }

    let mut dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    for _ in 0..5 {
        let candidate = dir.join("test_bags");
        if candidate.is_dir() {
            return Some(candidate);
        }
        if !dir.pop() {
            break;
        }
    }

    None
}

/// Return the test_bags/ directory, panicking if not present.
pub fn test_bags_dir() -> PathBuf {
    try_test_bags_dir().unwrap_or_else(|| {
        panic!(
            "test_bags/ directory not found. Download test bags first:\n\
             \n\
             \t./scripts/download-test-bags.sh small\n"
        )
    })
}

/// Returns true if large bag tests are enabled via `ROSBAG_DECK_LARGE_TESTS=1`.
pub fn large_tests_enabled() -> bool {
    std::env::var("ROSBAG_DECK_LARGE_TESTS")
        .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
        .unwrap_or(false)
}

/// Skip the current test if test bags are not downloaded.
macro_rules! skip_if_no_bags {
    () => {
        if crate::helpers::try_test_bags_dir().is_none() {
            eprintln!("SKIPPED: test_bags/ not found. Run: ./scripts/download-test-bags.sh small");
            return;
        }
    };
}

/// Skip the current test if large bag tests are not enabled.
macro_rules! skip_if_no_large_bags {
    () => {
        skip_if_no_bags!();
        if !crate::helpers::large_tests_enabled() {
            eprintln!("SKIPPED: Set ROSBAG_DECK_LARGE_TESTS=1 to enable large bag tests");
            return;
        }
    };
}

/// Open a rosbag2 bag via FFI, returning a boxed BagReader.
pub fn open_bag(path: &Path) -> Box<dyn BagReader> {
    Box::new(
        rosbag_deck_ffi::Rosbag2Reader::open(path, "")
            .unwrap_or_else(|e| panic!("failed to open bag at {}: {e}", path.display())),
    )
}

/// Open a Deck from a single bag path in best-effort mode.
pub fn open_deck(path: &Path) -> Deck {
    let reader = open_bag(path);
    let mut deck = Deck::open(vec![reader], DeckConfig::default()).unwrap();
    deck.set_mode(PlaybackMode::BestEffort);
    deck
}

/// Collect all messages from a playing deck.
pub fn drain_messages(deck: &mut Deck) -> Vec<rosbag_deck_core::TimedMessage> {
    deck.play();
    let mut msgs = Vec::new();
    while let Ok(Some(m)) = deck.next_message() {
        msgs.push(m);
    }
    msgs
}

/// Require that a specific bag sub-path exists, panicking with a helpful message.
macro_rules! bag_path {
    ($bags:expr, $rel:expr) => {{
        let p = $bags.join($rel);
        assert!(
            p.is_dir(),
            "test bag not found: {}\nDownload test bags: ./scripts/download-test-bags.sh small",
            p.display()
        );
        p
    }};
}
