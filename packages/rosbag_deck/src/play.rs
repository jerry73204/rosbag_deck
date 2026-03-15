use std::{collections::HashSet, path::PathBuf};

use regex::Regex;
use rosbag_deck_core::{BagReader, Deck, DeckConfig, LoopMode, PlaybackMode};
use rosbag_deck_ffi::Rosbag2Reader;

pub struct PlayOpts {
    pub bag_path: PathBuf,
    pub storage: String,
    pub rate: f64,
    pub topics: Option<Vec<String>>,
    pub regex: Option<String>,
    pub exclude: Option<String>,
    pub loop_mode: LoopMode,
}

/// Open the bag and create a Deck from PlayOpts.
pub fn open_deck(opts: &PlayOpts) -> anyhow::Result<Deck> {
    let reader = Rosbag2Reader::open(&opts.bag_path, &opts.storage)?;

    let meta = reader.metadata().clone();
    let secs = meta.duration.as_secs();
    eprintln!(
        "Opened: {} ({}, {:02}:{:02}:{:02}, {} messages)",
        opts.bag_path.display(),
        meta.storage_identifier,
        secs / 3600,
        (secs % 3600) / 60,
        secs % 60,
        meta.message_count,
    );

    let readers: Vec<Box<dyn BagReader>> = vec![Box::new(reader)];
    let mut deck = Deck::open(readers, DeckConfig::default())?;

    deck.set_speed(opts.rate);
    deck.set_loop_mode(opts.loop_mode);

    // Populate header cache for monotonic loop mode.
    if opts.loop_mode == LoopMode::Monotonic {
        deck.populate_header_cache(|t| rosbag_deck_ffi::type_has_header_first(t).unwrap_or(false));
    }

    let filter = resolve_topic_filter(
        &deck.topic_names(),
        opts.topics.as_deref(),
        opts.regex.as_deref(),
        opts.exclude.as_deref(),
    )?;
    if let Some(filter) = filter {
        deck.set_topic_filter(Some(filter));
    }

    Ok(deck)
}

/// Resolve `--topics`, `--regex`, `--exclude` into a concrete set of topic names.
///
/// Returns `None` if no filter is specified (all topics accepted).
fn resolve_topic_filter(
    all_topics: &[String],
    topics: Option<&[String]>,
    regex_pattern: Option<&str>,
    exclude_pattern: Option<&str>,
) -> anyhow::Result<Option<HashSet<String>>> {
    // Start with the explicit whitelist, or all topics if none specified.
    let mut accepted: HashSet<String> = if let Some(topics) = topics {
        topics.iter().cloned().collect()
    } else if regex_pattern.is_some() || exclude_pattern.is_some() {
        // If regex/exclude given but no explicit whitelist, start with all topics.
        all_topics.iter().cloned().collect()
    } else {
        return Ok(None); // No filter at all.
    };

    // Add topics matching --regex.
    if let Some(pattern) = regex_pattern {
        let re =
            Regex::new(pattern).map_err(|e| anyhow::anyhow!("invalid --regex pattern: {e}"))?;
        for topic in all_topics {
            if re.is_match(topic) {
                accepted.insert(topic.clone());
            }
        }
    }

    // Remove topics matching --exclude.
    if let Some(pattern) = exclude_pattern {
        let re =
            Regex::new(pattern).map_err(|e| anyhow::anyhow!("invalid --exclude pattern: {e}"))?;
        accepted.retain(|topic| !re.is_match(topic));
    }

    Ok(Some(accepted))
}

/// Headless playback — prints messages to stdout.
pub fn run_headless(opts: &PlayOpts) -> anyhow::Result<()> {
    let mut deck = open_deck(opts)?;

    deck.set_mode(PlaybackMode::BestEffort);
    deck.play();

    while let Some(timed) = deck.next_message()? {
        let ts_sec = timed.message.timestamp_ns as f64 / 1e9;
        println!(
            "{:.6}  {:<40}  [{} bytes]",
            ts_sec,
            timed.message.topic,
            timed.message.data.len(),
        );
    }

    Ok(())
}
