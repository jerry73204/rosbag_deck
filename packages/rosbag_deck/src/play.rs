use std::{collections::HashSet, path::PathBuf};

use rosbag_deck_core::{BagReader, Deck, DeckConfig, PlaybackMode};
use rosbag_deck_ffi::Rosbag2Reader;

pub struct PlayOpts {
    pub bag_path: PathBuf,
    pub storage: String,
    pub rate: f64,
    pub topics: Option<Vec<String>>,
    pub looping: bool,
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
    deck.set_looping(opts.looping);

    if let Some(ref topics) = opts.topics {
        deck.set_topic_filter(Some(topics.iter().cloned().collect::<HashSet<_>>()));
    }

    Ok(deck)
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
