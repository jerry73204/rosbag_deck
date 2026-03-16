use std::{
    collections::HashSet,
    io::{self, Write},
    path::PathBuf,
    time::Instant,
};

use regex::Regex;
use rosbag_deck_core::{BagReader, Deck, DeckConfig, LoopMode, QosPreset};
use rosbag_deck_ffi::Rosbag2Reader;

pub struct PlayOpts {
    pub bag_path: PathBuf,
    pub storage: String,
    pub rate: f64,
    pub topics: Option<Vec<String>>,
    pub regex: Option<String>,
    pub exclude: Option<String>,
    pub loop_mode: LoopMode,
    pub publish: bool,
    pub node_name: String,
    pub qos_depth: usize,
    pub qos_preset: QosPreset,
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

    // Set up ROS 2 topic publishing.
    if opts.publish {
        match rosbag_deck_ffi::RosPublisherBackend::new(&opts.node_name) {
            Ok(backend) => {
                let manager = rosbag_deck_core::PublisherManager::new(
                    Box::new(backend),
                    opts.qos_depth,
                    opts.qos_preset,
                );
                deck.enable_publishing(manager);
                eprintln!(
                    "Publishing enabled (node: {}, QoS: {})",
                    opts.node_name,
                    opts.qos_preset.label(),
                );
            }
            Err(e) => {
                eprintln!("Warning: failed to create ROS 2 node: {e}");
                eprintln!("Publishing disabled. Is ROS 2 running?");
            }
        }
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

    // Pre-create publishers for all (filtered) topics to avoid lazy-init
    // delays that cause stuttering on the first playback loop.
    if opts.publish {
        deck.precreate_publishers();
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

/// Headless playback — shows progress like `ros2 bag play`.
pub fn run_headless(opts: &PlayOpts) -> anyhow::Result<()> {
    // In headless mode, suppress noisy C++ INFO logs but keep WARN+ on stderr.
    rosbag_deck_ffi::set_ros_log_severity(rosbag_deck_ffi::RCUTILS_LOG_SEVERITY_WARN);

    let mut deck = open_deck(opts)?;
    let meta = deck.metadata().clone();
    let duration_ns = meta.end_time_ns - meta.start_time_ns;

    deck.play();

    let mut msg_count: u64 = 0;
    let mut last_print = Instant::now();
    let stderr = io::stderr();

    while let Some(_timed) = deck.next_message()? {
        msg_count += 1;

        // Update progress at most ~4 times per second.
        if last_print.elapsed().as_millis() >= 250 {
            let pos_ns = deck.cursor_ns() - meta.start_time_ns;
            let progress = if duration_ns > 0 {
                (pos_ns as f64 / duration_ns as f64 * 100.0).clamp(0.0, 100.0)
            } else {
                100.0
            };
            let pos_sec = pos_ns as f64 / 1e9;
            let dur_sec = duration_ns as f64 / 1e9;

            let mut handle = stderr.lock();
            let _ = write!(
                handle,
                "\r{:.1}% [{:.1}s / {:.1}s] {msg_count} messages published",
                progress, pos_sec, dur_sec,
            );
            let _ = handle.flush();
            last_print = Instant::now();
        }
    }

    // Final line.
    eprintln!("\r{:<60}", format!("Done. {msg_count} messages played."));

    Ok(())
}
