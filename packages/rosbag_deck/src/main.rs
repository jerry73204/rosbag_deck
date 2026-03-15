mod edit;
mod info;
mod play;
#[cfg(feature = "tui")]
mod tui;

use std::path::PathBuf;

use clap::{Parser, Subcommand};
use tracing_subscriber::EnvFilter;

#[derive(Parser)]
#[command(name = "rosbag-deck", about = "Interactive ROS 2 bag player")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Show bag metadata (topics, message counts, duration).
    Info {
        /// Path to the bag directory or file.
        bag_path: PathBuf,
        /// Storage backend (auto-detect if omitted).
        #[arg(long, default_value = "")]
        storage: String,
    },
    /// Play a bag file.
    Play {
        /// Path to the bag directory or file.
        bag_path: PathBuf,
        /// Storage backend (auto-detect if omitted).
        #[arg(long, default_value = "")]
        storage: String,
        /// Playback speed multiplier.
        #[arg(long, short, default_value_t = 1.0)]
        rate: f64,
        /// Topic whitelist (space or comma-separated).
        #[arg(long, short, value_delimiter = ',', num_args = 1..)]
        topics: Option<Vec<String>>,
        /// Include topics matching a regex pattern.
        #[arg(long, value_name = "PATTERN")]
        regex: Option<String>,
        /// Exclude topics matching a name or regex pattern.
        #[arg(long, short, value_name = "PATTERN")]
        exclude: Option<String>,
        /// Enable interactive TUI mode.
        #[arg(long)]
        tui: bool,
        /// Loop mode: off, restart, or monotonic.
        ///
        /// "restart" loops with original timestamps. "monotonic" shifts
        /// Header.stamp forward each iteration so timestamps never decrease.
        /// Use `--loop` alone for "restart" (backward compatible).
        #[arg(long, short, default_value = "off", default_missing_value = "restart", num_args = 0..=1)]
        r#loop: String,
        /// Disable ROS 2 topic publishing (publishing is on by default).
        #[arg(long)]
        no_publish: bool,
        /// ROS 2 node name for publishing.
        #[arg(long, default_value = "rosbag_deck")]
        node_name: String,
        /// Publisher queue depth (QoS KEEP_LAST).
        #[arg(long, default_value_t = 10)]
        qos_depth: usize,
        /// QoS preset: auto, sensor, or reliable.
        #[arg(long, default_value = "auto")]
        qos_profile: String,
    },
    /// Edit bag files: slice, merge, filter, and transform timestamps.
    Edit {
        /// One or more input bag paths.
        #[arg(required = true)]
        input: Vec<PathBuf>,
        /// Output bag path (required unless --dry-run).
        #[arg(long, short)]
        output: Option<PathBuf>,
        /// Output storage format.
        #[arg(long, default_value = "sqlite3")]
        format: String,
        /// Input storage backend (auto-detect if omitted).
        #[arg(long, default_value = "")]
        storage: String,
        /// Topic whitelist (space or comma-separated).
        #[arg(long, short, value_delimiter = ',', num_args = 1..)]
        topics: Option<Vec<String>>,
        /// Include topics matching a regex pattern.
        #[arg(long, value_name = "PATTERN")]
        regex: Option<String>,
        /// Exclude topics matching a name or regex pattern.
        #[arg(long, short, value_name = "PATTERN")]
        exclude: Option<String>,
        /// Start time (seconds from bag start).
        #[arg(long)]
        start: Option<f64>,
        /// End time (seconds from bag start).
        #[arg(long)]
        end: Option<f64>,
        /// Maximum duration from start (seconds).
        #[arg(long)]
        duration: Option<f64>,
        /// Shift all timestamps by ±N seconds.
        #[arg(long)]
        time_offset: Option<f64>,
        /// Scale timestamps relative to first message.
        #[arg(long, default_value_t = 1.0)]
        time_scale: f64,
        /// Run pipeline without writing, print summary.
        #[arg(long)]
        dry_run: bool,
        /// Print per-message progress.
        #[arg(long, short)]
        verbose: bool,
    },
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    // Initialize tracing for non-TUI modes. TUI mode manages its own output.
    let is_tui = matches!(&cli.command, Command::Play { tui: true, .. });
    if !is_tui {
        tracing_subscriber::fmt()
            .with_env_filter(
                EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
            )
            .init();
    }

    match cli.command {
        Command::Info { bag_path, storage } => info::run(&bag_path, &storage),
        Command::Edit {
            input,
            output,
            format,
            storage,
            topics,
            regex,
            exclude,
            start,
            end,
            duration,
            time_offset,
            time_scale,
            dry_run,
            verbose,
        } => {
            let opts = edit::EditOpts {
                input,
                output,
                format,
                storage,
                topics,
                regex,
                exclude,
                start,
                end,
                duration,
                time_offset,
                time_scale,
                dry_run,
                verbose,
            };
            edit::run(&opts)
        }
        Command::Play {
            bag_path,
            storage,
            rate,
            topics,
            regex,
            exclude,
            tui,
            r#loop,
            no_publish,
            node_name,
            qos_depth,
            qos_profile,
        } => {
            let loop_mode = match r#loop.as_str() {
                "off" => rosbag_deck_core::LoopMode::Off,
                "restart" => rosbag_deck_core::LoopMode::Restart,
                "monotonic" => rosbag_deck_core::LoopMode::Monotonic,
                other => anyhow::bail!(
                    "invalid --loop value '{}': expected off, restart, or monotonic",
                    other
                ),
            };
            let qos_preset = match qos_profile.as_str() {
                "auto" => rosbag_deck_core::QosPreset::Auto,
                "sensor" => rosbag_deck_core::QosPreset::Sensor,
                "reliable" => rosbag_deck_core::QosPreset::Reliable,
                other => anyhow::bail!(
                    "invalid --qos-profile '{}': expected auto, sensor, or reliable",
                    other
                ),
            };
            let opts = play::PlayOpts {
                bag_path,
                storage,
                rate,
                topics,
                regex,
                exclude,
                loop_mode,
                publish: !no_publish,
                node_name,
                qos_depth,
                qos_preset,
            };

            if tui {
                #[cfg(feature = "tui")]
                {
                    tui::run(&opts)
                }
                #[cfg(not(feature = "tui"))]
                {
                    anyhow::bail!("TUI support not compiled. Rebuild with the 'tui' feature.");
                }
            } else {
                play::run_headless(&opts)
            }
        }
    }
}
