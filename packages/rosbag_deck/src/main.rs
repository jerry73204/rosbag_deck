mod edit;
mod info;
mod play;
#[cfg(feature = "tui")]
mod tui;

use std::path::PathBuf;

use clap::{Parser, Subcommand};

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
        /// Disable TUI, run headless playback.
        #[arg(long)]
        no_tui: bool,
        /// Loop playback.
        #[arg(long, short)]
        r#loop: bool,
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
    tracing_subscriber::fmt::init();
    let cli = Cli::parse();

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
            no_tui,
            r#loop,
        } => {
            let opts = play::PlayOpts {
                bag_path,
                storage,
                rate,
                topics,
                regex,
                exclude,
                looping: r#loop,
            };

            if no_tui {
                play::run_headless(&opts)
            } else {
                #[cfg(feature = "tui")]
                {
                    tui::run(&opts)
                }
                #[cfg(not(feature = "tui"))]
                {
                    anyhow::bail!(
                        "TUI support not compiled. Use --no-tui or rebuild with the 'tui' feature."
                    );
                }
            }
        }
    }
}
