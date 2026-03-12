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
        /// Topic filter (comma-separated). If omitted, all topics are played.
        #[arg(long, short, value_delimiter = ',')]
        topics: Option<Vec<String>>,
        /// Disable TUI, run headless playback.
        #[arg(long)]
        no_tui: bool,
        /// Loop playback.
        #[arg(long, short)]
        r#loop: bool,
    },
}

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt::init();
    let cli = Cli::parse();

    match cli.command {
        Command::Info { bag_path, storage } => info::run(&bag_path, &storage),
        Command::Play {
            bag_path,
            storage,
            rate,
            topics,
            no_tui,
            r#loop,
        } => {
            let opts = play::PlayOpts {
                bag_path,
                storage,
                rate,
                topics,
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
