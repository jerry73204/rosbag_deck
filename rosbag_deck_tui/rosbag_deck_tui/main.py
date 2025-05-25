#!/usr/bin/env python3
"""
Main entry point for rosbag-deck-tui
"""

import sys
import argparse
from pathlib import Path
from typing import List

from .player import RosbagTUI


def parse_args() -> argparse.Namespace:
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        prog="rosbag-tui",
        description="Terminal-based rosbag player with tape deck controls",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  rosbag-tui data.bag                    # Play single bag file
  rosbag-tui bag1.bag bag2.bag          # Play multiple bag files
  rosbag-deck-tui /path/to/data.bag     # Alternative command name

Keyboard Controls:
  Space     - Play/Pause
  S         - Stop
  ←/→       - Step backward/forward
  G         - Seek (goto)
  O         - Open bag file
  Q         - Quit

The interface shows:
  - Tape deck controls (⏮ ⏪ ⏯ ⏩ ⏭ ⏹ 🔁)
  - Playback status and progress
  - Bag information (duration, topics, etc.)
  - Real-time message log
        """,
    )

    parser.add_argument("bag_files", nargs="*", help="Rosbag files to play")

    parser.add_argument(
        "--cache-size",
        type=int,
        default=1000,
        help="Cache size for message buffering (default: 1000)",
    )

    parser.add_argument(
        "--preload-ahead",
        type=int,
        default=50,
        help="Number of frames to preload ahead (default: 50)",
    )

    parser.add_argument(
        "--preload-behind",
        type=int,
        default=50,
        help="Number of frames to keep behind (default: 50)",
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="Playback rate multiplier (default: 1.0)",
    )

    parser.add_argument("--version", action="version", version="rosbag-deck-tui 0.1.0")

    return parser.parse_args()


def validate_bag_files(bag_files: List[str]) -> List[str]:
    """Validate that bag files exist"""
    valid_files = []
    errors = []

    for bag_file in bag_files:
        path = Path(bag_file)
        if not path.exists():
            errors.append(f"File not found: {bag_file}")
        elif not path.is_file():
            errors.append(f"Not a file: {bag_file}")
        else:
            valid_files.append(str(path.absolute()))

    if errors:
        print("Error: Invalid bag files:", file=sys.stderr)
        for error in errors:
            print(f"  {error}", file=sys.stderr)
        sys.exit(1)

    return valid_files


def main():
    """Main entry point"""
    args = parse_args()

    # Validate bag files if provided
    bag_files = []
    if args.bag_files:
        bag_files = validate_bag_files(args.bag_files)
        print(f"Loading {len(bag_files)} bag file(s)...")
    else:
        print(
            "No bag files specified. You can open files from within the TUI (press 'O')."
        )

    # Create and run the TUI app
    try:
        app = RosbagTUI(bag_files=bag_files)

        # Configure deck settings if bags are provided
        if bag_files and hasattr(app, "deck") and app.deck:
            app.deck.set_cache_size(args.cache_size)
            app.deck.set_preload_settings(args.preload_ahead, args.preload_behind)
            app.deck.set_playback_rate(args.rate)

        app.run()

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
