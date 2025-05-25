"""
Core TUI player logic using Textual framework
"""

import asyncio
import time
from typing import Optional, List
from pathlib import Path

from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical, Container
from textual.widgets import (
    Header,
    Footer,
    Static,
    Button,
    Label,
    ProgressBar,
    Input,
    ListView,
    ListItem,
    DataTable,
    Log,
)
from textual.reactive import reactive
from textual.message import Message
from rich.text import Text
from rich.panel import Panel
from rich.console import Console

from rosbag_deck import RosbagDeck, Status, Message as RosbagMessage, BagInfo


class PlayerControls(Static):
    """Tape deck style player controls"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.deck: Optional[RosbagDeck] = None
        self.is_playing = False

    def compose(self) -> ComposeResult:
        """Create the control interface"""
        with Horizontal(classes="controls"):
            yield Button("⏮", id="rewind", tooltip="Seek to beginning")
            yield Button("⏪", id="step_back", tooltip="Step backward")
            yield Button("⏯", id="play_pause", tooltip="Play/Pause")
            yield Button("⏩", id="step_forward", tooltip="Step forward")
            yield Button("⏭", id="fast_forward", tooltip="Seek to end")
            yield Button("⏹", id="stop", tooltip="Stop")
            yield Button("🔁", id="loop", tooltip="Toggle loop")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """Handle control button presses"""
        if not self.deck:
            return

        button_id = event.button.id

        if button_id == "play_pause":
            if self.is_playing:
                self.deck.stop_playback()
                self.is_playing = False
                event.button.label = "▶"
            else:
                self.deck.start_playback()
                self.is_playing = True
                event.button.label = "⏸"

        elif button_id == "stop":
            self.deck.stop_playback()
            self.is_playing = False
            self.query_one("#play_pause", Button).label = "▶"

        elif button_id == "step_forward":
            self.deck.step_forward()

        elif button_id == "step_back":
            self.deck.step_backward()

        elif button_id == "rewind":
            self.deck.seek_to_frame(0)

        elif button_id == "fast_forward":
            if hasattr(self, "_total_frames"):
                self.deck.seek_to_frame(self._total_frames - 1)

        elif button_id == "loop":
            # Toggle loop mode
            pass  # TODO: implement loop toggle

    def set_deck(self, deck: RosbagDeck, bag_info: BagInfo):
        """Set the rosbag deck instance"""
        self.deck = deck
        self._total_frames = bag_info.total_frames


class StatusDisplay(Static):
    """Display current playback status"""

    current_frame = reactive(0)
    total_frames = reactive(0)
    current_time = reactive(0)
    is_playing = reactive(False)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def compose(self) -> ComposeResult:
        """Create status display"""
        with Vertical():
            yield Label("Status: Stopped", id="status_label")
            yield ProgressBar(total=100, show_eta=False, id="progress")
            yield Label("Frame: 0 / 0", id="frame_label")
            yield Label("Time: 00:00.000 / 00:00.000", id="time_label")

    def watch_current_frame(self, frame: int) -> None:
        """Update frame display"""
        self.query_one("#frame_label", Label).update(
            f"Frame: {frame:,} / {self.total_frames:,}"
        )

        if self.total_frames > 0:
            progress = (frame / self.total_frames) * 100
            self.query_one("#progress", ProgressBar).update(progress=progress)

    def watch_is_playing(self, playing: bool) -> None:
        """Update playing status"""
        status = (
            "Playing" if playing else "Paused" if self.current_frame > 0 else "Stopped"
        )
        self.query_one("#status_label", Label).update(f"Status: {status}")

    def watch_current_time(self, time_ns: int) -> None:
        """Update time display"""
        current_sec = time_ns / 1e9
        total_sec = getattr(self, "_total_time_ns", 0) / 1e9

        current_str = f"{int(current_sec//60):02d}:{current_sec%60:06.3f}"
        total_str = f"{int(total_sec//60):02d}:{total_sec%60:06.3f}"

        self.query_one("#time_label", Label).update(
            f"Time: {current_str} / {total_str}"
        )

    def set_total_frames(self, total: int):
        """Set total frame count"""
        self.total_frames = total

    def set_total_time(self, total_time_ns: int):
        """Set total duration"""
        self._total_time_ns = total_time_ns
        self.watch_current_time(self.current_time)  # Refresh display


class BagInfoPanel(Static):
    """Display bag information"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def compose(self) -> ComposeResult:
        """Create bag info display"""
        yield Label("No bag loaded", id="bag_info")

    def update_info(self, bag_info: BagInfo):
        """Update with bag information"""
        if not bag_info.success:
            self.query_one("#bag_info", Label).update(f"Error: {bag_info.message}")
            return

        duration_sec = bag_info.total_duration_ns / 1e9
        duration_str = f"{int(duration_sec//60):02d}:{duration_sec%60:06.3f}"

        info_text = (
            f"Total Frames: {bag_info.total_frames:,}\n"
            f"Duration: {duration_str}\n"
            f"Topics: {len(bag_info.topic_names)}\n"
            + "\n".join(f"  • {topic}" for topic in bag_info.topic_names[:10])
        )

        if len(bag_info.topic_names) > 10:
            info_text += f"\n  ... and {len(bag_info.topic_names) - 10} more"

        self.query_one("#bag_info", Label).update(info_text)


class MessageLog(Log):
    """Display incoming messages"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.max_lines = 1000

    def log_message(self, msg: RosbagMessage):
        """Log a rosbag message"""
        timestamp = msg.virtual_timestamp / 1e9
        time_str = f"{timestamp:.3f}"

        log_line = f"[{time_str}] {msg.topic_name} ({msg.message_type}) - {len(msg.serialized_data)} bytes"
        self.write_line(log_line)


class SeekDialog(Container):
    """Dialog for seeking to specific time/frame"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.can_focus = True

    def compose(self) -> ComposeResult:
        """Create seek dialog"""
        with Vertical():
            yield Label("Seek to:")
            yield Input(placeholder="Frame number or time (MM:SS.sss)", id="seek_input")
            with Horizontal():
                yield Button("Seek", id="seek_button", variant="primary")
                yield Button("Cancel", id="cancel_button")


class RosbagTUI(App):
    """Main TUI application"""

    CSS = """
    .controls {
        height: 3;
        padding: 1;
    }
    
    #seek_dialog {
        background: $surface;
        border: solid $primary;
        width: 50;
        height: 8;
    }
    
    #message_log {
        height: 1fr;
        border: solid $primary;
    }
    
    #status_panel {
        height: 8;
        border: solid $accent;
    }
    
    #bag_info_panel {
        height: 1fr;
        border: solid $secondary;
    }
    """

    BINDINGS = [
        ("space", "play_pause", "Play/Pause"),
        ("s", "stop", "Stop"),
        ("left", "step_back", "Step back"),
        ("right", "step_forward", "Step forward"),
        ("g", "seek", "Seek"),
        ("q", "quit", "Quit"),
        ("o", "open", "Open bag"),
    ]

    def __init__(self, bag_files: List[str] = None, **kwargs):
        super().__init__(**kwargs)
        self.bag_files = bag_files or []
        self.deck: Optional[RosbagDeck] = None
        self.update_task: Optional[asyncio.Task] = None

    def compose(self) -> ComposeResult:
        """Compose the UI"""
        yield Header()

        with Horizontal():
            # Left panel - controls and status
            with Vertical(id="left_panel"):
                yield PlayerControls(id="controls")
                yield StatusDisplay(id="status_panel")
                yield BagInfoPanel(id="bag_info_panel")

            # Right panel - message log
            yield MessageLog(id="message_log")

        yield Footer()

    def on_mount(self) -> None:
        """Initialize when app starts"""
        if self.bag_files:
            self.load_bags(self.bag_files)

    def load_bags(self, bag_files: List[str]):
        """Load rosbag files"""
        try:
            # Validate files exist
            for bag_file in bag_files:
                if not Path(bag_file).exists():
                    self.notify(f"File not found: {bag_file}", severity="error")
                    return

            # Create deck instance
            self.deck = RosbagDeck()

            # Set callbacks
            self.deck.set_status_callback(self._on_status_update)
            self.deck.set_message_callback(self._on_message_update)

            # Configure deck
            self.deck.set_cache_size(1000)
            self.deck.set_preload_settings(ahead=50, behind=50)
            self.deck.set_playback_rate(1.0)

            # Build index
            self.notify("Building index...")
            self.deck.build_index(bag_files)

            # Get bag info
            bag_info = self.deck.get_bag_info()
            if not bag_info.success:
                self.notify(
                    f"Failed to load bags: {bag_info.message}", severity="error"
                )
                return

            # Update UI components
            self.query_one("#controls", PlayerControls).set_deck(self.deck, bag_info)
            self.query_one("#status_panel", StatusDisplay).set_total_frames(
                bag_info.total_frames
            )
            self.query_one("#status_panel", StatusDisplay).set_total_time(
                bag_info.total_duration_ns
            )
            self.query_one("#bag_info_panel", BagInfoPanel).update_info(bag_info)

            self.notify(f"Loaded {len(bag_files)} bag file(s)")

            # Start status update task
            self.update_task = asyncio.create_task(self._status_update_loop())

        except Exception as e:
            self.notify(f"Error loading bags: {e}", severity="error")

    def _on_status_update(self, status: Status):
        """Handle status updates from deck"""
        try:
            status_display = self.query_one("#status_panel", StatusDisplay)
            status_display.current_frame = status.current_frame
            status_display.current_time = status.current_time
            status_display.is_playing = status.is_playing
        except:
            pass  # Ignore errors during UI updates

    def _on_message_update(self, message: RosbagMessage):
        """Handle message updates from deck"""
        try:
            message_log = self.query_one("#message_log", MessageLog)
            message_log.log_message(message)
        except:
            pass  # Ignore errors during UI updates

    async def _status_update_loop(self):
        """Periodic status updates"""
        while True:
            if self.deck:
                try:
                    status = self.deck.get_status()
                    self._on_status_update(status)
                except:
                    pass
            await asyncio.sleep(0.1)  # 10 Hz updates

    def action_play_pause(self) -> None:
        """Toggle play/pause"""
        controls = self.query_one("#controls", PlayerControls)
        play_button = controls.query_one("#play_pause", Button)
        play_button.press()

    def action_stop(self) -> None:
        """Stop playback"""
        controls = self.query_one("#controls", PlayerControls)
        stop_button = controls.query_one("#stop", Button)
        stop_button.press()

    def action_step_forward(self) -> None:
        """Step forward"""
        controls = self.query_one("#controls", PlayerControls)
        step_button = controls.query_one("#step_forward", Button)
        step_button.press()

    def action_step_back(self) -> None:
        """Step backward"""
        controls = self.query_one("#controls", PlayerControls)
        step_button = controls.query_one("#step_back", Button)
        step_button.press()

    def action_seek(self) -> None:
        """Show seek dialog"""
        # TODO: Implement seek dialog
        self.notify("Seek dialog not implemented yet")

    def action_open(self) -> None:
        """Open file dialog"""
        # TODO: Implement file dialog
        self.notify("File dialog not implemented yet")

    def on_unmount(self) -> None:
        """Cleanup when app closes"""
        if self.update_task:
            self.update_task.cancel()
        if self.deck:
            self.deck.close()
