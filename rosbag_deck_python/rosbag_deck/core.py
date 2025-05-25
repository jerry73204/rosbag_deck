"""
Core Python API for RosbagDeck
"""

from typing import List, Optional, Callable, Dict, Any
from dataclasses import dataclass

from .ffi import RosbagDeckFFI, create_cffi_callback, ffi
from .exceptions import RosbagDeckError, RosbagDeckInitError, RosbagDeckIndexError


@dataclass
class BagInfo:
    """Information about a rosbag"""

    success: bool
    message: str
    start_time: int  # nanoseconds since epoch
    end_time: int  # nanoseconds since epoch
    total_duration_ns: int
    total_frames: int
    topic_names: List[str]


@dataclass
class Status:
    """Current playback status"""

    current_time: int  # nanoseconds since epoch
    is_playing: bool
    current_frame: int
    total_frames: int
    timeline_segment: int
    virtual_time: int  # nanoseconds since epoch


@dataclass
class Message:
    """A message from the rosbag"""

    original_timestamp: int  # nanoseconds since epoch
    virtual_timestamp: int  # nanoseconds since epoch
    topic_name: str
    message_type: str
    serialized_data: bytes
    frame_index: int


class RosbagDeck:
    """
    Python API for RosbagDeck - tape deck style rosbag playback

    Provides streaming architecture for efficient playback of large rosbag files
    with features like seeking, frame stepping, and configurable caching.
    """

    def __init__(self, library_path: Optional[str] = None):
        """
        Initialize RosbagDeck

        Args:
            library_path: Optional path to librosbag_deck_core.so

        Raises:
            RosbagDeckInitError: If initialization fails
        """
        try:
            self._ffi = RosbagDeckFFI(library_path)
            self._handle = self._ffi.lib.rosbag_deck_create()
            if self._handle == ffi.NULL:
                raise RosbagDeckInitError("Failed to create rosbag deck handle")

            # Keep references to callbacks to prevent garbage collection
            self._status_callback_ref = None
            self._message_callback_ref = None
            self._status_callback_func: Optional[Callable[[Status], None]] = None
            self._message_callback_func: Optional[Callable[[Message], None]] = None

        except Exception as e:
            raise RosbagDeckInitError(f"Failed to initialize RosbagDeck: {e}")

    def __del__(self):
        """Cleanup resources"""
        self.close()

    def close(self):
        """Close and cleanup the rosbag deck"""
        if hasattr(self, "_handle") and self._handle != ffi.NULL:
            self._ffi.lib.rosbag_deck_destroy(self._handle)
            self._handle = ffi.NULL

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()

    # Configuration methods

    def set_cache_size(self, size: int) -> None:
        """
        Set the cache size for message buffering

        Args:
            size: Number of messages to cache
        """
        self._ffi.lib.rosbag_deck_set_cache_size(self._handle, size)

    def set_preload_settings(self, ahead: int, behind: int) -> None:
        """
        Set preload settings for cache management

        Args:
            ahead: Number of frames to preload ahead of current position
            behind: Number of frames to keep behind current position
        """
        self._ffi.lib.rosbag_deck_set_preload_settings(self._handle, ahead, behind)

    def set_playback_rate(self, rate: float) -> None:
        """
        Set playback rate multiplier

        Args:
            rate: Playback rate (1.0 = normal speed, 2.0 = 2x speed, etc.)
        """
        self._ffi.lib.rosbag_deck_set_playback_rate(self._handle, rate)

    def set_loop_playback(self, loop: bool) -> None:
        """
        Enable or disable loop playback

        Args:
            loop: Whether to loop playback when reaching the end
        """
        self._ffi.lib.rosbag_deck_set_loop_playback(self._handle, loop)

    # Core operations

    def build_index(self, bag_paths: List[str]) -> None:
        """
        Build index from rosbag files

        Args:
            bag_paths: List of paths to rosbag files

        Raises:
            RosbagDeckIndexError: If index building fails
        """
        if not bag_paths:
            raise RosbagDeckIndexError("No bag paths provided")

        # Convert Python strings to C char arrays using CFFI
        c_paths = ffi.new("char*[]", len(bag_paths))
        for i, path in enumerate(bag_paths):
            c_paths[i] = ffi.new("char[]", path.encode("utf-8"))

        success = self._ffi.lib.rosbag_deck_build_index(
            self._handle, c_paths, len(bag_paths)
        )
        if not success:
            raise RosbagDeckIndexError("Failed to build index from bag files")

    def start_playback(self) -> None:
        """Start continuous playback"""
        self._ffi.lib.rosbag_deck_start_playback(self._handle)

    def stop_playback(self) -> None:
        """Stop continuous playback"""
        self._ffi.lib.rosbag_deck_stop_playback(self._handle)

    def step_forward(self) -> bool:
        """
        Step forward one frame

        Returns:
            True if step was successful, False if at end or error
        """
        return self._ffi.lib.rosbag_deck_step_forward(self._handle)

    def step_backward(self) -> bool:
        """
        Step backward one frame

        Returns:
            True if step was successful, False if at beginning or error
        """
        return self._ffi.lib.rosbag_deck_step_backward(self._handle)

    def seek_to_time(self, nanoseconds: int) -> bool:
        """
        Seek to specific time

        Args:
            nanoseconds: Time in nanoseconds since epoch

        Returns:
            True if seek was successful, False if time not found or error
        """
        timestamp = self._ffi.lib.rosbag_deck_to_timestamp(nanoseconds)
        return self._ffi.lib.rosbag_deck_seek_to_time(self._handle, timestamp)

    def seek_to_frame(self, frame_index: int) -> bool:
        """
        Seek to specific frame

        Args:
            frame_index: Frame index to seek to

        Returns:
            True if seek was successful, False if frame not found or error
        """
        return self._ffi.lib.rosbag_deck_seek_to_frame(self._handle, frame_index)

    # Information queries

    def get_bag_info(self) -> BagInfo:
        """
        Get information about the loaded bags

        Returns:
            BagInfo object with bag metadata
        """
        info = self._ffi.lib.rosbag_deck_get_bag_info(self._handle)

        # Convert topic names to Python list
        topics = []
        if info.topic_names != ffi.NULL and info.topic_names_count > 0:
            for i in range(info.topic_names_count):
                if info.topic_names[i] != ffi.NULL:
                    topic_str = ffi.string(info.topic_names[i]).decode("utf-8")
                    topics.append(topic_str)

        message = ""
        if info.message != ffi.NULL:
            message = ffi.string(info.message).decode("utf-8")

        return BagInfo(
            success=info.success,
            message=message,
            start_time=info.start_time.nanoseconds_since_epoch,
            end_time=info.end_time.nanoseconds_since_epoch,
            total_duration_ns=info.total_duration_ns,
            total_frames=info.total_frames,
            topic_names=topics,
        )

    def get_status(self) -> Status:
        """
        Get current playback status

        Returns:
            Status object with current state
        """
        status = self._ffi.lib.rosbag_deck_get_status(self._handle)

        return Status(
            current_time=status.current_time.nanoseconds_since_epoch,
            is_playing=status.is_playing,
            current_frame=status.current_frame,
            total_frames=status.total_frames,
            timeline_segment=status.timeline_segment,
            virtual_time=status.virtual_time.nanoseconds_since_epoch,
        )

    # Callback management

    def set_status_callback(self, callback: Optional[Callable[[Status], None]]) -> None:
        """
        Set callback function for status updates

        Args:
            callback: Function to call with Status objects, or None to disable
        """
        self._status_callback_func = callback

        if callback is None:
            self._status_callback_ref = None
            self._ffi.lib.rosbag_deck_set_status_callback(
                self._handle, ffi.NULL, ffi.NULL
            )
        else:

            def wrapper(status_ptr, user_data):
                try:
                    status = status_ptr[0]  # CFFI dereference
                    status_obj = Status(
                        current_time=status.current_time.nanoseconds_since_epoch,
                        is_playing=status.is_playing,
                        current_frame=status.current_frame,
                        total_frames=status.total_frames,
                        timeline_segment=status.timeline_segment,
                        virtual_time=status.virtual_time.nanoseconds_since_epoch,
                    )
                    callback(status_obj)
                except Exception as e:
                    # Log error but don't let it propagate to C code
                    print(f"Error in status callback: {e}")

            self._status_callback_ref = create_cffi_callback(
                "rosbag_deck_status_callback_t", wrapper
            )
            self._ffi.lib.rosbag_deck_set_status_callback(
                self._handle, self._status_callback_ref, ffi.NULL
            )

    def set_message_callback(
        self, callback: Optional[Callable[[Message], None]]
    ) -> None:
        """
        Set callback function for message updates

        Args:
            callback: Function to call with Message objects, or None to disable
        """
        self._message_callback_func = callback

        if callback is None:
            self._message_callback_ref = None
            self._ffi.lib.rosbag_deck_set_message_callback(
                self._handle, ffi.NULL, ffi.NULL
            )
        else:

            def wrapper(message_ptr, user_data):
                try:
                    message = message_ptr[0]  # CFFI dereference

                    # Extract strings safely
                    topic_name = ""
                    if message.topic_name != ffi.NULL:
                        topic_name = ffi.string(message.topic_name).decode("utf-8")

                    message_type = ""
                    if message.message_type != ffi.NULL:
                        message_type = ffi.string(message.message_type).decode("utf-8")

                    # Extract serialized data
                    data = b""
                    if (
                        message.serialized_data != ffi.NULL
                        and message.serialized_data_size > 0
                    ):
                        data = ffi.buffer(
                            message.serialized_data, message.serialized_data_size
                        )[:]

                    message_obj = Message(
                        original_timestamp=message.original_timestamp.nanoseconds_since_epoch,
                        virtual_timestamp=message.virtual_timestamp.nanoseconds_since_epoch,
                        topic_name=topic_name,
                        message_type=message_type,
                        serialized_data=data,
                        frame_index=message.frame_index,
                    )
                    callback(message_obj)
                except Exception as e:
                    # Log error but don't let it propagate to C code
                    print(f"Error in message callback: {e}")

            self._message_callback_ref = create_cffi_callback(
                "rosbag_deck_message_callback_t", wrapper
            )
            self._ffi.lib.rosbag_deck_set_message_callback(
                self._handle, self._message_callback_ref, ffi.NULL
            )

    # Utility methods

    @property
    def is_valid(self) -> bool:
        """Check if the deck handle is valid"""
        return hasattr(self, "_handle") and self._handle != ffi.NULL

    def __repr__(self) -> str:
        """String representation"""
        if not self.is_valid:
            return "RosbagDeck(closed)"

        try:
            status = self.get_status()
            return f"RosbagDeck(frame={status.current_frame}/{status.total_frames}, playing={status.is_playing})"
        except:
            return "RosbagDeck(unknown_status)"
