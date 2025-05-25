"""
RosbagDeck Python API

A Python wrapper for the RosbagDeck C++ library providing tape deck style
rosbag playback with streaming architecture for large bag files.
"""

from .core import RosbagDeck, BagInfo, Status, Message
from .exceptions import RosbagDeckError, RosbagDeckInitError, RosbagDeckIndexError

__version__ = "0.1.0"
__all__ = [
    "RosbagDeck",
    "BagInfo",
    "Status",
    "Message",
    "RosbagDeckError",
    "RosbagDeckInitError",
    "RosbagDeckIndexError",
]
