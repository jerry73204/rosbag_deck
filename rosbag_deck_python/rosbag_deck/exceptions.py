"""
RosbagDeck Python API Exceptions
"""


class RosbagDeckError(Exception):
    """Base exception for RosbagDeck errors"""

    pass


class RosbagDeckInitError(RosbagDeckError):
    """Raised when RosbagDeck initialization fails"""

    pass


class RosbagDeckIndexError(RosbagDeckError):
    """Raised when bag index building fails"""

    pass
