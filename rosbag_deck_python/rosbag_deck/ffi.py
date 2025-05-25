"""
Foreign Function Interface (FFI) bindings for RosbagDeck C API

This module automatically loads CFFI-generated bindings from the C header file.
"""

import os
from typing import Optional

# Try to import CFFI-generated bindings
try:
    from ._cffi_bindings import lib, ffi

    _CFFI_AVAILABLE = True
except ImportError:
    _CFFI_AVAILABLE = False
    lib = None
    ffi = None


class RosbagDeckFFI:
    """FFI wrapper for RosbagDeck C API using CFFI"""

    def __init__(self, library_path: Optional[str] = None):
        """Initialize FFI with library loading"""
        if not _CFFI_AVAILABLE:
            raise ImportError(
                "CFFI bindings not available. Please build the extension first by running:\n"
                "python ffi_builder.py"
            )

        # If library_path is specified, try to load it dynamically
        if library_path is not None:
            try:
                self.lib = ffi.dlopen(library_path)
            except Exception as e:
                raise RuntimeError(f"Failed to load library from {library_path}: {e}")
        else:
            # Use the pre-compiled library
            self.lib = lib

        # Store ffi for creating objects
        self.ffi = ffi

    def _find_library(self) -> str:
        """Find the rosbag_deck_core library (fallback method)"""
        # Common library search paths
        search_paths = [
            # Installed library paths
            "/usr/local/lib/librosbag_deck_core.so",
            "/usr/lib/librosbag_deck_core.so",
            # Build directory (relative to Python package)
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "..",
                "install",
                "rosbag_deck_core",
                "lib",
                "librosbag_deck_core.so",
            ),
            # Current directory
            "./librosbag_deck_core.so",
            "librosbag_deck_core.so",
        ]

        for path in search_paths:
            if os.path.exists(path):
                return path

        # Last resort - let system find it
        return "librosbag_deck_core.so"


# Provide backwards compatibility with ctypes-like interface
def create_cffi_callback(callback_type_name, callback_func):
    """Create a CFFI callback from a Python function"""
    if not _CFFI_AVAILABLE:
        raise ImportError(
            "CFFI bindings not available. Please build the extension first by running:\n"
            "python ffi_builder.py"
        )

    return ffi.callback(callback_type_name, callback_func)


# Export the types for convenience
if _CFFI_AVAILABLE:
    # These will be available as ffi.new("rosbag_deck_timestamp_t*")
    # and accessed via the lib object
    pass
else:
    # Fallback if CFFI is not available
    class _DummyFFI:
        @property
        def NULL(self):
            raise ImportError(
                "CFFI bindings not available. Please build the extension first by running:\n"
                "python ffi_builder.py"
            )

        def __getattr__(self, name):
            raise ImportError(
                "CFFI bindings not available. Please build the extension first by running:\n"
                "python ffi_builder.py"
            )

    lib = _DummyFFI()
    ffi = _DummyFFI()
