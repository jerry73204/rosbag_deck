"""
Shared pytest fixtures and configuration for rosbag_deck_python tests
"""

import os
import tempfile
import shutil
from pathlib import Path
from typing import Generator, List, Optional
from unittest.mock import Mock, MagicMock

import pytest


# Test environment setup
@pytest.fixture(scope="session", autouse=True)
def test_environment():
    """Set up test environment variables and configuration"""
    # Ensure we're in test mode
    os.environ["ROSBAG_DECK_TEST_MODE"] = "1"
    yield
    # Cleanup
    if "ROSBAG_DECK_TEST_MODE" in os.environ:
        del os.environ["ROSBAG_DECK_TEST_MODE"]


@pytest.fixture
def temp_dir() -> Generator[Path, None, None]:
    """Provide a temporary directory for test files"""
    temp_path = tempfile.mkdtemp(prefix="rosbag_deck_test_")
    try:
        yield Path(temp_path)
    finally:
        shutil.rmtree(temp_path, ignore_errors=True)


@pytest.fixture
def mock_library_path(temp_dir: Path) -> str:
    """Provide a mock library path for testing"""
    # Create a fake .so file for testing
    lib_file = temp_dir / "librosbag_deck_core.so"
    lib_file.touch()
    return str(lib_file)


# Mock CFFI components for isolated testing
@pytest.fixture
def mock_ffi():
    """Mock CFFI ffi object for testing without C dependencies"""
    mock = MagicMock()

    # Mock NULL constant
    mock.NULL = None

    # Mock common CFFI operations
    mock.new.return_value = Mock()
    mock.string.return_value = b"test_string"
    mock.buffer.return_value = b"test_data"

    return mock


@pytest.fixture
def mock_ffi_lib():
    """Mock CFFI lib object with C function signatures"""
    mock = MagicMock()

    # Mock all the C API functions with appropriate return values
    mock.rosbag_deck_create.return_value = 0x12345678  # Non-null handle
    mock.rosbag_deck_destroy.return_value = None
    mock.rosbag_deck_build_index.return_value = True
    mock.rosbag_deck_start_playback.return_value = None
    mock.rosbag_deck_stop_playback.return_value = None
    mock.rosbag_deck_step_forward.return_value = True
    mock.rosbag_deck_step_backward.return_value = True
    mock.rosbag_deck_seek_to_time.return_value = True
    mock.rosbag_deck_seek_to_frame.return_value = True
    mock.rosbag_deck_set_cache_size.return_value = None
    mock.rosbag_deck_set_preload_settings.return_value = None
    mock.rosbag_deck_set_playback_rate.return_value = None
    mock.rosbag_deck_set_loop_playback.return_value = None
    mock.rosbag_deck_set_status_callback.return_value = None
    mock.rosbag_deck_set_message_callback.return_value = None

    # Mock complex return structures
    mock_bag_info = Mock()
    mock_bag_info.success = True
    mock_bag_info.message = None
    mock_bag_info.start_time.nanoseconds_since_epoch = 1000000000
    mock_bag_info.end_time.nanoseconds_since_epoch = 2000000000
    mock_bag_info.total_duration_ns = 1000000000
    mock_bag_info.total_frames = 100
    mock_bag_info.topic_names = None
    mock_bag_info.topic_names_count = 0
    mock.rosbag_deck_get_bag_info.return_value = mock_bag_info

    mock_status = Mock()
    mock_status.current_time.nanoseconds_since_epoch = 1500000000
    mock_status.is_playing = False
    mock_status.current_frame = 50
    mock_status.total_frames = 100
    mock_status.timeline_segment = 1
    mock_status.virtual_time.nanoseconds_since_epoch = 1500000000
    mock.rosbag_deck_get_status.return_value = mock_status

    # Mock timestamp conversion
    mock_timestamp = Mock()
    mock_timestamp.nanoseconds_since_epoch = 1000000000
    mock.rosbag_deck_to_timestamp.return_value = mock_timestamp

    return mock


@pytest.fixture
def mock_rosbag_deck_ffi(mock_ffi, mock_ffi_lib):
    """Mock RosbagDeckFFI class for testing"""
    with pytest.MonkeyPatch.context() as mp:
        mock_class = Mock()
        mock_instance = Mock()
        mock_instance.lib = mock_ffi_lib
        mock_instance.ffi = mock_ffi
        mock_class.return_value = mock_instance

        # Import the actual modules to patch them correctly
        import rosbag_deck.ffi
        import rosbag_deck.core

        mp.setattr("rosbag_deck.ffi.RosbagDeckFFI", mock_class)
        mp.setattr("rosbag_deck.ffi.ffi", mock_ffi)
        mp.setattr("rosbag_deck.ffi.create_cffi_callback", Mock(return_value=Mock()))

        yield mock_instance


# Sample data fixtures
@pytest.fixture
def sample_bag_paths() -> List[str]:
    """Provide sample bag file paths for testing"""
    return [
        "/path/to/test_bag1.db3",
        "/path/to/test_bag2.db3",
    ]


@pytest.fixture
def sample_bag_info():
    """Provide sample BagInfo data for testing"""
    from rosbag_deck.core import BagInfo

    return BagInfo(
        success=True,
        message="Test bag loaded successfully",
        start_time=1000000000,  # 1 second in nanoseconds
        end_time=2000000000,  # 2 seconds in nanoseconds
        total_duration_ns=1000000000,
        total_frames=100,
        topic_names=["/test_topic1", "/test_topic2"],
    )


@pytest.fixture
def sample_status():
    """Provide sample Status data for testing"""
    from rosbag_deck.core import Status

    return Status(
        current_time=1500000000,
        is_playing=False,
        current_frame=50,
        total_frames=100,
        timeline_segment=1,
        virtual_time=1500000000,
    )


@pytest.fixture
def sample_message():
    """Provide sample Message data for testing"""
    from rosbag_deck.core import Message

    return Message(
        original_timestamp=1234567890,
        virtual_timestamp=1234567891,
        topic_name="/test_topic",
        message_type="std_msgs/msg/String",
        serialized_data=b"test message data",
        frame_index=42,
    )


# Callback testing fixtures
@pytest.fixture
def status_callback_mock():
    """Mock status callback for testing"""
    return Mock()


@pytest.fixture
def message_callback_mock():
    """Mock message callback for testing"""
    return Mock()


# Performance testing helpers
@pytest.fixture
def memory_profiler():
    """Memory profiling context manager for performance tests"""
    try:
        from memory_profiler import profile

        return profile
    except ImportError:
        pytest.skip("memory_profiler not available")


# Custom markers for test categorization
def pytest_configure(config):
    """Configure custom pytest markers"""
    config.addinivalue_line("markers", "unit: Unit tests")
    config.addinivalue_line("markers", "integration: Integration tests")
    config.addinivalue_line("markers", "performance: Performance tests")
    config.addinivalue_line("markers", "slow: Slow-running tests")


# Test utilities
class TestUtils:
    """Utility functions for testing"""

    @staticmethod
    def assert_callback_called_with_type(mock_callback, expected_type):
        """Assert that a callback was called with an object of expected type"""
        assert mock_callback.called
        args, kwargs = mock_callback.call_args
        assert len(args) == 1
        assert isinstance(args[0], expected_type)

    @staticmethod
    def create_mock_bag_file(temp_dir: Path, name: str) -> Path:
        """Create a mock bag file for testing"""
        bag_file = temp_dir / name
        bag_file.touch()
        return bag_file


@pytest.fixture
def test_utils():
    """Provide test utility functions"""
    return TestUtils
