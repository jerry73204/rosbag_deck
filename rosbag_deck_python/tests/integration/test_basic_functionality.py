"""
Basic integration tests for core functionality
"""

import pytest
from pathlib import Path
from unittest.mock import patch, Mock

# Test only if we can import the module
try:
    from rosbag_deck.core import RosbagDeck, BagInfo, Status, Message
    ROSBAG_DECK_AVAILABLE = True
except ImportError:
    ROSBAG_DECK_AVAILABLE = False


@pytest.mark.integration
@pytest.mark.skipif(not ROSBAG_DECK_AVAILABLE, reason="rosbag_deck not available")
class TestBasicFunctionality:
    """Test basic functionality with mocked C library"""

    def test_deck_lifecycle_with_mocks(self, mock_rosbag_deck_ffi):
        """Test complete deck lifecycle with mocked FFI"""
        
        # Create deck
        deck = RosbagDeck()
        assert deck.is_valid
        
        # Build index (mocked)
        deck.build_index(["/path/to/test.db3"])
        mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.assert_called()
        
        # Configure deck
        deck.set_cache_size(500)
        deck.set_preload_settings(20, 10)
        deck.set_playback_rate(1.5)
        deck.set_loop_playback(True)
        
        # Get information
        bag_info = deck.get_bag_info()
        assert isinstance(bag_info, BagInfo)
        assert bag_info.success
        
        status = deck.get_status()
        assert isinstance(status, Status)
        
        # Playback control
        deck.start_playback()
        mock_rosbag_deck_ffi.lib.rosbag_deck_start_playback.assert_called()
        
        deck.stop_playback()
        mock_rosbag_deck_ffi.lib.rosbag_deck_stop_playback.assert_called()
        
        # Seeking
        assert deck.seek_to_frame(50) is True
        assert deck.seek_to_time(1500000000) is True
        
        # Stepping
        assert deck.step_forward() is True
        assert deck.step_backward() is True
        
        # Close
        deck.close()
        assert not deck.is_valid

    def test_callback_integration(self, mock_rosbag_deck_ffi):
        """Test callback integration"""
        
        status_calls = []
        message_calls = []
        
        def status_callback(status):
            status_calls.append(status)
        
        def message_callback(message):
            message_calls.append(message)
        
        deck = RosbagDeck()
        
        # Set callbacks
        deck.set_status_callback(status_callback)
        deck.set_message_callback(message_callback)
        
        # Verify C API calls were made
        mock_rosbag_deck_ffi.lib.rosbag_deck_set_status_callback.assert_called()
        mock_rosbag_deck_ffi.lib.rosbag_deck_set_message_callback.assert_called()
        
        # Clear callbacks
        deck.set_status_callback(None)
        deck.set_message_callback(None)
        
        deck.close()

    def test_error_handling_integration(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test error handling scenarios"""
        
        # Test initialization failure
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = mock_ffi.NULL
        
        from rosbag_deck.exceptions import RosbagDeckInitError
        with pytest.raises(RosbagDeckInitError):
            RosbagDeck()
        
        # Reset for next test
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        
        # Test index building failure
        deck = RosbagDeck()
        mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.return_value = False
        
        from rosbag_deck.exceptions import RosbagDeckIndexError
        with pytest.raises(RosbagDeckIndexError):
            deck.build_index(["/path/to/test.db3"])
        
        deck.close()

    def test_configuration_persistence(self, mock_rosbag_deck_ffi):
        """Test that configuration calls are properly forwarded"""
        
        deck = RosbagDeck()
        
        # Set various configurations
        deck.set_cache_size(1000)
        deck.set_preload_settings(50, 25)
        deck.set_playback_rate(2.0)
        deck.set_loop_playback(False)
        
        # Verify all were called with correct parameters
        mock_rosbag_deck_ffi.lib.rosbag_deck_set_cache_size.assert_called_with(
            deck._handle, 1000
        )
        mock_rosbag_deck_ffi.lib.rosbag_deck_set_preload_settings.assert_called_with(
            deck._handle, 50, 25
        )
        mock_rosbag_deck_ffi.lib.rosbag_deck_set_playback_rate.assert_called_with(
            deck._handle, 2.0
        )
        mock_rosbag_deck_ffi.lib.rosbag_deck_set_loop_playback.assert_called_with(
            deck._handle, False
        )
        
        deck.close()

    def test_information_queries(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test information query functionality"""
        
        deck = RosbagDeck()
        
        # Configure mock returns for bag info with topics
        mock_info = mock_rosbag_deck_ffi.lib.rosbag_deck_get_bag_info.return_value
        mock_info.topic_names_count = 3
        mock_info.topic_names = [Mock(), Mock(), Mock()]
        mock_info.success = True
        mock_info.total_frames = 150
        
        # Mock string returns for topics
        mock_ffi.string.side_effect = [b"topic1", b"topic2", b"topic3"]
        
        bag_info = deck.get_bag_info()
        assert bag_info.success
        assert bag_info.total_frames == 150
        assert len(bag_info.topic_names) == 3
        
        # Test status query
        status = deck.get_status()
        assert status.current_frame == 50  # From mock
        assert status.total_frames == 100  # From mock
        
        deck.close()


@pytest.mark.integration
class TestWithoutMocks:
    """Tests that don't require the actual library"""

    def test_data_structure_creation(self):
        """Test that data structures can be created independently"""
        
        bag_info = BagInfo(
            success=True,
            message="Test",
            start_time=1000,
            end_time=2000,
            total_duration_ns=1000,
            total_frames=50,
            topic_names=["/test"]
        )
        
        assert bag_info.success
        assert len(bag_info.topic_names) == 1
        
        status = Status(
            current_time=1500,
            is_playing=True,
            current_frame=25,
            total_frames=50,
            timeline_segment=1,
            virtual_time=1500
        )
        
        assert status.is_playing
        assert status.current_frame == 25
        
        message = Message(
            original_timestamp=1000,
            virtual_timestamp=1001,
            topic_name="/test",
            message_type="std_msgs/msg/String",
            serialized_data=b"test data",
            frame_index=10
        )
        
        assert message.topic_name == "/test"
        assert len(message.serialized_data) == 9

    def test_exception_usage(self):
        """Test that exceptions can be used independently"""
        
        from rosbag_deck.exceptions import RosbagDeckError, RosbagDeckInitError, RosbagDeckIndexError
        
        # Test exception creation
        base_error = RosbagDeckError("Base error")
        init_error = RosbagDeckInitError("Init failed")
        index_error = RosbagDeckIndexError("Index failed")
        
        # Test inheritance
        assert isinstance(init_error, RosbagDeckError)
        assert isinstance(index_error, RosbagDeckError)
        
        # Test that they can be raised and caught
        with pytest.raises(RosbagDeckError):
            raise init_error
        
        with pytest.raises(RosbagDeckInitError):
            raise init_error