"""
Unit tests for RosbagDeck core class
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
from rosbag_deck.core import RosbagDeck, BagInfo, Status, Message
from rosbag_deck.exceptions import RosbagDeckInitError, RosbagDeckIndexError


class TestRosbagDeckInitialization:
    """Test RosbagDeck initialization and lifecycle"""

    @pytest.mark.unit
    def test_init_success(self, mock_rosbag_deck_ffi):
        """Test successful RosbagDeck initialization"""
        deck = RosbagDeck()
        
        assert deck.is_valid
        assert hasattr(deck, '_handle')
        assert hasattr(deck, '_ffi')

    @pytest.mark.unit
    def test_init_with_library_path(self, mock_rosbag_deck_ffi):
        """Test RosbagDeck initialization with custom library path"""
        library_path = "/custom/path/librosbag_deck_core.so"
        deck = RosbagDeck(library_path=library_path)
        
        assert deck.is_valid

    @pytest.mark.unit
    def test_init_failure_null_handle(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test initialization failure when C function returns NULL"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = mock_ffi.NULL
        
        with pytest.raises(RosbagDeckInitError, match="Failed to create rosbag deck handle"):
            RosbagDeck()

    @pytest.mark.unit
    def test_init_failure_exception(self, mock_ffi):
        """Test initialization failure when FFI raises exception"""
        with patch('rosbag_deck.core.RosbagDeckFFI') as mock_ffi_class:
            mock_ffi_class.side_effect = Exception("FFI initialization failed")
            
            with pytest.raises(RosbagDeckInitError, match="Failed to initialize RosbagDeck"):
                RosbagDeck()

    @pytest.mark.unit
    def test_context_manager(self, mock_rosbag_deck_ffi):
        """Test RosbagDeck as context manager"""
        with RosbagDeck() as deck:
            assert deck.is_valid
        
        # Should be closed after context exit
        mock_rosbag_deck_ffi.lib.rosbag_deck_destroy.assert_called()

    @pytest.mark.unit
    def test_close_method(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test explicit close method"""
        deck = RosbagDeck()
        assert deck.is_valid
        
        deck.close()
        mock_rosbag_deck_ffi.lib.rosbag_deck_destroy.assert_called()
        assert deck._handle == mock_ffi.NULL

    @pytest.mark.unit
    def test_double_close(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test that double close is safe"""
        deck = RosbagDeck()
        deck.close()
        deck.close()  # Should not raise exception
        
        # Destroy should only be called once
        assert mock_rosbag_deck_ffi.lib.rosbag_deck_destroy.call_count == 1

    @pytest.mark.unit
    def test_destructor(self, mock_rosbag_deck_ffi):
        """Test that destructor calls close"""
        deck = RosbagDeck()
        deck_handle = deck._handle
        
        # Simulate destructor
        deck.__del__()
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_destroy.assert_called()


class TestRosbagDeckConfiguration:
    """Test RosbagDeck configuration methods"""

    @pytest.mark.unit
    def test_set_cache_size(self, mock_rosbag_deck_ffi):
        """Test setting cache size"""
        deck = RosbagDeck()
        deck.set_cache_size(1000)
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_set_cache_size.assert_called_with(
            deck._handle, 1000
        )

    @pytest.mark.unit
    def test_set_preload_settings(self, mock_rosbag_deck_ffi):
        """Test setting preload settings"""
        deck = RosbagDeck()
        deck.set_preload_settings(10, 5)
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_set_preload_settings.assert_called_with(
            deck._handle, 10, 5
        )

    @pytest.mark.unit
    def test_set_playback_rate(self, mock_rosbag_deck_ffi):
        """Test setting playback rate"""
        deck = RosbagDeck()
        deck.set_playback_rate(2.0)
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_set_playback_rate.assert_called_with(
            deck._handle, 2.0
        )

    @pytest.mark.unit
    def test_set_loop_playback(self, mock_rosbag_deck_ffi):
        """Test setting loop playback"""
        deck = RosbagDeck()
        deck.set_loop_playback(True)
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_set_loop_playback.assert_called_with(
            deck._handle, True
        )

    @pytest.mark.unit
    def test_configuration_chain(self, mock_rosbag_deck_ffi):
        """Test chaining configuration calls"""
        deck = RosbagDeck()
        
        # Should be able to call multiple config methods
        deck.set_cache_size(500)
        deck.set_preload_settings(20, 10)
        deck.set_playback_rate(0.5)
        deck.set_loop_playback(False)
        
        # All should have been called
        assert mock_rosbag_deck_ffi.lib.rosbag_deck_set_cache_size.called
        assert mock_rosbag_deck_ffi.lib.rosbag_deck_set_preload_settings.called
        assert mock_rosbag_deck_ffi.lib.rosbag_deck_set_playback_rate.called
        assert mock_rosbag_deck_ffi.lib.rosbag_deck_set_loop_playback.called


class TestRosbagDeckIndexBuilding:
    """Test RosbagDeck index building"""

    @pytest.mark.unit
    def test_build_index_success(self, mock_rosbag_deck_ffi, sample_bag_paths, mock_ffi):
        """Test successful index building"""
        deck = RosbagDeck()
        deck.build_index(sample_bag_paths)
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.assert_called_once()
        # Verify the call was made with correct number of paths
        args = mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.call_args[0]
        assert len(args) == 3  # handle, paths, count
        assert args[2] == len(sample_bag_paths)  # count

    @pytest.mark.unit
    def test_build_index_empty_paths(self, mock_rosbag_deck_ffi):
        """Test build_index with empty path list"""
        deck = RosbagDeck()
        
        with pytest.raises(RosbagDeckIndexError, match="No bag paths provided"):
            deck.build_index([])

    @pytest.mark.unit
    def test_build_index_failure(self, mock_rosbag_deck_ffi, sample_bag_paths):
        """Test build_index failure from C library"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.return_value = False
        
        deck = RosbagDeck()
        with pytest.raises(RosbagDeckIndexError, match="Failed to build index"):
            deck.build_index(sample_bag_paths)

    @pytest.mark.unit
    def test_build_index_unicode_paths(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test build_index with unicode characters in paths"""
        unicode_paths = ["/path/with/ñáéíóú/bag.db3", "/パス/bag.db3"]
        
        deck = RosbagDeck()
        deck.build_index(unicode_paths)
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.assert_called_once()


class TestRosbagDeckPlaybackControl:
    """Test RosbagDeck playback control methods"""

    @pytest.mark.unit
    def test_start_playback(self, mock_rosbag_deck_ffi):
        """Test starting playback"""
        deck = RosbagDeck()
        deck.start_playback()
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_start_playback.assert_called_with(deck._handle)

    @pytest.mark.unit
    def test_stop_playback(self, mock_rosbag_deck_ffi):
        """Test stopping playback"""
        deck = RosbagDeck()
        deck.stop_playback()
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_stop_playback.assert_called_with(deck._handle)

    @pytest.mark.unit
    def test_step_forward_success(self, mock_rosbag_deck_ffi):
        """Test successful step forward"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_step_forward.return_value = True
        
        deck = RosbagDeck()
        result = deck.step_forward()
        
        assert result is True
        mock_rosbag_deck_ffi.lib.rosbag_deck_step_forward.assert_called_with(deck._handle)

    @pytest.mark.unit
    def test_step_forward_failure(self, mock_rosbag_deck_ffi):
        """Test step forward at end of bag"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_step_forward.return_value = False
        
        deck = RosbagDeck()
        result = deck.step_forward()
        
        assert result is False

    @pytest.mark.unit
    def test_step_backward_success(self, mock_rosbag_deck_ffi):
        """Test successful step backward"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_step_backward.return_value = True
        
        deck = RosbagDeck()
        result = deck.step_backward()
        
        assert result is True
        mock_rosbag_deck_ffi.lib.rosbag_deck_step_backward.assert_called_with(deck._handle)

    @pytest.mark.unit
    def test_step_backward_failure(self, mock_rosbag_deck_ffi):
        """Test step backward at beginning of bag"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_step_backward.return_value = False
        
        deck = RosbagDeck()
        result = deck.step_backward()
        
        assert result is False


class TestRosbagDeckSeeking:
    """Test RosbagDeck seeking methods"""

    @pytest.mark.unit
    def test_seek_to_time_success(self, mock_rosbag_deck_ffi):
        """Test successful seek to time"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_seek_to_time.return_value = True
        
        deck = RosbagDeck()
        result = deck.seek_to_time(1500000000)
        
        assert result is True
        mock_rosbag_deck_ffi.lib.rosbag_deck_to_timestamp.assert_called_with(1500000000)
        mock_rosbag_deck_ffi.lib.rosbag_deck_seek_to_time.assert_called()

    @pytest.mark.unit
    def test_seek_to_time_failure(self, mock_rosbag_deck_ffi):
        """Test seek to time failure (time not found)"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_seek_to_time.return_value = False
        
        deck = RosbagDeck()
        result = deck.seek_to_time(9999999999)
        
        assert result is False

    @pytest.mark.unit
    def test_seek_to_frame_success(self, mock_rosbag_deck_ffi):
        """Test successful seek to frame"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_seek_to_frame.return_value = True
        
        deck = RosbagDeck()
        result = deck.seek_to_frame(50)
        
        assert result is True
        mock_rosbag_deck_ffi.lib.rosbag_deck_seek_to_frame.assert_called_with(deck._handle, 50)

    @pytest.mark.unit
    def test_seek_to_frame_failure(self, mock_rosbag_deck_ffi):
        """Test seek to frame failure (frame not found)"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_seek_to_frame.return_value = False
        
        deck = RosbagDeck()
        result = deck.seek_to_frame(9999)
        
        assert result is False

    @pytest.mark.unit
    def test_seek_edge_cases(self, mock_rosbag_deck_ffi):
        """Test seeking edge cases"""
        deck = RosbagDeck()
        
        # Seek to frame 0
        deck.seek_to_frame(0)
        mock_rosbag_deck_ffi.lib.rosbag_deck_seek_to_frame.assert_called_with(deck._handle, 0)
        
        # Seek to time 0
        deck.seek_to_time(0)
        mock_rosbag_deck_ffi.lib.rosbag_deck_to_timestamp.assert_called_with(0)


class TestRosbagDeckInformation:
    """Test RosbagDeck information methods"""

    @pytest.mark.unit
    def test_get_bag_info(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test getting bag information"""
        deck = RosbagDeck()
        bag_info = deck.get_bag_info()
        
        assert isinstance(bag_info, BagInfo)
        assert bag_info.success is True
        assert bag_info.total_frames == 100
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_bag_info.assert_called_with(deck._handle)

    @pytest.mark.unit
    def test_get_bag_info_with_topics(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test getting bag info with topic names"""
        # Mock topic names
        mock_info = mock_rosbag_deck_ffi.lib.rosbag_deck_get_bag_info.return_value
        mock_info.topic_names_count = 2
        
        # Create mock topic name pointers
        topic1_ptr = Mock()
        topic2_ptr = Mock()
        mock_info.topic_names = [topic1_ptr, topic2_ptr]
        
        # Mock ffi.string to return topic names
        mock_ffi.string.side_effect = [b"topic1", b"topic2"]
        
        deck = RosbagDeck()
        bag_info = deck.get_bag_info()
        
        assert len(bag_info.topic_names) == 2

    @pytest.mark.unit
    def test_get_status(self, mock_rosbag_deck_ffi):
        """Test getting playback status"""
        deck = RosbagDeck()
        status = deck.get_status()
        
        assert isinstance(status, Status)
        assert status.current_frame == 50
        assert status.total_frames == 100
        assert status.is_playing is False
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_status.assert_called_with(deck._handle)


class TestRosbagDeckUtilities:
    """Test RosbagDeck utility methods"""

    @pytest.mark.unit
    def test_is_valid_property(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test is_valid property"""
        deck = RosbagDeck()
        assert deck.is_valid is True
        
        deck.close()
        assert deck.is_valid is False

    @pytest.mark.unit
    def test_repr_valid_deck(self, mock_rosbag_deck_ffi):
        """Test string representation of valid deck"""
        deck = RosbagDeck()
        repr_str = repr(deck)
        
        assert "RosbagDeck" in repr_str
        assert "frame=50/100" in repr_str
        assert "playing=False" in repr_str

    @pytest.mark.unit
    def test_repr_closed_deck(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test string representation of closed deck"""
        deck = RosbagDeck()
        deck.close()
        repr_str = repr(deck)
        
        assert "RosbagDeck(closed)" in repr_str

    @pytest.mark.unit
    def test_repr_status_error(self, mock_rosbag_deck_ffi):
        """Test string representation when status query fails"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_status.side_effect = Exception("Status error")
        
        deck = RosbagDeck()
        repr_str = repr(deck)
        
        assert "RosbagDeck(unknown_status)" in repr_str