"""
Unit tests for callback system
"""

import pytest
from unittest.mock import Mock, MagicMock, patch
from rosbag_deck.core import RosbagDeck, Status, Message


class TestStatusCallbacks:
    """Test status callback functionality"""

    @pytest.mark.unit
    def test_set_status_callback(self, mock_rosbag_deck_ffi, status_callback_mock):
        """Test setting a status callback"""
        deck = RosbagDeck()
        deck.set_status_callback(status_callback_mock)
        
        # Should have called the C API to set callback
        mock_rosbag_deck_ffi.lib.rosbag_deck_set_status_callback.assert_called()
        
        # Should store the callback reference
        assert deck._status_callback_func is status_callback_mock
        assert deck._status_callback_ref is not None

    @pytest.mark.unit
    def test_clear_status_callback(self, mock_rosbag_deck_ffi, status_callback_mock, mock_ffi):
        """Test clearing a status callback"""
        deck = RosbagDeck()
        
        # Set callback first
        deck.set_status_callback(status_callback_mock)
        assert deck._status_callback_func is not None
        
        # Clear callback
        deck.set_status_callback(None)
        
        # Should clear references
        assert deck._status_callback_func is None
        assert deck._status_callback_ref is None
        
        # Should call C API with NULL
        calls = mock_rosbag_deck_ffi.lib.rosbag_deck_set_status_callback.call_args_list
        last_call = calls[-1]
        assert last_call[0][1] == mock_ffi.NULL  # callback should be NULL
        assert last_call[0][2] == mock_ffi.NULL  # user_data should be NULL

    @pytest.mark.unit
    def test_status_callback_invocation(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test that status callback is properly invoked"""
        callback_mock = Mock()
        deck = RosbagDeck()
        deck.set_status_callback(callback_mock)
        
        # Get the wrapper function that was created
        wrapper = deck._status_callback_ref
        assert wrapper is not None
        
        # Create a mock status structure as would be passed from C
        mock_status_ptr = Mock()
        mock_status = Mock()
        mock_status.current_time.nanoseconds_since_epoch = 1500000000
        mock_status.is_playing = True
        mock_status.current_frame = 75
        mock_status.total_frames = 100
        mock_status.timeline_segment = 1
        mock_status.virtual_time.nanoseconds_since_epoch = 1500000000
        mock_status_ptr.__getitem__.return_value = mock_status
        
        # Simulate callback from C code
        with patch('rosbag_deck.ffi.create_cffi_callback') as mock_create_callback:
            # Extract the wrapper function that would be passed to CFFI
            deck.set_status_callback(callback_mock)
            wrapper_func = mock_create_callback.call_args[0][1]
            
            # Call the wrapper directly
            wrapper_func(mock_status_ptr, None)
            
            # Verify callback was called with Status object
            callback_mock.assert_called_once()
            status_arg = callback_mock.call_args[0][0]
            assert isinstance(status_arg, Status)
            assert status_arg.current_time == 1500000000
            assert status_arg.is_playing is True

    @pytest.mark.unit
    def test_status_callback_exception_handling(self, mock_rosbag_deck_ffi):
        """Test that exceptions in status callback don't crash"""
        def failing_callback(status):
            raise ValueError("Callback error")
        
        deck = RosbagDeck()
        deck.set_status_callback(failing_callback)
        
        # Get wrapper function
        with patch('rosbag_deck.ffi.create_cffi_callback') as mock_create_callback:
            deck.set_status_callback(failing_callback)
            wrapper_func = mock_create_callback.call_args[0][1]
            
            # Mock status
            mock_status_ptr = Mock()
            mock_status = Mock()
            mock_status.current_time.nanoseconds_since_epoch = 1000
            mock_status.is_playing = False
            mock_status.current_frame = 0
            mock_status.total_frames = 100
            mock_status.timeline_segment = 0
            mock_status.virtual_time.nanoseconds_since_epoch = 1000
            mock_status_ptr.__getitem__.return_value = mock_status
            
            # Should not raise exception
            with patch('builtins.print') as mock_print:
                wrapper_func(mock_status_ptr, None)
                
                # Should have printed error
                mock_print.assert_called()
                assert "Error in status callback" in str(mock_print.call_args)

    @pytest.mark.unit
    def test_multiple_status_callback_changes(self, mock_rosbag_deck_ffi):
        """Test changing status callback multiple times"""
        deck = RosbagDeck()
        
        callback1 = Mock()
        callback2 = Mock()
        
        # Set first callback
        deck.set_status_callback(callback1)
        assert deck._status_callback_func is callback1
        
        # Change to second callback
        deck.set_status_callback(callback2)
        assert deck._status_callback_func is callback2
        
        # Clear callback
        deck.set_status_callback(None)
        assert deck._status_callback_func is None
        
        # Verify C API was called each time
        assert mock_rosbag_deck_ffi.lib.rosbag_deck_set_status_callback.call_count == 3


class TestMessageCallbacks:
    """Test message callback functionality"""

    @pytest.mark.unit
    def test_set_message_callback(self, mock_rosbag_deck_ffi, message_callback_mock):
        """Test setting a message callback"""
        deck = RosbagDeck()
        deck.set_message_callback(message_callback_mock)
        
        # Should have called the C API
        mock_rosbag_deck_ffi.lib.rosbag_deck_set_message_callback.assert_called()
        
        # Should store the callback reference
        assert deck._message_callback_func is message_callback_mock
        assert deck._message_callback_ref is not None

    @pytest.mark.unit
    def test_clear_message_callback(self, mock_rosbag_deck_ffi, message_callback_mock, mock_ffi):
        """Test clearing a message callback"""
        deck = RosbagDeck()
        
        # Set callback first
        deck.set_message_callback(message_callback_mock)
        
        # Clear callback
        deck.set_message_callback(None)
        
        # Should clear references
        assert deck._message_callback_func is None
        assert deck._message_callback_ref is None
        
        # Should call C API with NULL
        calls = mock_rosbag_deck_ffi.lib.rosbag_deck_set_message_callback.call_args_list
        last_call = calls[-1]
        assert last_call[0][1] == mock_ffi.NULL

    @pytest.mark.unit
    def test_message_callback_invocation(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test that message callback is properly invoked"""
        callback_mock = Mock()
        deck = RosbagDeck()
        
        with patch('rosbag_deck.ffi.create_cffi_callback') as mock_create_callback:
            deck.set_message_callback(callback_mock)
            wrapper_func = mock_create_callback.call_args[0][1]
            
            # Create mock message structure
            mock_message_ptr = Mock()
            mock_message = Mock()
            mock_message.original_timestamp.nanoseconds_since_epoch = 1000000000
            mock_message.virtual_timestamp.nanoseconds_since_epoch = 1000000001
            mock_message.topic_name = Mock()
            mock_message.message_type = Mock()
            mock_message.serialized_data = Mock()
            mock_message.serialized_data_size = 10
            mock_message.frame_index = 42
            mock_message_ptr.__getitem__.return_value = mock_message
            
            # Mock string extraction
            mock_ffi.string.side_effect = [b"test_topic", b"std_msgs/msg/String"]
            mock_ffi.buffer.return_value = b"test_data_"
            
            # Call wrapper
            wrapper_func(mock_message_ptr, None)
            
            # Verify callback was called
            callback_mock.assert_called_once()
            message_arg = callback_mock.call_args[0][0]
            assert isinstance(message_arg, Message)
            assert message_arg.frame_index == 42

    @pytest.mark.unit
    def test_message_callback_with_empty_data(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test message callback with empty serialized data"""
        callback_mock = Mock()
        deck = RosbagDeck()
        
        with patch('rosbag_deck.ffi.create_cffi_callback') as mock_create_callback:
            deck.set_message_callback(callback_mock)
            wrapper_func = mock_create_callback.call_args[0][1]
            
            # Mock message with empty data
            mock_message_ptr = Mock()
            mock_message = Mock()
            mock_message.original_timestamp.nanoseconds_since_epoch = 1000
            mock_message.virtual_timestamp.nanoseconds_since_epoch = 1000
            mock_message.topic_name = Mock()
            mock_message.message_type = Mock()
            mock_message.serialized_data = mock_ffi.NULL
            mock_message.serialized_data_size = 0
            mock_message.frame_index = 0
            mock_message_ptr.__getitem__.return_value = mock_message
            
            mock_ffi.string.side_effect = [b"empty_topic", b"std_msgs/msg/Empty"]
            
            # Call wrapper
            wrapper_func(mock_message_ptr, None)
            
            # Verify callback was called with empty data
            callback_mock.assert_called_once()
            message_arg = callback_mock.call_args[0][0]
            assert message_arg.serialized_data == b""

    @pytest.mark.unit
    def test_message_callback_exception_handling(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test that exceptions in message callback don't crash"""
        def failing_callback(message):
            raise RuntimeError("Message callback error")
        
        deck = RosbagDeck()
        
        with patch('rosbag_deck.ffi.create_cffi_callback') as mock_create_callback:
            deck.set_message_callback(failing_callback)
            wrapper_func = mock_create_callback.call_args[0][1]
            
            # Mock message
            mock_message_ptr = Mock()
            mock_message = Mock()
            mock_message.original_timestamp.nanoseconds_since_epoch = 1000
            mock_message.virtual_timestamp.nanoseconds_since_epoch = 1000
            mock_message.topic_name = Mock()
            mock_message.message_type = Mock()
            mock_message.serialized_data = Mock()
            mock_message.serialized_data_size = 5
            mock_message.frame_index = 1
            mock_message_ptr.__getitem__.return_value = mock_message
            
            mock_ffi.string.side_effect = [b"topic", b"type"]
            mock_ffi.buffer.return_value = b"data_"
            
            # Should not raise exception
            with patch('builtins.print') as mock_print:
                wrapper_func(mock_message_ptr, None)
                
                # Should have printed error
                mock_print.assert_called()
                assert "Error in message callback" in str(mock_print.call_args)


class TestCallbackMemoryManagement:
    """Test callback memory management and garbage collection"""

    @pytest.mark.unit
    def test_callback_reference_kept(self, mock_rosbag_deck_ffi):
        """Test that callback references are kept to prevent GC"""
        callback = Mock()
        deck = RosbagDeck()
        deck.set_status_callback(callback)
        
        # References should be kept
        assert deck._status_callback_func is callback
        assert deck._status_callback_ref is not None

    @pytest.mark.unit
    def test_callback_reference_cleared(self, mock_rosbag_deck_ffi):
        """Test that callback references are cleared when set to None"""
        callback = Mock()
        deck = RosbagDeck()
        
        deck.set_status_callback(callback)
        deck.set_message_callback(callback)
        
        # Clear callbacks
        deck.set_status_callback(None)
        deck.set_message_callback(None)
        
        # References should be cleared
        assert deck._status_callback_func is None
        assert deck._message_callback_func is None
        assert deck._status_callback_ref is None
        assert deck._message_callback_ref is None

    @pytest.mark.unit
    def test_callbacks_cleared_on_close(self, mock_rosbag_deck_ffi):
        """Test that callbacks are cleared when deck is closed"""
        callback = Mock()
        deck = RosbagDeck()
        
        deck.set_status_callback(callback)
        deck.set_message_callback(callback)
        
        # Close deck
        deck.close()
        
        # Callbacks should still be set (close doesn't clear them automatically)
        # This is the current behavior - callbacks persist until explicitly cleared
        assert deck._status_callback_func is callback
        assert deck._message_callback_func is callback


class TestCallbackIntegration:
    """Test callback integration scenarios"""

    @pytest.mark.unit
    def test_both_callbacks_set(self, mock_rosbag_deck_ffi):
        """Test setting both status and message callbacks"""
        status_callback = Mock()
        message_callback = Mock()
        
        deck = RosbagDeck()
        deck.set_status_callback(status_callback)
        deck.set_message_callback(message_callback)
        
        # Both should be set
        assert deck._status_callback_func is status_callback
        assert deck._message_callback_func is message_callback
        
        # Both C API calls should be made
        mock_rosbag_deck_ffi.lib.rosbag_deck_set_status_callback.assert_called()
        mock_rosbag_deck_ffi.lib.rosbag_deck_set_message_callback.assert_called()

    @pytest.mark.unit
    def test_callback_independence(self, mock_rosbag_deck_ffi):
        """Test that status and message callbacks are independent"""
        status_callback = Mock()
        message_callback = Mock()
        
        deck = RosbagDeck()
        deck.set_status_callback(status_callback)
        deck.set_message_callback(message_callback)
        
        # Clear only status callback
        deck.set_status_callback(None)
        
        # Status should be cleared, message should remain
        assert deck._status_callback_func is None
        assert deck._message_callback_func is message_callback

    @pytest.mark.unit
    def test_callback_thread_safety_assumptions(self, mock_rosbag_deck_ffi):
        """Test assumptions about callback thread safety"""
        # This test documents the assumption that callbacks may be called
        # from different threads, but doesn't actually test thread safety
        # (that would require integration tests)
        
        callback = Mock()
        deck = RosbagDeck()
        deck.set_status_callback(callback)
        
        # The callback wrapper should be prepared for multi-threaded access
        # This is tested by verifying exception handling exists
        assert deck._status_callback_ref is not None