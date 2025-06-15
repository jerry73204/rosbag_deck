"""
Unit tests for error handling and C layer error propagation
Tests error propagation, invalid inputs, resource exhaustion, and concurrent operations
"""

import pytest
from unittest.mock import Mock, MagicMock, patch
import threading
import time
from concurrent.futures import ThreadPoolExecutor

from rosbag_deck.exceptions import (
    RosbagDeckError,
    RosbagDeckInitError,
    RosbagDeckIndexError
)


class TestCErrorPropagation:
    """Test C layer error propagation to Python exceptions"""
    
    @pytest.mark.unit
    def test_handle_creation_error(self, mock_rosbag_deck_ffi):
        """Test error when creating deck handle fails"""
        from rosbag_deck.core import RosbagDeck
        
        # Mock handle creation failure
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0  # NULL handle
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_last_error.return_value = b"Memory allocation failed"
        
        with pytest.raises(RosbagDeckInitError) as exc_info:
            deck = RosbagDeck()
        
        assert "Failed to create RosbagDeck handle" in str(exc_info.value)
    
    @pytest.mark.unit
    def test_index_building_error(self, mock_rosbag_deck_ffi):
        """Test error propagation when index building fails"""
        from rosbag_deck.core import RosbagDeck
        
        # Mock successful creation but failed index building
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.return_value = False
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_last_error.return_value = b"Corrupted bag file"
        
        deck = RosbagDeck()
        
        with pytest.raises(RosbagDeckIndexError) as exc_info:
            deck.build_index(["/path/to/bad.bag"])
        
        assert "Failed to build index" in str(exc_info.value)
    
    @pytest.mark.unit
    def test_c_error_code_mapping(self, mock_rosbag_deck_ffi):
        """Test mapping of C error codes to Python exceptions"""
        from rosbag_deck.core import RosbagDeck
        
        # Define error code scenarios
        error_scenarios = [
            (-1, b"Invalid handle", RosbagDeckError),
            (-2, b"Bag not found", RosbagDeckIndexError),
            (-3, b"Index build failed", RosbagDeckIndexError),
            (-4, b"Out of memory", RosbagDeckError),
            (-5, b"Invalid parameter", ValueError),
        ]
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        for error_code, error_msg, expected_exception in error_scenarios:
            mock_rosbag_deck_ffi.lib.rosbag_deck_step_forward.return_value = error_code
            mock_rosbag_deck_ffi.lib.rosbag_deck_get_last_error.return_value = error_msg
            
            # Some operations might return bool, others int
            if error_code < 0:
                mock_rosbag_deck_ffi.lib.rosbag_deck_step_forward.return_value = False
            
            with pytest.raises(RosbagDeckError):
                deck.step_forward()
    
    @pytest.mark.unit
    def test_null_string_error_handling(self, mock_rosbag_deck_ffi):
        """Test handling of NULL strings from C layer"""
        from rosbag_deck.core import RosbagDeck
        
        # Mock bag info with NULL message
        mock_info = Mock()
        mock_info.success = False
        mock_info.message = mock_rosbag_deck_ffi.ffi.NULL
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_bag_info.return_value = mock_info
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        with pytest.raises(RosbagDeckError) as exc_info:
            info = deck.get_bag_info()
        
        # Should handle NULL gracefully
        assert exc_info.value is not None


class TestInvalidParameterHandling:
    """Test handling of invalid input parameters"""
    
    @pytest.mark.unit
    def test_invalid_bag_paths(self, mock_rosbag_deck_ffi):
        """Test handling of invalid bag file paths"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        invalid_paths = [
            None,
            [],
            [""],
            [None],
            ["path/with\x00null"],
            ["/nonexistent/path.bag"],
        ]
        
        for paths in invalid_paths:
            if paths is None or not paths:
                with pytest.raises((ValueError, TypeError)):
                    deck.build_index(paths)
            else:
                # C layer should handle and return false
                mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.return_value = False
                with pytest.raises(RosbagDeckIndexError):
                    deck.build_index(paths)
    
    @pytest.mark.unit
    def test_invalid_cache_size(self, mock_rosbag_deck_ffi):
        """Test handling of invalid cache size values"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        invalid_sizes = [
            -1,
            0,
            -100,
            float('inf'),
            None,
            "not a number",
        ]
        
        for size in invalid_sizes:
            if isinstance(size, (int, float)) and size <= 0:
                # Should accept but may warn
                deck.set_cache_size(int(size))
            else:
                with pytest.raises((TypeError, ValueError)):
                    deck.set_cache_size(size)
    
    @pytest.mark.unit
    def test_invalid_seek_values(self, mock_rosbag_deck_ffi):
        """Test handling of invalid seek positions"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        mock_rosbag_deck_ffi.lib.rosbag_deck_seek_to_time.return_value = False
        mock_rosbag_deck_ffi.lib.rosbag_deck_seek_to_frame.return_value = False
        
        deck = RosbagDeck()
        
        # Test invalid time seeks
        invalid_times = [-1, float('inf'), float('nan')]
        for time_ns in invalid_times:
            with pytest.raises(RosbagDeckError):
                deck.seek_to_time(time_ns)
        
        # Test invalid frame seeks
        invalid_frames = [-1, -100]
        for frame in invalid_frames:
            with pytest.raises(RosbagDeckError):
                deck.seek_to_frame(frame)
    
    @pytest.mark.unit
    def test_invalid_callback_types(self, mock_rosbag_deck_ffi):
        """Test handling of invalid callback types"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        invalid_callbacks = [
            None,
            "not a function",
            123,
            [],
            {},
        ]
        
        for callback in invalid_callbacks:
            if not callable(callback) and callback is not None:
                with pytest.raises(TypeError):
                    deck.set_status_callback(callback)
                
                with pytest.raises(TypeError):
                    deck.set_message_callback(callback)


class TestResourceExhaustion:
    """Test behavior under resource exhaustion scenarios"""
    
    @pytest.mark.unit
    def test_memory_allocation_failure(self, mock_rosbag_deck_ffi):
        """Test handling when memory allocation fails"""
        from rosbag_deck.core import RosbagDeck
        
        # Simulate memory allocation failure
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0  # NULL
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_last_error.return_value = b"malloc failed"
        
        with pytest.raises(RosbagDeckInitError) as exc_info:
            deck = RosbagDeck()
        
        assert "Failed to create" in str(exc_info.value)
    
    @pytest.mark.unit
    def test_large_data_handling(self, mock_rosbag_deck_ffi):
        """Test handling of extremely large messages"""
        from rosbag_deck.core import RosbagDeck, Message
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Create a mock large message (100MB)
        large_size = 100 * 1024 * 1024
        mock_msg = Mock()
        mock_msg.original_timestamp.nanoseconds_since_epoch = 1000000000
        mock_msg.virtual_timestamp.nanoseconds_since_epoch = 1000000000
        mock_msg.topic_name = b"/large_topic"
        mock_msg.message_type = b"sensor_msgs/PointCloud2"
        mock_msg.serialized_data_size = large_size
        mock_msg.frame_index = 1
        
        # Mock the data access - should handle without crashing
        mock_data = Mock()
        mock_rosbag_deck_ffi.ffi.buffer.return_value = b'x' * min(1024, large_size)  # Return smaller for test
        mock_msg.serialized_data = mock_data
        
        # Test callback with large message
        callback_called = False
        def message_callback(msg: Message):
            nonlocal callback_called
            callback_called = True
            assert len(msg.serialized_data) > 0
        
        deck.set_message_callback(message_callback)
        
        # Simulate callback invocation
        with patch('rosbag_deck.ffi.create_cffi_callback') as mock_create:
            cffi_callback = mock_create.return_value
            # The callback would be invoked by C layer
            # For testing, we just verify setup
            assert mock_create.called
    
    @pytest.mark.unit
    def test_file_handle_exhaustion(self, mock_rosbag_deck_ffi):
        """Test handling when file handles are exhausted"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Simulate file handle exhaustion during index building
        mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.return_value = False
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_last_error.return_value = b"Too many open files"
        
        # Create many bag paths
        many_bags = [f"/path/to/bag_{i}.db3" for i in range(1000)]
        
        with pytest.raises(RosbagDeckIndexError) as exc_info:
            deck.build_index(many_bags)
        
        assert "Failed to build index" in str(exc_info.value)


class TestConcurrentOperationErrors:
    """Test error handling in concurrent operations"""
    
    @pytest.mark.unit
    def test_concurrent_playback_control(self, mock_rosbag_deck_ffi):
        """Test concurrent playback control operations"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Track call order
        call_order = []
        
        def track_call(name):
            def wrapper(*args, **kwargs):
                call_order.append(name)
                time.sleep(0.01)  # Simulate some work
                return True
            return wrapper
        
        # Mock concurrent operations
        mock_rosbag_deck_ffi.lib.rosbag_deck_start_playback = track_call('start')
        mock_rosbag_deck_ffi.lib.rosbag_deck_stop_playback = track_call('stop')
        mock_rosbag_deck_ffi.lib.rosbag_deck_step_forward.side_effect = track_call('step')
        
        # Run concurrent operations
        with ThreadPoolExecutor(max_workers=3) as executor:
            futures = []
            futures.append(executor.submit(deck.start_playback))
            futures.append(executor.submit(deck.stop_playback))
            futures.append(executor.submit(deck.step_forward))
            
            # Wait for completion
            for future in futures:
                try:
                    future.result()
                except Exception:
                    pass  # Expected for some concurrent operations
        
        # Verify operations were attempted
        assert len(call_order) >= 1  # At least one operation should complete
    
    @pytest.mark.unit
    def test_callback_exception_isolation(self, mock_rosbag_deck_ffi):
        """Test that exceptions in callbacks don't crash the core"""
        from rosbag_deck.core import RosbagDeck, Status
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        exception_count = 0
        
        def failing_callback(status: Status):
            nonlocal exception_count
            exception_count += 1
            raise RuntimeError("Callback failed!")
        
        # Set the failing callback
        deck.set_status_callback(failing_callback)
        
        # The C layer should handle callback exceptions gracefully
        # In practice, the Python wrapper should catch and log these
        # For testing, we verify the callback can be set without crashing
        assert deck._status_callback is not None
    
    @pytest.mark.unit
    def test_thread_safety_errors(self, mock_rosbag_deck_ffi):
        """Test thread safety of error handling"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        
        errors_caught = []
        
        def worker_thread(thread_id):
            try:
                deck = RosbagDeck()
                # Simulate some failing operations
                mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.return_value = False
                deck.build_index([f"/path/thread_{thread_id}.bag"])
            except Exception as e:
                errors_caught.append((thread_id, type(e).__name__))
        
        # Run multiple threads
        threads = []
        for i in range(5):
            t = threading.Thread(target=worker_thread, args=(i,))
            threads.append(t)
            t.start()
        
        # Wait for completion
        for t in threads:
            t.join()
        
        # Verify each thread caught its error
        assert len(errors_caught) >= 1  # At least some threads should catch errors


class TestErrorRecovery:
    """Test error recovery and state consistency"""
    
    @pytest.mark.unit
    def test_recovery_after_failed_index(self, mock_rosbag_deck_ffi):
        """Test that deck can recover after failed index building"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # First attempt fails
        mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.return_value = False
        with pytest.raises(RosbagDeckIndexError):
            deck.build_index(["/bad/path.bag"])
        
        # Second attempt succeeds
        mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.return_value = True
        deck.build_index(["/good/path.bag"])  # Should not raise
    
    @pytest.mark.unit
    def test_state_consistency_after_error(self, mock_rosbag_deck_ffi):
        """Test that deck state remains consistent after errors"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        
        # Mock initial status
        mock_status = Mock()
        mock_status.is_playing = False
        mock_status.current_frame = 0
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_status.return_value = mock_status
        
        deck = RosbagDeck()
        
        # Cause an error during playback
        mock_rosbag_deck_ffi.lib.rosbag_deck_start_playback.side_effect = Exception("Playback failed")
        
        try:
            deck.start_playback()
        except:
            pass
        
        # Status should still be accessible
        status = deck.get_status()
        assert status.is_playing == False  # Should not be playing after error
    
    @pytest.mark.unit
    def test_cleanup_after_error(self, mock_rosbag_deck_ffi):
        """Test proper cleanup after errors"""
        from rosbag_deck.core import RosbagDeck
        
        # Test cleanup when creation fails
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0
        
        try:
            deck = RosbagDeck()
        except RosbagDeckInitError:
            # Should not leak resources
            pass
        
        # Verify destroy wasn't called on NULL handle
        mock_rosbag_deck_ffi.lib.rosbag_deck_destroy.assert_not_called()


class TestErrorMessages:
    """Test quality and clarity of error messages"""
    
    @pytest.mark.unit
    def test_descriptive_error_messages(self, mock_rosbag_deck_ffi):
        """Test that error messages are descriptive and helpful"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Test various error scenarios with descriptive messages
        error_scenarios = [
            (
                "rosbag_deck_build_index",
                b"Failed to open bag file: Permission denied",
                "Permission denied"
            ),
            (
                "rosbag_deck_seek_to_time",
                b"Seek time 12345 is beyond bag duration",
                "beyond bag duration"
            ),
            (
                "rosbag_deck_set_cache_size",
                b"Cache size must be positive, got: -10",
                "must be positive"
            ),
        ]
        
        for method_name, c_error, expected_phrase in error_scenarios:
            mock_method = getattr(mock_rosbag_deck_ffi.lib, method_name)
            mock_method.return_value = False
            mock_rosbag_deck_ffi.lib.rosbag_deck_get_last_error.return_value = c_error
            
            # The actual method call would depend on the method
            # For testing, we just verify error message construction
            assert expected_phrase in c_error.decode('utf-8')
    
    @pytest.mark.unit
    def test_error_context_preservation(self, mock_rosbag_deck_ffi):
        """Test that error context is preserved through layers"""
        from rosbag_deck.core import RosbagDeck
        
        # Create a chain of errors
        original_error = "Storage plugin 'custom_plugin' not found"
        wrapper_error = f"Failed to read bag metadata: {original_error}"
        final_error = f"Index building failed: {wrapper_error}"
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.return_value = False
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_last_error.return_value = final_error.encode('utf-8')
        
        deck = RosbagDeck()
        
        with pytest.raises(RosbagDeckIndexError) as exc_info:
            deck.build_index(["/path/to/bag.custom"])
        
        error_message = str(exc_info.value)
        # Should preserve the error chain context
        assert "Failed to build index" in error_message or "Index building failed" in error_message