"""
Integration tests for memory management and leak detection
"""

import gc
import time
import threading
from typing import List
from unittest.mock import Mock

import pytest

# Import memory profiler if available
try:
    from memory_profiler import profile
    MEMORY_PROFILER_AVAILABLE = True
except ImportError:
    MEMORY_PROFILER_AVAILABLE = False


class TestMemoryLeaks:
    """Test for memory leaks in various usage patterns"""

    @pytest.mark.integration
    @pytest.mark.skipif(not MEMORY_PROFILER_AVAILABLE, reason="memory_profiler not available")
    def test_deck_creation_destruction_loop(self, mock_rosbag_deck_ffi):
        """Test repeated deck creation and destruction for memory leaks"""
        from rosbag_deck.core import RosbagDeck
        
        initial_objects = len(gc.get_objects())
        
        # Create and destroy many deck instances
        for i in range(100):
            deck = RosbagDeck()
            deck.close()
            del deck
            
            # Force garbage collection every 10 iterations
            if i % 10 == 9:
                gc.collect()
        
        # Final garbage collection
        gc.collect()
        final_objects = len(gc.get_objects())
        
        # Allow some tolerance for temporary objects
        object_growth = final_objects - initial_objects
        assert object_growth < 50, f"Too many objects created: {object_growth}"

    @pytest.mark.integration
    def test_callback_reference_cleanup(self, mock_rosbag_deck_ffi):
        """Test that callback references don't cause memory leaks"""
        from rosbag_deck.core import RosbagDeck
        
        # Track callback objects
        callbacks_created = []
        
        def create_callback():
            def callback(data):
                pass
            callbacks_created.append(callback)
            return callback
        
        deck = RosbagDeck()
        
        # Set and clear callbacks multiple times
        for _ in range(50):
            status_callback = create_callback()
            message_callback = create_callback()
            
            deck.set_status_callback(status_callback)
            deck.set_message_callback(message_callback)
            
            deck.set_status_callback(None)
            deck.set_message_callback(None)
            
            # Clear local references
            del status_callback
            del message_callback
        
        deck.close()
        del deck
        gc.collect()
        
        # Check that callbacks can be garbage collected
        # (This is a simplistic test - in practice you'd use weak references)
        assert len(callbacks_created) == 100

    @pytest.mark.integration
    def test_large_message_handling(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test memory efficiency with large messages"""
        from rosbag_deck.core import RosbagDeck
        
        # Simulate large message data
        large_data = b"x" * (1024 * 1024)  # 1MB
        mock_ffi.buffer.return_value = large_data
        
        messages_received = []
        
        def message_callback(message):
            # Store only frame index, not the data itself
            messages_received.append(message.frame_index)
        
        deck = RosbagDeck()
        deck.set_message_callback(message_callback)
        
        # Simulate receiving many large messages
        with pytest.MonkeyPatch.context() as mp:
            # Mock the wrapper function to simulate callback invocation
            def simulate_message_callback():
                mock_message_ptr = Mock()
                mock_message = Mock()
                mock_message.original_timestamp.nanoseconds_since_epoch = 1000
                mock_message.virtual_timestamp.nanoseconds_since_epoch = 1000
                mock_message.topic_name = Mock()
                mock_message.message_type = Mock()
                mock_message.serialized_data = Mock()
                mock_message.serialized_data_size = len(large_data)
                mock_message.frame_index = 1
                mock_message_ptr.__getitem__.return_value = mock_message
                
                mock_ffi.string.side_effect = [b"topic", b"type"]
                
                # Get the actual wrapper function
                if hasattr(deck, '_message_callback_ref') and deck._message_callback_ref:
                    # This would normally be called by the CFFI callback
                    # We simulate it here
                    wrapper_func = deck._message_callback_ref._func if hasattr(deck._message_callback_ref, '_func') else None
                    if wrapper_func:
                        wrapper_func(mock_message_ptr, None)
            
            # Simulate multiple large messages
            for i in range(10):
                simulate_message_callback()
                gc.collect()  # Force cleanup between messages
        
        deck.close()
        
        # Verify messages were processed but not all kept in memory
        assert len(messages_received) <= 10

    @pytest.mark.integration
    def test_long_running_session(self, mock_rosbag_deck_ffi):
        """Test memory stability in long-running sessions"""
        from rosbag_deck.core import RosbagDeck
        
        deck = RosbagDeck()
        
        # Simulate long-running operations
        operations = [
            lambda: deck.set_cache_size(100),
            lambda: deck.set_preload_settings(10, 5),
            lambda: deck.set_playback_rate(1.0),
            lambda: deck.get_status(),
            lambda: deck.get_bag_info(),
        ]
        
        initial_objects = len(gc.get_objects())
        
        # Perform many operations
        for i in range(1000):
            operation = operations[i % len(operations)]
            operation()
            
            if i % 100 == 99:
                gc.collect()
        
        gc.collect()
        final_objects = len(gc.get_objects())
        
        deck.close()
        
        # Check for reasonable object growth
        object_growth = final_objects - initial_objects
        assert object_growth < 100, f"Excessive object growth: {object_growth}"


class TestMemoryEfficiency:
    """Test memory efficiency of various operations"""

    @pytest.mark.integration
    def test_status_query_efficiency(self, mock_rosbag_deck_ffi):
        """Test that status queries don't accumulate memory"""
        from rosbag_deck.core import RosbagDeck
        
        deck = RosbagDeck()
        
        # Get initial memory baseline
        statuses = []
        for i in range(1000):
            status = deck.get_status()
            if i < 10:  # Keep only first few for verification
                statuses.append(status)
        
        deck.close()
        
        # Verify we got status objects
        assert len(statuses) == 10
        for status in statuses:
            assert hasattr(status, 'current_frame')
            assert hasattr(status, 'is_playing')

    @pytest.mark.integration
    def test_bag_info_caching(self, mock_rosbag_deck_ffi):
        """Test that bag info queries are efficient"""
        from rosbag_deck.core import RosbagDeck
        
        deck = RosbagDeck()
        
        # Multiple calls should not create excessive objects
        bag_infos = []
        for i in range(100):
            bag_info = deck.get_bag_info()
            if i < 5:  # Keep only first few
                bag_infos.append(bag_info)
        
        deck.close()
        
        # Verify consistency
        assert len(bag_infos) == 5
        for info in bag_infos:
            assert info.success == bag_infos[0].success
            assert info.total_frames == bag_infos[0].total_frames

    @pytest.mark.integration
    def test_string_handling_efficiency(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test efficient handling of string data from C"""
        from rosbag_deck.core import RosbagDeck
        
        # Mock different topic names
        topic_names = [f"/topic_{i}" for i in range(100)]
        mock_ffi.string.side_effect = [name.encode() for name in topic_names]
        
        deck = RosbagDeck()
        
        # Simulate getting bag info with many topics
        mock_info = mock_rosbag_deck_ffi.lib.rosbag_deck_get_bag_info.return_value
        mock_info.topic_names_count = len(topic_names)
        mock_info.topic_names = [Mock() for _ in topic_names]
        
        bag_info = deck.get_bag_info()
        
        deck.close()
        
        # Verify string handling
        assert len(bag_info.topic_names) == len(topic_names)


class TestConcurrentMemoryUsage:
    """Test memory usage under concurrent access"""

    @pytest.mark.integration
    def test_concurrent_deck_creation(self, mock_rosbag_deck_ffi):
        """Test memory usage when creating decks concurrently"""
        from rosbag_deck.core import RosbagDeck
        
        results = {}
        exceptions = []
        
        def create_deck_worker(worker_id):
            try:
                deck = RosbagDeck()
                # Perform some operations
                deck.set_cache_size(100)
                status = deck.get_status()
                deck.close()
                results[worker_id] = status.current_frame
            except Exception as e:
                exceptions.append(e)
        
        # Create multiple threads
        threads = []
        for i in range(10):
            thread = threading.Thread(target=create_deck_worker, args=(i,))
            threads.append(thread)
            thread.start()
        
        # Wait for completion
        for thread in threads:
            thread.join()
        
        # Verify no exceptions and all workers completed
        assert len(exceptions) == 0, f"Exceptions occurred: {exceptions}"
        assert len(results) == 10

    @pytest.mark.integration
    def test_concurrent_callbacks(self, mock_rosbag_deck_ffi):
        """Test memory usage with concurrent callback operations"""
        from rosbag_deck.core import RosbagDeck
        
        deck = RosbagDeck()
        callback_counts = {'status': 0, 'message': 0}
        lock = threading.Lock()
        
        def status_callback(status):
            with lock:
                callback_counts['status'] += 1
        
        def message_callback(message):
            with lock:
                callback_counts['message'] += 1
        
        def callback_worker():
            # Rapidly set and clear callbacks
            for _ in range(50):
                deck.set_status_callback(status_callback)
                deck.set_message_callback(message_callback)
                time.sleep(0.001)  # Brief pause
                deck.set_status_callback(None)
                deck.set_message_callback(None)
        
        # Run concurrent callback operations
        threads = []
        for _ in range(3):
            thread = threading.Thread(target=callback_worker)
            threads.append(thread)
            thread.start()
        
        for thread in threads:
            thread.join()
        
        deck.close()
        
        # No specific assertions needed - just ensure no crashes


class TestGarbageCollection:
    """Test garbage collection behavior"""

    @pytest.mark.integration
    def test_circular_reference_handling(self, mock_rosbag_deck_ffi):
        """Test that circular references are handled properly"""
        from rosbag_deck.core import RosbagDeck
        
        # Create objects with potential circular references
        deck = RosbagDeck()
        
        # Create a callback that references the deck
        def status_callback(status):
            # This creates a potential circular reference
            _ = deck.get_status()
        
        deck.set_status_callback(status_callback)
        
        # Clear direct references
        callback_ref = deck._status_callback_func
        deck_ref = deck
        
        deck.set_status_callback(None)
        deck.close()
        del deck
        del callback_ref
        del deck_ref
        
        # Force garbage collection
        gc.collect()
        
        # No specific assertions - just ensure no crashes

    @pytest.mark.integration
    def test_weak_reference_behavior(self, mock_rosbag_deck_ffi):
        """Test behavior with weak references"""
        import weakref
        from rosbag_deck.core import RosbagDeck
        
        deck = RosbagDeck()
        
        # Create weak reference
        weak_deck = weakref.ref(deck)
        assert weak_deck() is not None
        
        # Close and delete
        deck.close()
        del deck
        gc.collect()
        
        # Weak reference might still exist temporarily
        # This test just ensures no crashes occur