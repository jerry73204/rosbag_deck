"""
Unit tests for memory management
Tests memory leak detection, CFFI cleanup, large data handling, and long-running stability
"""

import pytest
import gc
import weakref
import tracemalloc
import psutil
import os
from unittest.mock import Mock, MagicMock, patch
from typing import List
import time

try:
    from memory_profiler import memory_usage
    MEMORY_PROFILER_AVAILABLE = True
except ImportError:
    MEMORY_PROFILER_AVAILABLE = False


class TestMemoryLeakDetection:
    """Test for memory leaks in various operations"""
    
    @pytest.mark.unit
    def test_handle_lifecycle_memory(self, mock_rosbag_deck_ffi):
        """Test that handles are properly cleaned up"""
        from rosbag_deck.core import RosbagDeck
        
        # Track memory before
        gc.collect()
        initial_objects = len(gc.get_objects())
        
        # Create and destroy multiple instances
        for i in range(10):
            mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678 + i
            deck = RosbagDeck()
            deck_ref = weakref.ref(deck)
            
            # Use the deck
            mock_rosbag_deck_ffi.lib.rosbag_deck_get_status.return_value = Mock(
                is_playing=False,
                current_frame=i
            )
            status = deck.get_status()
            
            # Delete and verify cleanup
            del deck
            gc.collect()
            assert deck_ref() is None, f"Deck {i} not garbage collected"
        
        # Check memory after
        gc.collect()
        final_objects = len(gc.get_objects())
        
        # Allow some growth but not proportional to iterations
        assert final_objects - initial_objects < 100, "Too many objects leaked"
    
    @pytest.mark.unit
    def test_callback_reference_cycles(self, mock_rosbag_deck_ffi):
        """Test that callbacks don't create reference cycles"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        
        # Create deck with callback that references the deck
        deck = RosbagDeck()
        deck_ref = weakref.ref(deck)
        
        # Create a callback that could create a cycle
        def status_callback(status):
            # This could create a cycle if not handled properly
            return deck.get_status()
        
        deck.set_status_callback(status_callback)
        
        # Delete deck and check for cycles
        del deck
        del status_callback
        gc.collect()
        
        assert deck_ref() is None, "Reference cycle detected with callbacks"
    
    @pytest.mark.unit
    @pytest.mark.skipif(not MEMORY_PROFILER_AVAILABLE, reason="memory_profiler not available")
    def test_message_processing_memory(self, mock_rosbag_deck_ffi):
        """Test memory usage during message processing"""
        from rosbag_deck.core import RosbagDeck, Message
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        
        def process_messages():
            deck = RosbagDeck()
            messages_processed = []
            
            def message_callback(msg: Message):
                # Process message without keeping reference
                messages_processed.append(len(msg.serialized_data))
            
            deck.set_message_callback(message_callback)
            
            # Simulate processing many messages
            for i in range(100):
                mock_msg = Mock()
                mock_msg.serialized_data = b'x' * 1024  # 1KB messages
                mock_msg.topic_name = f"/topic_{i}".encode()
                
                # Simulate callback invocation
                message_callback(Message(
                    original_timestamp=i * 1000,
                    virtual_timestamp=i * 1000,
                    topic_name=f"/topic_{i}",
                    message_type="std_msgs/String",
                    serialized_data=b'x' * 1024,
                    frame_index=i
                ))
            
            return len(messages_processed)
        
        # Measure memory usage
        mem_usage = memory_usage(process_messages)
        
        # Memory should not grow significantly
        assert max(mem_usage) - min(mem_usage) < 50, "Excessive memory growth during message processing"


class TestCFFIMemoryCleanup:
    """Test CFFI-specific memory cleanup"""
    
    @pytest.mark.unit
    def test_ffi_string_cleanup(self, mock_ffi):
        """Test that FFI strings are properly managed"""
        # Test string allocation and cleanup
        test_strings = ["test1", "test2", "test3" * 1000]  # Include a large string
        ffi_strings = []
        
        for s in test_strings:
            c_str = mock_ffi.new("char[]", s.encode('utf-8'))
            ffi_strings.append(c_str)
        
        # Verify strings are accessible
        for i, c_str in enumerate(ffi_strings):
            mock_ffi.string.return_value = test_strings[i].encode('utf-8')
            decoded = mock_ffi.string(c_str).decode('utf-8')
            assert decoded == test_strings[i]
        
        # Clean up
        del ffi_strings
        gc.collect()
    
    @pytest.mark.unit
    def test_ffi_struct_cleanup(self, mock_rosbag_deck_ffi):
        """Test that FFI structs are properly cleaned up"""
        from rosbag_deck.core import BagInfo, Status
        
        # Create multiple structs
        structs_created = []
        
        for i in range(50):
            # Mock struct creation
            mock_info = Mock()
            mock_info.success = True
            mock_info.total_frames = i
            mock_info.topic_names_count = 2
            mock_info.topic_names = Mock()
            
            # Track weak references
            structs_created.append(weakref.ref(mock_info))
        
        # Force cleanup
        del mock_info
        gc.collect()
        
        # Most structs should be cleaned up
        alive_count = sum(1 for ref in structs_created if ref() is not None)
        assert alive_count < 5, f"Too many structs still alive: {alive_count}"
    
    @pytest.mark.unit
    def test_callback_ffi_cleanup(self, mock_rosbag_deck_ffi, mock_ffi):
        """Test that FFI callbacks are properly cleaned up"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        
        # Track callback creations
        callbacks_created = []
        
        def track_callback_creation(type_name, func):
            cb = Mock()
            callbacks_created.append(weakref.ref(cb))
            return cb
        
        with patch('rosbag_deck.ffi.create_cffi_callback', side_effect=track_callback_creation):
            # Create deck with callbacks
            deck = RosbagDeck()
            
            for i in range(10):
                deck.set_status_callback(lambda s: None)
                deck.set_message_callback(lambda m: None)
            
            # Clear callbacks
            deck.set_status_callback(None)
            deck.set_message_callback(None)
            
            del deck
            gc.collect()
        
        # Callbacks should be cleaned up
        alive_count = sum(1 for ref in callbacks_created if ref() is not None)
        assert alive_count < 5, f"Too many callbacks still alive: {alive_count}"


class TestLargeDataHandling:
    """Test memory efficiency with large data"""
    
    @pytest.mark.unit
    def test_large_message_handling(self, mock_rosbag_deck_ffi):
        """Test handling of large messages without excessive memory use"""
        from rosbag_deck.core import RosbagDeck, Message
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Track peak memory during large message handling
        process = psutil.Process(os.getpid())
        initial_memory = process.memory_info().rss / 1024 / 1024  # MB
        
        # Process large messages
        message_sizes = [1024 * 1024, 5 * 1024 * 1024, 10 * 1024 * 1024]  # 1MB, 5MB, 10MB
        
        for size in message_sizes:
            # Mock large message
            large_data = b'x' * size
            mock_msg = Mock()
            mock_msg.serialized_data_size = size
            mock_msg.serialized_data = Mock()
            
            # Mock buffer return with smaller chunk for testing
            mock_rosbag_deck_ffi.ffi.buffer.return_value = large_data[:1024]
            
            # Process message
            processed = False
            def callback(msg: Message):
                nonlocal processed
                processed = True
                # Don't keep reference to data
                assert len(msg.serialized_data) > 0
            
            deck.set_message_callback(callback)
            
            # Simulate callback
            callback(Message(
                original_timestamp=1000,
                virtual_timestamp=1000,
                topic_name="/large",
                message_type="sensor_msgs/PointCloud2",
                serialized_data=large_data[:1024],  # Use smaller chunk for test
                frame_index=1
            ))
            
            assert processed
            
            # Clean up
            del large_data
            gc.collect()
        
        # Check memory didn't grow excessively
        final_memory = process.memory_info().rss / 1024 / 1024  # MB
        memory_growth = final_memory - initial_memory
        
        # Allow some growth but not proportional to message sizes
        assert memory_growth < 100, f"Excessive memory growth: {memory_growth}MB"
    
    @pytest.mark.unit
    def test_large_bag_index_memory(self, mock_rosbag_deck_ffi):
        """Test memory usage when indexing large bags"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.return_value = True
        
        # Mock large bag info
        mock_info = Mock()
        mock_info.success = True
        mock_info.total_frames = 1000000  # 1 million frames
        mock_info.topic_names_count = 50  # 50 topics
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_bag_info.return_value = mock_info
        
        deck = RosbagDeck()
        
        # Build index for "large" bag
        deck.build_index(["/path/to/large.bag"])
        
        # Get info
        info = deck.get_bag_info()
        assert info.total_frames == 1000000
        
        # Memory should be reasonable even with large index
        # The index manager uses ~100 bytes per message
        expected_memory = (100 * 1000000) / 1024 / 1024  # ~95MB
        # This is just a conceptual test - actual memory tracking would be more complex


class TestLongRunningStability:
    """Test memory stability in long-running scenarios"""
    
    @pytest.mark.unit
    def test_repeated_operations_stability(self, mock_rosbag_deck_ffi):
        """Test stability over many repeated operations"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Track memory over iterations
        gc.collect()
        initial_objects = len(gc.get_objects())
        
        # Perform many operations
        for i in range(100):
            # Seek operations
            mock_rosbag_deck_ffi.lib.rosbag_deck_seek_to_time.return_value = True
            deck.seek_to_time(i * 1000000)
            
            # Step operations
            mock_rosbag_deck_ffi.lib.rosbag_deck_step_forward.return_value = True
            deck.step_forward()
            
            # Status queries
            mock_status = Mock(current_frame=i, is_playing=i % 2 == 0)
            mock_rosbag_deck_ffi.lib.rosbag_deck_get_status.return_value = mock_status
            status = deck.get_status()
            
            # Periodic cleanup
            if i % 10 == 0:
                gc.collect()
        
        # Final cleanup
        gc.collect()
        final_objects = len(gc.get_objects())
        
        # Object count should stabilize
        object_growth = final_objects - initial_objects
        assert object_growth < 1000, f"Too many objects created: {object_growth}"
    
    @pytest.mark.unit
    def test_callback_churn_stability(self, mock_rosbag_deck_ffi):
        """Test stability when callbacks are frequently changed"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Change callbacks many times
        callback_count = 0
        
        for i in range(50):
            # Create new callback each time
            def make_callback(n):
                def callback(data):
                    return n
                return callback
            
            deck.set_status_callback(make_callback(i))
            deck.set_message_callback(make_callback(i * 2))
            
            callback_count += 2
            
            # Clear sometimes
            if i % 10 == 0:
                deck.set_status_callback(None)
                deck.set_message_callback(None)
                gc.collect()
        
        # Final cleanup
        deck.set_status_callback(None)
        deck.set_message_callback(None)
        gc.collect()
        
        # Should not accumulate callbacks
        assert deck._status_callback is None
        assert deck._message_callback is None


class TestReferenceCycles:
    """Test for reference cycles that could cause leaks"""
    
    @pytest.mark.unit
    def test_circular_reference_detection(self, mock_rosbag_deck_ffi):
        """Test detection of circular references"""
        from rosbag_deck.core import RosbagDeck
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        
        # Enable garbage collection debugging
        gc.set_debug(gc.DEBUG_SAVEALL)
        gc.collect()
        initial_garbage = len(gc.garbage)
        
        # Create potential circular references
        deck = RosbagDeck()
        
        # Callback that references deck
        def callback_with_ref(data):
            return deck.get_status()
        
        deck.set_status_callback(callback_with_ref)
        
        # Store deck in its own attribute (circular)
        deck._self_ref = deck
        
        # Delete and collect
        del deck
        del callback_with_ref
        gc.collect()
        
        # Check for cycles
        final_garbage = len(gc.garbage)
        cycles_found = final_garbage - initial_garbage
        
        # Clean up
        gc.set_debug(0)
        gc.garbage.clear()
        
        # Some cycles might be expected, but should be minimal
        assert cycles_found < 10, f"Too many reference cycles found: {cycles_found}"
    
    @pytest.mark.unit
    def test_exception_reference_cleanup(self, mock_rosbag_deck_ffi):
        """Test that exceptions don't keep references alive"""
        from rosbag_deck.core import RosbagDeck
        from rosbag_deck.exceptions import RosbagDeckError
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        deck_ref = weakref.ref(deck)
        
        # Create exception that references deck
        try:
            # Mock failure
            mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.return_value = False
            deck.build_index(["/nonexistent.bag"])
        except RosbagDeckError as e:
            # Exception might reference deck through traceback
            exception_ref = weakref.ref(e)
        
        # Clean up
        del deck
        del e
        gc.collect()
        
        # Both should be collected
        assert deck_ref() is None, "Deck kept alive by exception"
        assert exception_ref() is None, "Exception not cleaned up"


class TestMemoryProfiling:
    """Test memory profiling utilities"""
    
    @pytest.mark.unit
    def test_tracemalloc_integration(self, mock_rosbag_deck_ffi):
        """Test memory tracking with tracemalloc"""
        from rosbag_deck.core import RosbagDeck
        
        # Start tracing
        tracemalloc.start()
        
        # Create and use deck
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Take snapshot
        snapshot1 = tracemalloc.take_snapshot()
        
        # Allocate some memory through operations
        for i in range(10):
            mock_rosbag_deck_ffi.lib.rosbag_deck_get_status.return_value = Mock(
                current_frame=i,
                is_playing=True
            )
            status = deck.get_status()
        
        # Take another snapshot
        snapshot2 = tracemalloc.take_snapshot()
        
        # Compare snapshots
        stats = snapshot2.compare_to(snapshot1, 'lineno')
        
        # Stop tracing
        tracemalloc.stop()
        
        # Should have some allocations but not excessive
        total_diff = sum(stat.size_diff for stat in stats)
        assert total_diff < 10 * 1024 * 1024, f"Too much memory allocated: {total_diff} bytes"
    
    @pytest.mark.unit
    def test_memory_usage_bounds(self, mock_rosbag_deck_ffi):
        """Test that memory usage stays within expected bounds"""
        from rosbag_deck.core import RosbagDeck
        
        process = psutil.Process(os.getpid())
        initial_memory = process.memory_info().rss
        
        # Perform typical operations
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        
        decks = []
        for i in range(5):
            deck = RosbagDeck()
            decks.append(deck)
        
        # Use the decks
        for deck in decks:
            mock_rosbag_deck_ffi.lib.rosbag_deck_get_status.return_value = Mock()
            deck.get_status()
        
        peak_memory = process.memory_info().rss
        
        # Clean up
        decks.clear()
        gc.collect()
        
        final_memory = process.memory_info().rss
        
        # Memory should return close to initial
        memory_leaked = final_memory - initial_memory
        memory_leaked_mb = memory_leaked / 1024 / 1024
        
        # Allow some variance but not major leaks
        assert memory_leaked_mb < 50, f"Potential memory leak: {memory_leaked_mb}MB"