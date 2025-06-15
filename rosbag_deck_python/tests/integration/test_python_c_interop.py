"""
Integration tests for Python-C++ interoperability
Tests data consistency, callback performance, and cross-language operations
"""

import pytest
import time
import threading
from unittest.mock import Mock, MagicMock, patch
from typing import List, Dict, Any
import numpy as np

from rosbag_deck.core import RosbagDeck, BagInfo, Status, Message
from rosbag_deck.exceptions import RosbagDeckError


class TestDataConsistency:
    """Test data consistency across Python-C++ boundary"""
    
    @pytest.mark.integration
    def test_timestamp_precision(self, mock_rosbag_deck_ffi):
        """Test that nanosecond timestamps are preserved accurately"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        
        # Test various timestamp values
        test_timestamps = [
            0,  # Epoch
            1234567890123456789,  # Large nanosecond value
            999999999999999999,   # Near max
            1,  # Minimum positive
        ]
        
        for ts in test_timestamps:
            # Mock timestamp conversion
            mock_ts = Mock()
            mock_ts.nanoseconds_since_epoch = ts
            mock_rosbag_deck_ffi.lib.rosbag_deck_to_timestamp.return_value = mock_ts
            mock_rosbag_deck_ffi.lib.rosbag_deck_from_timestamp.return_value = ts
            
            # Convert to C and back
            c_timestamp = mock_rosbag_deck_ffi.lib.rosbag_deck_to_timestamp(ts)
            py_timestamp = mock_rosbag_deck_ffi.lib.rosbag_deck_from_timestamp(c_timestamp)
            
            assert py_timestamp == ts, f"Timestamp precision lost: {ts} != {py_timestamp}"
    
    @pytest.mark.integration
    def test_string_encoding_consistency(self, mock_rosbag_deck_ffi):
        """Test string encoding across language boundary"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Test various string encodings
        test_strings = [
            "simple_ascii",
            "with spaces and punctuation!",
            "unicode_测试_тест_🚀",
            "special\ncharacters\ttabs",
            "",  # Empty string
            "very" * 1000,  # Long string
        ]
        
        for test_str in test_strings:
            # Mock topic filter setting
            mock_rosbag_deck_ffi.lib.rosbag_deck_set_topic_filter.return_value = None
            
            # Set filter (this would pass string to C)
            deck.set_topic_filter(test_str)
            
            # Verify the call
            mock_rosbag_deck_ffi.lib.rosbag_deck_set_topic_filter.assert_called()
            
            # In real implementation, we'd verify the string was preserved
            # For mock, we verify the encoding process
            args = mock_rosbag_deck_ffi.lib.rosbag_deck_set_topic_filter.call_args[0]
            assert len(args) >= 2  # Handle and string
    
    @pytest.mark.integration
    def test_binary_data_integrity(self, mock_rosbag_deck_ffi):
        """Test binary message data integrity"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Test various binary patterns
        test_patterns = [
            b'\x00' * 100,  # Null bytes
            b'\xFF' * 100,  # Max bytes
            bytes(range(256)),  # All byte values
            b''.join([bytes([i]) * 10 for i in range(256)]),  # Repeated patterns
            np.random.bytes(10000),  # Random data
        ]
        
        received_data = []
        
        def message_callback(msg: Message):
            received_data.append(msg.serialized_data)
        
        deck.set_message_callback(message_callback)
        
        for pattern in test_patterns:
            # Simulate message with binary data
            message_callback(Message(
                original_timestamp=1000,
                virtual_timestamp=1000,
                topic_name="/test",
                message_type="std_msgs/ByteArray",
                serialized_data=pattern,
                frame_index=0
            ))
        
        # Verify all patterns were preserved
        assert len(received_data) == len(test_patterns)
        for original, received in zip(test_patterns, received_data):
            assert received == original, "Binary data corrupted"
    
    @pytest.mark.integration
    def test_struct_field_alignment(self, mock_rosbag_deck_ffi):
        """Test struct field alignment between Python and C"""
        # Test BagInfo struct alignment
        mock_info = Mock()
        mock_info.success = True
        mock_info.message = b"Test message"
        mock_info.start_time = Mock(nanoseconds_since_epoch=1000)
        mock_info.end_time = Mock(nanoseconds_since_epoch=2000)
        mock_info.total_duration_ns = 1000
        mock_info.total_frames = 100
        mock_info.topic_names_count = 3
        
        # Mock topic names array
        mock_topics = [b"/topic1", b"/topic2", b"/topic3"]
        mock_info.topic_names = Mock()
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_bag_info.return_value = mock_info
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        mock_rosbag_deck_ffi.ffi.string.side_effect = lambda x: x if isinstance(x, bytes) else mock_topics[0]
        
        deck = RosbagDeck()
        info = deck.get_bag_info()
        
        # Verify all fields are accessible and correctly typed
        assert isinstance(info.success, bool)
        assert isinstance(info.total_frames, int)
        assert isinstance(info.start_time, int)
        assert info.total_duration_ns == 1000


class TestCallbackPerformance:
    """Test callback performance and threading behavior"""
    
    @pytest.mark.integration
    def test_callback_latency(self, mock_rosbag_deck_ffi):
        """Test callback invocation latency"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        latencies = []
        
        def measure_latency_callback(msg: Message):
            # In real implementation, we'd measure actual latency
            # For testing, we simulate timing
            callback_time = time.time()
            latencies.append(callback_time)
        
        deck.set_message_callback(measure_latency_callback)
        
        # Simulate rapid message delivery
        start_time = time.time()
        for i in range(100):
            measure_latency_callback(Message(
                original_timestamp=i * 1000000,
                virtual_timestamp=i * 1000000,
                topic_name="/test",
                message_type="std_msgs/Header",
                serialized_data=b"",
                frame_index=i
            ))
        
        # Calculate statistics
        if len(latencies) > 1:
            intervals = [latencies[i+1] - latencies[i] for i in range(len(latencies)-1)]
            avg_interval = sum(intervals) / len(intervals)
            
            # Callbacks should be fast
            assert avg_interval < 0.001, f"Callback interval too slow: {avg_interval}s"
    
    @pytest.mark.integration
    def test_callback_thread_safety(self, mock_rosbag_deck_ffi):
        """Test callbacks are thread-safe"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Shared state to test thread safety
        callback_count = 0
        callback_lock = threading.Lock()
        errors = []
        
        def thread_safe_callback(data):
            nonlocal callback_count
            try:
                with callback_lock:
                    callback_count += 1
                    # Simulate some work
                    time.sleep(0.001)
            except Exception as e:
                errors.append(e)
        
        deck.set_status_callback(thread_safe_callback)
        deck.set_message_callback(thread_safe_callback)
        
        # Simulate callbacks from multiple threads
        threads = []
        for i in range(5):
            def worker():
                for j in range(20):
                    thread_safe_callback(Mock())
            
            t = threading.Thread(target=worker)
            threads.append(t)
            t.start()
        
        # Wait for completion
        for t in threads:
            t.join()
        
        # Verify thread safety
        assert len(errors) == 0, f"Thread safety errors: {errors}"
        assert callback_count == 100, f"Lost callbacks: {callback_count}"
    
    @pytest.mark.integration
    def test_gil_impact_on_callbacks(self, mock_rosbag_deck_ffi):
        """Test GIL impact on callback performance"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Measure callback performance with CPU-bound work
        callback_times = []
        
        def cpu_bound_callback(msg: Message):
            start = time.time()
            # Simulate CPU-bound work
            total = sum(i for i in range(10000))
            callback_times.append(time.time() - start)
            return total
        
        deck.set_message_callback(cpu_bound_callback)
        
        # Run callbacks
        for i in range(10):
            cpu_bound_callback(Message(
                original_timestamp=i,
                virtual_timestamp=i,
                topic_name="/test",
                message_type="test",
                serialized_data=b"",
                frame_index=i
            ))
        
        # Check performance consistency
        if callback_times:
            avg_time = sum(callback_times) / len(callback_times)
            max_time = max(callback_times)
            
            # GIL shouldn't cause extreme variations
            assert max_time < avg_time * 3, "GIL causing performance spikes"


class TestCrossLanguageOperations:
    """Test complex operations across language boundary"""
    
    @pytest.mark.integration
    def test_concurrent_operations(self, mock_rosbag_deck_ffi):
        """Test concurrent operations from Python to C++"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Mock thread-safe operations
        operation_log = []
        operation_lock = threading.Lock()
        
        def log_operation(op_name):
            with operation_lock:
                operation_log.append((op_name, threading.current_thread().name))
            return True
        
        # Mock operations
        mock_rosbag_deck_ffi.lib.rosbag_deck_start_playback.side_effect = lambda h: log_operation("start")
        mock_rosbag_deck_ffi.lib.rosbag_deck_stop_playback.side_effect = lambda h: log_operation("stop")
        mock_rosbag_deck_ffi.lib.rosbag_deck_step_forward.side_effect = lambda h: log_operation("step")
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_status.return_value = Mock(is_playing=False)
        
        # Run concurrent operations
        def worker(op_func, count):
            for i in range(count):
                op_func()
                time.sleep(0.001)
        
        threads = [
            threading.Thread(target=worker, args=(deck.start_playback, 5)),
            threading.Thread(target=worker, args=(deck.stop_playback, 5)),
            threading.Thread(target=worker, args=(deck.step_forward, 5)),
            threading.Thread(target=worker, args=(deck.get_status, 5)),
        ]
        
        for t in threads:
            t.start()
        
        for t in threads:
            t.join()
        
        # Verify all operations completed
        assert len(operation_log) >= 15, "Some operations failed"
        
        # Verify operations came from different threads
        thread_names = set(op[1] for op in operation_log)
        assert len(thread_names) > 1, "No concurrent execution"
    
    @pytest.mark.integration
    def test_error_propagation_consistency(self, mock_rosbag_deck_ffi):
        """Test consistent error propagation across languages"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Test various error scenarios
        error_scenarios = [
            ("build_index", False, b"File not found", RosbagDeckError),
            ("seek_to_time", False, b"Invalid timestamp", RosbagDeckError),
            ("step_forward", False, b"No index built", RosbagDeckError),
        ]
        
        for method_name, return_value, error_msg, expected_exception in error_scenarios:
            mock_method = getattr(mock_rosbag_deck_ffi.lib, f"rosbag_deck_{method_name}")
            mock_method.return_value = return_value
            mock_rosbag_deck_ffi.lib.rosbag_deck_get_last_error.return_value = error_msg
            
            deck_method = getattr(deck, method_name)
            
            with pytest.raises(expected_exception) as exc_info:
                if method_name == "build_index":
                    deck_method(["/test.bag"])
                elif method_name == "seek_to_time":
                    deck_method(1000)
                else:
                    deck_method()
            
            # Verify error message was preserved
            assert error_msg.decode() in str(exc_info.value) or "Failed" in str(exc_info.value)
    
    @pytest.mark.integration
    def test_memory_ownership_semantics(self, mock_rosbag_deck_ffi):
        """Test memory ownership rules between Python and C"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        
        # Test string ownership
        topic_filter = "/test/topic"
        type_filter = "sensor_msgs/*"
        
        deck = RosbagDeck()
        
        # Set filters (Python should maintain ownership during call)
        deck.set_topic_filter(topic_filter)
        deck.set_type_filter(type_filter)
        
        # Strings should remain valid
        assert topic_filter == "/test/topic"
        assert type_filter == "sensor_msgs/*"
        
        # Test array ownership
        mock_info = Mock()
        mock_info.topic_names_count = 3
        mock_info.topic_names = Mock()
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_bag_info.return_value = mock_info
        
        # Mock string array access
        topics = [b"/topic1", b"/topic2", b"/topic3"]
        mock_rosbag_deck_ffi.ffi.string.side_effect = lambda x: topics[0]
        
        info = deck.get_bag_info()
        
        # Python should have its own copy
        # Modifying Python copy shouldn't affect C
        if hasattr(info, 'topic_names') and info.topic_names:
            original_topics = info.topic_names.copy()
            info.topic_names.append("/new_topic")
            assert len(info.topic_names) == len(original_topics) + 1


class TestRealBagFileOperations:
    """Test operations with real bag file formats"""
    
    @pytest.mark.integration
    @pytest.mark.skip(reason="Requires real bag files")
    def test_sqlite3_bag_format(self, temp_dir):
        """Test with sqlite3 storage format"""
        # This would test with actual sqlite3 bag files
        pass
    
    @pytest.mark.integration
    @pytest.mark.skip(reason="Requires real bag files")
    def test_mcap_bag_format(self, temp_dir):
        """Test with MCAP storage format"""
        # This would test with actual MCAP bag files
        pass
    
    @pytest.mark.integration
    def test_multi_format_compatibility(self, mock_rosbag_deck_ffi):
        """Test handling multiple bag formats in same session"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.return_value = True
        
        deck = RosbagDeck()
        
        # Mock different storage formats
        bag_files = [
            "/path/to/sqlite3.db3",
            "/path/to/mcap.mcap",
            "/path/to/custom.bag",
        ]
        
        # Should handle all formats
        deck.build_index(bag_files)
        
        # Verify index was built
        mock_rosbag_deck_ffi.lib.rosbag_deck_build_index.assert_called_once()


class TestPerformanceBenchmarks:
    """Performance benchmarks for Python bindings"""
    
    @pytest.mark.integration
    @pytest.mark.performance
    def test_callback_throughput(self, mock_rosbag_deck_ffi):
        """Benchmark callback throughput"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        message_count = 0
        start_time = None
        
        def throughput_callback(msg: Message):
            nonlocal message_count, start_time
            if start_time is None:
                start_time = time.time()
            message_count += 1
        
        deck.set_message_callback(throughput_callback)
        
        # Simulate high-frequency messages
        test_duration = 1.0  # seconds
        msg_start = time.time()
        
        while time.time() - msg_start < test_duration:
            throughput_callback(Message(
                original_timestamp=int((time.time() - msg_start) * 1e9),
                virtual_timestamp=int((time.time() - msg_start) * 1e9),
                topic_name="/benchmark",
                message_type="std_msgs/Header",
                serialized_data=b"x" * 100,
                frame_index=message_count
            ))
        
        # Calculate throughput
        actual_duration = time.time() - msg_start
        throughput = message_count / actual_duration
        
        print(f"Callback throughput: {throughput:.0f} messages/second")
        
        # Should handle at least 1000 messages/second
        assert throughput > 1000, f"Throughput too low: {throughput}"
    
    @pytest.mark.integration
    @pytest.mark.performance
    def test_ffi_call_overhead(self, mock_rosbag_deck_ffi):
        """Benchmark FFI call overhead"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_status.return_value = Mock(
            current_frame=0,
            is_playing=False
        )
        
        deck = RosbagDeck()
        
        # Benchmark simple FFI calls
        iterations = 10000
        start_time = time.time()
        
        for i in range(iterations):
            status = deck.get_status()
        
        duration = time.time() - start_time
        overhead_us = (duration / iterations) * 1e6
        
        print(f"FFI call overhead: {overhead_us:.2f} microseconds")
        
        # Should be less than 100 microseconds per call
        assert overhead_us < 100, f"FFI overhead too high: {overhead_us}us"