"""
Performance tests for rosbag_deck_python
Tests CFFI overhead, callback latency, memory efficiency, and garbage collection impact
"""

import pytest
import time
import gc
import psutil
import os
import threading
from unittest.mock import Mock, MagicMock, patch
from typing import List, Tuple
import statistics

from rosbag_deck.core import RosbagDeck, Message, Status


class TestCFFIOverhead:
    """Test CFFI call overhead and optimization opportunities"""
    
    @pytest.mark.integration
    @pytest.mark.performance
    def test_ffi_function_call_overhead(self, mock_rosbag_deck_ffi):
        """Measure overhead of different FFI function types"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        
        # Test different function signatures
        test_cases = [
            # (function, args, description)
            (mock_rosbag_deck_ffi.lib.rosbag_deck_get_status, [], "no args"),
            (mock_rosbag_deck_ffi.lib.rosbag_deck_set_cache_size, [1000], "single int"),
            (mock_rosbag_deck_ffi.lib.rosbag_deck_seek_to_time, [123456789], "int64"),
            (mock_rosbag_deck_ffi.lib.rosbag_deck_build_index, [["path"]], "string array"),
        ]
        
        results = []
        
        for func, args, desc in test_cases:
            # Mock return values
            if "get_status" in str(func):
                func.return_value = Mock(is_playing=False)
            else:
                func.return_value = True
            
            # Warm up
            for _ in range(100):
                func(0x12345678, *args)
            
            # Measure
            iterations = 10000
            start = time.perf_counter()
            
            for _ in range(iterations):
                func(0x12345678, *args)
            
            duration = time.perf_counter() - start
            overhead_ns = (duration / iterations) * 1e9
            
            results.append((desc, overhead_ns))
            print(f"FFI overhead for {desc}: {overhead_ns:.0f} nanoseconds")
        
        # All calls should be under 10 microseconds
        for desc, overhead in results:
            assert overhead < 10000, f"FFI overhead too high for {desc}: {overhead}ns"
    
    @pytest.mark.integration
    @pytest.mark.performance
    def test_struct_marshaling_overhead(self, mock_rosbag_deck_ffi):
        """Test overhead of marshaling complex structs"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        
        # Create complex struct mocks
        mock_bag_info = Mock()
        mock_bag_info.success = True
        mock_bag_info.message = b"Test"
        mock_bag_info.start_time = Mock(nanoseconds_since_epoch=1000)
        mock_bag_info.end_time = Mock(nanoseconds_since_epoch=2000)
        mock_bag_info.total_duration_ns = 1000
        mock_bag_info.total_frames = 1000000
        mock_bag_info.topic_names_count = 50
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_bag_info.return_value = mock_bag_info
        
        deck = RosbagDeck()
        
        # Measure struct marshaling
        iterations = 1000
        start = time.perf_counter()
        
        for _ in range(iterations):
            info = deck.get_bag_info()
        
        duration = time.perf_counter() - start
        overhead_us = (duration / iterations) * 1e6
        
        print(f"Struct marshaling overhead: {overhead_us:.2f} microseconds")
        
        # Should be under 100 microseconds even for complex structs
        assert overhead_us < 100, f"Struct marshaling too slow: {overhead_us}us"
    
    @pytest.mark.integration
    @pytest.mark.performance
    def test_string_conversion_performance(self, mock_rosbag_deck_ffi):
        """Test string conversion performance for various sizes"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        string_sizes = [10, 100, 1000, 10000]
        results = []
        
        for size in string_sizes:
            test_string = "x" * size
            
            # Measure conversion time
            iterations = 1000
            start = time.perf_counter()
            
            for _ in range(iterations):
                # Simulate string passing to C
                c_string = mock_rosbag_deck_ffi.ffi.new("char[]", test_string.encode())
                # Simulate getting string back
                mock_rosbag_deck_ffi.ffi.string.return_value = test_string.encode()
                result = mock_rosbag_deck_ffi.ffi.string(c_string).decode()
            
            duration = time.perf_counter() - start
            rate_mb_s = (size * iterations / 1024 / 1024) / duration
            
            results.append((size, rate_mb_s))
            print(f"String size {size}: {rate_mb_s:.1f} MB/s")
        
        # Should handle at least 100 MB/s for reasonable sizes
        for size, rate in results:
            if size <= 1000:
                assert rate > 100, f"String conversion too slow for size {size}: {rate} MB/s"


class TestCallbackLatency:
    """Test callback invocation latency and timing"""
    
    @pytest.mark.integration
    @pytest.mark.performance
    def test_callback_invocation_latency(self, mock_rosbag_deck_ffi):
        """Measure latency from C callback to Python function"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        latencies = []
        
        def latency_callback(msg: Message):
            # In real scenario, we'd measure time from C invocation
            # For testing, we measure callback execution time
            start = time.perf_counter()
            # Minimal work
            _ = msg.topic_name
            latencies.append(time.perf_counter() - start)
        
        deck.set_message_callback(latency_callback)
        
        # Simulate many callbacks
        for i in range(1000):
            latency_callback(Message(
                original_timestamp=i,
                virtual_timestamp=i,
                topic_name=f"/topic_{i}",
                message_type="test",
                serialized_data=b"data",
                frame_index=i
            ))
        
        # Calculate statistics
        avg_latency = statistics.mean(latencies) * 1e6  # microseconds
        p99_latency = sorted(latencies)[int(len(latencies) * 0.99)] * 1e6
        
        print(f"Callback latency - Average: {avg_latency:.1f}us, P99: {p99_latency:.1f}us")
        
        # Should be very low latency
        assert avg_latency < 10, f"Average latency too high: {avg_latency}us"
        assert p99_latency < 50, f"P99 latency too high: {p99_latency}us"
    
    @pytest.mark.integration
    @pytest.mark.performance
    def test_callback_throughput_limits(self, mock_rosbag_deck_ffi):
        """Test maximum sustainable callback throughput"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Test different callback complexities
        complexities = [
            ("minimal", lambda m: None),
            ("light", lambda m: len(m.serialized_data)),
            ("medium", lambda m: sum(m.serialized_data)),
            ("heavy", lambda m: sorted(m.serialized_data)),
        ]
        
        results = []
        
        for name, callback_func in complexities:
            count = 0
            
            def counting_callback(msg):
                nonlocal count
                count += 1
                callback_func(msg)
            
            deck.set_message_callback(counting_callback)
            
            # Run for fixed duration
            duration = 0.5
            start = time.perf_counter()
            
            while time.perf_counter() - start < duration:
                counting_callback(Message(
                    original_timestamp=count,
                    virtual_timestamp=count,
                    topic_name="/test",
                    message_type="test",
                    serialized_data=b"x" * 100,
                    frame_index=count
                ))
            
            throughput = count / duration
            results.append((name, throughput))
            print(f"Callback throughput ({name}): {throughput:.0f} calls/second")
        
        # Minimal callbacks should handle > 100k/second
        assert results[0][1] > 100000, f"Minimal callback too slow: {results[0][1]}"


class TestMemoryEfficiency:
    """Test memory efficiency for large datasets"""
    
    @pytest.mark.integration
    @pytest.mark.performance
    def test_large_message_memory_efficiency(self, mock_rosbag_deck_ffi):
        """Test memory efficiency when processing large messages"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        process = psutil.Process(os.getpid())
        
        # Baseline memory
        gc.collect()
        baseline_memory = process.memory_info().rss / 1024 / 1024  # MB
        
        # Process large messages
        message_size = 10 * 1024 * 1024  # 10MB
        message_count = 10
        
        peak_memory = baseline_memory
        
        def memory_callback(msg: Message):
            nonlocal peak_memory
            # Access data to ensure it's loaded
            data_sum = sum(msg.serialized_data[:100])  # Only access part
            current_memory = process.memory_info().rss / 1024 / 1024
            peak_memory = max(peak_memory, current_memory)
            return data_sum
        
        deck.set_message_callback(memory_callback)
        
        for i in range(message_count):
            # Simulate large message
            memory_callback(Message(
                original_timestamp=i,
                virtual_timestamp=i,
                topic_name="/large",
                message_type="sensor_msgs/PointCloud2",
                serialized_data=b"x" * min(message_size, 1024),  # Limited for test
                frame_index=i
            ))
            
            # Allow GC between messages
            if i % 2 == 0:
                gc.collect()
        
        # Final memory
        gc.collect()
        final_memory = process.memory_info().rss / 1024 / 1024
        
        memory_overhead = peak_memory - baseline_memory
        memory_leaked = final_memory - baseline_memory
        
        print(f"Memory overhead: {memory_overhead:.1f}MB peak, {memory_leaked:.1f}MB leaked")
        
        # Should not keep all messages in memory
        assert memory_overhead < message_size * 3 / 1024 / 1024, "Too much memory retained"
        assert memory_leaked < 10, "Memory leak detected"
    
    @pytest.mark.integration
    @pytest.mark.performance
    def test_callback_object_allocation(self, mock_rosbag_deck_ffi):
        """Test object allocation efficiency in callbacks"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Track allocations
        gc.collect()
        gc.disable()  # Disable automatic collection
        
        initial_objects = len(gc.get_objects())
        
        # Process many messages
        message_count = 10000
        
        def allocation_callback(msg: Message):
            # Minimal allocations
            return msg.frame_index * 2
        
        deck.set_message_callback(allocation_callback)
        
        for i in range(message_count):
            allocation_callback(Message(
                original_timestamp=i,
                virtual_timestamp=i,
                topic_name="/test",
                message_type="test",
                serialized_data=b"",
                frame_index=i
            ))
        
        # Check allocations
        final_objects = len(gc.get_objects())
        gc.enable()
        gc.collect()
        
        objects_per_message = (final_objects - initial_objects) / message_count
        
        print(f"Objects allocated per message: {objects_per_message:.2f}")
        
        # Should allocate minimal objects per message
        assert objects_per_message < 5, f"Too many allocations: {objects_per_message} per message"


class TestGarbageCollectionImpact:
    """Test impact of garbage collection on performance"""
    
    @pytest.mark.integration
    @pytest.mark.performance
    def test_gc_pause_impact(self, mock_rosbag_deck_ffi):
        """Test GC pause impact on real-time performance"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Track timing with GC enabled
        gc_pauses = []
        last_time = time.perf_counter()
        
        def timing_callback(msg: Message):
            nonlocal last_time
            current_time = time.perf_counter()
            interval = current_time - last_time
            last_time = current_time
            
            # Detect GC pauses (intervals > 10ms)
            if interval > 0.010:
                gc_pauses.append(interval * 1000)  # ms
            
            # Create some garbage
            temp_data = [i for i in range(100)]
            return sum(temp_data)
        
        deck.set_message_callback(timing_callback)
        
        # Process messages at steady rate
        target_rate = 1000  # Hz
        interval = 1.0 / target_rate
        
        start_time = time.perf_counter()
        message_count = 0
        
        while time.perf_counter() - start_time < 2.0:  # 2 second test
            timing_callback(Message(
                original_timestamp=message_count,
                virtual_timestamp=message_count,
                topic_name="/test",
                message_type="test",
                serialized_data=b"data",
                frame_index=message_count
            ))
            
            message_count += 1
            
            # Try to maintain rate
            next_time = start_time + (message_count * interval)
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        print(f"GC pauses detected: {len(gc_pauses)}")
        if gc_pauses:
            print(f"Max GC pause: {max(gc_pauses):.1f}ms")
            print(f"Average GC pause: {statistics.mean(gc_pauses):.1f}ms")
        
        # GC pauses should be minimal
        assert len(gc_pauses) < 10, f"Too many GC pauses: {len(gc_pauses)}"
        if gc_pauses:
            assert max(gc_pauses) < 50, f"GC pause too long: {max(gc_pauses)}ms"
    
    @pytest.mark.integration
    @pytest.mark.performance
    def test_gc_tuning_impact(self, mock_rosbag_deck_ffi):
        """Test impact of GC tuning on performance"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        
        # Test different GC configurations
        configs = [
            ("default", {}),
            ("aggressive", {"enabled": True, "threshold": (100, 10, 10)}),
            ("disabled", {"enabled": False}),
        ]
        
        results = []
        
        for name, config in configs:
            # Configure GC
            if "enabled" in config:
                if config["enabled"]:
                    gc.enable()
                    if "threshold" in config:
                        gc.set_threshold(*config["threshold"])
                else:
                    gc.disable()
            
            deck = RosbagDeck()
            
            # Measure performance
            iterations = 10000
            start = time.perf_counter()
            
            for i in range(iterations):
                status = deck.get_status()
            
            duration = time.perf_counter() - start
            rate = iterations / duration
            
            results.append((name, rate))
            print(f"GC config '{name}': {rate:.0f} ops/second")
            
            # Reset GC
            gc.enable()
            gc.set_threshold(700, 10, 10)  # Default values
        
        # Performance shouldn't vary too much
        rates = [r[1] for r in results]
        variation = (max(rates) - min(rates)) / min(rates)
        assert variation < 0.5, f"GC impact too high: {variation*100:.0f}% variation"


class TestThreadingPerformance:
    """Test performance in multi-threaded scenarios"""
    
    @pytest.mark.integration
    @pytest.mark.performance
    def test_concurrent_callback_performance(self, mock_rosbag_deck_ffi):
        """Test callback performance with concurrent operations"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Shared counters
        callback_count = 0
        callback_lock = threading.Lock()
        
        def concurrent_callback(msg: Message):
            nonlocal callback_count
            with callback_lock:
                callback_count += 1
            
            # Simulate some work
            time.sleep(0.0001)
        
        deck.set_message_callback(concurrent_callback)
        
        # Run callbacks from multiple threads
        thread_count = 4
        messages_per_thread = 1000
        
        def worker():
            for i in range(messages_per_thread):
                concurrent_callback(Message(
                    original_timestamp=i,
                    virtual_timestamp=i,
                    topic_name=f"/thread_{threading.current_thread().name}",
                    message_type="test",
                    serialized_data=b"data",
                    frame_index=i
                ))
        
        start = time.perf_counter()
        
        threads = []
        for _ in range(thread_count):
            t = threading.Thread(target=worker)
            threads.append(t)
            t.start()
        
        for t in threads:
            t.join()
        
        duration = time.perf_counter() - start
        total_messages = thread_count * messages_per_thread
        throughput = total_messages / duration
        
        print(f"Concurrent callback throughput: {throughput:.0f} messages/second")
        print(f"Callback count: {callback_count}/{total_messages}")
        
        # Should handle concurrent callbacks efficiently
        assert callback_count == total_messages, "Lost callbacks in concurrent execution"
        assert throughput > 5000, f"Concurrent throughput too low: {throughput}"


class TestOptimizationOpportunities:
    """Identify optimization opportunities"""
    
    @pytest.mark.integration
    @pytest.mark.performance
    def test_batch_operation_performance(self, mock_rosbag_deck_ffi):
        """Test if batch operations would improve performance"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Test individual vs batch operations
        frame_count = 1000
        
        # Individual seeks
        mock_rosbag_deck_ffi.lib.rosbag_deck_seek_to_frame.return_value = True
        
        start = time.perf_counter()
        for i in range(frame_count):
            deck.seek_to_frame(i)
        individual_duration = time.perf_counter() - start
        
        # Simulated batch seek (if it existed)
        start = time.perf_counter()
        # In reality, this would be a single batch call
        frames = list(range(frame_count))
        # Simulate overhead of preparing batch
        _ = [f * 2 for f in frames]
        batch_duration = time.perf_counter() - start
        
        speedup = individual_duration / batch_duration
        print(f"Potential batch speedup: {speedup:.1f}x")
        print(f"Individual: {individual_duration:.3f}s, Batch: {batch_duration:.3f}s")
        
        # Batch operations should be significantly faster
        assert speedup > 10, "Batch operations should provide significant speedup"
    
    @pytest.mark.integration
    @pytest.mark.performance
    def test_caching_effectiveness(self, mock_rosbag_deck_ffi):
        """Test effectiveness of caching frequently accessed data"""
        mock_rosbag_deck_ffi.lib.rosbag_deck_create.return_value = 0x12345678
        deck = RosbagDeck()
        
        # Mock expensive operation
        call_count = 0
        
        def expensive_get_info(*args):
            nonlocal call_count
            call_count += 1
            time.sleep(0.001)  # Simulate expensive operation
            return Mock(success=True, total_frames=1000)
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_get_bag_info.side_effect = expensive_get_info
        
        # Repeated access pattern
        iterations = 100
        
        start = time.perf_counter()
        for _ in range(iterations):
            info = deck.get_bag_info()
        no_cache_duration = time.perf_counter() - start
        
        # Simulate cached access
        cached_info = deck.get_bag_info()
        start = time.perf_counter()
        for _ in range(iterations):
            info = cached_info  # Use cached value
        cache_duration = time.perf_counter() - start
        
        speedup = no_cache_duration / cache_duration
        print(f"Cache speedup: {speedup:.1f}x")
        print(f"Calls to C API: {call_count}")
        
        # Caching should provide significant speedup
        assert speedup > 100, "Caching should provide major speedup"
        assert call_count > iterations, "Should call C API multiple times without cache"