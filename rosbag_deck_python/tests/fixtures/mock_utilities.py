"""
Mock utilities for testing rosbag_deck_python
Provides mock C API, bag fixtures, and error simulation framework
"""

import os
import tempfile
from pathlib import Path
from typing import List, Dict, Any, Optional, Callable
from unittest.mock import Mock, MagicMock
import struct
import time


class MockCAPI:
    """Mock C API for isolated Python testing"""
    
    def __init__(self):
        self.handles = {}
        self.next_handle = 0x12345678
        self.last_error = b""
        self.callbacks = {}
        self.playback_state = {
            "is_playing": False,
            "current_frame": 0,
            "total_frames": 0,
            "current_time": 0,
            "timeline_segment": 0,
        }
        self.bag_info = None
        self.message_queue = []
        
    def create_handle(self) -> int:
        """Create a new deck handle"""
        handle = self.next_handle
        self.handles[handle] = {
            "created": time.time(),
            "cache_size": 100,
            "preload_size": 10,
            "playback_rate": 1.0,
            "loop_enabled": False,
            "topic_filter": None,
            "type_filter": None,
            "status_callback": None,
            "message_callback": None,
        }
        self.next_handle += 1
        return handle
    
    def destroy_handle(self, handle: int) -> None:
        """Destroy a deck handle"""
        if handle in self.handles:
            del self.handles[handle]
    
    def is_valid_handle(self, handle: int) -> bool:
        """Check if handle is valid"""
        return handle in self.handles
    
    def set_error(self, error: str) -> None:
        """Set last error message"""
        self.last_error = error.encode('utf-8')
    
    def get_last_error(self) -> bytes:
        """Get last error message"""
        return self.last_error
    
    def build_index(self, handle: int, bag_paths: List[str]) -> bool:
        """Mock index building"""
        if not self.is_valid_handle(handle):
            self.set_error("Invalid handle")
            return False
        
        # Simulate index building
        self.playback_state["total_frames"] = 1000  # Mock frame count
        self.bag_info = {
            "success": True,
            "message": "Index built successfully",
            "start_time": 1000000000,  # 1 second
            "end_time": 2000000000,    # 2 seconds
            "total_duration_ns": 1000000000,
            "total_frames": 1000,
            "topic_names": ["/camera/image", "/lidar/points", "/imu/data"],
        }
        return True
    
    def get_status(self, handle: int) -> Dict[str, Any]:
        """Get current playback status"""
        if not self.is_valid_handle(handle):
            return None
        
        return {
            "current_time": self.playback_state["current_time"],
            "is_playing": self.playback_state["is_playing"],
            "current_frame": self.playback_state["current_frame"],
            "total_frames": self.playback_state["total_frames"],
            "timeline_segment": self.playback_state["timeline_segment"],
            "virtual_time": self.playback_state["current_time"],
        }
    
    def start_playback(self, handle: int) -> None:
        """Start playback"""
        if self.is_valid_handle(handle):
            self.playback_state["is_playing"] = True
    
    def stop_playback(self, handle: int) -> None:
        """Stop playback"""
        if self.is_valid_handle(handle):
            self.playback_state["is_playing"] = False
    
    def step_forward(self, handle: int) -> bool:
        """Step forward one frame"""
        if not self.is_valid_handle(handle):
            return False
        
        if self.playback_state["current_frame"] < self.playback_state["total_frames"] - 1:
            self.playback_state["current_frame"] += 1
            self.playback_state["current_time"] += 1000000  # 1ms per frame
            self._trigger_callbacks(handle)
            return True
        return False
    
    def step_backward(self, handle: int) -> bool:
        """Step backward one frame"""
        if not self.is_valid_handle(handle):
            return False
        
        if self.playback_state["current_frame"] > 0:
            self.playback_state["current_frame"] -= 1
            self.playback_state["current_time"] -= 1000000  # 1ms per frame
            self._trigger_callbacks(handle)
            return True
        return False
    
    def _trigger_callbacks(self, handle: int) -> None:
        """Trigger registered callbacks"""
        deck_data = self.handles.get(handle, {})
        
        # Trigger status callback
        if deck_data.get("status_callback"):
            status = self.get_status(handle)
            deck_data["status_callback"](status)
        
        # Trigger message callback
        if deck_data.get("message_callback") and self.message_queue:
            message = self.message_queue.pop(0)
            deck_data["message_callback"](message)
    
    def add_test_message(self, message: Dict[str, Any]) -> None:
        """Add a test message to the queue"""
        self.message_queue.append(message)


class TestBagCreator:
    """Create test bag files for integration testing"""
    
    def __init__(self, temp_dir: Path):
        self.temp_dir = temp_dir
        
    def create_simple_bag(self, name: str = "test.db3") -> Path:
        """Create a simple test bag file"""
        bag_path = self.temp_dir / name
        
        # Create a mock bag file (just a file with some data)
        with open(bag_path, 'wb') as f:
            # Write mock bag header
            f.write(b'ROSBAG V2.0\n')
            # Write some mock data
            f.write(struct.pack('<I', 1000))  # Frame count
            f.write(struct.pack('<Q', 1000000000))  # Start time
            f.write(struct.pack('<Q', 2000000000))  # End time
            
        return bag_path
    
    def create_large_bag(self, name: str = "large.db3", size_mb: int = 100) -> Path:
        """Create a large test bag file"""
        bag_path = self.temp_dir / name
        
        # Create a large file
        with open(bag_path, 'wb') as f:
            f.write(b'ROSBAG V2.0\n')
            # Write data in chunks
            chunk_size = 1024 * 1024  # 1MB chunks
            for _ in range(size_mb):
                f.write(b'x' * chunk_size)
        
        return bag_path
    
    def create_multi_topic_bag(self, name: str = "multi_topic.db3") -> Path:
        """Create a bag with multiple topics"""
        bag_path = self.temp_dir / name
        
        topics = [
            "/camera/image_raw",
            "/camera/camera_info", 
            "/lidar/points",
            "/imu/data",
            "/gps/fix",
            "/tf",
            "/tf_static",
        ]
        
        with open(bag_path, 'wb') as f:
            f.write(b'ROSBAG V2.0\n')
            # Write topic count
            f.write(struct.pack('<I', len(topics)))
            # Write topic names
            for topic in topics:
                topic_bytes = topic.encode('utf-8')
                f.write(struct.pack('<I', len(topic_bytes)))
                f.write(topic_bytes)
        
        return bag_path


class ErrorSimulator:
    """Simulate various error conditions for testing"""
    
    def __init__(self, mock_api: MockCAPI):
        self.mock_api = mock_api
        self.error_mode = None
        
    def enable_error_mode(self, mode: str) -> None:
        """Enable a specific error simulation mode"""
        self.error_mode = mode
        
    def disable_error_mode(self) -> None:
        """Disable error simulation"""
        self.error_mode = None
        
    def maybe_fail(self, operation: str) -> bool:
        """Check if operation should fail based on error mode"""
        if not self.error_mode:
            return False
            
        error_conditions = {
            "memory_exhaustion": ["create_handle", "build_index"],
            "file_not_found": ["build_index"],
            "corrupted_data": ["get_message", "step_forward"],
            "permission_denied": ["build_index", "create_handle"],
            "network_timeout": ["get_status", "get_message"],
        }
        
        if self.error_mode in error_conditions:
            if operation in error_conditions[self.error_mode]:
                self.mock_api.set_error(f"Simulated error: {self.error_mode}")
                return True
        
        return False


class PerformanceMonitor:
    """Monitor performance metrics during tests"""
    
    def __init__(self):
        self.metrics = {
            "callback_count": 0,
            "callback_times": [],
            "memory_samples": [],
            "operation_times": {},
        }
        self.start_time = time.time()
        
    def record_callback(self, duration: float) -> None:
        """Record callback execution time"""
        self.metrics["callback_count"] += 1
        self.metrics["callback_times"].append(duration)
        
    def record_memory(self, memory_mb: float) -> None:
        """Record memory usage sample"""
        self.metrics["memory_samples"].append({
            "time": time.time() - self.start_time,
            "memory_mb": memory_mb,
        })
        
    def record_operation(self, operation: str, duration: float) -> None:
        """Record operation timing"""
        if operation not in self.metrics["operation_times"]:
            self.metrics["operation_times"][operation] = []
        self.metrics["operation_times"][operation].append(duration)
        
    def get_summary(self) -> Dict[str, Any]:
        """Get performance summary"""
        summary = {
            "total_callbacks": self.metrics["callback_count"],
            "avg_callback_time": 0,
            "max_callback_time": 0,
            "peak_memory_mb": 0,
            "operation_stats": {},
        }
        
        if self.metrics["callback_times"]:
            summary["avg_callback_time"] = sum(self.metrics["callback_times"]) / len(self.metrics["callback_times"])
            summary["max_callback_time"] = max(self.metrics["callback_times"])
            
        if self.metrics["memory_samples"]:
            summary["peak_memory_mb"] = max(s["memory_mb"] for s in self.metrics["memory_samples"])
            
        for op, times in self.metrics["operation_times"].items():
            if times:
                summary["operation_stats"][op] = {
                    "count": len(times),
                    "avg_time": sum(times) / len(times),
                    "max_time": max(times),
                }
                
        return summary


def create_mock_ffi_with_api() -> tuple:
    """Create a complete mock FFI setup with C API"""
    mock_api = MockCAPI()
    mock_ffi = MagicMock()
    mock_lib = MagicMock()
    
    # Wire up the mock lib to use mock API
    mock_lib.rosbag_deck_create.side_effect = lambda: mock_api.create_handle()
    mock_lib.rosbag_deck_destroy.side_effect = lambda h: mock_api.destroy_handle(h)
    mock_lib.rosbag_deck_build_index.side_effect = lambda h, paths, count: mock_api.build_index(h, paths)
    mock_lib.rosbag_deck_get_status.side_effect = lambda h: mock_api.get_status(h)
    mock_lib.rosbag_deck_start_playback.side_effect = lambda h: mock_api.start_playback(h)
    mock_lib.rosbag_deck_stop_playback.side_effect = lambda h: mock_api.stop_playback(h)
    mock_lib.rosbag_deck_step_forward.side_effect = lambda h: mock_api.step_forward(h)
    mock_lib.rosbag_deck_step_backward.side_effect = lambda h: mock_api.step_backward(h)
    mock_lib.rosbag_deck_get_last_error.side_effect = lambda: mock_api.get_last_error()
    
    # Set up FFI NULL
    mock_ffi.NULL = None
    
    return mock_ffi, mock_lib, mock_api