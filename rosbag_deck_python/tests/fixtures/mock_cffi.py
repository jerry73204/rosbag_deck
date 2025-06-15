"""
Mock CFFI infrastructure for testing without C dependencies
"""

from typing import Any, Dict, List, Optional, Callable
from unittest.mock import Mock, MagicMock


class MockFFI:
    """Mock CFFI ffi object for testing"""
    
    def __init__(self):
        self.NULL = None
        self._new_objects = {}
        self._string_mapping = {}
        self._buffer_mapping = {}
    
    def new(self, type_name: str, init=None):
        """Mock ffi.new() method"""
        if type_name.endswith('[]'):
            # Array type
            base_type = type_name[:-2]
            if init is not None:
                size = len(init) if hasattr(init, '__len__') else init
                return [Mock() for _ in range(size)]
            return []
        else:
            # Single object
            mock_obj = Mock()
            if type_name == "char[]" and init:
                mock_obj._value = init
            return mock_obj
    
    def string(self, c_string) -> bytes:
        """Mock ffi.string() method"""
        if c_string in self._string_mapping:
            return self._string_mapping[c_string]
        return b"mock_string"
    
    def buffer(self, data, size: int) -> bytes:
        """Mock ffi.buffer() method"""
        if data in self._buffer_mapping:
            return self._buffer_mapping[data][:size]
        return b"mock_data" * (size // 9 + 1)[:size]
    
    def set_string_return(self, c_string, value: bytes):
        """Set what ffi.string() should return for a specific input"""
        self._string_mapping[c_string] = value
    
    def set_buffer_return(self, data, value: bytes):
        """Set what ffi.buffer() should return for a specific input"""
        self._buffer_mapping[data] = value


class MockCLib:
    """Mock C library object for testing"""
    
    def __init__(self):
        # Core lifecycle
        self.rosbag_deck_create = Mock(return_value=0x12345678)
        self.rosbag_deck_destroy = Mock()
        
        # Index building
        self.rosbag_deck_build_index = Mock(return_value=True)
        
        # Playback control
        self.rosbag_deck_start_playback = Mock()
        self.rosbag_deck_stop_playback = Mock()
        self.rosbag_deck_step_forward = Mock(return_value=True)
        self.rosbag_deck_step_backward = Mock(return_value=True)
        
        # Seeking
        self.rosbag_deck_seek_to_time = Mock(return_value=True)
        self.rosbag_deck_seek_to_frame = Mock(return_value=True)
        self.rosbag_deck_to_timestamp = Mock(return_value=self._create_mock_timestamp())
        
        # Configuration
        self.rosbag_deck_set_cache_size = Mock()
        self.rosbag_deck_set_preload_settings = Mock()
        self.rosbag_deck_set_playback_rate = Mock()
        self.rosbag_deck_set_loop_playback = Mock()
        
        # Information
        self.rosbag_deck_get_bag_info = Mock(return_value=self._create_mock_bag_info())
        self.rosbag_deck_get_status = Mock(return_value=self._create_mock_status())
        
        # Callbacks
        self.rosbag_deck_set_status_callback = Mock()
        self.rosbag_deck_set_message_callback = Mock()
    
    def _create_mock_timestamp(self):
        """Create a mock timestamp structure"""
        timestamp = Mock()
        timestamp.nanoseconds_since_epoch = 1000000000
        return timestamp
    
    def _create_mock_bag_info(self):
        """Create a mock bag info structure"""
        info = Mock()
        info.success = True
        info.message = None
        info.start_time = self._create_mock_timestamp()
        info.end_time = Mock()
        info.end_time.nanoseconds_since_epoch = 2000000000
        info.total_duration_ns = 1000000000
        info.total_frames = 100
        info.topic_names = None
        info.topic_names_count = 0
        return info
    
    def _create_mock_status(self):
        """Create a mock status structure"""
        status = Mock()
        status.current_time = self._create_mock_timestamp()
        status.current_time.nanoseconds_since_epoch = 1500000000
        status.is_playing = False
        status.current_frame = 50
        status.total_frames = 100
        status.timeline_segment = 1
        status.virtual_time = self._create_mock_timestamp()
        status.virtual_time.nanoseconds_since_epoch = 1500000000
        return status
    
    def set_bag_info(self, **kwargs):
        """Update mock bag info with custom values"""
        info = self.rosbag_deck_get_bag_info.return_value
        for key, value in kwargs.items():
            setattr(info, key, value)
    
    def set_status(self, **kwargs):
        """Update mock status with custom values"""
        status = self.rosbag_deck_get_status.return_value
        for key, value in kwargs.items():
            if key in ['current_time', 'virtual_time']:
                timestamp = Mock()
                timestamp.nanoseconds_since_epoch = value
                setattr(status, key, timestamp)
            else:
                setattr(status, key, value)


class MockRosbagDeckFFI:
    """Mock RosbagDeckFFI class for testing"""
    
    def __init__(self, library_path: Optional[str] = None):
        self.lib = MockCLib()
        self.ffi = MockFFI()
        self.library_path = library_path
    
    def reset_mocks(self):
        """Reset all mock call counts and side effects"""
        for attr_name in dir(self.lib):
            attr = getattr(self.lib, attr_name)
            if isinstance(attr, Mock):
                attr.reset_mock()


def create_mock_callback(callback_type: str, func: Callable) -> Mock:
    """Mock version of create_cffi_callback"""
    mock_callback = Mock()
    mock_callback._func = func  # Store for testing
    mock_callback._type = callback_type
    return mock_callback


class MockBagCreator:
    """Mock bag file creator for testing"""
    
    def __init__(self, temp_dir):
        self.temp_dir = temp_dir
        self.created_bags = []
    
    def create_bag(self, name: str, num_messages: int = 10, topics: Optional[List[str]] = None) -> str:
        """Create a mock bag file"""
        bag_path = self.temp_dir / name
        bag_path.touch()
        
        bag_info = {
            'path': str(bag_path),
            'num_messages': num_messages,
            'topics': topics or ['/test_topic'],
            'start_time': 1000000000,
            'end_time': 1000000000 + num_messages * 100000000,  # 100ms per message
        }
        
        self.created_bags.append(bag_info)
        return str(bag_path)
    
    def create_multi_topic_bag(self, name: str, num_messages: int = 10) -> str:
        """Create a mock multi-topic bag file"""
        topics = ['/pointcloud', '/cmd_vel', '/status']
        return self.create_bag(name, num_messages * len(topics), topics)
    
    def get_bag_info(self, bag_path: str) -> Dict[str, Any]:
        """Get info for a created bag"""
        for bag in self.created_bags:
            if bag['path'] == bag_path:
                return bag
        return {}
    
    def cleanup(self):
        """Clean up created bag files"""
        self.created_bags.clear()


class ErrorSimulator:
    """Helper class for simulating various error conditions"""
    
    def __init__(self, mock_lib: MockCLib):
        self.mock_lib = mock_lib
        self._original_returns = {}
    
    def simulate_init_failure(self):
        """Simulate initialization failure"""
        self.mock_lib.rosbag_deck_create.return_value = None
    
    def simulate_index_failure(self):
        """Simulate index building failure"""
        self.mock_lib.rosbag_deck_build_index.return_value = False
    
    def simulate_seek_failure(self):
        """Simulate seek operation failure"""
        self.mock_lib.rosbag_deck_seek_to_time.return_value = False
        self.mock_lib.rosbag_deck_seek_to_frame.return_value = False
    
    def simulate_step_at_boundary(self):
        """Simulate stepping at bag boundaries"""
        self.mock_lib.rosbag_deck_step_forward.return_value = False
        self.mock_lib.rosbag_deck_step_backward.return_value = False
    
    def simulate_exception(self, method_name: str, exception: Exception):
        """Simulate an exception being raised by a mock method"""
        method = getattr(self.mock_lib, method_name)
        method.side_effect = exception
    
    def reset_all(self):
        """Reset all simulated errors"""
        self.mock_lib.rosbag_deck_create.return_value = 0x12345678
        self.mock_lib.rosbag_deck_create.side_effect = None
        self.mock_lib.rosbag_deck_build_index.return_value = True
        self.mock_lib.rosbag_deck_build_index.side_effect = None
        self.mock_lib.rosbag_deck_seek_to_time.return_value = True
        self.mock_lib.rosbag_deck_seek_to_time.side_effect = None
        self.mock_lib.rosbag_deck_seek_to_frame.return_value = True
        self.mock_lib.rosbag_deck_seek_to_frame.side_effect = None
        self.mock_lib.rosbag_deck_step_forward.return_value = True
        self.mock_lib.rosbag_deck_step_forward.side_effect = None
        self.mock_lib.rosbag_deck_step_backward.return_value = True
        self.mock_lib.rosbag_deck_step_backward.side_effect = None