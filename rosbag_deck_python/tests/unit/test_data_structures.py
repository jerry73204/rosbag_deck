"""
Unit tests for data structures (BagInfo, Status, Message)
"""

import pytest
from rosbag_deck.core import BagInfo, Status, Message


class TestBagInfo:
    """Test BagInfo dataclass"""

    @pytest.mark.unit
    def test_bag_info_creation(self):
        """Test BagInfo object creation"""
        bag_info = BagInfo(
            success=True,
            message="Test message",
            start_time=1000,
            end_time=2000,
            total_duration_ns=1000,
            total_frames=100,
            topic_names=["topic1", "topic2"]
        )
        
        assert bag_info.success is True
        assert bag_info.message == "Test message"
        assert bag_info.start_time == 1000
        assert bag_info.end_time == 2000
        assert bag_info.total_duration_ns == 1000
        assert bag_info.total_frames == 100
        assert bag_info.topic_names == ["topic1", "topic2"]

    @pytest.mark.unit
    def test_bag_info_immutable(self):
        """Test that BagInfo is immutable (dataclass with frozen=False by default)"""
        bag_info = BagInfo(
            success=True,
            message="Test",
            start_time=1000,
            end_time=2000,
            total_duration_ns=1000,
            total_frames=100,
            topic_names=[]
        )
        
        # Should be able to modify since we didn't freeze the dataclass
        bag_info.success = False
        assert bag_info.success is False

    @pytest.mark.unit
    def test_bag_info_empty_topics(self):
        """Test BagInfo with empty topic list"""
        bag_info = BagInfo(
            success=True,
            message="",
            start_time=0,
            end_time=0,
            total_duration_ns=0,
            total_frames=0,
            topic_names=[]
        )
        
        assert len(bag_info.topic_names) == 0

    @pytest.mark.unit
    def test_bag_info_repr(self):
        """Test BagInfo string representation"""
        bag_info = BagInfo(
            success=True,
            message="Test",
            start_time=1000,
            end_time=2000,
            total_duration_ns=1000,
            total_frames=100,
            topic_names=["topic1"]
        )
        
        repr_str = repr(bag_info)
        assert "BagInfo" in repr_str
        assert "success=True" in repr_str


class TestStatus:
    """Test Status dataclass"""

    @pytest.mark.unit
    def test_status_creation(self):
        """Test Status object creation"""
        status = Status(
            current_time=1500,
            is_playing=True,
            current_frame=50,
            total_frames=100,
            timeline_segment=1,
            virtual_time=1500
        )
        
        assert status.current_time == 1500
        assert status.is_playing is True
        assert status.current_frame == 50
        assert status.total_frames == 100
        assert status.timeline_segment == 1
        assert status.virtual_time == 1500

    @pytest.mark.unit
    def test_status_not_playing(self):
        """Test Status when not playing"""
        status = Status(
            current_time=0,
            is_playing=False,
            current_frame=0,
            total_frames=100,
            timeline_segment=0,
            virtual_time=0
        )
        
        assert status.is_playing is False
        assert status.current_frame == 0

    @pytest.mark.unit
    def test_status_edge_cases(self):
        """Test Status with edge case values"""
        # Test with maximum frame
        status = Status(
            current_time=1000,
            is_playing=False,
            current_frame=99,
            total_frames=100,
            timeline_segment=1,
            virtual_time=1000
        )
        
        assert status.current_frame == status.total_frames - 1

    @pytest.mark.unit
    def test_status_repr(self):
        """Test Status string representation"""
        status = Status(
            current_time=1500,
            is_playing=True,
            current_frame=50,
            total_frames=100,
            timeline_segment=1,
            virtual_time=1500
        )
        
        repr_str = repr(status)
        assert "Status" in repr_str
        assert "is_playing=True" in repr_str


class TestMessage:
    """Test Message dataclass"""

    @pytest.mark.unit
    def test_message_creation(self):
        """Test Message object creation"""
        message = Message(
            original_timestamp=1000,
            virtual_timestamp=1001,
            topic_name="/test_topic",
            message_type="std_msgs/msg/String",
            serialized_data=b"test data",
            frame_index=42
        )
        
        assert message.original_timestamp == 1000
        assert message.virtual_timestamp == 1001
        assert message.topic_name == "/test_topic"
        assert message.message_type == "std_msgs/msg/String"
        assert message.serialized_data == b"test data"
        assert message.frame_index == 42

    @pytest.mark.unit
    def test_message_empty_data(self):
        """Test Message with empty serialized data"""
        message = Message(
            original_timestamp=1000,
            virtual_timestamp=1001,
            topic_name="/empty_topic",
            message_type="std_msgs/msg/Empty",
            serialized_data=b"",
            frame_index=0
        )
        
        assert len(message.serialized_data) == 0
        assert message.frame_index == 0

    @pytest.mark.unit
    def test_message_large_data(self):
        """Test Message with large serialized data"""
        large_data = b"x" * 10000  # 10KB
        message = Message(
            original_timestamp=1000,
            virtual_timestamp=1001,
            topic_name="/large_topic",
            message_type="sensor_msgs/msg/PointCloud2",
            serialized_data=large_data,
            frame_index=1000
        )
        
        assert len(message.serialized_data) == 10000
        assert message.serialized_data[0:5] == b"xxxxx"

    @pytest.mark.unit
    def test_message_unicode_topic(self):
        """Test Message with unicode characters in topic name"""
        message = Message(
            original_timestamp=1000,
            virtual_timestamp=1001,
            topic_name="/test_topic_ñáéíóú",
            message_type="std_msgs/msg/String",
            serialized_data=b"test",
            frame_index=1
        )
        
        assert "ñáéíóú" in message.topic_name

    @pytest.mark.unit
    def test_message_repr(self):
        """Test Message string representation"""
        message = Message(
            original_timestamp=1000,
            virtual_timestamp=1001,
            topic_name="/test",
            message_type="std_msgs/msg/String",
            serialized_data=b"data",
            frame_index=42
        )
        
        repr_str = repr(message)
        assert "Message" in repr_str
        assert "frame_index=42" in repr_str


class TestDataStructureInteraction:
    """Test interactions between data structures"""

    @pytest.mark.unit
    def test_timestamp_consistency(self):
        """Test timestamp consistency between structures"""
        timestamp = 1234567890
        
        status = Status(
            current_time=timestamp,
            is_playing=True,
            current_frame=10,
            total_frames=100,
            timeline_segment=1,
            virtual_time=timestamp
        )
        
        message = Message(
            original_timestamp=timestamp,
            virtual_timestamp=timestamp,
            topic_name="/test",
            message_type="std_msgs/msg/String",
            serialized_data=b"test",
            frame_index=10
        )
        
        assert status.current_time == message.original_timestamp
        assert status.current_frame == message.frame_index

    @pytest.mark.unit
    def test_frame_consistency(self):
        """Test frame numbering consistency"""
        total_frames = 100
        current_frame = 50
        
        bag_info = BagInfo(
            success=True,
            message="Test",
            start_time=1000,
            end_time=2000,
            total_duration_ns=1000,
            total_frames=total_frames,
            topic_names=[]
        )
        
        status = Status(
            current_time=1500,
            is_playing=False,
            current_frame=current_frame,
            total_frames=total_frames,
            timeline_segment=1,
            virtual_time=1500
        )
        
        assert bag_info.total_frames == status.total_frames
        assert status.current_frame < status.total_frames