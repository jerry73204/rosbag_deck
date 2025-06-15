"""
Unit tests for exception handling and error propagation
"""

import pytest
from rosbag_deck.exceptions import (
    RosbagDeckError,
    RosbagDeckInitError,
    RosbagDeckIndexError
)


class TestExceptionHierarchy:
    """Test exception class hierarchy and inheritance"""

    @pytest.mark.unit
    def test_base_exception(self):
        """Test RosbagDeckError base exception"""
        error = RosbagDeckError("Base error message")
        
        assert str(error) == "Base error message"
        assert isinstance(error, Exception)

    @pytest.mark.unit
    def test_init_error_inheritance(self):
        """Test RosbagDeckInitError inherits from base"""
        error = RosbagDeckInitError("Initialization failed")
        
        assert isinstance(error, RosbagDeckError)
        assert isinstance(error, Exception)
        assert str(error) == "Initialization failed"

    @pytest.mark.unit
    def test_index_error_inheritance(self):
        """Test RosbagDeckIndexError inherits from base"""
        error = RosbagDeckIndexError("Index building failed")
        
        assert isinstance(error, RosbagDeckError)
        assert isinstance(error, Exception)
        assert str(error) == "Index building failed"

    @pytest.mark.unit
    def test_exception_catching(self):
        """Test that specific exceptions can be caught as base exception"""
        with pytest.raises(RosbagDeckError):
            raise RosbagDeckInitError("Init error")
        
        with pytest.raises(RosbagDeckError):
            raise RosbagDeckIndexError("Index error")

    @pytest.mark.unit
    def test_exception_distinction(self):
        """Test that specific exceptions can be distinguished"""
        init_error = RosbagDeckInitError("Init")
        index_error = RosbagDeckIndexError("Index")
        
        assert not isinstance(init_error, RosbagDeckIndexError)
        assert not isinstance(index_error, RosbagDeckInitError)
        assert isinstance(init_error, RosbagDeckInitError)
        assert isinstance(index_error, RosbagDeckIndexError)


class TestExceptionMessages:
    """Test exception message handling"""

    @pytest.mark.unit
    def test_empty_message(self):
        """Test exception with empty message"""
        error = RosbagDeckError("")
        assert str(error) == ""

    @pytest.mark.unit
    def test_unicode_message(self):
        """Test exception with unicode message"""
        message = "Error with unicode: ñáéíóú 中文 🚀"
        error = RosbagDeckError(message)
        assert str(error) == message

    @pytest.mark.unit
    def test_long_message(self):
        """Test exception with very long message"""
        long_message = "Error: " + "x" * 1000
        error = RosbagDeckError(long_message)
        assert str(error) == long_message
        assert len(str(error)) == 1007

    @pytest.mark.unit
    def test_formatted_message(self):
        """Test exception with formatted message"""
        error = RosbagDeckInitError(f"Failed to initialize with code {42}")
        assert "Failed to initialize with code 42" in str(error)

    @pytest.mark.unit
    def test_multiline_message(self):
        """Test exception with multiline message"""
        message = "Error occurred:\nLine 1\nLine 2"
        error = RosbagDeckError(message)
        assert str(error) == message
        assert "\n" in str(error)


class TestExceptionContext:
    """Test exception context and chaining"""

    @pytest.mark.unit
    def test_exception_chaining(self):
        """Test exception chaining with cause"""
        try:
            try:
                raise ValueError("Original error")
            except ValueError as e:
                raise RosbagDeckInitError("Wrapper error") from e
        except RosbagDeckInitError as wrapper_error:
            assert str(wrapper_error) == "Wrapper error"
            assert isinstance(wrapper_error.__cause__, ValueError)
            assert str(wrapper_error.__cause__) == "Original error"

    @pytest.mark.unit
    def test_exception_context_suppressed(self):
        """Test exception context suppression"""
        try:
            try:
                raise ValueError("Original error")
            except ValueError:
                raise RosbagDeckError("New error") from None
        except RosbagDeckError as error:
            assert str(error) == "New error"
            assert error.__cause__ is None
            assert error.__suppress_context__ is True

    @pytest.mark.unit
    def test_nested_exception_handling(self):
        """Test handling nested exceptions"""
        def raise_init_error():
            raise RosbagDeckInitError("Init failed")
        
        def raise_index_error():
            try:
                raise_init_error()
            except RosbagDeckInitError as e:
                raise RosbagDeckIndexError("Index failed after init error") from e
        
        with pytest.raises(RosbagDeckIndexError) as exc_info:
            raise_index_error()
        
        assert "Index failed after init error" in str(exc_info.value)
        assert isinstance(exc_info.value.__cause__, RosbagDeckInitError)


class TestExceptionUsagePatterns:
    """Test common exception usage patterns"""

    @pytest.mark.unit
    def test_catch_specific_reraise_general(self):
        """Test catching specific exception and re-raising as general"""
        def problematic_function():
            raise ValueError("Low-level error")
        
        def wrapper_function():
            try:
                problematic_function()
            except ValueError as e:
                raise RosbagDeckError(f"High-level error: {e}") from e
        
        with pytest.raises(RosbagDeckError) as exc_info:
            wrapper_function()
        
        assert "High-level error: Low-level error" in str(exc_info.value)

    @pytest.mark.unit
    def test_multiple_exception_types(self):
        """Test handling multiple exception types"""
        def handle_errors(error_type):
            if error_type == "init":
                raise RosbagDeckInitError("Init error")
            elif error_type == "index":
                raise RosbagDeckIndexError("Index error")
            else:
                raise RosbagDeckError("General error")
        
        # Test catching each type specifically
        with pytest.raises(RosbagDeckInitError):
            handle_errors("init")
        
        with pytest.raises(RosbagDeckIndexError):
            handle_errors("index")
        
        with pytest.raises(RosbagDeckError):
            handle_errors("other")

    @pytest.mark.unit
    def test_exception_in_context_manager(self):
        """Test exception handling in context manager pattern"""
        class MockResource:
            def __init__(self, should_fail=False):
                self.should_fail = should_fail
                if should_fail:
                    raise RosbagDeckInitError("Resource creation failed")
            
            def __enter__(self):
                return self
            
            def __exit__(self, exc_type, exc_val, exc_tb):
                if exc_type is not None:
                    # Exception occurred during usage
                    return False  # Don't suppress
                return False
        
        # Test exception during resource creation
        with pytest.raises(RosbagDeckInitError):
            with MockResource(should_fail=True):
                pass
        
        # Test exception during resource usage
        with pytest.raises(RosbagDeckIndexError):
            with MockResource(should_fail=False):
                raise RosbagDeckIndexError("Usage error")


class TestExceptionDocumentation:
    """Test that exceptions provide good error messages for documentation"""

    @pytest.mark.unit
    def test_init_error_scenarios(self):
        """Test common RosbagDeckInitError scenarios"""
        scenarios = [
            "Failed to load shared library",
            "Invalid library path provided",
            "Library version mismatch",
            "Insufficient permissions",
            "Memory allocation failed"
        ]
        
        for scenario in scenarios:
            error = RosbagDeckInitError(scenario)
            assert scenario in str(error)
            assert isinstance(error, RosbagDeckError)

    @pytest.mark.unit
    def test_index_error_scenarios(self):
        """Test common RosbagDeckIndexError scenarios"""
        scenarios = [
            "Bag file not found",
            "Corrupted bag file format",
            "Unsupported storage plugin",
            "No topics found in bag",
            "Index building timeout"
        ]
        
        for scenario in scenarios:
            error = RosbagDeckIndexError(scenario)
            assert scenario in str(error)
            assert isinstance(error, RosbagDeckError)

    @pytest.mark.unit
    def test_error_with_suggestions(self):
        """Test errors that include suggestions for resolution"""
        error_with_suggestion = RosbagDeckInitError(
            "Failed to load library 'librosbag_deck_core.so'. "
            "Please ensure the library is installed and LD_LIBRARY_PATH is set correctly."
        )
        
        message = str(error_with_suggestion)
        assert "Failed to load library" in message
        assert "Please ensure" in message
        assert "LD_LIBRARY_PATH" in message