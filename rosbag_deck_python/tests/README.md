# RosBag Deck Python Testing Framework

This directory contains the comprehensive test suite for the `rosbag_deck_python` package.

## Test Structure

```
tests/
├── unit/                      # Unit tests (fast, isolated)
│   ├── test_data_structures.py    # BagInfo, Status, Message tests
│   ├── test_deck_core.py          # RosbagDeck class tests  
│   ├── test_callbacks.py          # Callback system tests
│   └── test_exceptions.py         # Exception handling tests
├── integration/               # Integration tests (slower, with dependencies)
│   ├── test_basic_functionality.py    # End-to-end workflow tests
│   └── test_memory_management.py      # Memory leak and efficiency tests
├── fixtures/                  # Test utilities and fixtures
│   └── mock_cffi.py              # Mock CFFI infrastructure
├── conftest.py               # Shared pytest fixtures
└── README.md                 # This file
```

## Running Tests

### Prerequisites

Install test dependencies:
```bash
# If using rye (recommended)
rye sync

# Or with pip
pip install pytest pytest-cov pytest-mock memory-profiler
```

### Basic Usage

```bash
# Run all tests
python -m pytest

# Run only unit tests (fast)
python -m pytest -m unit

# Run only integration tests
python -m pytest -m integration

# Run with coverage report
python -m pytest --cov=rosbag_deck --cov-report=html

# Run specific test file
python -m pytest tests/unit/test_data_structures.py

# Run with verbose output
python -m pytest -v
```

### Using the Test Runner

A convenient test runner script is provided:

```bash
# Run all tests
python run_tests.py

# Run only unit tests
python run_tests.py unit

# Run only integration tests  
python run_tests.py integration

# Run with coverage
python run_tests.py --coverage
```

## Test Categories

### Unit Tests (`pytest -m unit`)

Fast, isolated tests that don't require external dependencies:

- **Data Structures**: Test BagInfo, Status, Message dataclasses
- **Deck Core**: Test RosbagDeck class with mocked CFFI
- **Callbacks**: Test callback registration and invocation
- **Exceptions**: Test error handling and exception hierarchy

### Integration Tests (`pytest -m integration`)

Slower tests that may require real dependencies or simulate complex scenarios:

- **Basic Functionality**: End-to-end workflow testing
- **Memory Management**: Memory leak detection and efficiency testing

### Performance Tests (`pytest -m performance`)

Tests that measure timing and memory usage:

- **Memory Profiling**: Using `memory_profiler` for leak detection
- **Performance Benchmarks**: Measuring CFFI overhead and callback latency

## Test Configuration

Configuration is handled through `pyproject.toml`:

```toml
[tool.pytest.ini_options]
testpaths = ["tests"]
addopts = [
    "--strict-markers",
    "--cov=rosbag_deck", 
    "--cov-fail-under=85",
]
markers = [
    "unit: Unit tests that run quickly",
    "integration: Integration tests that may require real dependencies",
    "performance: Performance tests that measure timing and memory",
    "slow: Tests that take a long time to run",
]
```

## Mock Infrastructure

The test suite uses extensive mocking to test Python code without requiring the C library:

### Mock CFFI (`tests/fixtures/mock_cffi.py`)

- **MockFFI**: Simulates CFFI `ffi` object
- **MockCLib**: Simulates C library functions
- **MockRosbagDeckFFI**: Complete mock of the FFI interface
- **ErrorSimulator**: Helps simulate various error conditions

### Shared Fixtures (`conftest.py`)

- **temp_dir**: Temporary directory for test files
- **mock_ffi**: Mock CFFI ffi object
- **mock_ffi_lib**: Mock C library with realistic function signatures
- **sample_data**: Pre-configured test data (BagInfo, Status, Message)
- **callback_mocks**: Mock callback functions for testing

## Writing Tests

### Unit Test Example

```python
import pytest
from rosbag_deck.core import RosbagDeck

class TestMyFeature:
    @pytest.mark.unit
    def test_basic_functionality(self, mock_rosbag_deck_ffi):
        deck = RosbagDeck()
        deck.set_cache_size(100)
        
        mock_rosbag_deck_ffi.lib.rosbag_deck_set_cache_size.assert_called_with(
            deck._handle, 100
        )
```

### Integration Test Example

```python
import pytest

class TestIntegration:
    @pytest.mark.integration
    def test_end_to_end_workflow(self, mock_rosbag_deck_ffi):
        deck = RosbagDeck()
        deck.build_index(["/path/to/bag.db3"])
        deck.start_playback()
        status = deck.get_status()
        assert status.is_playing
```

### Memory Test Example

```python
import pytest

class TestMemory:
    @pytest.mark.performance
    def test_no_memory_leaks(self, mock_rosbag_deck_ffi):
        # Test that repeated operations don't leak memory
        for _ in range(100):
            deck = RosbagDeck()
            deck.close()
```

## Coverage Goals

- **Line Coverage**: 85%+ (enforced by pytest)
- **Branch Coverage**: 75%+ (monitored)
- **Function Coverage**: 95%+ (for public API)

## Continuous Integration

The test suite is designed for CI/CD integration:

```bash
# Quick CI check (unit tests only)
pytest -m unit --tb=short

# Full CI check (all tests with coverage)
pytest --cov=rosbag_deck --cov-fail-under=85

# Memory leak detection
pytest -m performance --tb=short
```

## Troubleshooting

### Common Issues

1. **Import Errors**: Ensure `rosbag_deck` is in PYTHONPATH
2. **CFFI Errors**: Unit tests should use mocks, integration tests may skip if CFFI unavailable
3. **Memory Tests Failing**: Install `memory-profiler`: `pip install memory-profiler`
4. **Coverage Too Low**: Write more unit tests, current minimum is 85%

### Debug Options

```bash
# Run with debug output
pytest -v -s --tb=long

# Run single test with debugging
pytest tests/unit/test_deck_core.py::TestRosbagDeckInitialization::test_init_success -v -s

# Generate detailed coverage report
pytest --cov=rosbag_deck --cov-report=html --cov-report=term-missing
```

## Contributing

When adding new functionality:

1. **Write unit tests first** for new classes/methods
2. **Add integration tests** for new workflows
3. **Update fixtures** if new mock behavior is needed
4. **Maintain coverage** above 85%
5. **Add performance tests** for performance-critical code

The test suite follows the principle of testing the Python interface thoroughly while mocking the C dependencies, ensuring robust validation of the Python binding layer.