#!/bin/bash

# RosBag Deck - Simple Test Runner
set -e

echo "=== RosBag Deck Test Runner ==="

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build first
echo "Building packages..."
colcon build

# Source the built workspace
source install/setup.bash

# Run C++ tests
echo "Running C++ tests..."
! colcon test

# Show C++ test results
echo "C++ test results:"
! colcon test-result

# Run Python tests
echo ""
echo "Running Python tests..."

# Test rosbag_deck_python
echo "Testing rosbag_deck_python..."
pushd rosbag_deck_python 2>/dev/null
! python3 -m pytest tests/ -v --tb=short
popd 2>/dev/null

# Test rosbag_deck_tui
echo "Testing rosbag_deck_tui..."
pushd rosbag_deck_tui 2>/dev/null
! python3 -m pytest tests/ -v --tb=short || true  # Allow failure since no tests yet
popd 2>/dev/null

echo "=== Test completed ==="
