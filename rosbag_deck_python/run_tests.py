#!/usr/bin/env python3
"""
Test runner for rosbag_deck_python

Usage:
    python run_tests.py                    # Run all tests
    python run_tests.py unit               # Run only unit tests
    python run_tests.py integration        # Run only integration tests
    python run_tests.py --coverage         # Run with coverage report
"""

import sys
import subprocess
from pathlib import Path


def run_tests(test_type=None, coverage=False):
    """Run tests with pytest"""
    
    # Base pytest command
    cmd = ["python", "-m", "pytest"]
    
    # Add coverage if requested
    if coverage:
        cmd.extend([
            "--cov=rosbag_deck",
            "--cov-report=term-missing",
            "--cov-report=html:htmlcov"
        ])
    
    # Add test selection
    if test_type == "unit":
        cmd.extend(["-m", "unit", "tests/unit/"])
    elif test_type == "integration":
        cmd.extend(["-m", "integration", "tests/integration/"])
    elif test_type is None:
        cmd.append("tests/")
    else:
        print(f"Unknown test type: {test_type}")
        return 1
    
    # Add verbose output
    cmd.extend(["-v", "--tb=short"])
    
    print(f"Running: {' '.join(cmd)}")
    return subprocess.call(cmd)


def main():
    """Main entry point"""
    test_type = None
    coverage = False
    
    # Parse simple command line arguments
    for arg in sys.argv[1:]:
        if arg in ["unit", "integration"]:
            test_type = arg
        elif arg == "--coverage":
            coverage = True
        elif arg in ["-h", "--help"]:
            print(__doc__)
            return 0
        else:
            print(f"Unknown argument: {arg}")
            print(__doc__)
            return 1
    
    # Check if we're in the right directory
    if not Path("tests").exists():
        print("Error: tests directory not found. Run from rosbag_deck_python directory.")
        return 1
    
    return run_tests(test_type, coverage)


if __name__ == "__main__":
    sys.exit(main())