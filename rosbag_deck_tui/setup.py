#!/usr/bin/env python3

from setuptools import setup, find_packages

setup(
    name="rosbag_deck_tui",
    version="0.1.0",
    description="Terminal-based rosbag player using tape deck metaphor",
    author="LCTK Team",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "rosbag_deck>=0.1.0",  # Our Python wrapper
        "rich>=10.0.0",  # Rich terminal UI library
        "textual>=0.1.0",  # Modern TUI framework
        "numpy>=1.17.3,<1.25.0",  # Compatible with system scipy
    ],
    entry_points={
        "console_scripts": [
            "rosbag-tui=rosbag_deck_tui.main:main",
            "rosbag-deck-tui=rosbag_deck_tui.main:main",
        ],
    },
    zip_safe=True,
    package_data={
        "": ["package.xml"],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/rosbag_deck_tui"]),
        ("share/rosbag_deck_tui", ["package.xml"]),
    ],
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Multimedia :: Sound/Audio :: Players",
        "Topic :: System :: Distributed Computing",
        "Environment :: Console :: Curses",
    ],
)
