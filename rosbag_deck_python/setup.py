#!/usr/bin/env python3

import os
from setuptools import setup, find_packages
from setuptools.command.build_py import build_py
from setuptools.command.develop import develop


class BuildWithCFFI(build_py):
    """Build command that also builds CFFI extension"""

    def run(self):
        # Run normal build first
        super().run()
        # Then build CFFI extension
        self.build_cffi()

    def build_cffi(self):
        """Build CFFI extension if possible"""
        try:
            print("Building CFFI extension...")
            import ffi_builder

            ffi = ffi_builder.build_ffi()
            ffi.compile(verbose=False)
            print("CFFI extension built successfully")
        except Exception as e:
            print(f"Warning: Failed to build CFFI extension: {e}")
            print(
                "The package will work but you may need to run 'python ffi_builder.py' manually"
            )


class DevelopWithCFFI(develop):
    """Develop command that also builds CFFI extension"""

    def run(self):
        # Run normal develop first
        super().run()
        # Then build CFFI extension
        self.build_cffi()

    def build_cffi(self):
        """Build CFFI extension if possible"""
        try:
            print("Building CFFI extension for development...")
            import ffi_builder

            ffi = ffi_builder.build_ffi()
            ffi.compile(verbose=False)
            print("CFFI extension built successfully")
        except Exception as e:
            print(f"Warning: Failed to build CFFI extension: {e}")
            print("You may need to run 'python ffi_builder.py' manually")


setup(
    name="rosbag_deck",
    version="0.1.0",
    description="Python API for RosbagDeck - tape deck style rosbag playback",
    author="LCTK Team",
    packages=find_packages(),
    python_requires=">=3.8",
    setup_requires=[
        "cffi>=1.0.0",
    ],
    install_requires=[
        "cffi>=1.0.0",
    ],
    cmdclass={
        "build_py": BuildWithCFFI,
        "develop": DevelopWithCFFI,
    },
    zip_safe=True,
    package_data={
        "": ["package.xml"],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/rosbag_deck_python"]),
        ("share/rosbag_deck_python", ["package.xml"]),
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
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: System :: Distributed Computing",
    ],
)
