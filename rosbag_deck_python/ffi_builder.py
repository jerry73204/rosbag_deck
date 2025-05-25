#!/usr/bin/env python3
"""
CFFI builder for RosbagDeck C API

This script automatically generates Python FFI bindings from the C header file.
"""

import os
import sys
from cffi import FFI

def build_ffi():
    """Build CFFI bindings from C header"""
    ffi = FFI()
    
    # Find the C header file
    header_path = find_header_file()
    if not header_path:
        raise FileNotFoundError("Could not find rosbag_deck_c.h header file")
    
    print(f"Found header file: {header_path}")
    
    # Read and preprocess the header
    header_content = read_and_preprocess_header(header_path)
    
    # Define the C API for CFFI
    ffi.cdef(header_content)
    
    # Set the source - this will be loaded at runtime
    ffi.set_source(
        "rosbag_deck._cffi_bindings",
        '#include "rosbag_deck_core/rosbag_deck_c.h"',
        libraries=['rosbag_deck_core'],
        library_dirs=get_library_search_paths(),
        include_dirs=get_include_search_paths()
    )
    
    return ffi

def find_header_file():
    """Find the rosbag_deck_c.h header file"""
    # Search paths relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    search_paths = [
        # Relative to Python package (build directory)
        os.path.join(script_dir, "..", "rosbag_deck_core", "include", "rosbag_deck_core", "rosbag_deck_c.h"),
        # Installed location
        os.path.join(script_dir, "..", "install", "rosbag_deck_core", "include", "rosbag_deck_core", "rosbag_deck_c.h"),
        # System locations
        "/usr/local/include/rosbag_deck_core/rosbag_deck_c.h",
        "/usr/include/rosbag_deck_core/rosbag_deck_c.h",
    ]
    
    for path in search_paths:
        if os.path.exists(path):
            return path
    
    return None

def read_and_preprocess_header(header_path):
    """Read and preprocess the header file for CFFI"""
    with open(header_path, 'r') as f:
        content = f.read()
    
    # Remove includes that CFFI doesn't need
    lines = content.split('\n')
    processed_lines = []
    
    for line in lines:
        # Skip include directives
        if line.strip().startswith('#include'):
            continue
        # Skip header guards
        if line.strip().startswith('#ifndef') or line.strip().startswith('#define') or line.strip().startswith('#endif'):
            continue
        # Skip extern "C" blocks - CFFI handles this automatically
        if 'extern "C"' in line or line.strip() == '{' or (line.strip() == '}' and len(processed_lines) > 0):
            continue
        
        processed_lines.append(line)
    
    # Join and clean up
    processed_content = '\n'.join(processed_lines)
    
    # Remove any remaining preprocessor directives that might cause issues
    processed_content = processed_content.replace('#ifdef __cplusplus', '')
    processed_content = processed_content.replace('#endif', '')
    
    return processed_content

def get_library_search_paths():
    """Get library search paths for linking"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    return [
        # Build directory
        os.path.join(script_dir, "..", "install", "rosbag_deck_core", "lib"),
        # System paths
        "/usr/local/lib",
        "/usr/lib",
        # Current directory
        "."
    ]

def get_include_search_paths():
    """Get include search paths for compilation"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    return [
        # Build directory
        os.path.join(script_dir, "..", "rosbag_deck_core", "include"),
        os.path.join(script_dir, "..", "install", "rosbag_deck_core", "include"),
        # System paths
        "/usr/local/include",
        "/usr/include",
    ]

# Command-line interface functions
def build_main():
    """Build CFFI extension (from build_cffi.py)"""
    try:
        print("Building CFFI extension for RosbagDeck...")
        
        # Build the FFI
        ffi = build_ffi()
        
        # Compile the extension
        print("Compiling CFFI extension...")
        ffi.compile(verbose=True)
        
        print("✅ CFFI extension built successfully!")
        print("You can now use the rosbag_deck Python API.")
        
        return 0
        
    except ImportError as e:
        print(f"❌ Error: {e}")
        print("Please install CFFI: pip install cffi")
        return 1
    except Exception as e:
        print(f"❌ Error building CFFI extension: {e}")
        import traceback
        traceback.print_exc()
        return 1

def test_header_parsing():
    """Test if we can find and parse the header file"""
    print("Testing header file parsing...")
    
    try:
        header_path = find_header_file()
        if not header_path:
            print("❌ Could not find header file")
            return False
        
        print(f"✅ Found header file: {header_path}")
        
        # Test parsing
        content = read_and_preprocess_header(header_path)
        print(f"✅ Parsed header ({len(content)} characters)")
        
        # Show a sample of the parsed content
        lines = content.split('\n')[:10]
        print("Sample parsed content:")
        for i, line in enumerate(lines):
            if line.strip():
                print(f"  {i+1}: {line}")
        
        return True
        
    except Exception as e:
        print(f"❌ Error parsing header: {e}")
        return False

def test_cffi_build():
    """Test if CFFI extension can be built"""
    print("\nTesting CFFI build...")
    
    try:
        ffi = build_ffi()
        print("✅ CFFI FFI object created successfully")
        
        # Try to compile (this will create the extension)
        print("Compiling CFFI extension...")
        ffi.compile(verbose=True)
        print("✅ CFFI extension compiled successfully")
        
        return True
        
    except Exception as e:
        print(f"❌ Error building CFFI extension: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_import():
    """Test if the generated extension can be imported"""
    print("\nTesting import of generated extension...")
    
    try:
        from rosbag_deck._cffi_bindings import lib, ffi
        print("✅ Successfully imported CFFI bindings")
        
        # Test a simple function call
        handle = lib.rosbag_deck_create()
        if handle != ffi.NULL:
            print("✅ Successfully called rosbag_deck_create()")
            lib.rosbag_deck_destroy(handle)
            print("✅ Successfully called rosbag_deck_destroy()")
        else:
            print("⚠️  rosbag_deck_create() returned NULL (library might not be available)")
        
        return True
        
    except Exception as e:
        print(f"❌ Error importing CFFI bindings: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_main():
    """Run all tests (from test_build.py)"""
    print("CFFI Extension Build Test")
    print("=" * 40)
    
    tests = [
        ("Header Parsing", test_header_parsing),
        ("CFFI Build", test_cffi_build),
        ("Import Test", test_import),
    ]
    
    results = []
    for name, test_func in tests:
        print(f"\n{name}:")
        print("-" * len(name))
        success = test_func()
        results.append((name, success))
    
    print("\n" + "=" * 40)
    print("SUMMARY:")
    
    all_passed = True
    for name, success in results:
        status = "✅ PASS" if success else "❌ FAIL"
        print(f"  {name}: {status}")
        if not success:
            all_passed = False
    
    if all_passed:
        print("\n🎉 All tests passed! CFFI extension is working.")
        return 0
    else:
        print("\n❌ Some tests failed. Check the errors above.")
        return 1

if __name__ == "__main__":
    """Command-line interface"""
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "test":
        # Run tests
        sys.exit(test_main())
    else:
        # Build extension
        sys.exit(build_main())