"""
Test script for the generator module
Run this to verify the generator setup is working correctly
"""

import sys
import os
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from generator import (
    get_executable_path,
    get_generator_base_path,
    GenerationError
)


def test_executable_exists():
    """Test that the main executable exists"""
    print("Testing: Executable existence...")
    try:
        exe_path = get_executable_path('main')
        print(f"✓ Executable found: {exe_path}")
        return True
    except GenerationError as e:
        print(f"✗ Executable not found: {e}")
        return False


def test_generator_base_path():
    """Test that we can get the generator base path"""
    print("\nTesting: Generator base path...")
    try:
        base_path = get_generator_base_path()
        print(f"✓ Base path: {base_path}")
        print(f"✓ Path exists: {base_path.exists()}")
        return True
    except Exception as e:
        print(f"✗ Error getting base path: {e}")
        return False


def test_outputs_directory():
    """Test that outputs directory can be created"""
    print("\nTesting: Outputs directory...")
    try:
        base_path = get_generator_base_path()
        outputs_dir = base_path / 'outputs'
        outputs_dir.mkdir(parents=True, exist_ok=True)
        print(f"✓ Outputs directory: {outputs_dir}")
        print(f"✓ Directory exists: {outputs_dir.exists()}")
        return True
    except Exception as e:
        print(f"✗ Error creating outputs directory: {e}")
        return False


def test_executable_callable():
    """Test that the executable can be called"""
    print("\nTesting: Executable callable...")
    try:
        import subprocess
        exe_path = get_executable_path('main')
        
        # Try to run with --help or without args to see if it responds
        result = subprocess.run(
            [str(exe_path)],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        print(f"✓ Executable ran (return code: {result.returncode})")
        if result.stdout:
            print(f"  Output: {result.stdout[:100]}")
        return True
    except subprocess.TimeoutExpired:
        print(f"✗ Executable timed out")
        return False
    except Exception as e:
        print(f"✗ Error running executable: {e}")
        return False


def run_all_tests():
    """Run all tests"""
    print("=" * 50)
    print("Generator Module Tests")
    print("=" * 50)
    
    tests = [
        test_generator_base_path,
        test_executable_exists,
        test_outputs_directory,
        test_executable_callable,
    ]
    
    results = []
    for test in tests:
        try:
            results.append(test())
        except Exception as e:
            print(f"✗ Test error: {e}")
            results.append(False)
    
    print("\n" + "=" * 50)
    print(f"Results: {sum(results)}/{len(results)} tests passed")
    print("=" * 50)
    
    return all(results)


if __name__ == '__main__':
    success = run_all_tests()
    sys.exit(0 if success else 1)
