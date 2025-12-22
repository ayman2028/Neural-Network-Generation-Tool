"""
Manual test script to test the generate_network function
This creates a mock NetworkConfig and tests the generation
"""

import sys
import os
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from generator import generate_network, GenerationError


class MockNetworkConfig:
    """Mock NetworkConfig for testing"""
    def __init__(self, mode=1, input_size=10, T=16, R=False):
        self.id = 1
        self.mode = mode
        self.input_size = input_size
        self.T = T
        self.R = R
        self.name = f"test_network_{mode}_{input_size}_{T}"


def test_generate_network():
    """Test the generate_network function"""
    print("=" * 50)
    print("Manual Generation Test")
    print("=" * 50)
    
    # Create a mock network config
    network = MockNetworkConfig(
        mode=1,
        input_size=10,
        T=16,
        R=False
    )
    
    print(f"\nTesting with network config:")
    print(f"  Mode: {network.mode}")
    print(f"  Input Size: {network.input_size}")
    print(f"  Bit Width (T): {network.T}")
    print(f"  ReLU (R): {network.R}")
    
    try:
        print("\nGenerating network files...")
        zip_path = generate_network(network)
        
        if zip_path:
            print(f"✓ Generation successful!")
            print(f"  ZIP file: {zip_path}")
            print(f"  File exists: {Path(zip_path).exists()}")
            print(f"  File size: {Path(zip_path).stat().st_size} bytes")
            return True
        else:
            print(f"✗ Generation returned no path")
            return False
    
    except GenerationError as e:
        print(f"✗ Generation error: {e}")
        return False
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_different_modes():
    """Test generation with different modes"""
    print("\n" + "=" * 50)
    print("Testing Different Modes")
    print("=" * 50)
    
    modes = [1, 2, 3]
    results = []
    
    for mode in modes:
        try:
            network = MockNetworkConfig(mode=mode, input_size=10)
            print(f"\nTesting Mode {mode}...")
            zip_path = generate_network(network)
            print(f"✓ Mode {mode} successful: {Path(zip_path).name}")
            results.append(True)
        except GenerationError as e:
            print(f"✗ Mode {mode} failed: {e}")
            results.append(False)
        except Exception as e:
            print(f"✗ Mode {mode} error: {e}")
            results.append(False)
    
    print(f"\nMode test results: {sum(results)}/{len(results)} passed")
    return all(results)


if __name__ == '__main__':
    # Run basic test
    basic_result = test_generate_network()
    
    # Uncomment to test different modes
    # modes_result = test_different_modes()
    
    print("\n" + "=" * 50)
    print("Test Complete")
    print("=" * 50)
    
    sys.exit(0 if basic_result else 1)
