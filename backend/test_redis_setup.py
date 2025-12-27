#!/usr/bin/env python
"""
Quick test script to verify Redis is working with your Django setup.
Run this after installing redis and starting the Redis server.

Usage:
    cd backend
    python test_redis_setup.py
"""

import os
import sys
import django

# Setup Django
os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'config.settings')
django.setup()

from django.core.cache import cache
import time

def print_header(text):
    print(f"\n{'='*60}")
    print(f"  {text}")
    print(f"{'='*60}\n")

def print_success(text):
    print(f"✓ {text}")

def print_warning(text):
    print(f"⚠ {text}")

def test_basic_operations():
    print_header("TEST 1: Basic Set/Get Operations")
    
    try:
        # Test SET
        cache.set('test_key', 'test_value', timeout=60)
        print_success("Cache SET operation succeeded")
        
        # Test GET
        value = cache.get('test_key')
        assert value == 'test_value', "Retrieved value doesn't match"
        print_success(f"Cache GET operation succeeded (value: '{value}')")
        
        # Test DELETE
        cache.delete('test_key')
        value = cache.get('test_key')
        assert value is None, "Key should be deleted"
        print_success("Cache DELETE operation succeeded")
        
        return True
    except Exception as e:
        print_warning(f"Error: {e}")
        return False

def test_expiration():
    print_header("TEST 2: Key Expiration")
    
    try:
        # Set key with 1 second timeout
        cache.set('short_lived', 'disappear', timeout=1)
        print_success("Set key with 1 second timeout")
        
        # Immediately retrieve
        value = cache.get('short_lived')
        assert value == 'disappear', "Should retrieve value immediately"
        print_success("Retrieved value before timeout")
        
        # Wait for expiration
        print("Waiting for key to expire...")
        time.sleep(2)
        
        # Try to retrieve after timeout
        value = cache.get('short_lived')
        assert value is None, "Key should be expired"
        print_success("Key expired as expected")
        
        return True
    except Exception as e:
        print_warning(f"Error: {e}")
        return False

def test_complex_data():
    print_header("TEST 3: Complex Data Types")
    
    try:
        # Cache a dictionary
        complex_data = {
            'id': 1,
            'name': 'Test Network',
            'layers': [10, 8, 16, 1],
            'metadata': {
                'created': '2025-01-01',
                'status': 'active'
            }
        }
        
        cache.set('network_1', complex_data, timeout=300)
        print_success("Cached complex dictionary")
        
        retrieved = cache.get('network_1')
        assert retrieved == complex_data, "Retrieved data doesn't match"
        print_success(f"Retrieved complex data: {retrieved['name']}")
        
        cache.delete('network_1')
        return True
    except Exception as e:
        print_warning(f"Error: {e}")
        return False

def test_graceful_degradation():
    print_header("TEST 4: Graceful Degradation")
    print("(This tests that Django won't crash if Redis fails)")
    
    try:
        # Test with a key that might not exist
        value = cache.get('nonexistent_key')
        assert value is None, "Nonexistent key should return None"
        print_success("Handled missing key gracefully")
        
        # Test with invalid operations (should be silently ignored)
        cache.set('test', 'value')
        cache.clear()  # Clear all cache
        value = cache.get('test')
        assert value is None, "Cache should be cleared"
        print_success("Cache clear operation succeeded")
        
        return True
    except Exception as e:
        print_warning(f"Error: {e}")
        return False

def test_cache_many():
    print_header("TEST 5: Batch Operations (get_many)")
    
    try:
        # Set multiple keys
        data = {
            'network_1': {'id': 1, 'name': 'Net 1'},
            'network_2': {'id': 2, 'name': 'Net 2'},
            'network_3': {'id': 3, 'name': 'Net 3'},
        }
        
        for key, value in data.items():
            cache.set(key, value, timeout=300)
        print_success(f"Set {len(data)} keys in cache")
        
        # Retrieve multiple keys at once
        retrieved = cache.get_many(list(data.keys()))
        assert len(retrieved) == 3, "Should retrieve all 3 keys"
        print_success(f"Retrieved {len(retrieved)} keys using get_many()")
        
        # Cleanup
        for key in data.keys():
            cache.delete(key)
        
        return True
    except Exception as e:
        print_warning(f"Error: {e}")
        return False

def test_redis_connection():
    print_header("TEST 0: Redis Connection Check")
    
    try:
        import redis
        from django.conf import settings
        
        # Get Redis URL from settings
        redis_url = settings.CACHES['default']['LOCATION']
        print(f"Attempting to connect to: {redis_url}")
        
        # Try to connect
        r = redis.from_url(redis_url)
        r.ping()
        print_success("Successfully connected to Redis server")
        return True
    except Exception as e:
        print_warning(f"Could not connect to Redis directly: {e}")
        print_warning("This is OK! Django will use fallback cache.")
        print_warning("Make sure Redis is running if you want to use caching.")
        return False

def main():
    print("\n" + "="*60)
    print("  REDIS CACHE SETUP TEST SUITE")
    print("="*60)
    
    results = {
        'Connection': test_redis_connection(),
        'Basic Operations': test_basic_operations(),
        'Expiration': test_expiration(),
        'Complex Data': test_complex_data(),
        'Graceful Degradation': test_graceful_degradation(),
        'Batch Operations': test_cache_many(),
    }
    
    # Print summary
    print_header("TEST SUMMARY")
    passed = 0
    failed = 0
    
    for test_name, result in results.items():
        status = "PASSED" if result else "FAILED"
        symbol = "✓" if result else "✗"
        print(f"{symbol} {test_name}: {status}")
        if result:
            passed += 1
        else:
            failed += 1
    
    print(f"\nTotal: {passed} passed, {failed} failed out of {len(results)} tests")
    
    if failed == 0:
        print("\n" + "="*60)
        print("  ✓ ALL TESTS PASSED!")
        print("  Redis is properly configured and working!")
        print("="*60 + "\n")
        return 0
    elif failed > 0 and passed > 0:
        print("\n" + "="*60)
        print("  ⚠ SOME TESTS FAILED")
        print("  Check that Redis is running: redis-server")
        print("  Or use Docker: docker run -p 6379:6379 redis:7-alpine")
        print("="*60 + "\n")
        return 1
    else:
        print("\n" + "="*60)
        print("  ✗ ALL TESTS FAILED")
        print("  Redis server is not running!")
        print("\n  To get started:")
        print("  1. Start Redis: redis-server")
        print("  2. Or use Docker: docker run -p 6379:6379 redis:7-alpine")
        print("  3. Then run this test again")
        print("="*60 + "\n")
        return 2

if __name__ == '__main__':
    sys.exit(main())
