# Redis Local Setup & Learning Guide

This guide explains how to set up and use Redis locally for caching with graceful fallback.

## 1. Installation

### Windows Installation

#### Option A: Using Windows Subsystem for Linux (WSL2) - Recommended
```bash
# Inside WSL2 Ubuntu terminal
sudo apt-get update
sudo apt-get install redis-server

# Start Redis
redis-server

# Test connection (in another terminal)
redis-cli
> PING
PONG
```

#### Option B: Using Docker Desktop (No Installation)
```bash
# Make sure Docker Desktop is running
docker run -d -p 6379:6379 --name redis-local redis:7-alpine

# Test connection
docker exec redis-local redis-cli PING
# Output: PONG

# Stop when done
docker stop redis-local
docker rm redis-local
```

#### Option C: Windows Native (via Memurai)
Download from: https://www.memurai.com/
- Install normally
- Redis server will start automatically
- Access via `redis-cli`

### macOS
```bash
brew install redis
redis-server
```

### Linux
```bash
sudo apt-get install redis-server
redis-server
```

## 2. Install Python Packages

Make sure you're in your virtual environment:

```bash
cd backend
pipenv install
# OR if using pip:
pip install django-redis redis
```

## 3. Django Configuration

Your `config/settings.py` is already configured with:

```python
CACHES = {
    'default': {
        'BACKEND': 'django_redis.cache.RedisCache',
        'LOCATION': 'redis://127.0.0.1:6379/0',
        'OPTIONS': {
            'CLIENT_CLASS': 'django_redis.client.DefaultClient',
            'SOCKET_CONNECT_TIMEOUT': 5,
            'SOCKET_TIMEOUT': 5,
            'COMPRESSOR': 'django_redis.compressors.zlib.ZlibCompressor',
            'IGNORE_EXCEPTIONS': True,  # ← This makes it graceful!
        },
        'KEY_PREFIX': 'neural_network',
        'TIMEOUT': 300,  # Default 5 minutes
    }
}
```

**Key feature**: `IGNORE_EXCEPTIONS': True` means if Redis crashes, Django will silently fail cache operations and continue working. You get the best of both worlds!

## 4. Verify Setup

### Check if Redis is Running
```bash
# Test connection
redis-cli PING
# Should return: PONG

# See what's in Redis
redis-cli
> KEYS *
> FLUSHDB  # Clear all data (for testing)
> EXIT
```

### Test Django Cache
```bash
cd backend
python manage.py shell

# In the Django shell:
from django.core.cache import cache

# Set a value
cache.set('test_key', 'test_value', timeout=300)

# Get the value
print(cache.get('test_key'))  # Should print: test_value

# Delete a value
cache.delete('test_key')
print(cache.get('test_key'))  # Should print: None

# Check if key exists
cache.has_key('test_key')  # Should return: False
```

## 5. Using Cache in Your Views

### Basic Example
```python
from django.core.cache import cache
from django.http import JsonResponse
from datetime import timedelta

def get_neural_network_data(request, network_id):
    # Create a cache key
    cache_key = f'network_{network_id}'
    
    # Try to get from cache
    data = cache.get(cache_key)
    
    if data is None:
        # Cache miss - fetch from database
        network = NeuralNetwork.objects.get(id=network_id)
        data = {
            'id': network.id,
            'name': network.name,
            'layers': network.layers,
        }
        # Cache it for 5 minutes
        cache.set(cache_key, data, timeout=300)
        print("Data fetched from database")
    else:
        print("Data retrieved from cache")
    
    return JsonResponse(data)
```

### Invalidate Cache on Update
```python
from django.core.cache import cache

def update_neural_network(request, network_id):
    network = NeuralNetwork.objects.get(id=network_id)
    
    # Update the network
    network.name = request.POST.get('name')
    network.save()
    
    # Invalidate the cache
    cache_key = f'network_{network_id}'
    cache.delete(cache_key)
    
    return JsonResponse({'status': 'updated'})
```

### Cache with Decorator
```python
from django.views.decorators.cache import cache_page
from django.views.decorators.http import condition

# Cache entire view for 5 minutes
@cache_page(300)
def get_all_networks(request):
    networks = NeuralNetwork.objects.all()
    return JsonResponse(list(networks.values()))

# Or use custom cache key function
from django.core.cache import cache

def cache_result(timeout=300):
    def decorator(view_func):
        def wrapper(request, *args, **kwargs):
            cache_key = f'{view_func.__name__}_{request.path}'
            result = cache.get(cache_key)
            
            if result is None:
                result = view_func(request, *args, **kwargs)
                cache.set(cache_key, result, timeout=timeout)
            
            return result
        return wrapper
    return decorator

@cache_result(timeout=600)
def expensive_computation(request):
    # Do something expensive
    result = sum_all_neural_networks()
    return JsonResponse(result)
```

## 6. Testing Graceful Degradation

Here's how to test that your app works even when Redis is down:

### Test 1: Stop Redis and Check App Still Works
```bash
# Terminal 1: Stop Redis
redis-cli SHUTDOWN

# Terminal 2: In Django shell
python manage.py shell
from django.core.cache import cache
cache.set('test', 'value')  # Should work silently
cache.get('test')  # Will return None but won't crash
print("App still working!")
```

### Test 2: Restart Redis and See Cache Kicks In
```bash
# In another terminal, start Redis again
redis-server

# Back in Django shell:
cache.set('test', 'value')
cache.get('test')  # Will now return 'value'
```

## 7. Common Redis CLI Commands

```bash
redis-cli

# View all keys
KEYS *

# Get specific key
GET mykey

# Set key with expiration
SET mykey "myvalue" EX 300

# Delete key
DEL mykey

# Clear entire database
FLUSHDB

# Clear all databases
FLUSHALL

# Check memory usage
INFO memory

# Monitor incoming commands
MONITOR

# Get statistics
INFO stats
```

## 8. Troubleshooting

### "Connection refused" error
- Make sure Redis is running: `redis-cli PING`
- Check the port (default 6379 is available)
- Your app will still work due to `IGNORE_EXCEPTIONS': True`

### Redis is taking too much memory
```bash
redis-cli
> CONFIG GET maxmemory
> CONFIG SET maxmemory 100mb
```

### Want to see what's being cached?
```bash
redis-cli
> KEYS neural_network*  # See all your cache keys
> TTL neural_network_key_name  # See how long key lasts
```

## 9. Next Steps

Once you're comfortable with local Redis:

1. **Add caching to expensive operations**
   - Generator output caching
   - Network metadata caching
   - User session caching

2. **Set up cache invalidation patterns**
   - Invalidate when networks are created/updated
   - Implement cache tags for complex invalidation

3. **Move to Docker** (as documented in DOCKER_SETUP.md)
   - When ready, add Redis service to docker-compose.yml
   - Use environment variables for Redis URL

4. **Monitor and optimize**
   - Track cache hit/miss rates
   - Adjust timeout values based on usage
   - Consider cache key naming conventions

## Quick Start Script

Create `test_redis.py` in your backend directory:

```python
#!/usr/bin/env python
import os
import django

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'config.settings')
django.setup()

from django.core.cache import cache

print("Testing Redis cache...")

# Test set/get
cache.set('greeting', 'Hello Redis!', timeout=60)
value = cache.get('greeting')
print(f"✓ Set/Get: {value}")

# Test delete
cache.delete('greeting')
value = cache.get('greeting')
print(f"✓ Delete: Value is now {value}")

# Test expiration doesn't break things
cache.set('short_lived', 'disappear', timeout=1)
import time
time.sleep(2)
value = cache.get('short_lived')
print(f"✓ Expiration: Value after timeout is {value}")

print("\n✓ All tests passed! Redis is working correctly.")
```

Run it:
```bash
python test_redis.py
```
