#!/usr/bin/env python
"""
Redis Cache Examples for Neural Network Generation Tool

This file demonstrates practical ways to use Redis caching in your Django app.
Copy these examples into your views.py to start using caching!
"""

from django.core.cache import cache
from django.http import JsonResponse
from django.views.decorators.http import require_http_methods
import time


# ============================================================================
# EXAMPLE 1: Simple Cache Set/Get
# ============================================================================

def example_simple_cache(request):
    """
    Demonstrates basic cache operations.
    """
    cache.set('simple_key', 'Hello from Redis!', timeout=300)
    value = cache.get('simple_key')
    
    return JsonResponse({
        'status': 'success',
        'cached_value': value
    })


# ============================================================================
# EXAMPLE 2: Cache with Fallback (What we recommend!)
# ============================================================================

def get_network_with_cache(request, network_id):
    """
    Fetch network data with caching.
    If Redis is down, it still works - just fetches from DB every time.
    """
    cache_key = f'network_{network_id}'
    
    # Try to get from cache
    network_data = cache.get(cache_key)
    from_cache = True
    
    if network_data is None:
        # Cache miss - fetch from database
        from_cache = False
        try:
            # Example: replace with your actual model
            # network = NeuralNetwork.objects.get(id=network_id)
            network_data = {
                'id': network_id,
                'name': f'Network {network_id}',
                'layers': [10, 8, 16, 1],
                'timestamp': time.time()
            }
            # Cache it for 5 minutes
            cache.set(cache_key, network_data, timeout=300)
        except Exception as e:
            return JsonResponse({'error': str(e)}, status=404)
    
    return JsonResponse({
        'data': network_data,
        'from_cache': from_cache,
        'cache_working': True
    })


# ============================================================================
# EXAMPLE 3: Cache Invalidation on Update
# ============================================================================

@require_http_methods(["POST"])
def update_network(request, network_id):
    """
    When you update something, invalidate its cache so fresh data is fetched next time.
    """
    # Do your update
    # network = NeuralNetwork.objects.get(id=network_id)
    # network.name = request.POST.get('name')
    # network.save()
    
    # Invalidate the cache
    cache_key = f'network_{network_id}'
    cache.delete(cache_key)
    
    return JsonResponse({
        'status': 'updated',
        'message': f'Network {network_id} updated and cache cleared'
    })


# ============================================================================
# EXAMPLE 4: Cache Multiple Related Items
# ============================================================================

def get_network_with_generator_results(request, network_id):
    """
    Cache both the network and its generator results together.
    """
    network_cache_key = f'network_{network_id}'
    results_cache_key = f'network_{network_id}_results'
    
    # Get both from cache
    network_data = cache.get(network_cache_key)
    results = cache.get(results_cache_key)
    
    if network_data is None or results is None:
        # Fetch from database if either is missing
        network_data = {
            'id': network_id,
            'name': f'Network {network_id}',
        }
        results = {
            'output_file': f'output_{network_id}.sv',
            'status': 'generated'
        }
        
        # Cache both together
        cache.set(network_cache_key, network_data, timeout=600)
        cache.set(results_cache_key, results, timeout=600)
    
    return JsonResponse({
        'network': network_data,
        'results': results
    })


# ============================================================================
# EXAMPLE 5: Delete Pattern (Clear All Related Cache)
# ============================================================================

def clear_network_cache(request, network_id):
    """
    When you delete or significantly change something, clear all related cache.
    """
    keys_to_delete = [
        f'network_{network_id}',
        f'network_{network_id}_results',
        f'network_{network_id}_metadata',
    ]
    
    for key in keys_to_delete:
        cache.delete(key)
    
    return JsonResponse({
        'status': 'cache_cleared',
        'keys_deleted': keys_to_delete
    })


# ============================================================================
# EXAMPLE 6: Cache with Conditional Logic
# ============================================================================

def get_expensive_computation(request):
    """
    Cache the result of an expensive operation.
    """
    cache_key = 'expensive_computation_result'
    result = cache.get(cache_key)
    
    if result is None:
        # Simulate expensive computation
        print("Performing expensive computation...")
        time.sleep(2)  # Simulate work
        result = {
            'computation': 'done',
            'data': list(range(1000)),
            'timestamp': time.time()
        }
        
        # Cache for 10 minutes since it's expensive
        cache.set(cache_key, result, timeout=600)
    
    return JsonResponse(result)


# ============================================================================
# EXAMPLE 7: Test Cache Health
# ============================================================================

def test_cache_health(request):
    """
    Check if Redis is working and return diagnostics.
    Useful for monitoring!
    """
    test_key = 'health_check_test'
    test_value = f'test_{time.time()}'
    
    try:
        # Try to set and get
        cache.set(test_key, test_value, timeout=10)
        retrieved = cache.get(test_key)
        
        if retrieved == test_value:
            cache.delete(test_key)
            return JsonResponse({
                'status': 'healthy',
                'redis': 'connected',
                'cache_working': True
            })
        else:
            return JsonResponse({
                'status': 'degraded',
                'message': 'Cache set/get mismatch',
                'cache_working': False
            })
    except Exception as e:
        # If we get here, Redis is definitely down
        # But app still works due to IGNORE_EXCEPTIONS!
        return JsonResponse({
            'status': 'offline',
            'redis': 'not_connected',
            'cache_working': False,
            'message': 'App will continue working without caching',
            'error': str(e)
        })


# ============================================================================
# EXAMPLE 8: Batch Cache Operations
# ============================================================================

def get_multiple_networks(request):
    """
    Efficiently cache multiple items.
    """
    network_ids = [1, 2, 3, 4, 5]
    
    # Try to get all from cache
    cache_keys = {nid: f'network_{nid}' for nid in network_ids}
    cached_data = cache.get_many(cache_keys.values())
    
    networks = {}
    for nid in network_ids:
        key = cache_keys[nid]
        if key in cached_data:
            networks[nid] = cached_data[key]
        else:
            # Simulate fetching from DB
            networks[nid] = {
                'id': nid,
                'name': f'Network {nid}'
            }
            # Cache it
            cache.set(key, networks[nid], timeout=300)
    
    return JsonResponse({'networks': networks})


# ============================================================================
# EXAMPLE 9: Cache with Custom Timeout
# ============================================================================

def cache_by_importance(request):
    """
    Different cache durations for different types of data.
    """
    # Short-lived cache (1 minute)
    cache.set('frequently_changing_data', 'value', timeout=60)
    
    # Medium-lived cache (30 minutes)
    cache.set('moderately_stable_data', 'value', timeout=1800)
    
    # Long-lived cache (1 hour)
    cache.set('rarely_changing_data', 'value', timeout=3600)
    
    return JsonResponse({'status': 'cache_set'})


# ============================================================================
# EXAMPLE 10: Debug - View All Cache Keys
# ============================================================================

def debug_view_cache_keys(request):
    """
    CAUTION: Only use this in development!
    Never expose cache keys in production.
    """
    if not request.user.is_staff:
        return JsonResponse({'error': 'Unauthorized'}, status=403)
    
    # Note: You'll need to use redis-cli to list all keys
    # This is a limitation of django-redis
    # For production monitoring, use dedicated Redis tools
    
    return JsonResponse({
        'message': 'To see all cache keys, use: redis-cli KEYS neural_network*',
        'tip': 'Run this in your terminal when Redis is running'
    })


# ============================================================================
# URLS Configuration
# ============================================================================

"""
Add these to your urls.py to test the examples:

from django.urls import path
from . import views

urlpatterns = [
    path('test/simple/', views.example_simple_cache),
    path('network/<int:network_id>/', views.get_network_with_cache),
    path('network/<int:network_id>/update/', views.update_network),
    path('network/<int:network_id>/results/', views.get_network_with_generator_results),
    path('network/<int:network_id>/clear/', views.clear_network_cache),
    path('expensive/', views.get_expensive_computation),
    path('health/', views.test_cache_health),
    path('networks/', views.get_multiple_networks),
    path('debug/cache-keys/', views.debug_view_cache_keys),
]
"""
