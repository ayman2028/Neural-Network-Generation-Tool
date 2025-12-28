"""
Custom Redis-backed rate limiter for Django views.
Uses a sliding window counter approach for accurate rate limiting.
"""

import redis
import logging
from datetime import datetime
from typing import Tuple
from functools import wraps
from django.http import HttpResponse
from django_redis import get_redis_connection

logger = logging.getLogger(__name__)


class RedisRateLimiter:
    """
    Rate limiter using Redis sorted sets for sliding window tracking.
    
    Example usage:
        redis_client = get_redis_connection("default")
        limiter = RedisRateLimiter(redis_client)
        
        # In view:
        client_ip = request.META.get('REMOTE_ADDR')
        if not limiter.is_allowed(client_ip, limit=100, window_seconds=3600):
            return HttpResponse('Rate limit exceeded', status=429)
    """
    
    def __init__(self, redis_client: redis.Redis, key_prefix: str = 'rate_limit'):
        """
        Initialize the rate limiter.
        
        Args:
            redis_client: Redis connection instance
            key_prefix: Prefix for Redis keys (default: 'rate_limit')
        """
        self.redis = redis_client
        self.key_prefix = key_prefix
    
    def is_allowed(self, identifier: str, limit: int = 100, window_seconds: int = 3600) -> bool:
        """
        Check if an action is allowed within rate limit using sliding window.
        
        Args:
            identifier: Unique identifier (user_id, IP address, etc.)
            limit: Maximum requests allowed in the window
            window_seconds: Time window in seconds (default: 1 hour)
        
        Returns:
            True if request is allowed, False if rate limited
        """
        key = f"{self.key_prefix}:{identifier}"
        now = datetime.now().timestamp()
        window_start = now - window_seconds
        
        # Remove entries outside the current window
        self.redis.zremrangebyscore(key, 0, window_start)
        
        # Count requests in current window
        current_count = self.redis.zcard(key)
        
        if current_count < limit:
            # Add current request with current timestamp as score
            self.redis.zadd(key, {str(now): now})
            # Set key expiration to clean up old keys
            self.redis.expire(key, window_seconds + 1)
            return True
        
        return False
    
    def get_remaining(self, identifier: str, limit: int = 100, window_seconds: int = 3600) -> int:
        """
        Get the number of remaining requests for an identifier.
        
        Args:
            identifier: Unique identifier (user_id, IP address, etc.)
            limit: Maximum requests allowed in the window
            window_seconds: Time window in seconds (default: 1 hour)
        
        Returns:
            Number of remaining requests allowed
        """
        key = f"{self.key_prefix}:{identifier}"
        now = datetime.now().timestamp()
        window_start = now - window_seconds
        
        # Remove old entries
        self.redis.zremrangebyscore(key, 0, window_start)
        current_count = self.redis.zcard(key)
        
        return max(0, limit - current_count)
    
    def get_stats(self, identifier: str, limit: int = 100, window_seconds: int = 3600) -> dict:
        """
        Get detailed rate limit stats for an identifier.
        
        Args:
            identifier: Unique identifier (user_id, IP address, etc.)
            limit: Maximum requests allowed in the window
            window_seconds: Time window in seconds (default: 1 hour)
        
        Returns:
            Dictionary with limit stats
        """
        key = f"{self.key_prefix}:{identifier}"
        now = datetime.now().timestamp()
        window_start = now - window_seconds
        
        # Remove old entries
        self.redis.zremrangebyscore(key, 0, window_start)
        current_count = self.redis.zcard(key)
        
        remaining = max(0, limit - current_count)
        
        return {
            'current_count': current_count,
            'limit': limit,
            'remaining': remaining,
            'window_seconds': window_seconds,
            'reset_in_seconds': self.redis.ttl(key) if self.redis.exists(key) else 0
        }
    
    def reset(self, identifier: str) -> bool:
        """
        Reset rate limit for an identifier.
        
        Args:
            identifier: Unique identifier to reset
        
        Returns:
            True if key was deleted, False if it didn't exist
        """
        key = f"{self.key_prefix}:{identifier}"
        return bool(self.redis.delete(key))


def rate_limit(requests: int = 3, window: int = 60):
    """
    Decorator to rate limit a Django view by IP address.
    
    Default: 3 requests per 60 seconds (per IP)
    
    Each view has its OWN separate rate limit counter.
    
    Graceful degradation: If Redis is unavailable, requests are allowed through
    (fails open for better user experience)
    
    Args:
        requests: Number of requests allowed in the window (default: 3)
        window: Time window in seconds (default: 60)
    
    Usage:
        @rate_limit()
        def my_view(request):
            return HttpResponse("Hello")
        
        # With custom limits:
        @rate_limit(requests=5, window=120)
        def my_view(request):
            return HttpResponse("Hello")
    
    Returns:
        429 Too Many Requests if rate limit exceeded
        Response with X-RateLimit headers if allowed
        Normal response if Redis is unavailable (graceful fallback)
    """
    def decorator(view_func):
        @wraps(view_func)
        def wrapper(request, *args, **kwargs):
            try:
                # Get Redis connection and create limiter
                redis_client = get_redis_connection("default")
                # Use view function name in key prefix to keep separate counters per view
                view_name = view_func.__name__
                limiter = RedisRateLimiter(redis_client, key_prefix=f'rate_limit:{view_name}')
                
                # Get client IP address
                client_ip = request.META.get('REMOTE_ADDR', 'unknown')
                
                # Check rate limit
                if not limiter.is_allowed(client_ip, limit=requests, window_seconds=window):
                    # Rate limited - return 429
                    response = HttpResponse(
                        'Rate limit exceeded. Maximum {} requests per {} seconds.'.format(requests, window),
                        status=429
                    )
                    stats = limiter.get_stats(client_ip, limit=requests, window_seconds=window)
                    response['X-RateLimit-Limit'] = str(requests)
                    response['X-RateLimit-Remaining'] = str(stats['remaining'])
                    response['X-RateLimit-Reset'] = str(int(datetime.now().timestamp()) + stats['reset_in_seconds'])
                    return response
                
                # Request allowed - call the view
                response = view_func(request, *args, **kwargs)
                
                # Add rate limit headers to response
                stats = limiter.get_stats(client_ip, limit=requests, window_seconds=window)
                response['X-RateLimit-Limit'] = str(requests)
                response['X-RateLimit-Remaining'] = str(stats['remaining'])
                response['X-RateLimit-Reset'] = str(int(datetime.now().timestamp()) + stats['reset_in_seconds'])
                
                return response
            
            except Exception as e:
                # Redis is unavailable or other error occurred
                # Fail open: allow the request through but log the error
                logger.warning(
                    f"Rate limiter error for view '{view_func.__name__}' on IP {request.META.get('REMOTE_ADDR', 'unknown')}: {str(e)}. "
                    f"Allowing request through (Redis may be unavailable)."
                )
                
                # Call the view normally without rate limiting
                response = view_func(request, *args, **kwargs)
                
                # Add header indicating rate limiting was skipped
                response['X-RateLimit-Status'] = 'disabled'
                
                return response
        
        return wrapper
    return decorator
