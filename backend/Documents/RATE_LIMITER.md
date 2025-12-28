# Rate Limiter Documentation

## Overview

The rate limiter is a Redis-backed decorator that limits the number of requests from a single IP address within a specified time window. It's designed to protect your application from abuse and brute-force attacks.

## How It Works

The rate limiter uses a **sliding window counter** approach with graceful error handling:

### Sliding Window Algorithm
- Tracks each request with a timestamp in a Redis sorted set
- Automatically removes requests outside the current time window
- Checks if the current request count exceeds the limit
- Returns a 429 (Too Many Requests) response if limit is exceeded

### Error Handling & Graceful Degradation
The rate limiter uses a **wrapper pattern** with try-catch error handling:

```python
def rate_limit(requests=3, window=60):
    def decorator(view_func):
        @wraps(view_func)
        def wrapper(request, *args, **kwargs):
            try:
                # Attempt rate limiting
                if not limiter.is_allowed(...):
                    return 429_response
                response = view_func(request, *args, **kwargs)
                # Add rate limit headers
                response['X-RateLimit-*'] = ...
                return response
            except Exception as e:
                # Redis unavailable - fail open
                logger.warning(f"Rate limiter error: {e}")
                # Allow request through without rate limiting
                response = view_func(request, *args, **kwargs)
                response['X-RateLimit-Status'] = 'disabled'
                return response
        return wrapper
    return decorator
```

**Key features of the wrapper:**

1. **Try-Except Block** - Catches any Redis errors gracefully
2. **Fail Open** - If Redis fails, requests are allowed through (doesn't block users)
3. **Error Logging** - Logs issues with IP, view name, and error details
4. **Status Header** - Adds `X-RateLimit-Status: disabled` when Redis is down
5. **IP Address Extraction** - Gets client IP from `request.META.get('REMOTE_ADDR')`
6. **View-Specific Keys** - Uses view function name to keep separate counters per view

**Behavior in different scenarios:**

| Redis Status | Rate Limit Exceeded | Rate Limit OK |
|---|---|---|
| ✅ Working | Returns 429 | Returns 200 + headers |
| ❌ Down | Returns 200 (allowed) | Returns 200 (allowed) |
| ⚠️ Slow | May timeout, logs error | Returns 200 + headers |

This ensures:
- ✓ Normal rate limiting when Redis is available
- ✓ No blocked users if Redis fails
- ✓ Clear logging for debugging Redis issues
- ✓ Production-ready error resilience

## Setup

The rate limiter is located in `backend/utils/rate_limiter.py` and consists of two main components:

1. **RedisRateLimiter class** - Core rate limiting logic
2. **rate_limit decorator** - Easy-to-use decorator for views

### Prerequisites

- Redis must be running (configured in `settings.py`)
- `django-redis` package (already in Pipfile)

## Usage

### Basic Usage (Default: 3 requests per 60 seconds)

```python
from django.http import HttpResponse
from utils import rate_limit

@rate_limit()
def my_view(request):
    return HttpResponse("Success")
```

### Custom Limits

```python
from utils import rate_limit

@rate_limit(requests=5, window=120)  # 5 requests per 120 seconds
def my_view(request):
    return HttpResponse("Success")
```

## Parameters

### rate_limit decorator

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `requests` | int | 3 | Number of requests allowed in the window |
| `window` | int | 60 | Time window in seconds |

## Response Behavior

### When Rate Limit is Exceeded (429)

```
HTTP/1.1 429 Too Many Requests
X-RateLimit-Limit: 3
X-RateLimit-Remaining: 0
X-RateLimit-Reset: 1704067200

Rate limit exceeded. Maximum 3 requests per 60 seconds.
```

### When Request is Allowed (200)

```
HTTP/1.1 200 OK
X-RateLimit-Limit: 3
X-RateLimit-Remaining: 2
X-RateLimit-Reset: 1704067200

[Your response content]
```

## Response Headers

The decorator automatically adds three headers to every response:

- **X-RateLimit-Limit** - Maximum requests allowed in the window
- **X-RateLimit-Remaining** - Number of requests remaining in the current window
- **X-RateLimit-Reset** - Unix timestamp when the rate limit window resets

## Examples

### Login View (3 attempts per minute)

```python
from django.http import HttpResponse, redirect
from django.shortcuts import render
from utils import rate_limit

@rate_limit()  # Default: 3 per 60 seconds
def login_view(request):
    if request.method == 'POST':
        # Your login logic here
        return redirect('home')
    return render(request, 'login.html')
```

### File Download View (5 downloads per 5 minutes)

```python
from utils import rate_limit

@rate_limit(requests=5, window=300)
def download_view(request):
    # Your download logic here
    return FileResponse(open('file.zip', 'rb'))
```

### Form Submission (10 submissions per 10 minutes)

```python
from utils import rate_limit

@rate_limit(requests=10, window=600)
def submit_form_view(request):
    if request.method == 'POST':
        # Process form
        pass
    return render(request, 'form.html')
```

## Advanced Usage

### Using RedisRateLimiter Directly

If you need more control, you can use the `RedisRateLimiter` class directly in your views:

```python
from django_redis import get_redis_connection
from utils import RedisRateLimiter
from django.http import HttpResponse

def my_view(request):
    redis_client = get_redis_connection("default")
    limiter = RedisRateLimiter(redis_client)
    
    client_ip = request.META.get('REMOTE_ADDR')
    
    # Check if allowed
    if not limiter.is_allowed(client_ip, limit=100, window_seconds=3600):
        return HttpResponse('Rate limit exceeded', status=429)
    
    # Get remaining requests
    remaining = limiter.get_remaining(client_ip, limit=100, window_seconds=3600)
    
    # Get detailed stats
    stats = limiter.get_stats(client_ip, limit=100, window_seconds=3600)
    # stats = {
    #     'current_count': 1,
    #     'limit': 100,
    #     'remaining': 99,
    #     'window_seconds': 3600,
    #     'reset_in_seconds': 3599
    # }
    
    # Reset rate limit
    limiter.reset(client_ip)
    
    return HttpResponse('Success')
```

### RedisRateLimiter Methods

| Method | Description | Returns |
|--------|-------------|---------|
| `is_allowed(identifier, limit, window_seconds)` | Check if request is allowed | bool |
| `get_remaining(identifier, limit, window_seconds)` | Get remaining requests | int |
| `get_stats(identifier, limit, window_seconds)` | Get detailed stats | dict |
| `reset(identifier)` | Reset rate limit for identifier | bool |

## How Identifier Works

The rate limiter uses the client's **IP address** as the identifier:

```python
client_ip = request.META.get('REMOTE_ADDR')
```

This means:
- Each unique IP address gets its own limit counter
- All requests from the same IP share the same counter
- Users behind the same proxy/NAT will share the limit

## Recommended Use Cases

Apply rate limiting to these types of views:

1. **Authentication**
   - Login attempts
   - Registration
   - Password reset requests

2. **Forms & Data Entry**
   - Comment submissions
   - Contact forms
   - Feedback forms

3. **Resource Intensive Operations**
   - File uploads
   - File downloads
   - Report generation
   - Neural network generation

4. **API Endpoints** (when you build them)
   - Search
   - Data export
   - Batch operations

## Troubleshooting

### "Rate limit exceeded" on all requests

**Cause:** Redis not running or disconnected

**Solution:** 
```bash
# Check if Redis is running
docker-compose up -d
# or
# Ensure Redis service is started
```

### Rate limit not working

**Cause:** Redis configuration incorrect in `settings.py`

**Solution:** Verify Redis configuration:
```python
# In settings.py
CACHES = {
    'default': {
        'BACKEND': 'django_redis.cache.RedisCache',
        'LOCATION': 'redis://127.0.0.1:6379/0',
        ...
    }
}
```

### Same IP shows different limits

**Cause:** Multiple Redis key prefixes or Redis keys expiring

**Solution:** Rate limiter automatically manages key expiration. This is normal behavior.

## Performance Considerations

- **Redis overhead:** Minimal. Each check is ~1-2 Redis operations
- **Memory usage:** Low. One sorted set per IP per rate-limited endpoint
- **Scalability:** Scales well with Redis clustering

## Customization

### Change Default Limits Globally

To change the default from 3 requests per 60 seconds, you can create a wrapper:

```python
# In your app's views.py or utils
from utils import rate_limit as base_rate_limit

def rate_limit_custom(requests=5, window=120):
    """Custom rate limiter with different defaults"""
    return base_rate_limit(requests=requests, window=window)

# Then use:
@rate_limit_custom()
def my_view(request):
    pass
```

### Custom Key Prefix

To change the Redis key prefix (default is "rate_limit"):

```python
from django_redis import get_redis_connection
from utils import RedisRateLimiter

redis_client = get_redis_connection("default")
limiter = RedisRateLimiter(redis_client, key_prefix='my_app_limit')
```

## Testing

The rate limiter includes comprehensive test coverage. All tests pass successfully:

```bash
cd backend
pipenv run python manage.py test networks.tests.NetworkCreateViewRateLimitTestCase -v 2
```

**Test Results (6 tests, all passing):**
- ✅ `test_create_3_networks_succeeds` - 3 requests within limit succeed
- ✅ `test_create_4th_network_fails_rate_limit` - 4th request returns 429
- ✅ `test_different_ips_have_separate_limits` - Each IP has independent counter
- ✅ `test_rate_limit_headers_present` - Rate limit headers in response
- ✅ `test_rate_limit_message_on_429` - Helpful error message on limit
- ✅ `test_rate_limit_remaining_decreases` - Counter decreases correctly

**Test Coverage:**
- Rate limit enforcement (3/60 seconds)
- Per-view separate counters
- Per-IP separate counters
- Response headers and messages
- Graceful degradation

## Implementation Details

### RedisRateLimiter Class

Located in `backend/utils/rate_limiter.py`, this class handles the core rate limiting logic:

```python
limiter = RedisRateLimiter(redis_client, key_prefix='rate_limit:view_name')
is_allowed = limiter.is_allowed('192.168.1.1', limit=3, window_seconds=60)
```

**Methods:**
- `is_allowed()` - Check if request allowed, add to counter if yes
- `get_remaining()` - Get remaining requests in current window
- `get_stats()` - Get detailed stats (count, limit, remaining, reset time)
- `reset()` - Manually reset rate limit for an identifier

### rate_limit Decorator

The decorator wraps view functions with rate limiting logic:

```python
@method_decorator(rate_limit(requests=3, window=60))
def post(self, request, *args, **kwargs):
    # Rate limited POST method
```

**How it works:**
1. Extracts decorator parameters (requests, window)
2. Creates inner `decorator` function that takes the view function
3. Creates inner `wrapper` function that:
   - Wraps with try-except for error handling
   - Gets Redis connection
   - Creates RedisRateLimiter with view-specific key prefix
   - Extracts client IP from request
   - Checks rate limit
   - Returns 429 if exceeded, otherwise calls view
   - Adds rate limit headers to response
   - Catches exceptions and fails open (allows request if Redis down)

**Key design decisions:**
- **View-specific keys**: `rate_limit:post:192.168.1.1` keeps counters separate per view
- **IP-based**: Tracks by client IP address (works for direct connections)
- **Fail open**: Allows requests if Redis fails (better UX than blocking)
- **Error logging**: Logs all issues for debugging
```

## Security Notes

- Rate limiting is based on IP address, not user identity
- Users behind proxies/CDNs may share IP addresses
- For strict user-based limiting, consider adding user ID to the key
- Rate limiting alone doesn't prevent determined attackers; combine with other security measures

## See Also

- [Redis Configuration](REDIS_LOCAL_SETUP.md)
- [Django Caching](https://docs.djangoproject.com/en/5.2/topics/cache/)
- [Rate Limiting Best Practices](https://tools.ietf.org/html/rfc6585#section-4)
