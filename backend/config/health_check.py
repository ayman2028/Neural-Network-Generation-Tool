"""
Health check views for Docker and monitoring
"""
from django.http import JsonResponse
from django.db import connections
from django.db.utils import OperationalError
import redis
import os

def health_check(request):
    """
    Health check endpoint for Docker and monitoring systems
    Returns JSON status of database and cache connectivity
    """
    try:
        # Check database connection
        db_conn = connections['default']
        db_conn.ensure_connection()
        db_status = 'ok'
        db_msg = 'Database connected'
    except OperationalError as e:
        db_status = 'error'
        db_msg = f'Database error: {str(e)}'
    
    try:
        # Check Redis connection
        redis_url = os.getenv('REDIS_URL', 'redis://127.0.0.1:6379/0')
        redis_client = redis.from_url(redis_url)
        redis_client.ping()
        cache_status = 'ok'
        cache_msg = 'Redis connected'
    except Exception as e:
        cache_status = 'warning'
        cache_msg = f'Redis warning: {str(e)}'
    
    status_code = 200
    if db_status != 'ok':
        status_code = 503
    
    return JsonResponse({
        'status': 'healthy' if status_code == 200 else 'unhealthy',
        'database': {
            'status': db_status,
            'message': db_msg
        },
        'cache': {
            'status': cache_status,
            'message': cache_msg
        }
    }, status=status_code)


def liveness(request):
    """
    Liveness probe for Kubernetes/Docker
    Returns 200 if the service is running
    """
    return JsonResponse({'status': 'alive'}, status=200)


def readiness(request):
    """
    Readiness probe for Kubernetes/Docker
    Returns 200 if the service is ready to handle requests
    """
    try:
        # Check database
        db_conn = connections['default']
        db_conn.ensure_connection()
        
        # Check Redis
        redis_url = os.getenv('REDIS_URL', 'redis://127.0.0.1:6379/0')
        redis_client = redis.from_url(redis_url)
        redis_client.ping()
        
        return JsonResponse({'status': 'ready'}, status=200)
    except Exception as e:
        return JsonResponse(
            {'status': 'not ready', 'error': str(e)},
            status=503
        )
