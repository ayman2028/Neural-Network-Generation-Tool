# Celery Setup Guide

## Overview
Celery is configured to handle asynchronous tasks using Redis as the message broker. This setup allows long-running operations (like network generation) to run in the background without blocking HTTP requests.

## Architecture

- **Message Broker**: Redis (DB 1)
- **Result Backend**: Redis (DB 2)
- **Cache**: Redis (DB 0)
- **Message Format**: JSON

## Files Created/Modified

1. **`config/celery.py`** - Celery app initialization and configuration
2. **`config/__init__.py`** - Auto-imports Celery app when Django starts
3. **`config/settings.py`** - Added Celery configuration settings
4. **`networks/tasks.py`** - Example async tasks
5. **`backend/Pipfile`** - Added Celery dependency

## Installation

After updating the Pipfile, install Celery:

```bash
pipenv install
```

Or if you prefer pip:
```bash
pip install celery
```

## Running Celery Worker

### Development (single worker)

```bash
celery -A config worker -l info
```

### With auto-reload (useful during development)

```bash
celery -A config worker -l info --loglevel=debug
```

### Multiple worker instances (for production)

```bash
celery -A config worker -l info -c 4  # 4 concurrent tasks
```

### With Celery Beat scheduler (for periodic tasks)

Open a separate terminal and run:

```bash
celery -A config beat -l info
```

Or combine them in one process (not recommended for production):

```bash
celery -A config worker -B -l info
```

## Using Celery Tasks

### In Views

```python
from networks.tasks import generate_network
from django.http import JsonResponse

def create_network(request):
    # ... create network object ...
    
    # Queue the task asynchronously
    task = generate_network.delay(network.id)
    
    return JsonResponse({
        'message': 'Network generation started',
        'task_id': task.id,
        'status_url': f'/api/task-status/{task.id}/'
    })
```

### Task Status

To check task status, create an endpoint like:

```python
from celery.result import AsyncResult

def task_status(request, task_id):
    task_result = AsyncResult(task_id)
    return JsonResponse({
        'task_id': task_id,
        'status': task_result.status,
        'result': task_result.result if task_result.status == 'SUCCESS' else None,
    })
```

## Task Examples

### Simple Task (no retries)

```python
@shared_task
def send_email(email, subject):
    # send email logic
    pass

# Usage
send_email.delay('user@example.com', 'Hello')
```

### Task with Retries

```python
@shared_task(bind=True, max_retries=3)
def generate_network(self, network_id):
    try:
        # logic
        pass
    except Exception as exc:
        raise self.retry(exc=exc, countdown=60)
```

### Task with Time Limits

```python
@shared_task(time_limit=600)  # 10 minutes
def long_running_task():
    # logic
    pass
```

### Periodic Tasks (Celery Beat)

Configure in `config/celery.py`:

```python
from celery.schedules import crontab

app.conf.beat_schedule = {
    'cleanup-stale-networks': {
        'task': 'networks.tasks.cleanup_stale_networks',
        'schedule': crontab(hour=0, minute=0),  # Run daily at midnight
    },
}
```

## Monitoring Tasks

### Celery Flower (Web UI)

Install and run Flower for real-time monitoring:

```bash
pip install flower
celery -A config flower
```

Then visit `http://localhost:5555`

### Command Line

```bash
# Inspect active tasks
celery -A config inspect active

# Check stats
celery -A config inspect stats

# Revoke a task
celery -A config revoke <task_id>
```

## Redis Database Separation

- **DB 0**: Cache (Django default)
- **DB 1**: Celery broker/message queue
- **DB 2**: Celery result backend

This separation prevents cache invalidation when Celery processes tasks.

## Configuration Reference

Key settings in `config/settings.py`:

- `CELERY_BROKER_URL` - Redis connection for task queue
- `CELERY_RESULT_BACKEND` - Redis connection for storing results
- `CELERY_TASK_SERIALIZER` - Message format (JSON)
- `CELERY_TASK_TRACK_STARTED` - Track task start status
- `CELERY_TASK_TIME_LIMIT` - Hard time limit (kills task)
- `CELERY_TASK_SOFT_TIME_LIMIT` - Soft limit (graceful shutdown)

## Troubleshooting

### Tasks not executing

1. Check Redis is running: `redis-cli ping` (should return PONG)
2. Check worker is running: `celery -A config worker -l info`
3. Check task is imported: Verify `tasks.py` exists and is properly importable
4. Check logs for errors

### "No module named 'celery'"

Install Celery:
```bash
pip install celery
```

### Connection refused to Redis

Start Redis:
```bash
redis-server
```

Or Docker:
```bash
docker run -d -p 6379:6379 redis
```

## Next Steps

1. Update `networks/models.py` to add status and timestamp fields for tracking network generation
2. Implement actual network generation logic in `networks/tasks.py`
3. Create API endpoints for task status monitoring
4. Set up Flower for task monitoring in production
5. Configure Celery Beat for periodic cleanup tasks
