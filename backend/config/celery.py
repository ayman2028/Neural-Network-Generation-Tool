"""
Celery configuration for the Neural Network Generation Tool.
This module configures Celery to use Redis as the message broker.
"""

import os
from celery import Celery
from celery.schedules import crontab

# Set the default Django settings module for the 'celery' program.
os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'config.settings')

app = Celery('neural_network')

# Load configuration from Django settings, all celery configuration keys should have a `CELERY_` prefix.
app.config_from_object('django.conf:settings', namespace='CELERY')

# Auto-discover tasks from all registered Django apps.
app.autodiscover_tasks()

# Optional: Configure periodic tasks (Beat schedule)
app.conf.beat_schedule = {
    # Example: cleanup stale networks every hour
    # 'cleanup-stale-networks': {
    #     'task': 'networks.tasks.cleanup_stale_networks',
    #     'schedule': crontab(minute=0),  # Run at the top of every hour
    # },
}
