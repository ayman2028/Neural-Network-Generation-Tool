#!/bin/bash
set -e

# Wait for database to be ready (DB on EC2 host, RDS URL, or compose service "db")
DB_HOST="${DB_HOST:-db}"
DB_PORT="${DB_PORT:-5432}"
echo "Waiting for PostgreSQL at ${DB_HOST}:${DB_PORT}..."
while ! nc -z "$DB_HOST" "$DB_PORT"; do
  sleep 0.1
done
echo "PostgreSQL is ready"

# Wait for Redis to be ready (same hostnames as REDIS_URL / compose)
REDIS_WAIT_HOST="${REDIS_HOST:-redis}"
REDIS_WAIT_PORT="${REDIS_PORT:-6379}"
echo "Waiting for Redis at ${REDIS_WAIT_HOST}:${REDIS_WAIT_PORT}..."
while ! nc -z "$REDIS_WAIT_HOST" "$REDIS_WAIT_PORT"; do
  sleep 0.1
done
echo "Redis is ready"

# Run migrations
echo "Running Django migrations..."
python manage.py migrate --noinput || true

# Create default user if it doesn't exist
echo "Creating default superuser if needed..."
python manage.py shell << END
from django.contrib.auth import get_user_model
User = get_user_model()
if not User.objects.filter(username='admin').exists():
    User.objects.create_superuser('admin', 'admin@example.com', 'admin')
    print('Superuser created')
else:
    print('Superuser already exists')
END

# Collect static files
echo "Collecting static files..."
python manage.py collectstatic --noinput || true

echo "Starting application..."
exec "$@"
