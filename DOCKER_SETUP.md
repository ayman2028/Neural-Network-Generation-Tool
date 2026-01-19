# Docker Setup Guide

This guide will help you deploy the Neural Network Generation Tool using Docker.

## Prerequisites

- Docker (version 20.10 or higher)
- Docker Compose (version 1.29 or higher)
- 4GB+ RAM available
- 10GB+ disk space

## Quick Start

### 1. Clone and Navigate to Project

```bash
cd Neural-Network-Generation-Tool
```

### 2. Create Environment File

```bash
cp .env.example .env
```

Edit `.env` and update the following for production:
- `SECRET_KEY`: Generate a strong secret key
- `DEBUG`: Set to `False` for production
- `ALLOWED_HOSTS`: Add your domain/IP addresses
- `DB_PASSWORD`: Change the default database password
- Database and Redis credentials

### 3. Build and Start Services

```bash
# Build the Docker images
docker-compose build

# Start all services
docker-compose up -d

# View logs
docker-compose logs -f
```

### 4. Initialize Database

```bash
# Run migrations
docker-compose exec web python manage.py migrate

# Create superuser
docker-compose exec web python manage.py createsuperuser

# Collect static files
docker-compose exec web python manage.py collectstatic --noinput
```

### 5. Access the Application

- **Web Application**: http://localhost:8000 (or your configured port)
- **Admin Panel**: http://localhost:8000/admin
- **Nginx Reverse Proxy**: http://localhost:80

## Services Overview

### Web (Django)
- Main application server
- Runs Gunicorn with 4 workers
- Handles HTTP requests
- Port: 8000

### Database (PostgreSQL)
- Primary data store
- Replaces SQLite for production
- Port: 5432
- Volume: `postgres_data`

### Redis
- Cache backend
- Celery message broker
- Session storage
- Port: 6379
- Volume: `redis_data`

### Celery Worker
- Processes asynchronous tasks
- Generator job queue
- 4 concurrent workers

### Celery Beat
- Scheduled task scheduler
- Periodic task execution

### Nginx
- Reverse proxy server
- Static file serving
- SSL/TLS termination (when configured)
- Gzip compression
- Security headers
- Port: 80, 443

## Common Commands

### View Logs
```bash
# All services
docker-compose logs -f

# Specific service
docker-compose logs -f web
docker-compose logs -f celery
docker-compose logs -f db
```

### Access Container Shell
```bash
# Django shell
docker-compose exec web python manage.py shell

# Bash in web container
docker-compose exec web bash

# Database connection
docker-compose exec db psql -U postgres -d neural_network
```

### Database Operations
```bash
# Create migration
docker-compose exec web python manage.py makemigrations

# Run migrations
docker-compose exec web python manage.py migrate

# Backup database
docker-compose exec db pg_dump -U postgres neural_network > backup.sql

# Restore database
docker-compose exec -T db psql -U postgres neural_network < backup.sql
```

### Stop Services
```bash
# Stop all services (keep data)
docker-compose down

# Stop and remove volumes (clean slate)
docker-compose down -v

# Stop specific service
docker-compose stop web
```

### Restart Services
```bash
# Restart all
docker-compose restart

# Rebuild after code changes
docker-compose up -d --build
```

## Production Deployment

### 1. SSL/TLS Configuration

```bash
# Generate self-signed certificate (testing only)
mkdir ssl
openssl req -x509 -newkey rsa:4096 -nodes -out ssl/cert.pem -keyout ssl/key.pem -days 365

# For production, use Let's Encrypt with Certbot
docker run -it --rm -v certbot-data:/etc/letsencrypt \
  -v /path/to/project/.well-known:/var/www/certbot \
  certbot/certbot certonly --webroot -w /var/www/certbot \
  -d yourdomain.com
```

### 2. Uncomment SSL in nginx.conf

Edit `nginx.conf` and uncomment the SSL configuration block.

### 3. Environment Configuration

Create a `.env.production` file with production values:
```bash
DEBUG=False
SECRET_KEY=<generate-strong-key>
ALLOWED_HOSTS=yourdomain.com,www.yourdomain.com
DB_PASSWORD=<strong-password>
```

### 4. Update Settings

Update Django settings in `backend/config/settings.py`:
```python
ALLOWED_HOSTS = ['yourdomain.com', 'www.yourdomain.com']
SECURE_SSL_REDIRECT = True
SESSION_COOKIE_SECURE = True
CSRF_COOKIE_SECURE = True
```

## Troubleshooting

### Database Connection Issues
```bash
# Check database status
docker-compose exec db pg_isready -U postgres

# Verify network connectivity
docker-compose exec web ping db
```

### Redis Connection Issues
```bash
# Check Redis status
docker-compose exec redis redis-cli ping

# Monitor Redis
docker-compose exec redis redis-cli monitor
```

### Static Files Not Loading
```bash
# Rebuild static files
docker-compose exec web python manage.py collectstatic --noinput

# Check permissions
docker-compose exec web ls -la /app/staticfiles/
```

### Permission Issues
```bash
# Check user in container
docker-compose exec web whoami

# Fix permissions
docker-compose exec web chown -R djangouser:djangouser /app
```

### Out of Memory
```bash
# Increase Docker memory limit
# Edit Docker Desktop settings or docker daemon.json

# Check current resource usage
docker stats
```

## Scaling

### Increase Celery Workers
```yaml
# In docker-compose.yml, scale celery workers
docker-compose up -d --scale celery=3
```

### Multiple Web Workers
```yaml
# Update WORKERS environment variable or modify Dockerfile CMD
gunicorn config.wsgi:application --bind 0.0.0.0:8000 --workers 8
```

## Monitoring & Maintenance

### Health Checks
```bash
# Check service health
docker-compose ps

# Manual health check
curl http://localhost:8000/health
```

### Database Maintenance
```bash
# Vacuum database (cleanup)
docker-compose exec db vacuumdb -U postgres neural_network

# Analyze database
docker-compose exec db analyzedb -U postgres neural_network
```

### Clean Up
```bash
# Remove unused images
docker image prune -a

# Remove unused volumes
docker volume prune

# Remove unused networks
docker network prune
```

## Advanced Configuration

### Custom Gunicorn Settings
Edit the `gunicorn` command in `docker-compose.yml`:
```bash
gunicorn config.wsgi:application \
  --bind 0.0.0.0:8000 \
  --workers 4 \
  --worker-class sync \
  --max-requests 1000 \
  --max-requests-jitter 100 \
  --timeout 120 \
  --access-logfile - \
  --error-logfile -
```

### Custom Docker Network
```bash
docker network create neural-network-custom
# Update docker-compose.yml networks section
```

### Using Docker Registry
```bash
# Tag image
docker tag neural-network-app:latest myregistry.azurecr.io/neural-network:latest

# Push to registry
docker push myregistry.azurecr.io/neural-network:latest

# Pull from registry
docker pull myregistry.azurecr.io/neural-network:latest
```

## Security Best Practices

1. **Change Default Passwords**: Update all default credentials
2. **Use Strong Secret Key**: Generate with `python -c "from django.core.management.utils import get_random_secret_key; print(get_random_secret_key())"`
3. **Enable HTTPS**: Configure SSL certificates
4. **Restrict Database Access**: Only expose on internal network
5. **Use Environment Variables**: Never hardcode sensitive data
6. **Regular Updates**: Keep Docker images and dependencies updated
7. **Network Isolation**: Use custom networks and security groups
8. **Backup Strategy**: Implement regular database backups

## Support & Documentation

For more information:
- Django Documentation: https://docs.djangoproject.com/
- Celery Documentation: https://docs.celeryproject.org/
- PostgreSQL Documentation: https://www.postgresql.org/docs/
- Redis Documentation: https://redis.io/documentation
- Nginx Documentation: https://nginx.org/en/docs/
