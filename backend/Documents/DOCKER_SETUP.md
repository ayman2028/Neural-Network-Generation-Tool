# Docker Setup Guide - Neural Network Generation Tool

## Overview
This document outlines the steps required to containerize the Neural Network Generation Tool using Docker for consistent deployment and development environments.
## Backward Compatibility: Local Development Still Works

**Important:** Setting up Docker does NOT change local development. Your current workflow remains unchanged:

### Running Locally (Windows/macOS/Linux)
```bash
# Works exactly as before
python manage.py runserver
```

The generator uses this logic:
1. Checks for `OUTPUT_DIR` environment variable
2. If not set, falls back to default: `backend/generator/outputs/`
3. Files are stored locally in your repository

**Local behavior is identical to before Docker setup.**

### Running in Docker
```bash
docker-compose up
```

The docker-compose.yml sets `OUTPUT_DIR=/app/generator/outputs` and mounts a volume:
```yaml
volumes:
  - ./backend/generator/outputs:/app/generator/outputs
```

This syncs files from Docker back to your host machine while keeping everything isolated.

**Summary:** Local development is 100% unchanged. Docker is optional for deployment/consistency.

---
## Pre-Setup Questions & Decisions

Before implementing Docker, clarify the following:

### 1. C++ Executable Handling
- [ ] Is the C++ `main` executable pre-compiled?
  - Location: `backend/generator/main` or `src/main`
  - Decision: Copy pre-compiled binary OR compile from source in Docker?
  
### 2. Database Management
- [ ] Use SQLite (db.sqlite3) with volume persistence?
- [ ] Switch to PostgreSQL/MySQL for production?
- [ ] Where should database files be stored?

### 3. Source Code
- [ ] Is `/src/` directory needed in Docker, or just the pre-compiled executable?
- [ ] Do you need to recompile the C++ tool during Docker build?

---

## Implementation Steps

### Step 1: Update Generator for Environment Variables
**File:** `backend/generator/generator.py`

**Goal:** Make output directory configurable via environment variables

**Changes needed:**
- Modify `generate_network()` to check for `OUTPUT_DIR` environment variable
- Fall back to default `backend/generator/outputs/` if not set
- Update any other hardcoded paths

**Example:**
```python
import os

def get_generator_base_path():
    """Get the base path for the generator (this module's directory)"""
    return Path(__file__).parent.absolute()

def get_output_directory():
    """Get output directory from environment or use default"""
    output_dir = os.getenv('OUTPUT_DIR')
    if output_dir:
        return Path(output_dir)
    return get_generator_base_path() / 'outputs'
```

**Impact:** Allows Docker containers to write to mounted volumes

---

### Step 2: Create Dockerfile
**File:** Create `Dockerfile` in project root

**Considerations:**
- Base image: `python:3.11-slim` (adjust version as needed)
- Install system dependencies (if any required by your C++ executable)
- Copy Pipfile/Pipenv configuration
- Copy application code
- Handle C++ executable (copy if pre-compiled, compile if source included)
- Set environment variables
- Expose ports (8000 for Django)
- Define entry point (runserver, gunicorn, etc.)

**Template:**
```dockerfile
FROM python:3.11-slim

WORKDIR /app

# Install system dependencies (add as needed)
RUN apt-get update && apt-get install -y \
    && rm -rf /var/lib/apt/lists/*

# Copy Pipfile
COPY backend/Pipfile backend/Pipfile.lock ./

# Install Python dependencies
RUN pip install pipenv && pipenv install --deploy --ignore-pipfile

# Copy application
COPY backend/ .

# Create output directory
RUN mkdir -p /app/outputs

# Set environment variables
ENV DJANGO_SETTINGS_MODULE=config.settings
ENV OUTPUT_DIR=/app/outputs
ENV PYTHONUNBUFFERED=1

# Expose port
EXPOSE 8000

# Run Django app
CMD ["python", "manage.py", "runserver", "0.0.0.0:8000"]
```

---

### Step 3: Create docker-compose.yml
**File:** Create `docker-compose.yml` in project root

**Features:**
- Define Django web service
- Configure volume mounts for:
  - Generated output files
  - Database persistence
  - Static files (if needed)
- Set environment variables
- Configure port mappings
- (Optional) Add database service (PostgreSQL/MySQL)

**Template:**
```yaml
version: '3.8'

services:
  web:
    build: .
    container_name: neural-network-tool
    ports:
      - "8000:8000"
    volumes:
      - ./backend/generator/outputs:/app/outputs
      - ./backend/db.sqlite3:/app/db.sqlite3
    environment:
      - DEBUG=True
      - OUTPUT_DIR=/app/outputs
      - DJANGO_SETTINGS_MODULE=config.settings
    command: python manage.py runserver 0.0.0.0:8000
```

---

### Step 4: Update Django Settings
**File:** `backend/config/settings.py`

**Changes needed:**
- Update `ALLOWED_HOSTS` for Docker environment
- Configure database path to work in container
- Update static/media file paths (if needed)
- Set `DEBUG` based on environment variable

**Example:**
```python
import os

DEBUG = os.getenv('DEBUG', 'False') == 'True'

ALLOWED_HOSTS = os.getenv('ALLOWED_HOSTS', 'localhost,127.0.0.1,0.0.0.0').split(',')

# Database
DATABASES = {
    'default': {
        'ENGINE': 'django.db.backends.sqlite3',
        'NAME': os.getenv('DATABASE_URL', os.path.join(BASE_DIR, 'db.sqlite3')),
    }
}
```

---

### Step 5: Handle C++ Executable in Docker

**Option A: Pre-compiled Binary**
- Copy the pre-compiled `main` executable into Docker image
- Update Dockerfile to copy from correct location

**Option B: Compile in Docker**
- Include C++ build tools in Dockerfile
- Copy `/src/` directory
- Compile during Docker build
- Move executable to correct location

**In both cases:**
- Ensure executable has correct permissions (chmod +x)
- Test that it runs in the container environment

---

### Step 6: Create .dockerignore
**File:** `.dockerignore` in project root

**Purpose:** Exclude unnecessary files from Docker build context

**Content:**
```
.git
.gitignore
.vscode
__pycache__
*.pyc
*.pyo
*.egg-info
.env
.DS_Store
node_modules
```

---

### Step 7: Test Docker Locally

**Build the image:**
```bash
docker-compose build
```

**Run the container:**
```bash
docker-compose up
```

**Test the application:**
- Access web interface: http://localhost:8000
- Try generating a network
- Verify output files appear in `backend/generator/outputs/`

**Check logs:**
```bash
docker-compose logs -f web
```

**Stop container:**
```bash
docker-compose down
```

---

## Troubleshooting Checklist

- [ ] C++ executable runs in container (test with `docker exec`)
- [ ] Output directory is writable from container
- [ ] Database migrations run successfully
- [ ] Generated files persist after container stops
- [ ] Network/port configurations are correct
- [ ] Environment variables are properly passed to Django
- [ ] Static files load correctly (if needed)

---

## Production Considerations

Once local testing works, consider:

- [ ] Use `gunicorn` instead of Django runserver
- [ ] Implement proper logging
- [ ] Set up database service (PostgreSQL recommended)
- [ ] Use environment file (.env) for sensitive configs
- [ ] Implement health checks
- [ ] Add restart policies
- [ ] Set up reverse proxy (nginx)
- [ ] Configure HTTPS/SSL

---

## Next Steps

1. Answer the pre-setup questions above
2. Modify `generator.py` to support `OUTPUT_DIR` environment variable
3. Create Dockerfile
4. Create docker-compose.yml
5. Update Django settings
6. Test locally
7. Debug and iterate
