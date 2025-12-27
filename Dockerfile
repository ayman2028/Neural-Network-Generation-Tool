FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    g++ \
    make \
    && rm -rf /var/lib/apt/lists/*

# Copy Pipfile for dependency management
COPY backend/Pipfile* ./

# Install Python dependencies
RUN pip install --upgrade pip && \
    if [ -f Pipfile ]; then \
        pip install pipenv && pipenv install --deploy --ignore-pipfile; \
    fi

# Copy application code
COPY backend/ .

# Build the C++ generator
COPY src/ /tmp/src/
RUN cd /tmp/src && \
    make generator && \
    cp gen /app/generator/main && \
    chmod +x /app/generator/main

# Create output directory
RUN mkdir -p /app/generator/outputs

# Set environment variables
ENV DJANGO_SETTINGS_MODULE=config.settings
ENV OUTPUT_DIR=/app/generator/outputs
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1

# Expose port
EXPOSE 8000

# Run Django app
CMD ["python", "manage.py", "runserver", "0.0.0.0:8000"]
