# Stage 1: Build C++ generator
FROM gcc:latest as cpp-builder

WORKDIR /build

# Copy src directory
COPY src/ .

# Build the C++ generator
RUN make clean || true && \
    make generator && \
    make tester

# Stage 2: Build Python application
FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    g++ \
    make \
    postgresql-client \
    netcat-openbsd \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user for security
RUN useradd -m -u 1000 djangouser

# Copy Pipfile for dependency management
COPY backend/Pipfile backend/Pipfile.lock* ./

# Install Python dependencies
RUN pip install --upgrade pip setuptools wheel && \
    pip install pipenv && \
    if [ -f Pipfile.lock ]; then \
        pipenv install --deploy --system; \
    else \
        pip install django django-redis redis celery gunicorn psycopg2-binary django-celery-beat requests pillow python-dotenv; \
    fi

# Copy application code
COPY backend/ .

# Copy C++ generator from builder stage
COPY --from=cpp-builder /build/gen /app/generator/main
RUN chmod +x /app/generator/main

# Create necessary directories
RUN mkdir -p /app/generator/outputs \
    /app/staticfiles \
    /app/media \
    && chown -R djangouser:djangouser /app

# Copy entrypoint script (strip CRLF so Linux exec/shebang works on Windows checkouts)
COPY docker-entrypoint.sh /
RUN sed -i 's/\r$//' /docker-entrypoint.sh && chmod +x /docker-entrypoint.sh

# Set environment variables
ENV DJANGO_SETTINGS_MODULE=config.settings
ENV OUTPUT_DIR=/app/generator/outputs
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1
ENV PATH="/app:${PATH}"

# Switch to non-root user
USER djangouser

# Expose port
EXPOSE 8000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=40s --retries=3 \
    CMD python -c "import requests; requests.get('http://localhost:8000/health', timeout=5)"

# Run entrypoint script
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["gunicorn", "config.wsgi:application", "--bind", "0.0.0.0:8000", "--workers", "4"]
