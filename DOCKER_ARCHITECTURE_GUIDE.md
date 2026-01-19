# Docker Architecture & Deployment Guide

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         CLIENT REQUESTS                              │
│                    (Browser, API Client, etc.)                       │
└────────────────────────────────┬────────────────────────────────────┘
                                 │
                    Port 80 (HTTP) / 443 (HTTPS)
                                 │
                ┌────────────────▼────────────────┐
                │      NGINX Reverse Proxy        │
                │  ────────────────────────────   │
                │  • SSL/TLS Termination          │
                │  • Static File Serving          │
                │  • GZIP Compression             │
                │  • Security Headers             │
                │  • Load Balancing               │
                │  • Caching                      │
                └────────────────┬────────────────┘
                                 │
                    Port 8000 (Docker Internal)
                                 │
                ┌────────────────▼────────────────┐
                │    Django Web Application       │
                │  ────────────────────────────   │
                │  • 4 Gunicorn Workers           │
                │  • Handles HTTP Requests        │
                │  • Renders Templates            │
                │  • API Endpoints                │
                │  • User Authentication          │
                └────────┬───────────────┬────────┘
                         │               │
        ┌────────────────▼───┐   ┌──────▼──────────────┐
        │   Database         │   │  Cache & Broker    │
        │   PostgreSQL 15    │   │  Redis             │
        ├────────────────────┤   ├────────────────────┤
        │ • User Data        │   │ • Session Cache    │
        │ • Network Configs  │   │ • Task Queue       │
        │ • Generated Code   │   │ • Real-time Data   │
        │ • Persistent Data  │   │ • Temporary Cache  │
        └────────────────────┘   └──────┬─────────────┘
                                        │
                    ┌───────────────────┴───────────────────┐
                    │                                       │
        ┌───────────▼─────────────┐      ┌────────────────▼────┐
        │  Celery Worker          │      │   Celery Beat      │
        │  ─────────────────────  │      │  ───────────────   │
        │  • Process Async Tasks  │      │  • Scheduler       │
        │  • Generate Networks    │      │  • Periodic Tasks  │
        │  • Long Operations      │      │  • Cleanup Jobs    │
        │  • 4 Concurrent Workers │      │  • Report Gen      │
        └─────────────────────────┘      └────────────────────┘
```

## Docker Container Flow

```
        BUILD PHASE
        ───────────

        [Source Code]
               │
         ┌─────▼─────┐
         │ Dockerfile │
         └──┬────┬───┘
            │    │
    ┌───────▼┐ ┌─▼──────────┐
    │ Stage1 │ │ Stage 2    │
    │ (C++   │ │ (Python    │
    │Builder)│ │  Runtime)  │
    └────┬───┘ └──┬────┬─┘
         │        │    │
    [gen binary] [libs][deps]
         │        │    │
         └────────┴────┴─────┐
                             │
                    ┌────────▼───────┐
                    │  Docker Image  │
                    │                │
                    │ neural-net:v1  │
                    └────────────────┘


        RUN PHASE
        ────────

[docker-compose.yml]
        │
        ├─── web (port 8000)
        │    └─ Django + Gunicorn
        │
        ├─── celery (bg)
        │    └─ Worker Process
        │
        ├─── celery-beat (bg)
        │    └─ Scheduler
        │
        ├─── db (port 5432)
        │    └─ PostgreSQL Container
        │
        ├─── redis (port 6379)
        │    └─ Redis Container
        │
        └─── nginx (port 80/443)
             └─ Reverse Proxy
```

## Data Flow for Neural Network Generation

```
1. USER REQUEST
   └─ POST /networks/generate/
         └─ Parameters: input_dim, layers, etc.

2. DJANGO RECEIVES REQUEST
   └─ Validates input
   └─ Saves to database
   └─ Creates Celery task
   └─ Returns task_id to client

3. CELERY WORKER PROCESSES TASK
   └─ Loads parameters from DB
   └─ Calls C++ generator (gen binary)
   └─ Generates SystemVerilog code
   └─ Saves output files
   └─ Updates database with status

4. USER POLLS FOR RESULT
   └─ GET /networks/{task_id}/status/
   └─ Celery returns: PENDING/SUCCESS/FAILURE
   └─ Upon completion, returns download link

5. FILE DELIVERY
   └─ GET /networks/{task_id}/download/
   └─ Nginx serves file from /app/generator/outputs/
   └─ With proper headers and caching
```

## Environment Variable Structure

```
.env File Structure:
┌─────────────────────────────────────┐
│  DJANGO SETTINGS                    │
│  ├─ DEBUG: False/True               │
│  ├─ SECRET_KEY: random string       │
│  ├─ ALLOWED_HOSTS: domains          │
│  └─ PYTHONUNBUFFERED: 1             │
├─────────────────────────────────────┤
│  DATABASE SETTINGS                  │
│  ├─ DB_ENGINE: postgresql           │
│  ├─ DB_NAME: database name          │
│  ├─ DB_USER: username               │
│  ├─ DB_PASSWORD: strong password    │
│  ├─ DB_HOST: db (hostname)          │
│  ├─ DB_PORT: 5432                   │
│  └─ DATABASE_URL: full connection   │
├─────────────────────────────────────┤
│  REDIS SETTINGS                     │
│  ├─ REDIS_URL: full URI             │
│  ├─ REDIS_HOST: redis               │
│  └─ REDIS_PORT: 6379                │
├─────────────────────────────────────┤
│  CELERY SETTINGS                    │
│  ├─ CELERY_BROKER_URL: redis        │
│  ├─ CELERY_RESULT_BACKEND: redis    │
│  └─ CELERY_ALWAYS_EAGER: False      │
├─────────────────────────────────────┤
│  PORT SETTINGS                      │
│  ├─ WEB_PORT: 8000                  │
│  ├─ DB_PORT: 5432                   │
│  ├─ REDIS_PORT: 6379                │
│  ├─ NGINX_PORT: 80                  │
│  └─ NGINX_HTTPS_PORT: 443           │
└─────────────────────────────────────┘
```

## Deployment Workflow

```
┌──────────────────────────────────────┐
│   1. PREPARATION                     │
│   ├─ cp .env.example .env            │
│   ├─ Edit .env with values           │
│   ├─ Generate SECRET_KEY             │
│   └─ Update DB_PASSWORD              │
└────────────────┬─────────────────────┘
                 │
┌────────────────▼─────────────────────┐
│   2. BUILD                           │
│   ├─ docker-compose build            │
│   ├─ Pulls base images               │
│   ├─ Builds app image                │
│   └─ Creates layers (takes 5-10min)  │
└────────────────┬─────────────────────┘
                 │
┌────────────────▼─────────────────────┐
│   3. START SERVICES                  │
│   ├─ docker-compose up -d            │
│   ├─ Starts all containers           │
│   ├─ Waits for health checks         │
│   └─ Creates networks                │
└────────────────┬─────────────────────┘
                 │
┌────────────────▼─────────────────────┐
│   4. INITIALIZE                      │
│   ├─ python manage.py migrate        │
│   ├─ Creates database schema         │
│   ├─ Creates tables                  │
│   └─ Applies migrations              │
└────────────────┬─────────────────────┘
                 │
┌────────────────▼─────────────────────┐
│   5. CREATE ADMIN                    │
│   ├─ createsuperuser command         │
│   ├─ Prompts for username            │
│   ├─ Prompts for email               │
│   └─ Prompts for password            │
└────────────────┬─────────────────────┘
                 │
┌────────────────▼─────────────────────┐
│   6. TEST                            │
│   ├─ curl http://localhost:8000      │
│   ├─ Check /health endpoint          │
│   ├─ Access /admin                   │
│   └─ Verify services running         │
└────────────────┬─────────────────────┘
                 │
┌────────────────▼─────────────────────┐
│   7. PRODUCTION READY                │
│   ├─ Apply SSL certificates          │
│   ├─ Configure domain DNS            │
│   ├─ Set up monitoring               │
│   ├─ Enable backups                  │
│   └─ Deploy!                         │
└──────────────────────────────────────┘
```

## Service Dependencies

```
Service Startup Order:
─────────────────────

1. INFRASTRUCTURE (no dependencies)
   ├─ Redis (cache/broker)
   └─ PostgreSQL (database)

2. APPLICATIONS (depend on infrastructure)
   ├─ Django Web (depends on: DB, Redis)
   ├─ Celery Worker (depends on: DB, Redis)
   ├─ Celery Beat (depends on: DB, Redis)
   └─ Nginx (depends on: Web)

Health Check Status:
│
├─ Initial (0-10s): Services starting
├─ Healthy (10-30s): Core services healthy
├─ Ready (30-40s): All dependencies available
└─ Running (40s+): Accepting requests

Docker automatically manages this with:
• Health checks (every 30s)
• Depends_on with conditions
• Startup scripts
• Service networking
```

## Volume Structure

```
Docker Volumes:
───────────────

┌─ postgres_data ──────────────┐
│  PostgreSQL Data             │
│  ├─ Tables                   │
│  ├─ Indexes                  │
│  ├─ Transaction Logs         │
│  └─ Backups (optional)       │
└──────────────────────────────┘

┌─ redis_data ─────────────────┐
│  Redis Data (AOF)            │
│  ├─ Cache data               │
│  ├─ Broker messages          │
│  └─ Session data             │
└──────────────────────────────┘

┌─ neural_outputs ─────────────┐
│  Generated Neural Networks   │
│  ├─ *.sv files               │
│  ├─ *.txt files              │
│  └─ configs                  │
└──────────────────────────────┘

┌─ static_volume ──────────────┐
│  Static Files                │
│  ├─ CSS                      │
│  ├─ JavaScript               │
│  ├─ Images                   │
│  └─ Admin assets             │
└──────────────────────────────┘

┌─ media_volume ───────────────┐
│  User Uploaded Files         │
│  ├─ User documents           │
│  ├─ Generated reports        │
│  └─ Temporary files          │
└──────────────────────────────┘

All mounted to:
  /var/lib/docker/volumes/
(Automatically managed by Docker)
```

## Health Check Flow

```
Docker Health Checks:
────────────────────

Every 30 seconds:

Health Check Endpoint: /health
         │
         ├─ Check Database
         │  └─ pg_isready (PostgreSQL)
         │     └─ Status: OK/ERROR
         │
         ├─ Check Redis
         │  └─ redis-cli ping
         │     └─ Status: OK/WARNING
         │
         └─ Return JSON Response

{
  "status": "healthy",
  "database": {
    "status": "ok",
    "message": "Database connected"
  },
  "cache": {
    "status": "ok",
    "message": "Redis connected"
  }
}

Docker Actions:
  • 200 → Container healthy
  • 503 → Container unhealthy
  • Timeout → Container unhealthy
  • After 3 failures → Restart container
```

## Scaling Architecture

```
Scale Horizontally:
───────────────────

Default Setup:
┌─────────────┐
│   Nginx     │
└──────┬──────┘
       │
       └─ Django x1 (1 instance)
       │  └─ Gunicorn: 4 workers
       │
       └─ Celery x1 (1 worker)
       │  └─ 4 concurrent tasks
       │
       └─ Beat x1 (1 scheduler)

Scaled Setup:
┌─────────────┐
│   Nginx     │ (Load Balancing)
└──┬────────┬──────┬──────┐
   │        │      │      │
   └─ Django x4 (multiple instances)
   │  └─ Each: Gunicorn 4 workers
   │
   └─ Celery x6 (6 workers)
   │  └─ Each: 4 concurrent tasks
   │
   └─ Beat x1 (1 scheduler, no duplicate)
   │
   └─ PostgreSQL x1 (shared)
   │
   └─ Redis x1 (shared)

Scale command:
$ docker-compose up -d --scale celery=6
$ docker-compose up -d --scale web=4
```

## Production Deployment Checklist

```
SECURITY:
☐ SECRET_KEY is randomly generated
☐ DEBUG = False
☐ ALLOWED_HOSTS configured
☐ SSL/TLS certificates installed
☐ HTTPS enabled in nginx.conf
☐ Security headers in place
☐ CSRF tokens enabled
☐ Session cookies secure
☐ Database password strong
☐ Redis requires auth (if exposed)

DATABASE:
☐ PostgreSQL running
☐ Database created
☐ Migrations applied
☐ Admin user created
☐ Backup scheduled
☐ Connection pooling configured

CACHE:
☐ Redis running
☐ Persistence enabled (AOF)
☐ Memory limits set
☐ Eviction policy configured

CELERY:
☐ Broker URL configured
☐ Result backend configured
☐ Worker started
☐ Beat scheduler started
☐ Task timeouts set

MONITORING:
☐ Health checks enabled
☐ Logging configured
☐ Alert system set up
☐ Metrics collection started
☐ Error tracking enabled

DEPLOYMENT:
☐ Environment file (.env) configured
☐ Volume mounting verified
☐ Port mappings correct
☐ Network isolation verified
☐ Resource limits set
☐ Auto-restart enabled

TESTING:
☐ All services start
☐ Health endpoints return 200
☐ Admin panel accessible
☐ API endpoints responding
☐ Generate network task completes
☐ Database persists data
☐ Logs collected properly
☐ Performance acceptable
```

---

**Ready to deploy your containerized Neural Network Generation Tool!** 🐳🚀
