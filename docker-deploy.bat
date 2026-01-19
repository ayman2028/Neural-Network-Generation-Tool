@echo off
REM Docker deployment helper script for Windows
REM Usage: docker-deploy.bat [command] [options]

setlocal enabledelayedexpansion

set "PROJECT_NAME=neural-network"
set "DOCKER_COMPOSE_CMD=docker-compose"

if "%1"=="" (
    call :show_usage
    exit /b 0
)

set "COMMAND=%1"
shift

goto %COMMAND%
if errorlevel 1 goto unknown_command

:build
echo [INFO] Building Docker images...
%DOCKER_COMPOSE_CMD% build
echo [INFO] Build complete!
exit /b 0

:up
echo [INFO] Building and starting services...
call :build
%DOCKER_COMPOSE_CMD% up -d
echo [INFO] Services started!
echo [INFO] Web: http://localhost:8000
echo [INFO] Nginx: http://localhost:80
exit /b 0

:down
echo [INFO] Stopping services...
%DOCKER_COMPOSE_CMD% down
echo [INFO] Services stopped!
exit /b 0

:logs
if "%1"=="follow" (
    %DOCKER_COMPOSE_CMD% logs -f
) else (
    %DOCKER_COMPOSE_CMD% logs
)
exit /b 0

:shell
echo [INFO] Opening Django shell...
%DOCKER_COMPOSE_CMD% exec web python manage.py shell
exit /b 0

:bash
echo [INFO] Opening bash shell in web container...
%DOCKER_COMPOSE_CMD% exec web bash
exit /b 0

:manage
if "%1"=="" (
    echo [ERROR] No management command specified
    exit /b 1
)
echo [INFO] Running: python manage.py %*
%DOCKER_COMPOSE_CMD% exec web python manage.py %*
exit /b 0

:migrate
echo [INFO] Running database migrations...
%DOCKER_COMPOSE_CMD% exec web python manage.py migrate
echo [INFO] Migrations complete!
exit /b 0

:createsuperuser
echo [INFO] Creating superuser...
%DOCKER_COMPOSE_CMD% exec web python manage.py createsuperuser
exit /b 0

:collectstatic
echo [INFO] Collecting static files...
%DOCKER_COMPOSE_CMD% exec web python manage.py collectstatic --noinput
echo [INFO] Static files collected!
exit /b 0

:backup-db
for /f "tokens=2-4 delims=/ " %%a in ('date /t') do (set mydate=%%c%%a%%b)
for /f "tokens=1-2 delims=/:" %%a in ('time /t') do (set mytime=%%a%%b)
set "BACKUP_FILE=backup_%mydate%_%mytime%.sql"
echo [INFO] Backing up database to %BACKUP_FILE%...
%DOCKER_COMPOSE_CMD% exec -T db pg_dump -U postgres neural_network > "%BACKUP_FILE%"
echo [INFO] Database backed up to %BACKUP_FILE%
exit /b 0

:restore-db
if "%1"=="" (
    echo [ERROR] No backup file specified
    exit /b 1
)
if not exist "%1" (
    echo [ERROR] Backup file not found: %1
    exit /b 1
)
echo [WARNING] This will restore the database from %1
set /p "confirm=Are you sure? (yes/no): "
if /i "!confirm!"=="yes" (
    echo [INFO] Restoring database...
    %DOCKER_COMPOSE_CMD% exec -T db psql -U postgres neural_network < "%1"
    echo [INFO] Database restored!
) else (
    echo [INFO] Restore cancelled
)
exit /b 0

:test
echo [INFO] Running tests...
%DOCKER_COMPOSE_CMD% exec web python manage.py test
exit /b 0

:ps
echo [INFO] Container status:
%DOCKER_COMPOSE_CMD% ps
exit /b 0

:restart
echo [INFO] Restarting all services...
%DOCKER_COMPOSE_CMD% restart
echo [INFO] Services restarted!
exit /b 0

:restart-service
if "%1"=="" (
    echo [ERROR] No service specified
    exit /b 1
)
echo [INFO] Restarting %1...
%DOCKER_COMPOSE_CMD% restart %1
echo [INFO] Service restarted!
exit /b 0

:clean
echo [WARNING] This will remove all Docker resources for this project
set /p "confirm=Are you sure? (yes/no): "
if /i "!confirm!"=="yes" (
    echo [INFO] Cleaning up...
    %DOCKER_COMPOSE_CMD% down -v
    docker image prune -f
    docker volume prune -f
    echo [INFO] Cleanup complete!
) else (
    echo [INFO] Cleanup cancelled
)
exit /b 0

:help
call :show_usage
exit /b 0

:unknown_command
echo [ERROR] Unknown command: %COMMAND%
call :show_usage
exit /b 1

:show_usage
echo Usage: %0 [command] [options]
echo.
echo Commands:
echo   build               Build Docker images
echo   up                  Start all services
echo   down                Stop all services
echo   logs                Show service logs
echo   logs follow         Follow service logs in real-time
echo   shell               Open Django shell
echo   bash                Open bash in web container
echo   manage              Run Django management command
echo   migrate             Run database migrations
echo   createsuperuser     Create a superuser account
echo   collectstatic       Collect static files
echo   backup-db           Backup PostgreSQL database
echo   restore-db [file]   Restore PostgreSQL database from backup
echo   test                Run tests
echo   ps                  Show container status
echo   restart             Restart all services
echo   restart-service     Restart specific service
echo   clean               Clean up Docker resources
echo   help                Show this help message
echo.
echo Examples:
echo   %0 build
echo   %0 up
echo   %0 logs
echo   %0 manage makemigrations
echo   %0 bash
echo   %0 backup-db
echo   %0 restore-db backup.sql
exit /b 0
