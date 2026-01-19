#!/bin/bash

# Docker deployment helper script
# Usage: ./docker-deploy.sh [command] [options]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME="neural-network"
DOCKER_COMPOSE_CMD="docker-compose"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

show_usage() {
    cat << EOF
Usage: $0 [command] [options]

Commands:
    build               Build Docker images
    up                  Start all services
    down                Stop all services
    logs                Show service logs
    logs-follow         Follow service logs in real-time
    shell               Open Django shell
    bash                Open bash in web container
    manage              Run Django management command
    migrate             Run database migrations
    createsuperuser     Create a superuser account
    collectstatic       Collect static files
    backup-db           Backup PostgreSQL database
    restore-db [file]   Restore PostgreSQL database from backup
    clean               Clean up Docker resources
    test                Run tests
    ps                  Show container status
    restart             Restart all services
    restart-service     Restart specific service
    help                Show this help message

Examples:
    $0 build
    $0 up
    $0 logs
    $0 manage makemigrations
    $0 bash
    $0 backup-db
    $0 restore-db backup.sql
EOF
}

build_images() {
    print_info "Building Docker images..."
    $DOCKER_COMPOSE_CMD build
    print_info "Build complete!"
}

start_services() {
    print_info "Starting services..."
    $DOCKER_COMPOSE_CMD up -d
    print_info "Services started!"
    print_info "Web: http://localhost:8000"
    print_info "Nginx: http://localhost:80"
}

stop_services() {
    print_info "Stopping services..."
    $DOCKER_COMPOSE_CMD down
    print_info "Services stopped!"
}

show_logs() {
    if [ "$1" = "follow" ]; then
        $DOCKER_COMPOSE_CMD logs -f
    else
        $DOCKER_COMPOSE_CMD logs
    fi
}

django_shell() {
    print_info "Opening Django shell..."
    $DOCKER_COMPOSE_CMD exec web python manage.py shell
}

bash_shell() {
    print_info "Opening bash shell in web container..."
    $DOCKER_COMPOSE_CMD exec web bash
}

run_manage_command() {
    if [ -z "$1" ]; then
        print_error "No management command specified"
        exit 1
    fi
    print_info "Running: python manage.py $@"
    $DOCKER_COMPOSE_CMD exec web python manage.py "$@"
}

run_migrations() {
    print_info "Running database migrations..."
    $DOCKER_COMPOSE_CMD exec web python manage.py migrate
    print_info "Migrations complete!"
}

create_superuser() {
    print_info "Creating superuser..."
    $DOCKER_COMPOSE_CMD exec web python manage.py createsuperuser
}

collect_static() {
    print_info "Collecting static files..."
    $DOCKER_COMPOSE_CMD exec web python manage.py collectstatic --noinput
    print_info "Static files collected!"
}

backup_database() {
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    BACKUP_FILE="${SCRIPT_DIR}/backup_${TIMESTAMP}.sql"
    print_info "Backing up database to ${BACKUP_FILE}..."
    $DOCKER_COMPOSE_CMD exec -T db pg_dump -U postgres neural_network > "${BACKUP_FILE}"
    print_info "Database backed up to ${BACKUP_FILE}"
}

restore_database() {
    if [ -z "$1" ]; then
        print_error "No backup file specified"
        exit 1
    fi
    
    if [ ! -f "$1" ]; then
        print_error "Backup file not found: $1"
        exit 1
    fi
    
    print_warning "This will restore the database from ${1}"
    read -p "Are you sure? (yes/no): " -r
    if [[ $REPLY =~ ^[Yy][Ee][Ss]$ ]]; then
        print_info "Restoring database..."
        $DOCKER_COMPOSE_CMD exec -T db psql -U postgres neural_network < "$1"
        print_info "Database restored!"
    else
        print_info "Restore cancelled"
    fi
}

run_tests() {
    print_info "Running tests..."
    $DOCKER_COMPOSE_CMD exec web python manage.py test
}

show_status() {
    print_info "Container status:"
    $DOCKER_COMPOSE_CMD ps
}

restart_all() {
    print_info "Restarting all services..."
    $DOCKER_COMPOSE_CMD restart
    print_info "Services restarted!"
}

restart_service() {
    if [ -z "$1" ]; then
        print_error "No service specified"
        exit 1
    fi
    print_info "Restarting ${1}..."
    $DOCKER_COMPOSE_CMD restart "$1"
    print_info "Service restarted!"
}

cleanup() {
    print_warning "This will remove all Docker resources for this project"
    read -p "Are you sure? (yes/no): " -r
    if [[ $REPLY =~ ^[Yy][Ee][Ss]$ ]]; then
        print_info "Cleaning up..."
        $DOCKER_COMPOSE_CMD down -v
        docker image prune -f
        docker volume prune -f
        print_info "Cleanup complete!"
    else
        print_info "Cleanup cancelled"
    fi
}

# Main script logic
if [ $# -eq 0 ]; then
    show_usage
    exit 0
fi

COMMAND="$1"
shift

case "$COMMAND" in
    build)
        build_images
        ;;
    up)
        build_images
        start_services
        ;;
    down)
        stop_services
        ;;
    logs)
        show_logs
        ;;
    logs-follow)
        show_logs follow
        ;;
    shell)
        django_shell
        ;;
    bash)
        bash_shell
        ;;
    manage)
        run_manage_command "$@"
        ;;
    migrate)
        run_migrations
        ;;
    createsuperuser)
        create_superuser
        ;;
    collectstatic)
        collect_static
        ;;
    backup-db)
        backup_database
        ;;
    restore-db)
        restore_database "$1"
        ;;
    test)
        run_tests
        ;;
    ps)
        show_status
        ;;
    restart)
        restart_all
        ;;
    restart-service)
        restart_service "$1"
        ;;
    clean)
        cleanup
        ;;
    help)
        show_usage
        ;;
    *)
        print_error "Unknown command: $COMMAND"
        show_usage
        exit 1
        ;;
esac
