# Neural-Network-Generation-Tool

A C++ program that generates SystemVerilog code for implementing neural network hardware.

## Features

- Generate SystemVerilog code for neural network hardware implementations
- Three operation modes:
  - Mode 1: Generate a single unparallelized neural network layer
  - Mode 2: Generate a single parallelized neural network layer
  - Mode 3: Generate a three-layer neural network with optimized parallelism
- Interactive mode for entering parameters directly through console prompts
- Default weight generation (all 1's) instead of requiring manual input
- Organized output directory structure for each neural network configuration
- Automatic generation of supporting SystemVerilog files (controller.sv, memory.sv, datapath modules)

## Usage

### Interactive Mode
```
./neural_net_gen
```
or
```
./neural_net_gen interactive
```

### Command Line Mode
```
# Mode 1: Single unparallelized layer
./neural_net_gen 1 M N T R const_file
./neural_net_gen 1 M N T R console    # Use default weights

# Mode 2: Single parallelized layer
./neural_net_gen 2 M N T R P const_file
./neural_net_gen 2 M N T R P console  # Use default weights

# Mode 3: Three-layer network
./neural_net_gen 3 N M1 M2 M3 T R B const_file
./neural_net_gen 3 N M1 M2 M3 T R B console  # Use default weights
```

Where:
- M, M1, M2, M3: Output dimensions
- N: Input dimension
- T: Bit width
- R: ReLU activation (1 = yes, 0 = no)
- P: Parallelism factor (must be a factor of M)
- B: Multiplier budget for the three-layer network
- const_file: File containing weight values
- console: Use default weight values (all 1's)

## Output

The program creates a directory structure with all necessary SystemVerilog files:
- Main neural network module (.sv)
- controller.sv
- memory.sv
- datapath_gen_p3.sv (or datapath_gen_p3_relu.sv for ReLU activation)

## Building

```
cd src
g++ main.cc -o neural_net_gen
```

## Backend

The project includes a Django REST API backend that provides a web interface for neural network generation, user management, and file handling.

### Backend Technologies

- **Framework**: Django 4.x
- **API**: Django REST Framework (if applicable)
- **Database**: SQLite (default), PostgreSQL (production-ready)
- **Authentication**: Django Custom User Model (CustomUser)
- **Task Queue**: Celery with Redis
- **Caching & Rate Limiting**: Redis
- **File Storage**: Local filesystem with ZIP archive support

### Backend Features

#### Core Components

1. **Networks App** (`backend/networks/`)
   - Network configuration management (Mode 1, 2, and 3)
   - Auto-generation of unique network identifiers based on parameters
   - Generated file tracking and download management
   - ManyToMany relationship between users and networks for sharing configurations

2. **Accounts App** (`backend/accounts/`)
   - Custom user model with email authentication
   - User registration and account management
   - User-network relationship tracking

3. **Generator Module** (`backend/generator/`)
   - Integration with C++ neural network generator
   - Command execution and file handling
   - Error handling and validation

#### Key Models

- **NetworkConfig**: Stores neural network configuration parameters
  - `name`: User-defined network name (unique)
  - `generated_name`: Auto-generated identifier based on parameters
  - `mode`: Operation mode (1, 2, or 3)
  - `input_size`, `output_size`, `T`, `R`, `P`, `layer_sizes`, `B`: Network parameters
  - `users`: ManyToMany field linking networks to users
  - `generated_files_path`: Path to cached generated files

- **CustomUser**: Extended Django User model
  - `email`: Unique email field
  - `networks`: Reverse relationship to NetworkConfig (ManyToMany)

### API Endpoints

- `GET/POST /networks/create/` - Create new network configuration
- `GET /networks/<id>/` - View network details
- `POST /networks/<id>/` - Download generated network files
- `GET /networks/` - List all networks (user-filtered in dashboard)
- `GET /dashboard/` - User dashboard showing their networks

### Installation

1. **Install Python dependencies**:
   ```bash
   cd backend
   pipenv install
   ```

2. **Run migrations**:
   ```bash
   python manage.py makemigrations
   python manage.py migrate
   ```

3. **Create superuser** (optional):
   ```bash
   python manage.py createsuperuser
   ```

4. **Start Redis** (required for rate limiting):
   ```bash
   redis-server
   ```

5. **Run development server**:
   ```bash
   python manage.py runserver
   ```

### Configuration

Key settings in `backend/config/settings.py`:
- `INSTALLED_APPS`: Core Django apps plus custom apps (accounts, networks, generator)
- `DATABASES`: Database configuration (SQLite by default)
- `CACHES`: Redis cache for rate limiting and session management
- `CELERY_BROKER_URL`: Redis connection for Celery task queue

### Testing

Run all tests:
```bash
python manage.py test
```

Run specific app tests:
```bash
python manage.py test networks
python manage.py test accounts
```

### File Structure

```
backend/
├── accounts/           # User management app
│   ├── models.py      # CustomUser model
│   ├── views.py       # User views
│   └── tests.py       # User tests
├── networks/          # Network configuration app
│   ├── models.py      # NetworkConfig model
│   ├── views.py       # Network views
│   ├── forms.py       # Network forms
│   ├── urls.py        # Network URL routing
│   └── tests.py       # Network tests
├── generator/         # C++ generator integration
│   ├── generator.py   # Generator wrapper
│   └── tests.py       # Generator tests
├── config/            # Django project configuration
├── templates/         # HTML templates
├── static/           # Static files (CSS, JS)
├── manage.py         # Django management script
└── Pipfile           # Python dependencies
```

### Future Enhancements

- Celery tasks for asynchronous network generation
- Automatic cleanup of duplicate networks
- Network sharing and collaboration features
- Advanced caching strategies
- Webhook notifications for generation completion