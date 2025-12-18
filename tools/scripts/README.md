# Scripts Directory

Organized utility scripts for development, testing, production deployment, CI/CD, and maintenance.

## Directory Structure

```
scripts/
├── production/          # Production deployment and validation
├── development/         # Development workflows and testing
├── ci-cd/              # Continuous integration and deployment
└── maintenance/        # System maintenance and setup
```

## Production Scripts (`production/`)

### Core Validation
- **`validate_config.py`** - Validates configuration files for production readiness
- **`production_health_check.py`** - Comprehensive system health check for deployment

### GitHub Integration
- **`extract-todos-to-issues.py`** - Automated TODO extraction from `*_TODO.md` files and GitHub issue creation

## Development Scripts (`development/`)

### System Management
- **`launch_autonomy_system.py`** - Launch complete autonomy system with all components

### Demos & Testing
- **`software_system_demo.py`** - Comprehensive system demonstration script

### Testing Framework (`testing/`)
- **`testing/quick_test.sh`** - Fast integration test for autonomy-teleoperation
- **`testing/test_manual_integration.sh`** - Manual integration testing procedures

## CI/CD Scripts (`ci-cd/`)

### Docker & Deployment
- **`build_universal_docker.sh`** - Build universal Docker images for different environments

### Documentation
- **`build_docs.sh`** - Build Sphinx documentation from source

## Maintenance Scripts (`maintenance/`)

### Environment Setup
- **`setup_ros2_env.sh`** - Setup ROS2 development environment

### Repository Management
- **`manage_submodules.sh`** - Git submodule management and synchronization

## Usage

### Production Deployment
```bash
# Validate configuration
python3 scripts/production/validate_config.py

# Health check
python3 scripts/production/production_health_check.py

# Extract TODOs to GitHub issues
python3 scripts/production/extract-todos-to-issues.py
```

### Development
```bash
# Launch system
python3 scripts/development/launch_autonomy_system.py

# Run demo
python3 scripts/development/software_system_demo.py

# Quick integration test
./scripts/development/testing/quick_test.sh
```

### CI/CD
```bash
# Build Docker images
./scripts/ci-cd/build_universal_docker.sh

# Build documentation
./scripts/ci-cd/build_docs.sh
```

### Maintenance
```bash
# Setup ROS2 environment
./scripts/maintenance/setup_ros2_env.sh

# Manage submodules
./scripts/maintenance/manage_submodules.sh
```

## Dependencies

- **Production**: PyYAML, requests (GitHub API)
- **Development**: ROS2 Humble, Python dependencies
- **CI/CD**: Docker, Sphinx
- **Maintenance**: ROS2 Humble, Git

## Notes

- Scripts are organized by purpose and usage context
- Each subdirectory has its own README with detailed usage
- Production scripts are validated and tested for deployment
- Development scripts may require ROS2 environment and hardware access
