.. _development_workflow:

=====================
Development Workflow
=====================

This guide explains the day-to-day development process for the URC 2026 rover project.

Daily Development Routine
=======================

1. **Start Fresh**
   ```bash
   # Get latest changes
   git pull --recurse-submodules
   
   # Create feature branch
   git checkout -b feature/my-feature-name
   ```

2. **Setup Environment**
   ```bash
   # Source ROS2
   source /opt/ros/humble/setup.bash
   
   # Check everything builds
   ./scripts/build.sh dev
   ```

3. **Make Changes**
   - Edit code in appropriate ``src/`` directory
   - Follow the style guidelines in :doc:`code_quality`
   - Add tests for new functionality

4. **Test Your Changes**
   ```bash
   # Quick development tests
   python scripts/run_tests.py dev
   
   # Full quality checks before committing
   python scripts/run_tests.py pre-commit
   ```

5. **Commit and Push**
   ```bash
   # Stage changes
   git add .
   
   # Commit with good message
   git commit -m "feat: Add new feature description"
   
   # Push and create PR
   git push origin feature/my-feature-name
   ```

Code Organization
================

The project follows a layered architecture:

```
src/
├── infrastructure/          # 🔧 Unified infrastructure
│   ├── config/            # Configuration management
│   ├── bridges/           # Communication bridges
│   └── monitoring/        # System monitoring
├── autonomy/              # 🤖 Core robotics stack
│   ├── autonomy_core/     # Navigation, safety, perception
│   ├── interfaces/        # ROS2 messages/services
│   └── bt/              # Behavior trees
├── dashboard/             # 💻 React web interface
└── simulation/            # 🎮 Gazebo integration
```

Key Development Areas
--------------------

**Autonomy Systems** (``src/autonomy/``)
- Navigation and path planning
- Safety monitoring and emergency stops
- Computer vision and perception
- State machines and behavior trees

**Infrastructure** (``src/infrastructure/``)
- Configuration management
- WebSocket bridges for web interface
- System monitoring and health checks
- Network resilience

**Web Dashboard** (``src/dashboard/``)
- React components for monitoring
- Real-time data visualization
- Mission control interface

**Simulation** (``src/simulation/``)
- Gazebo world files
- Robot models and physics
- Testing scenarios

Build System
============

The project uses a unified build system:

```bash
# Development build (fast, debug info)
./scripts/build.sh dev

# Production build (optimized)
./scripts/build.sh prod

# Competition build (safety-first)
./scripts/build.sh comp

# Clean all build artifacts
./scripts/build.sh clean
```

Testing Strategy
===============

Smart Test Runner
----------------

Use the smart test runner for different scenarios:

```bash
# Fast feedback (<30s)
python scripts/run_tests.py dev

# Quality gates (<2min) 
python scripts/run_tests.py pre-commit

# Full validation (<10min)
python scripts/run_tests.py ci

# Hardware-in-the-loop
python scripts/run_tests.py hardware

# Performance testing
python scripts/run_tests.py performance
```

Test Categories
---------------

- **Unit Tests**: Individual component testing
- **Integration Tests**: Cross-component validation  
- **Hardware Tests**: Real hardware validation
- **Performance Tests**: System benchmarks
- **Simulation Tests**: Gazebo-based testing

Code Quality Standards
=====================

Pre-commit Hooks
----------------

The project uses pre-commit hooks for quality:

```bash
# Run all quality checks
pre-commit run --all-files

# Individual tools
python -m ruff check src/        # Linting
python -m ruff format src/       # Formatting  
python -m isort --profile=black src/  # Import sorting
python -m mypy src/              # Type checking
```

Coverage Requirements
-------------------

- **Minimum**: 85% code coverage for CI/CD
- **Target**: 90% for new code
- **Critical Path**: 95%+ for safety systems

Documentation
=============

Public APIs require documentation:

```python
def navigate_to_waypoint(x: float, y: float, timeout: float = 30.0) -> bool:
    """
    Navigate the rover to a specific waypoint.
    
    Args:
        x: Target X coordinate in meters
        y: Target Y coordinate in meters  
        timeout: Maximum time to wait in seconds
        
    Returns:
        True if waypoint reached successfully, False otherwise
        
    Raises:
        NavigationError: If navigation fails
        TimeoutError: If waypoint not reached within timeout
    """
```

Debugging and Troubleshooting
============================

Common Issues
-------------

**Build Failures**
```bash
# Check dependencies
./scripts/build.sh dev --test

# Clean and rebuild
./scripts/build.sh clean
./scripts/build.sh dev
```

**Test Failures**
```bash
# Run with verbose output
python scripts/run_tests.py dev --verbose

# Run specific test
python -m pytest tests/unit/test_navigation.py -v
```

**ROS2 Issues**
```bash
# Check ROS2 environment
ros2 doctor

# Check topics/nodes
ros2 topic list
ros2 node list
```

Performance Monitoring
--------------------

The system includes comprehensive monitoring:

```bash
# Check system health
ros2 topic echo /system/service_health

# View performance metrics
ros2 topic echo /system/metrics
```

Getting Help
============

- **📖 Documentation**: Check ``docs/`` directory
- **🧪 Testing**: See :doc:`testing` guide  
- **🤝 Team Chat**: Ask in development channels
- **📋 Issues**: Check GitHub issues first
- **📧 Email**: Contact maintainers for urgent issues

Remember: This is a complex robotics system. Don't hesitate to ask questions!