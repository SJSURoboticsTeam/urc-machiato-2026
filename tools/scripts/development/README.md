# Development Scripts

Scripts for development workflows, testing, and system integration.

## Scripts

### System Management
- **`launch_autonomy_system.py`** - Launch complete autonomy system with all components

### Demos & Testing
- **`software_system_demo.py`** - Comprehensive system demonstration script

### Testing Framework (`testing/`)
- **`testing/quick_test.sh`** - Fast integration test for autonomy-teleoperation
- **`testing/test_manual_integration.sh`** - Manual integration testing procedures

## Usage

### Launch System
```bash
python3 scripts/development/launch_autonomy_system.py
```

### Run Demo
```bash
python3 scripts/development/software_system_demo.py
```

### Quick Integration Test
```bash
./scripts/development/testing/quick_test.sh
```

## Dependencies

- ROS2 Humble (for integration tests)
- Python dependencies as specified in `pyproject.toml`

## Notes

- Development scripts may require ROS2 environment setup
- Some scripts need hardware access for full functionality
- Integration tests use mock components where hardware is unavailable
