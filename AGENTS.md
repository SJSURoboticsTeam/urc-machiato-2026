# AGENTS.md - URC Machiato 2026 Development Guide

Essential commands and code style guidelines for agentic coding assistants.

## Essential Commands

### ROS 2 workspace (autonomy packages)

Build with **system Python** (same major.minor as ROS: Jazzy/3.12, Humble/3.10). Deactivate conda/venv first, then:

```bash
source /opt/ros/jazzy/setup.bash   # or humble
export PYTHON_EXECUTABLE="$(command -v python3)"
./scripts/build_ros_for_bt_tests.sh   # or ./scripts/clean_ros_build.sh
source install/setup.bash
```

If Python imports of `autonomy_interfaces` fail with `UnsupportedTypeSupport` or `libpython3.N.so`, see `docs/development/ros2_python_environment.rst` and clean-rebuild.

### Build System
```bash
./scripts/build.sh dev           # Development build (fast)
./scripts/build.sh prod          # Production build (optimized)  
./scripts/build.sh comp          # Competition build (safety-first)
./scripts/build.sh clean         # Clean all artifacts
./scripts/build.sh dev --test    # Build + run tests
./scripts/build.sh prod --deploy # Build + deploy
```

### Testing (pytest + markers)
```bash
# Run specific test types
python -m pytest tests/unit/ -v                    # Unit tests only
python -m pytest tests/integration/ -v             # Integration tests  

# Run single test (KEY COMMAND)
python -m pytest path/to/test_file.py::test_name    # Run single test
python -m pytest tests/unit/test_example.py::test_function -v

# Pattern matching and markers
python -m pytest -k "test_pattern"                 # Name pattern match
python -m pytest -m "unit"                         # By marker
python -m pytest -m "not slow"                     # Exclude slow tests

# Coverage reporting
python -m pytest tests/ --cov=src --cov-report=html # Full suite + coverage
```

### Code Quality (Primary: ruff)
```bash
./scripts/check_quality.sh    # Complete check (format, lint, test)
black .                       # Format Python code  
ruff check .                  # Lint (replaces flake8+isort)
ruff check --fix .           # Auto-fix linting issues
mypy .                        # Type checking
```

### Frontend (services/dashboard/)
```bash
npm run dev                   # Development server
npm run build                 # Production build  
npm run lint                  # ESLint
npm run test                  # Vitest
npm run test:coverage        # Coverage report
```

### System Launch
```bash
./start.py dev frontend       # Frontend development server
./start.py dev dashboard      # Testing dashboard (backend + frontend)  
./start.py prod autonomy      # Full autonomy system
./start.py dev simulation     # Gazebo simulation
```

## Code Style Guidelines

### Python Code Style

#### Import Order (Strict)
1. Standard library imports (alphabetical)
2. Third-party imports (alphabetical)  
3. Local imports with intelligent fallbacks

```python
# Standard library imports
import asyncio
import math
import os
from typing import Any, Dict, List, Optional

# Third-party imports
import numpy as np
import rclpy
from std_msgs.msg import String

# Local imports
try:
    from autonomy_navigation.gnss_processor import GNSSProcessor
except ImportError:
    from gnss_processor import GNSSProcessor
```

#### Naming Conventions
- **Classes**: `PascalCase` (e.g., `NavigationNode`, `SafetyMonitor`)
- **Functions/Variables**: `snake_case` (e.g., `start_navigation`, `sensor_data`)
- **Constants**: `UPPER_SNAKE_CASE` (e.g., `TOAST_LIMIT`, `MAX_RETRIES`)
- **Private methods**: `_snake_case` with underscore prefix
- **ROS2 nodes**: `PascalCase` ending in `Node` (e.g., `PerceptionNode`)

#### Type Hints (Required)
```python
def process_sensor_data(
    data: Dict[str, Any], 
    timeout: Optional[float] = None
) -> Tuple[bool, Optional[Dict[str, float]]]:
    """Process sensor data with timeout.
    
    Args:
        data: Sensor data dictionary
        timeout: Optional timeout in seconds
        
    Returns:
        Tuple of (success, processed_data)
    """
    pass
```

#### Error Handling Pattern
```python
from .exceptions import NavigationError, ProcessingError

try:
    result = some_operation()
    if isinstance(result, Failure):
        self.logger.error("Operation failed", error=result.error)
        return failure(ProcessingError("operation", result.error))
except NavigationError as e:
    self.logger.error(f"Navigation failed: {e}")
    return failure(e)
except Exception as e:
    self.logger.error(f"Unexpected error: {e}")
    return failure(ProcessingError("unexpected", str(e)))
```

### TypeScript/React Code Style

#### Component Structure
```typescript
// Type imports first
import type { ToastActionElement, ToastProps } from '@/components/ui/toast';

// React imports
import { useState, useEffect, useRef, useCallback } from 'react';

// Third-party imports
import ROSLIB from '../utils/rosbridge';

// Local imports
import { UI_CONSTANTS } from '../constants/uiConstants';

interface ComponentProps {
  /** Component description */
  data: SensorData[];
  onAction?: (action: string) => void;
  timeout?: number;
}

export function SensorDashboard({ data, onAction, timeout = 5000 }: ComponentProps) {
  // Hook-first architecture
  const [state, setState] = useState<SensorState>({});
  const rosRef = useRef<ROSLIB.Ros | null>(null);
  
  // Use custom hooks
  const { toast } = useToast();
  const { isConnected } = useROS();
  
  // Implementation...
}
```

#### Naming Conventions
- **Interfaces**: `PascalCase` (e.g., `DashboardStat`, `ChartDataPoint`)
- **Functions/Variables**: `camelCase` (e.g., `useToast`, `generateMockGPS`)
- **Constants**: `UPPER_SNAKE_CASE` (e.g., `UI_CONSTANTS`, `MAX_RETRIES`)
- **React Hooks**: `camelCase` with `use` prefix (e.g., `useROS`, `useToast`)

## Cursor Rules Reference

Based on `.cursorrules` for project-specific AI assistant guidelines:

### Project Context
- **URC 2026 University Rover Challenge**: ROS2-based autonomous Mars rover system
- **Core Components**: Autonomy Stack, Mission System, Web Dashboard, Simulation, Hardware
- **Key Technologies**: Python 3.10+, ROS2 Humble, React 18+, TypeScript, Gazebo, Behavior Trees

### Code Organization Principles
- ROS2 packages follow functional organization (not layered architecture)
- Missions are separate from core autonomy for modularity
- Hardware interfaces are abstracted for testing
- Configuration is centralized in `config/rover.yaml`

### Development Guidelines
- **Testing First**: Write tests before implementation
- **Documentation**: Update docs for any API changes
- **Code Style**: Black formatting, type hints, comprehensive error handling
- **ROS2 Best Practices**: Proper package.xml, launch files, and message definitions

### Common Tasks
- Adding new missions: Create in `missions/`, add BT in `src/autonomy/bt/`
- Modifying autonomy: Update ROS2 packages in `src/autonomy/`
- Changing UI: Modify React components in `src/frontend/`
- Adding hardware: Create interfaces in `src/autonomy/control/`

### File Locations Reference
- ROS2 messages: `src/autonomy/interfaces/msg/`
- Mission logic: `missions/*.py`
- Navigation code: `src/autonomy/core/navigation/`
- Web components: `src/frontend/src/components/`
- Configuration: `config/rover.yaml`
- Tests: `tests/` directory
- Documentation: `docs/` directory

### Quality Standards
- Test coverage >80% for new code
- ROS2 linting passes
- Documentation updated for public APIs
- Code reviewed before merging
- Works in simulation before hardware testing

## Development Workflow

### Before Committing
1. Run quality check: `./scripts/check_quality.sh`
2. Run tests: `python -m pytest tests/unit/ -v`
3. Format code: `black . && ruff check --fix .`
4. Type check: `mypy .`

### Key Requirements
- Test coverage: >80% for new code
- Documentation: Update docs for API changes
- Safety: All robotics code must have safety checks
- Performance: Consider resource constraints in robotics environment