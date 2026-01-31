# AGENTS.md - URC Machiato 2026 Development Guide

Development guidelines and commands for agentic coding assistants on URC 2026 robotics platform.

## Essential Commands

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
python -m pytest tests/unit/ -v                    # Unit tests only
python -m pytest tests/integration/ -v             # Integration tests  
python -m pytest tests/ --cov=src --cov-report=html # Full suite + coverage
python -m pytest path/to/test_file.py::test_name    # Run single test
python -m pytest -k "test_pattern"                 # Name pattern match
python -m pytest -m "unit"                         # By marker
python -m pytest -m "not slow"                     # Exclude slow tests
```

### Code Quality (Primary: ruff)
```bash
./scripts/check_quality.sh    # Complete check (format, lint, test)
black .                       # Format Python code  
ruff check .                  # Lint (replaces flake8+isort)
ruff check --fix .           # Auto-fix linting issues
mypy .                        # Type checking
```

### Frontend (src/dashboard/)
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

#### Imports (Strict Order)
```python
# Standard library imports (alphabetical)
import asyncio
import math
import os
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

# Third-party imports
import numpy as np
import rclpy
from std_msgs.msg import String

# Local imports with intelligent fallbacks
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

## Architecture Patterns

### ROS2 Node Structure
```python
class NavigationNode(rclpy.lifecycle.LifecycleNode):
    """ROS2 lifecycle node for navigation."""
    
    def __init__(self):
        super().__init__('navigation_node')
        self.logger = self.get_logger()
        
    def on_configure(self, state: rclpy.lifecycle.State) -> rclpy.lifecycle.CallbackReturn:
        """Configuration phase - setup publishers/subscribers."""
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 
            qos_profile=QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        )
        return rclpy.lifecycle.CallbackReturn.SUCCESS
```

### Safety Systems (Critical)
All safety-critical code must:
1. Use redundant safety layers (primary, secondary, tertiary)
2. Implement comprehensive health monitoring
3. Provide emergency response coordination
4. Include structured logging with context

```python
class SafetyMonitor:
    """Multi-layer safety monitoring system."""
    
    def check_system_health(self) -> SafetyStatus:
        """Comprehensive safety check."""
        primary_result = self.primary_safety.check()
        if not primary_result.is_safe:
            return self.emergency_handler.handle(primary_result)
        return SafetyStatus.from_results(primary_result, self.secondary_safety.check())
```

## Testing Guidelines

### Python Test Structure
```python
import pytest
from unittest.mock import Mock
from autonomy.navigation import NavigationNode

class TestNavigationNode:
    @pytest.fixture
    def navigation_node(self):
        return NavigationNode()
        
    def test_navigation_success(self, navigation_node):
        mock_twist = Twist()
        mock_twist.linear.x = 1.0
        result = navigation_node.process_command(mock_twist)
        assert result.success is True
```

### Frontend Test Structure
```typescript
import { render, screen } from '@testing-library/react';
import { SensorDashboard } from './SensorDashboard';

describe('SensorDashboard', () => {
  it('renders sensor data correctly', async () => {
    const mockData = [{ id: 1, value: 100 }];
    render(<SensorDashboard data={mockData} />);
    expect(screen.getByText('100')).toBeInTheDocument();
  });
});
```

## Development Workflow

### Before Committing
1. **Run quality check**: `./scripts/check_quality.sh`
2. **Run tests**: `python -m pytest tests/unit/ -v`
3. **Format code**: `black . && ruff check --fix .`
4. **Type check**: `mypy .`

### File Organization
- **ROS2 packages**: `src/autonomy/` (functional organization)
- **Mission logic**: `missions/` (separate from core autonomy)
- **Web dashboard**: `src/dashboard/` (React/TypeScript)
- **Configuration**: `config/rover.yaml` (centralized config)
- **Tests**: `tests/unit/`, `tests/integration/`, `tests/performance/`

### Key Requirements
- **Test coverage**: >80% for new code
- **Documentation**: Update docs for API changes
- **Safety**: All robotics code must have safety checks
- **Performance**: Consider resource constraints in robotics environment

## Project Context

University Rover Challenge autonomous Mars rover system with:
- **ROS2 Humble/Jazzy** backend for robotics
- **React 18 + TypeScript** frontend for dashboard
- **Gazebo** simulation environment
- **STM32** hardware interfaces (via submodules)
- **Behavior Trees** for mission logic

Emphasizes safety, reliability, and comprehensive testing for competition robotics.