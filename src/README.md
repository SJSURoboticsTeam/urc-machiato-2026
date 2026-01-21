# Source Code Directory

Main source code for the URC 2026 Mars Rover project.

## Structure

### Core Systems
- `autonomy/` - ROS2 packages for autonomous operation
  - `bt/` - Behavior trees
  - `control/` - Motor control, hardware interfaces
  - `core/` - Navigation, safety, state management
  - `perception/` - Computer vision, SLAM, sensors
  - `utilities/` - Shared utilities

### Bridges & Communication
- `bridges/` - Protocol adapters and communication bridges
  - CAN bridge (SLCAN protocol)
  - WebSocket/Socket.IO bridge
  - Protocol adapters

### User Interfaces
- `dashboard/` - Operator dashboard (teleoperation UI)
  - HTML/React dashboard for rover control
  - Connects via WebSocket to teleoperation backend
- `frontend/` - React web frontend (monitoring/visualization)
  - General-purpose web UI
  - Separate from operator dashboard

### Other
- `cli/` - Command-line interface tools
- `comms/` - Communication systems
- `config/` - Configuration management (Dynaconf)
- `core/` - Core system components
  - State machines
  - Data management
  - Security
- `launch/` - ROS2 launch files
  - `integrated_bridge_system.launch.py` - Bridge system launch
  - `jazzy/` - Jazzy-specific launch files
- `motion/` - Motion planning and control
- `sensors/` - Sensor interfaces and drivers
- `simulation/` - Simulation components
- `testing/` - Testing utilities

## Related Directories

- `../config/` - Configuration files (rover.yaml, etc.)
- `../missions/` - Mission implementations
- `../tests/` - Test suites
- `../output/` - Generated reports, metrics, logs
- `../vendor/` - Submodules (control-systems, teleoperation)

## Development

See main `README.md` for development setup and guidelines.
