# Onboarding by Specialization

This section provides standalone onboarding guides for different development areas. Choose the document that matches your role or area of focus.

## First-time ROS 2 + dashboard bridge setup

| Document | Audience | Contents |
|----------|----------|----------|
| [ros2_workspace_and_bridges](ros2_workspace_and_bridges.md) | All developers using ROS 2 + dashboard | **Complete** `colcon` build (this repo’s `shared/` + `services/` paths), **system Python / CMake** alignment, **`bt_orchestrator`** lifecycle, **`rosbridge_stack`**, verification commands |
| [ROS 2 + Python](../development/ros2_python_environment.rst) | Anyone hitting `UnsupportedTypeSupport` / wrong `libpython` | Distro Python versions, conda/venv, `PYTHON_EXECUTABLE`, clean rebuild |

## Pillar Documents

| Document | Audience | Contents |
|----------|----------|----------|
| [PILLAR_1_PERCEPTION](PILLAR_1_PERCEPTION.md) | Perception / Vision / SLAM | Computer vision, SLAM, sensor fusion, sensor bridge |
| [PILLAR_2_COGNITION](PILLAR_2_COGNITION.md) | Autonomy / Mission / BT | Behavior trees, state machines, mission execution, blackboard |
| [PILLAR_3_MOTION_CONTROL](PILLAR_3_MOTION_CONTROL.md) | Motion / Hardware / Control | Motor control, hardware interfaces, safety integration |
| [PILLAR_4_COMMUNICATION](PILLAR_4_COMMUNICATION.md) | Comms / Bridges / Network | CAN bridge, WebSocket, circuit breakers, network resilience |

## Code Examples

Minimal runnable examples are in `onboarding/examples/`:

- `component_registry.py` - Component registry usage
- `state_machine.py` - State machine patterns
- `safety_system.py` - Safety system integration

## Quick Links

- [Getting Started](../getting_started.rst) - Full setup guide
- [Build System](../operations/build_system.rst) - Unified build commands
- [Architecture Overview](../architecture/overview.rst) - System design
