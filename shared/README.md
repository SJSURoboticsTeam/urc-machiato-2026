# Shared Libraries

The shared library contains common code utilized across multiple microservices. This includes core observability systems, shared utilities, common data structures, and infrastructure tools like bridges and logging.

## Structure
- `core/`: Observability, logging, feature flags, configuration managers
- `infrastructure/`: Networking, WebSocket/CAN bridges
- `interfaces/`: Shared ROS2 message definitions

## How to Test
```bash
python -m pytest tests/unit/core/
```