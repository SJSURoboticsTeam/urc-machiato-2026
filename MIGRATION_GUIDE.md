# Migration Guide - Maintainability Cleanup

This guide describes breaking changes and migration steps from the maintainability cleanup (January 2026).

## Dependencies Removed

- **asyncio-mqtt** - No imports found; removed.
- **aioredis** - Deprecated; removed (use redis package if needed).
- **dependency-injector** - Not used; custom ComponentRegistry is used instead; removed.

**Migration:** None. No code referenced these packages.

## Docker

- **Dockerfile.optimized** was merged into **Dockerfile.universal** as build targets: performance-optimized, sensor-bridge-optimized, computer-vision-optimized, autonomy-system-optimized, mission-control-optimized.
- **docker-compose.optimized.yml** now uses `docker/Dockerfile.universal` with the above targets.

**Migration:** If you had local scripts that built from Dockerfile.optimized, switch to building the appropriate target from Dockerfile.universal (e.g. sensor-bridge-optimized).

## State Management

- **JazzyStateMachineBridge** was removed. Jazzy launch and component manager use **AdaptiveStateMachine** instead.
- Launch lifecycle commands use `/adaptive_state_machine` instead of `/jazzy_state_machine_bridge`.
- Topics: use `/adaptive_state_machine/state` and `/adaptive_state_machine/commands` instead of `/jazzy_state_machine_bridge/telemetry` and `/jazzy_state_machine/commands`.

**Migration:** Update any custom launch or scripts that referenced JazzyStateMachineBridge or the old topic names to use AdaptiveStateMachine and the new topics.

## Bridge Architecture

- **unified_bridge_interface.py**, **protocol_adapter.py**, and **teleop_can_adapter.py** were removed.
- **CANBridge** is now a direct implementation in `src/bridges/can_bridge.py` with inline SLCAN/teleoperation logic.
- **BridgeMessage** and **BridgeStatus** are defined in `can_bridge.py` (no longer in unified_bridge_interface).
- **simple_bridge** is a stub: `SimpleBridge = CANBridge`, `get_simple_bridge()` returns a default CANBridge instance.

**Migration:**
- Replace `from src.bridges.unified_bridge_interface import BridgeStatus` (or similar) with `from src.bridges.can_bridge import BridgeStatus` (or BridgeMessage, CANBridge).
- Replace `from src.bridges.teleop_can_adapter import TeleopCANAdapter` with use of **CANBridge**; encoding/decoding are internal to CANBridge.
- **create_bridge()** and **register_bridge_factory** no longer exist; instantiate **CANBridge(config)** directly.

## Configuration

- **config_manager** remains the canonical API for typed config (RoverConfig, get_system_config()).
- **dynaconf_config** provides **get_urc_config()** and **get_settings()** for environment-based loading (rover.yaml + env yaml + local.yaml + URC_* env vars).

**Migration:** No breaking change. To use unified Dynaconf loading, call `get_urc_config()` or `get_settings()` from `src.config.dynaconf_config` instead of (or in addition to) config_manager where appropriate.

## Backup Files

- A pre-commit hook blocks `*.backup` files from being committed.

**Migration:** Rename or remove any `.backup` files you need to keep; do not commit them.

## Documentation

- **docs/architecture/STATE_MANAGEMENT.md** - Runtime vs dashboard state.
- **docs/architecture/BRIDGE_ARCHITECTURE.md** - Simplified bridge design.
- **docs/architecture/CONFIGURATION.md** - Config systems and when to use each.
