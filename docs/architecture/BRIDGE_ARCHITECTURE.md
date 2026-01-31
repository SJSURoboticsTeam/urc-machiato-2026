# Bridge Architecture

## Overview

Bridges provide communication between ROS2 and hardware or external services. The architecture uses direct implementations without a shared abstract interface or factory.

## CAN Bridge

**Module:** `src/bridges/can_bridge.py`

- **CANBridge** is the only CAN implementation.
- SLCAN/teleoperation protocol logic is implemented inline (no separate protocol adapter).
- **API:**
  - `connect()` / `disconnect()` / `shutdown()`: Connection lifecycle.
  - `send_message(BridgeMessage)`: Send velocity, motor, emergency stop, heartbeat, homing, or sensor request.
  - `get_status()`: Returns `BridgeStatus` (is_connected, messages_sent, messages_received, errors, uptime, additional_info).
- **Message types:** velocity_command, motor_command, emergency_stop, heartbeat, homing, sensor_request.
- **Protocol:** SLCAN with teleoperation message IDs (e.g. 0x00C velocity, 0x00E heartbeat).

**Usage:** `src/autonomy/control/integrated_critical_systems.py` instantiates `CANBridge(config)` and uses it for CAN communication.

## Simple Bridge (Compatibility)

**Module:** `src/bridges/simple_bridge.py`

- **SimpleBridge** is an alias for **CANBridge**.
- **get_simple_bridge()** returns a singleton CANBridge with default config.
- New code should use **CANBridge** directly.

## Teleop WebSocket Bridge

**Module:** `src/bridges/teleop_websocket_bridge.py`

- **TeleopWebSocketBridge** is a ROS2 Node (not a CANBridge-style bridge).
- Handles Socket.IO and ROS2 (e.g. Twist, diagnostics).
- Separate from CAN bridge; used for frontend/teleop when enabled.

## Removed Abstractions

- **unified_bridge_interface.py** (BridgeInterface, factory, BridgeMessage/BridgeStatus): Removed; CAN bridge is standalone.
- **protocol_adapter.py** (ProtocolAdapter, VelocityScaler): Removed; scaling and encoding are inline in can_bridge.
- **teleop_can_adapter.py** (TeleopCANAdapter): Removed; logic inlined into CANBridge.

## Data Types

- **BridgeMessage:** message_type (str), data (dict). Defined in `can_bridge.py`.
- **BridgeStatus:** is_connected, last_message_time, messages_sent, messages_received, errors, uptime, additional_info. Defined in `can_bridge.py`.
