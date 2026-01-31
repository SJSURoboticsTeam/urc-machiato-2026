# CAN, ROS2, and Blackboard Testing

## Does the current test suite test CAN to blackboard?

**No.** The existing tests do **not** exercise communication between CAN and the ROS2 blackboard.

| What is tested | What is not tested |
|----------------|--------------------|
| CAN message encoding/decoding (SLCAN, velocity scaling) | CAN data flowing into ROS2 topics |
| Hardware interface node file presence and config | Hardware interface publishing to topics that blackboard writers use |
| Protocol message IDs and firmware compatibility | Safety watchdog or SLAM node updating blackboard from those topics |

**Reason:** The blackboard is updated by ROS2 nodes that subscribe to specific topics:

- **Safety watchdog** subscribes to `/battery/status` and calls `blackboard.set("battery_level", ...)`.
- **SLAM node** subscribes to `/odom` and updates blackboard keys such as `slam_confidence`, `feature_count`, `map_quality`, `perception_confidence`.

The hardware interface node publishes to **different** topic names:

- Publishes `/hardware/battery_state` (watchdog expects `/battery/status`).
- Publishes `/hardware/chassis_velocity` (SLAM expects `/odom`).

So with the current setup, CAN → hardware_interface → ROS2 does **not** by itself feed the blackboard. You need either topic remaps or a bridge that republishes `/hardware/*` onto `/battery/status` and `/odom`.

---

## How to use the simulator to visualize the blackboard

To check that the blackboard updates correctly **before** using real CAN:

1. **Run a bridge** that publishes the topics the blackboard writers expect:
   - `/battery/status` (for safety watchdog → `battery_level`, etc.)
   - `/odom` (for SLAM node → `slam_confidence`, `feature_count`, `map_quality`, `perception_confidence`)

   Options:
   - Use the **CAN-to-blackboard bridge** node: it republishes `/hardware/battery_state` → `/battery/status` and `/hardware/chassis_velocity` → `/odom`. In simulation, it can also publish mock `/battery/status` and `/odom` when hardware topics are not present.
   - Or use the existing **sensor_simulator** (publishes `/odom`) and add something that publishes `/battery/status` (e.g. the bridge in “sim only” mode).

2. **Run the nodes that write to the blackboard** (e.g. safety watchdog, SLAM node), so they receive `/battery/status` and `/odom` and call blackboard `set_value`.

3. **Run the blackboard visualizer** script, which periodically calls `GetBlackboardValue` for the main keys and prints them (or publishes to a topic). That gives you a live view of blackboard state driven by simulated (or bridged) CAN data.

**Minimal flow:**

```text
[Simulated CAN / HIL or mock data]
        ↓
[Bridge: /hardware/* → /battery/status, /odom]   (or sensor_simulator + mock battery)
        ↓
[Safety watchdog, SLAM node, etc.]
        ↓
[Blackboard (set_value)]
        ↑
[Blackboard visualizer (get_value and print)]
```

---

## Does simulated CAN need to be updated?

**Simulated CAN itself** (e.g. `control_systems_hil.py`, `slcan_protocol_simulator.py`, `swerve_simulator.py`) does **not** publish ROS2 topics. It only simulates CAN/SLCAN or internal state.

So:

- **No change to the core simulators is required** for protocol or firmware behavior.
- **You do need a ROS2 bridge** that turns simulated (or real) hardware data into the topics the blackboard writers use:
  - Either:
    - **From real CAN:** hardware_interface already publishes `/hardware/battery_state` and `/hardware/chassis_velocity`; add a node that republishes these to `/battery/status` and `/odom`, or
    - **From simulation:** a node that publishes mock `/battery/status` and `/odom` (and optionally subscribes later to `/hardware/*` when you connect real CAN).

The **CAN-to-blackboard bridge** node is that piece: it subscribes to `/hardware/battery_state` and `/hardware/chassis_velocity` and republishes to `/battery/status` and `/odom`. It can also run in a “sim only” mode with no hardware, publishing mock data so you can validate blackboard updates using only the simulator and this bridge.

---

## Summary

| Question | Answer |
|----------|--------|
| Will current tests verify CAN ↔ blackboard? | No; they only test encoding and node presence. |
| How to visualize blackboard with the simulator? | Run bridge (mock or republish `/hardware/*`) → run watchdog + SLAM (or other writers) → run blackboard visualizer. |
| Does simulated CAN need to be updated? | No; add a ROS2 bridge from `/hardware/*` (or mock) to `/battery/status` and `/odom` so blackboard writers get data. |

**How to run (simulator flow):**

1. **Terminal 1 – bridge (mock data so blackboard writers get input):**
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash  # if you use colcon
   python3 scripts/hardware/can_to_blackboard_bridge.py
   ```
   With `use_sim_fallback=true` (default), it publishes mock `/battery/status` and `/odom` so you do not need hardware or the hardware interface node.

2. **Terminal 2 – nodes that write to the blackboard** (safety watchdog, SLAM node, etc.), so they receive `/battery/status` and `/odom` and call blackboard `set_value`. Run your normal launch that starts these nodes.

3. **Terminal 3 – blackboard visualizer:**
   ```bash
   python3 scripts/hardware/blackboard_visualizer.py --rate 2.0
   ```
   Requires the blackboard service (e.g. from BT.CPP or another node that advertises `/blackboard/get_value`). If the service is not available, the visualizer reports that and exits.

**Files:**

- `scripts/hardware/can_to_blackboard_bridge.py` – republishes `/hardware/*` to `/battery/status` and `/odom`; in sim mode publishes mock data.
- `scripts/hardware/blackboard_visualizer.py` – prints blackboard keys for inspection.
