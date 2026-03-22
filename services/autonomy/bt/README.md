# Behavior Tree System - URC 2026 Rover Autonomy

## Overview

The Behavior Tree (BT) system provides hierarchical, reactive mission execution for the URC 2026 rover. Built on BT.CPP 4.x, it orchestrates complex mission sequences with built-in error recovery and real-time adaptation.

## Architecture

### Core Components

1. **BT Orchestrator** (`bt_orchestrator`): Lifecycle-managed ROS2 node that executes behavior trees
2. **BT Debugger** (`bt_debugger`): Offline validation and visualization tool
3. **Blackboard System**: Shared memory for inter-node communication
4. **Telemetry Integration**: Real-time execution monitoring

### BT Nodes Implemented

#### Control Flow Nodes
- `Sequence`: Execute children in order
- `Fallback`: Try alternatives on failure
- `Parallel`: Execute multiple branches simultaneously

#### Action Nodes
- `CallService`: ROS2 service calls for state transitions
- `SensorCheck`: Validate sensor health before mission start
- `NavigateToWaypoint`: Coordinate navigation to specific positions
- `SampleCollection`: Execute sample collection procedures
- `EmergencyStop`: Immediate safety shutdown

#### Condition Nodes
- `SensorAvailable`: Check sensor readiness
- `BatteryLevel`: Monitor power constraints
- `ObstacleFree`: Path safety validation

## Mission Templates

### Sample Collection Mission
```xml
<Sequence name="SampleCollectionMission">
    <NavigateToWaypoint x="10.0" y="5.0" tolerance="1.0"/>
    <SampleCollection site_id="1" timeout="60.0"/>
    <NavigateToWaypoint x="15.0" y="10.0" tolerance="1.0"/>
    <SampleCollection site_id="2" timeout="60.0"/>
</Sequence>
```

### Competition Mission Structure
```xml
<Sequence name="CompetitionMission">
    <Sequence name="SystemStartup">
        <CallService service_name="/state_machine/change_state" command="BOOT"/>
        <SensorCheck sensor_type="imu" timeout="5.0"/>
        <SensorCheck sensor_type="gps" timeout="5.0"/>
    </Sequence>

    <Fallback name="MissionExecution">
        <Sequence name="TaskSequence">
            <!-- Mission-specific tasks -->
        </Sequence>
        <EmergencyStop reason="mission_failure"/>
    </Fallback>
</Sequence>
```

## Blackboard Variables

The BT system uses a centralized blackboard for state sharing:

| Variable | Type | Description |
|----------|------|-------------|
| `robot_x` | double | Current X position |
| `robot_y` | double | Current Y position |
| `robot_yaw` | double | Current heading |
| `slam_x` | double | SLAM-estimated X |
| `slam_y` | double | SLAM-estimated Y |
| `slam_confidence` | double | SLAM confidence [0.0-1.0] |
| `mission_active` | bool | Mission execution state |

## Telemetry Integration

The BT system publishes detailed execution telemetry:

```json
{
  "component": "bt_orchestrator",
  "timestamp": 1703123456.789,
  "event_type": "tick|completed|shutdown",
  "tree_status": "RUNNING|SUCCESS|FAILURE",
  "blackboard": {
    "robot_x": 10.5,
    "robot_y": 5.2,
    "mission_active": true
  }
}
```

## Usage

### Running the BT System

1. **Start the orchestrator:**
   ```bash
   ros2 run autonomy_bt bt_orchestrator
   ```

2. **Validate BT files:**
   ```bash
   ros2 run autonomy_bt bt_debugger behavior_trees/main_mission.xml
   ```

3. **Monitor execution:**
   ```bash
   ros2 topic echo /bt/telemetry
   ```

### Creating Custom Missions

1. **Define BT XML** in `behavior_trees/` directory
2. **Register new nodes** in `bt_orchestrator.cpp`
3. **Update blackboard** variables as needed
4. **Test with debugger** before deployment

## Debugging Tools

### BT Debugger
Validates tree structure and node connectivity:
```bash
bt_debugger behavior_trees/mission.xml
```

### Execution Logs
- Console logging: Real-time execution status
- File logging: Complete execution traces in `/tmp/bt_execution.log`
- Telemetry: Structured execution data for analysis

### Visualization
BT execution can be visualized using Groot (BehaviorTree visualization tool):
```bash
# Connect to running BT orchestrator
groot --monitor
```

## Integration Points

### State Machine Integration
BT nodes can trigger state machine transitions:
```xml
<CallService service_name="/state_machine/change_state" command="AUTONOMOUS"/>
```

### Navigation Integration
Direct navigation action calls:
```xml
<NavigateToWaypoint x="10.0" y="5.0" tolerance="1.0"/>
```

### Sensor Integration
Health checks before mission execution:
```xml
<SensorCheck sensor_type="imu" timeout="5.0"/>
```

## Performance Characteristics

- **Execution Rate**: 10 Hz (configurable)
- **Memory Usage**: ~50MB baseline
- **Latency**: <10ms per tick
- **Recovery Time**: <100ms on failures

## Testing

### Unit Tests
```bash
colcon test --packages-select autonomy_bt
```

### Integration Tests
```bash
python3 tests/system/comprehensive_system_test.py
```

### Mission Validation
```bash
# Test specific mission
bt_debugger behavior_trees/science_mission.xml
bt_debugger behavior_trees/equipment_servicing_mission.xml
bt_debugger behavior_trees/autonomous_navigation_mission.xml
bt_debugger behavior_trees/delivery_mission.xml
```

### Mission Templates

The system includes complete BT templates for all URC 2026 missions:

- **Science Mission**: Sample collection, analysis, and caching
- **Equipment Servicing**: Autonomous typing, dexterous operations
- **Autonomous Navigation**: GNSS waypoints, AR tags, object detection
- **Delivery Mission**: Object pickup/delivery, terrain navigation

See `behavior_trees/README_MISSIONS.md` for detailed mission specifications.

## Future Enhancements

1. **Dynamic Tree Loading**: Runtime mission updates
2. **Learning Adaptation**: Self-optimizing execution
3. **Multi-Robot Coordination**: Swarm mission execution
4. **Advanced Recovery**: Predictive failure handling

## Dependencies

- `behaviortree_cpp` >= 4.0
- `rclcpp_lifecycle`
- `autonomy_interfaces`
- `geometry_msgs`

## References

- [BT.CPP Documentation](https://www.behaviortree.dev/)
- [Groot Visualizer](https://github.com/BehaviorTree/Groot)
- [URC 2026 Competition Rules](https://urc.marssociety.org/)
