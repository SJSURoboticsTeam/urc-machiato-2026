# 🎯 URC Mission Implementations

This directory contains the high-level mission logic for all University Rover Challenge 2026 competitions. Each mission implements a complete URC challenge with planning, execution, and recovery.

## 📋 Available Missions

| Mission | URC Challenge | Key Features | Status |
|---------|---------------|--------------|--------|
| **Sample Collection** | Science | Autonomous sample detection, approach, collection | ✅ Complete |
| **Delivery** | Delivery | Multi-phase object pickup and delivery | ✅ Complete |
| **Waypoint Navigation** | Navigation | GPS waypoint following with obstacle avoidance | ✅ Complete |
| **Autonomous Keyboard** | Equipment Service | Computer interaction and typing | ✅ Framework |
| **Follow Me** | Follow-Me | ArUco tag tracking and following | ✅ Complete |
| **Return to Operator** | Recovery | Safe return with multi-modal localization | ✅ Complete |

## 🏗️ Mission Architecture

Each mission follows a consistent pattern:

### Core Components
```python
class SampleCollectionMission:
    def __init__(self):           # Setup and configuration
    def plan_mission(self):       # Generate waypoints/tasks
    def execute_mission(self):    # Run mission logic
    def monitor_progress(self):   # Track completion status
    def handle_failures(self):    # Error recovery
```

### Integration Points
- **ROS2 Actions**: Long-running mission execution
- **Behavior Trees**: Complex decision logic (see `src/autonomy/bt/`)
- **Web Dashboard**: Real-time mission monitoring
- **Hardware Interfaces**: Actuator control and sensor feedback

## 🚀 Using Missions

### Starting a Mission
```python
# Via ROS2 action client
ros2 action send_goal /mission/execute_mission autonomy_interfaces/action/ExecuteMission "{
  mission_type: 'sample_collection',
  mission_id: 'mission_001'
}"
```

### Via Mission Executor
```python
from missions.mission_executor import MissionExecutor
from missions.sample_collection_mission import SampleCollectionMission

executor = MissionExecutor()
mission = SampleCollectionMission()
executor.start_mission(mission)
```

### Via Web Dashboard
1. Open dashboard: `./start.py dev dashboard`
2. Select mission from dropdown
3. Configure parameters
4. Click "Start Mission"

## 🔧 Adding a New Mission

### 1. Create Mission Class
```python
# missions/new_mission.py
from missions.base_mission import BaseMission

class NewMission(BaseMission):
    def __init__(self):
        super().__init__("new_mission")

    def plan_mission(self, params):
        # Generate mission plan
        return mission_plan

    def execute_mission(self, plan):
        # Execute mission steps
        pass
```

### 2. Add Behavior Tree Logic
Create BT XML in `src/autonomy/bt/behavior_trees/new_mission.xml`

### 3. Update ROS2 Interfaces
Add mission type to `src/autonomy/interfaces/action/ExecuteMission.action`

### 4. Register with Executor
Add to `mission_executor.py` mission registry

### 5. Add Dashboard Support
Update frontend mission selection and parameter forms

## 🧪 Testing Missions

### Unit Tests
```bash
# Test mission logic
python -m pytest tests/unit/test_sample_collection_mission.py -v
```

### Integration Tests
```bash
# Test with ROS2 system
python -m pytest tests/integration/test_mission_execution.py -v
```

### Simulation Testing
```bash
# Test in Gazebo
./start.py dev simulation
# Then execute mission via dashboard
```

## 📊 Mission Monitoring

### Key Metrics
- **Success Rate**: Mission completion percentage
- **Execution Time**: Time to complete objectives
- **Error Recovery**: Failed attempt handling
- **Resource Usage**: CPU, memory, network during execution

### Logging
Missions log to ROS2 with structured data:
```bash
# Monitor mission progress
ros2 topic echo /mission/progress

# Check mission status
ros2 topic echo /mission/status
```

## 🛠️ Mission Development Tools

### Mission Validator
```bash
python scripts/validate_config.py --mission sample_collection
```

### Mission Simulator
```bash
python tools/mission_simulator.py --mission sample_collection --dry-run
```

### Performance Profiler
```bash
python tools/performance/profiler.py --mission sample_collection
```

## 🔍 Troubleshooting

### Common Issues
- **Mission not starting**: Check ROS2 action server is running
- **Hardware not responding**: Verify actuator interfaces
- **Navigation failing**: Check GPS and IMU data quality
- **Timeouts occurring**: Adjust timing parameters in config

### Debug Mode
```python
# Enable verbose logging
export ROS_LOG_LEVEL=debug
ros2 launch autonomy mission_system.launch.py
```

## 📚 Related Documentation

- **Behavior Trees**: `src/autonomy/bt/README.md`
- **ROS2 Actions**: `docs/api/ros2_actions.rst`
- **Hardware Interfaces**: `src/autonomy/control/README.md`
- **Testing Guide**: `docs/testing/mission_testing.rst`