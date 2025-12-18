# Manual Testing Guide

This guide provides step-by-step instructions for testing the autonomy-teleoperation integration using simulation.

## 🚀 Quick Start (5 minutes)

For the fastest way to test the integration:

```bash
# From workspace root
./quick_test.sh
```

This will:

- ✅ Set up ROS2 environment
- ✅ Start mock teleoperation data
- ✅ Launch autonomy mission executor
- ✅ Verify all components are working
- ✅ Provide interactive testing commands

## 📋 Comprehensive Testing

For detailed testing with full control:

```bash
# From workspace root
./test_manual_integration.sh
```

This provides:

- 🔍 Automated verification tests
- 🎮 Interactive testing menu
- 📊 Detailed status reporting
- 🧹 Automatic cleanup

## 🧪 Testing Components

### Mock Teleoperation Data

**Purpose:** Simulates real teleoperation system without hardware

- Publishes realistic motor, velocity, temperature, and status data
- Runs at 10Hz with configurable noise
- Located: `test_teleoperation_integration.py`

### Autonomy Mission Executor

**Purpose:** Tests autonomy data processing and decision making

- Subscribes to all teleoperation topics
- Validates and filters incoming data
- Makes autonomous decisions based on system state
- Located: `missions/mission_executor.py`

### Configuration System

**Purpose:** Tests configurable behavior thresholds

- Thermal limits, battery thresholds, motor constraints
- Loaded from `config/production.yaml`
- Hot-reloadable for testing different scenarios

## 🎮 Testing Scenarios

### Basic Integration Test

```bash
./quick_test.sh
# Then run these commands:
ros2 topic echo /teleoperation/joint_states  # Monitor motor data
ros2 topic hz /teleoperation/chassis_velocity  # Check publishing rate
ros2 node list  # Verify autonomy is running
```

### Thermal Stress Test

```bash
./quick_test.sh
# Monitor temperature handling:
ros2 topic echo /teleoperation/motor_temperatures
# Watch autonomy logs for thermal throttling messages
```

### Battery Management Test

```bash
./quick_test.sh
# Monitor battery handling:
ros2 topic echo /teleoperation/system_status
# Watch for battery conservation messages
```

## 🔍 Verification Commands

### Check System Status

```bash
# View all ROS2 topics
ros2 topic list

# Check node status
ros2 node list

# View computation graph
ros2 run rqt_graph rqt_graph
```

### Monitor Data Flow

```bash
# Monitor motor data
ros2 topic echo /teleoperation/joint_states

# Check publishing rates
ros2 topic hz /teleoperation/joint_states

# Monitor autonomy decisions
# (Check ROS2 logging output for autonomy messages)
```

### Test Autonomy Responses

```bash
# Send mission commands
ros2 topic pub /mission/commands std_msgs/String "data: 'start_mission'"

# Monitor mission status
ros2 topic echo /mission/status

# Check mission progress
ros2 topic echo /mission/progress
```

## 🛠️ Troubleshooting

### Common Issues

**"ROS2 not found"**

```bash
# Ensure ROS2 is installed and sourced
source /opt/ros/humble/setup.bash
```

**"Mock teleoperation won't start"**

```bash
# Check Python dependencies
pip install rclpy

# Verify script permissions
chmod +x test_teleoperation_integration.py
```

**"Autonomy node not starting"**

```bash
# Check if mission_executor.py exists and compiles
python3 -m py_compile missions/mission_executor.py

# Verify ROS2 environment
ros2 --version
```

**"No teleoperation topics visible"**

```bash
# Check if mock publisher is running
ps aux | grep test_teleoperation

# Restart mock publisher
pkill -f test_teleoperation
python3 test_teleoperation_integration.py &
```

### Debug Mode

```bash
# Enable verbose logging
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# Run tests with debug output
./quick_test.sh  # Will show detailed logs
```

## 📊 Expected Results

### Successful Test Indicators

- ✅ **4 teleoperation topics active**: `joint_states`, `chassis_velocity`, `motor_temperatures`, `system_status`
- ✅ **Publishing rate ~10Hz**: Topics update at expected frequency
- ✅ **Autonomy node running**: `simple_mission_executor` visible in node list
- ✅ **No error messages**: Clean ROS2 logging output
- ✅ **Data validation working**: No "invalid data" warnings

### Performance Benchmarks

- **Topic latency**: < 100ms from publish to subscribe
- **CPU usage**: < 20% for mock testing (single core)
- **Memory usage**: < 200MB total for test environment
- **Data validation**: 95%+ of messages pass validation

## 🧹 Cleanup

### Automatic Cleanup

Both testing scripts automatically clean up on exit:

- Stop mock teleoperation processes
- Terminate autonomy nodes
- Remove temporary files

### Manual Cleanup

```bash
# Kill all test processes
pkill -f "test_teleoperation_integration"
pkill -f "mission_executor"
pkill -f "ros2"

# Clean up temporary files
rm -f .mock_pid .autonomy_pid
```

## 📈 Next Steps After Testing

### With Real Teleoperation Data

1. Replace mock publisher with real teleoperation ROS2 publishing
2. Update configuration thresholds for real hardware
3. Test with actual CAN/serial data instead of simulated
4. Validate data quality and filtering with real-world conditions

### Performance Optimization

1. Profile data processing latency
2. Optimize filtering algorithms for real-time performance
3. Tune configuration parameters for production use
4. Add performance monitoring and alerting

### Production Deployment

1. Integrate with full autonomy system (SLAM, navigation, etc.)
2. Add health monitoring and automatic recovery
3. Implement production logging and telemetry
4. Create automated integration tests for CI/CD

---

## 📞 Support

For testing issues:

1. Check this troubleshooting guide
2. Verify ROS2 environment setup
3. Run individual components separately to isolate issues
4. Check ROS2 logging output for detailed error messages

**The testing infrastructure is designed to work reliably and provide clear feedback on integration status.**
