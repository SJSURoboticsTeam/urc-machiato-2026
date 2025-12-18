# Integrated Testing Dashboard

## Overview

The **Integrated Testing Dashboard** provides comprehensive visual testing and monitoring for the URC 2026 rover system. It enables real-time verification of system integration, communication flows, and component functionality.

## Key Features

### PRIMARY: Communication & State Verification
- **WebSocket ↔ ROS2 ↔ CAN Communication Flow**
  - Real-time visualization of message routing
  - Latency measurement across all channels
  - Error rate tracking
  - Message count and rate monitoring

- **State Machine Monitoring**
  - Current state display
  - State transition history (last 10 transitions)
  - Transition timing analysis
  - Invalid transition detection

### SECONDARY: Mock Data Streaming
- **CAN Bus Data Stream**
  - IMU sensor data (accelerometer, gyroscope)
  - GPS position and heading
  - Battery voltage and current
  - Motor encoder positions

- **Sensor Simulation**
  - Configurable update rates
  - Realistic noise injection
  - Fault injection capabilities

### TERTIARY: Simulation Monitoring
- **Drive System Simulation**
- **Robotic Arm Simulation**
- **Science Payload Simulation**

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      FRONTEND (React)                        │
│  ┌──────────────────────────────────────────────────────┐  │
│  │     IntegratedTestingDashboard.jsx                   │  │
│  │  - Test Controls Sidebar                             │  │
│  │  - Communication Flow Visualization                  │  │
│  │  - State Machine Monitor                             │  │
│  │  - Topic Monitor                                     │  │
│  │  - CAN Data Stream Display                           │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            ↕ WebSocket (port 8766)
┌─────────────────────────────────────────────────────────────┐
│              BACKEND (Python WebSocket Server)               │
│  ┌──────────────────────────────────────────────────────┐  │
│  │     test_dashboard_backend.py                        │  │
│  │  - Test Execution Engine                             │  │
│  │  - Communication Metrics Collector                   │  │
│  │  - ROS2 Integration                                  │  │
│  │  - CAN Simulator Integration                         │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            ↕ ROS2 Topics
┌─────────────────────────────────────────────────────────────┐
│                       ROS2 SYSTEM                            │
│  - State Machine Director                                   │
│  - SLAM Nodes                                               │
│  - Navigation Service                                       │
│  - Mock Publishers                                          │
└─────────────────────────────────────────────────────────────┘
                            ↕ CAN Bus (Mock)
┌─────────────────────────────────────────────────────────────┐
│                    CAN BUS SIMULATOR                         │
│  - Motor Controllers                                        │
│  - Sensor Interfaces (IMU, GPS, Encoders)                  │
│  - Battery Monitoring                                       │
└─────────────────────────────────────────────────────────────┘
```

## Getting Started

### Prerequisites

1. **Frontend Dependencies**
   ```bash
   cd /home/ubuntu/urc-machiato-2026/frontend
   npm install
   ```

2. **Backend Dependencies**
   ```bash
   pip install websockets dataclasses-json
   # Optional: ROS2 for full integration
   ```

### Running the Dashboard

#### Step 1: Start the Backend Service

```bash
cd /home/ubuntu/urc-machiato-2026
python3 scripts/testing/test_dashboard_backend.py
```

This will start the WebSocket server on `ws://localhost:8766` and begin streaming test data.

#### Step 2: Start the Frontend

```bash
cd /home/ubuntu/urc-machiato-2026/frontend
npm run dev
```

Navigate to `http://localhost:5173` and click the **Testing** tab.

#### Step 3: Switch to Integrated Dashboard

In the Testing tab, click the **Integrated Dashboard** button to access the new visualization.

### Running With Full ROS2 Integration

For full system integration with ROS2:

```bash
# Terminal 1: Start ROS2 system
cd /home/ubuntu/urc-machiato-2026
source /opt/ros/humble/setup.bash
ros2 launch launch/integrated_system.launch.py

# Terminal 2: Start backend
python3 scripts/testing/test_dashboard_backend.py

# Terminal 3: Start frontend
cd frontend && npm run dev
```

## Using the Dashboard

### Test Controls Sidebar

Located on the left side, this panel allows you to:

1. **Run Tests**
   - Click individual test checkboxes to enable/disable
   - Use "Run All High Priority" to execute communication and state tests
   - Use "Stop All Tests" to halt running tests
   - Use "Reset" to clear test results

2. **Test Categories**
   - **Communication (High Priority)**: Verify WebSocket/ROS2/CAN integration
   - **State Machine (High Priority)**: Test state transitions and recovery
   - **Mock Data (Medium Priority)**: Stream and validate mock sensor data
   - **Simulation (Low Priority)**: Run system simulations

### Communication Flow Visualization

The central panel shows real-time communication between systems:

- **Green nodes**: Active and connected
- **Yellow nodes**: Mock/simulated data
- **Gray nodes**: Inactive
- **Animated lines**: Active data flow
- **Metrics below**: Message counts, latency, error rates

### State Machine Monitor

Displays:
- **Current State**: Large display of active state
- **State History**: Last 10 state transitions with durations
- **Transition Tests**: Validate state changes work correctly

### Topic Monitor

Shows all active ROS2 topics:
- **Topic Name**: Full ROS2 topic path
- **Type**: ROS2 message type
- **Rate**: Publishing frequency in Hz
- **Source**: Origin (websocket, ros2, or can)
- **Status Indicator**: Green pulse = active, Gray = inactive

Click any topic to view detailed message contents.

### CAN Data Stream

When "CAN Bus Stream" test is enabled:
- Real-time display of mock CAN data
- IMU accelerometer readings
- GPS latitude/longitude
- Battery voltage/current
- Motor encoder positions

**Note**: All CAN data is clearly marked as MOCK/SIMULATED.

### Test Results

Bottom panel shows:
- Test execution status (running/passed/failed)
- Execution time
- Pass/fail indicators
- Detailed messages for failures

## Test Definitions

### Communication Tests

| Test ID | Name | Duration | Description |
|---------|------|----------|-------------|
| `comm-ws-ros2` | WebSocket ↔ ROS2 | 2s | Verify message routing between WebSocket and ROS2 |
| `comm-ros2-can` | ROS2 ↔ CAN | 2s | Verify message routing between ROS2 and CAN bus |
| `comm-bidirectional` | Bidirectional Flow | 3s | Verify bidirectional message flow across all systems |
| `comm-latency` | Latency Check | 5s | Measure end-to-end latency across communication channels |

### State Machine Tests

| Test ID | Name | Duration | Description |
|---------|------|----------|-------------|
| `state-transitions` | State Transitions | 4s | Test all valid state transitions |
| `state-recovery` | Error Recovery | 3s | Test error state recovery mechanisms |
| `state-timing` | Transition Timing | 6s | Measure state transition timing and delays |

### Mock Data Tests

| Test ID | Name | Duration | Description |
|---------|------|----------|-------------|
| `mock-can-stream` | CAN Bus Stream | 10s | Continuously stream mock CAN bus data |
| `mock-sensor-data` | Sensor Data | 5s | Generate and validate mock sensor data |
| `mock-fault-injection` | Fault Injection | 8s | Inject faults into mock data streams |

### Simulation Tests

| Test ID | Name | Duration | Description |
|---------|------|----------|-------------|
| `sim-drive` | Drive System | 15s | Run drive system simulation |
| `sim-arm` | Arm System | 12s | Run robotic arm simulation |
| `sim-science` | Science Payload | 10s | Run science payload simulation |

## WebSocket API

The backend exposes the following WebSocket API at `ws://localhost:8766`:

### Client → Server Messages

```json
{
  "type": "start_test",
  "test_id": "comm-ws-ros2"
}
```

```json
{
  "type": "stop_test",
  "test_id": "comm-ws-ros2"
}
```

```json
{
  "type": "get_state"
}
```

### Server → Client Messages

#### Full State Update
```json
{
  "type": "full_state",
  "timestamp": 1234567890.123,
  "metrics": {
    "websocket": {
      "message_count": 100,
      "message_rate": 2.5,
      "latency": 15.0,
      "errors": 0,
      "last_activity": 1234567890.123
    },
    "ros2": { ... },
    "can": { ... }
  },
  "state_history": [...],
  "active_topics": [...],
  "test_statuses": {...},
  "can_data": {...}
}
```

#### Test Update
```json
{
  "type": "test_update",
  "timestamp": 1234567890.123,
  "result": {
    "test_id": "comm-ws-ros2",
    "status": "passed",
    "start_time": 1234567880.0,
    "end_time": 1234567882.0,
    "message": "Test completed successfully",
    "metrics": {
      "duration": 2.0,
      "assertions_passed": 15,
      "assertions_total": 15
    }
  }
}
```

#### Metrics Update
```json
{
  "type": "metrics_update",
  "timestamp": 1234567890.123,
  "metrics": {...},
  "can_data": {...}
}
```

## Customization

### Adding New Tests

Edit `scripts/testing/test_dashboard_backend.py` and add to `test_definitions`:

```python
'my-test-id': TestDefinition(
    id='my-test-id',
    name='My Test Name',
    category='communication',  # or 'state_machine', 'mock_data', 'simulation'
    priority=TestPriority.HIGH,
    duration_estimate=3.0,
    description='Description of what this test does'
)
```

Then add the test to the frontend sidebar in `IntegratedTestingDashboard.jsx`.

### Modifying Communication Metrics

Edit the `update_metrics_loop()` method in `TestDashboardBackend` class to:
- Connect to actual ROS2 topics
- Read from real CAN bus
- Calculate real latency measurements

### Extending Visualizations

The dashboard uses Lucide React icons and Tailwind CSS. To add new visualizations:

1. Create a new component in the main render area
2. Subscribe to backend WebSocket messages
3. Update state based on incoming data
4. Use existing styling patterns for consistency

## Troubleshooting

### Backend Won't Start

**Error**: `Address already in use`
- **Solution**: Another service is using port 8766. Kill it or change the port in the backend script.

```bash
lsof -i :8766
kill <PID>
```

### Frontend Can't Connect

**Error**: `WebSocket connection failed`
- **Solution**: Ensure backend is running and accessible
- Check browser console for detailed error messages
- Verify firewall isn't blocking port 8766

### No Test Results

**Problem**: Tests don't execute or show results
- **Solution**: Check browser console for JavaScript errors
- Verify WebSocket connection is established (green indicator)
- Check backend logs for errors

### ROS2 Integration Issues

**Problem**: Topics don't appear or show as inactive
- **Solution**: Ensure ROS2 is running: `ros2 topic list`
- Verify `integrated_system.launch.py` is running
- Check ROS2_DOMAIN_ID matches: `echo $ROS2_DOMAIN_ID`

## Performance Considerations

- **Update Rate**: Backend sends metrics every 1 second
- **Message History**: Last 10 state transitions kept in memory
- **Test History**: All test results kept until reset
- **WebSocket**: Uses minimal bandwidth (~1-2 KB/s when idle)

## Future Enhancements

- [ ] Integration with pytest for actual test execution
- [ ] Historical test result database
- [ ] Performance regression detection
- [ ] Export test reports to PDF/CSV
- [ ] Real-time log streaming
- [ ] Video recording of test sessions
- [ ] Automated test scheduling
- [ ] Multi-rover testing support

## Support

For issues or questions:
1. Check this README
2. Review backend logs: `scripts/testing/test_dashboard_backend.py`
3. Check frontend console (F12 in browser)
4. Review integration test documentation in `/tests/`

---

**Last Updated**: December 2025
**Author**: URC Machiato 2026 Team
