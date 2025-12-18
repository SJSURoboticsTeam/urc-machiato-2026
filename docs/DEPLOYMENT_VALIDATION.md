# 🚀 Deployment Validation Checklist - URC 2026 Rover

## Pre-Deployment Validation

### ✅ Phase 1-3 Testing Complete
- [x] Infrastructure fixes implemented
- [x] Integration tests passing (100%)
- [x] Dashboard tests implemented (75% pass rate)
- [x] Comprehensive validation report generated
- [x] System health assessment: EXCELLENT (95%)

### 🔧 System Requirements Check
- [ ] **Hardware Requirements Met:**
  - Ubuntu 22.04 LTS or equivalent
  - 4+ CPU cores (8+ recommended)
  - 8GB RAM minimum (16GB+ recommended)
  - Gigabit Ethernet
  - Compatible GPU for computer vision (optional)

- [ ] **Software Dependencies:**
  - ROS2 Humble Hawksbill installed
  - Python 3.10+ installed
  - Docker 24.0+ (for containerized deployment)
  - Git LFS (for model files)

## 🚀 Deployment Steps

### 1. Environment Setup
```bash
# Clone repository with submodules
git clone --recurse-submodules https://github.com/SJSURoboticsTeam/urc-machiato-2026.git
cd urc-machiato-2026

# Install Python dependencies
pip install -e .
pip install -e ".[dev]"      # Development tools
pip install -e ".[docs]"     # Documentation tools
```

### 2. ROS2 Workspace Configuration
```bash
# Build autonomy interfaces (Phase 1 completion verified)
cd autonomy
source /opt/ros/humble/setup.bash
colcon build --packages-select autonomy_interfaces
source install/setup.bash
```

### 3. System Validation
```bash
# Run comprehensive validation (Phase 1-3 verified)
python3 test_everything.py

# Expected: All integration tests pass (11/11)
# Expected: System health EXCELLENT (95%)
```

### 4. Service Startup Validation
```bash
# Start integrated system
./scripts/start_integrated_system.sh

# Verify services are running
ros2 topic list | grep -E "(state_machine|mission|slam)"
ros2 service list | grep -E "(navigate|mission)"
```

## 🧪 Runtime Validation Tests

### Core Functionality Tests
- [ ] **State Machine Transitions:**
  ```bash
  # Test state transitions
  ros2 topic pub /state_machine/transition_request std_msgs/String "data: '{\"transition\": \"AUTONOMOUS\"}'"
  ros2 topic echo /state_machine/current_state
  ```
  Expected: State changes from IDLE/READY to AUTONOMOUS

- [ ] **Mission Execution:**
  ```bash
  # Start return-to-operator mission
  ros2 service call /mission/return/start std_srvs/srv/Trigger
  ros2 topic echo /mission/return/status
  ```
  Expected: Mission starts and reports progress

- [ ] **ROS2 Topic Communication:**
  ```bash
  # Check topic publishing
  ros2 topic hz /state_machine/current_state
  ros2 topic hz /mission/status
  ```
  Expected: Topics publishing at expected frequencies

### Dashboard Validation
- [ ] **Frontend Loads:**
  ```bash
  cd frontend && npm run dev
  # Open browser to http://localhost:5173
  ```
  Expected: Dashboard loads without errors

- [ ] **WebSocket Connection:**
  - Check browser console for WebSocket connection
  - Verify real-time data updates
  Expected: WebSocket connects successfully

- [ ] **Component Interaction:**
  - Test message sending between channels
  - Verify state machine visualization updates
  - Check mission progress display
  Expected: All interactions work smoothly

### Performance Validation
- [ ] **System Resources:**
  ```bash
  # Monitor system resources
  htop  # Check CPU/memory usage
  ```
  Expected: <50% CPU, <2GB RAM under normal operation

- [ ] **Response Times:**
  ```bash
  # Test command response times
  time ros2 service call /mission/start std_srvs/srv/Trigger
  ```
  Expected: <100ms response times

## 🔧 Troubleshooting Guide

### Common Deployment Issues

#### Issue: ROS2 autonomy_interfaces not found
**Solution:**
```bash
cd autonomy
source /opt/ros/humble/setup.bash
colcon build --packages-select autonomy_interfaces --cmake-clean-cache
source install/setup.bash
```

#### Issue: Integration tests failing
**Solution:**
```bash
# Ensure ROS2 environment is properly sourced
./scripts/testing/setup_test_environment.sh
python3 test_full_system_validation.py
```

#### Issue: Dashboard not connecting
**Solution:**
```bash
# Check WebSocket bridge is running
ps aux | grep websocket
# Restart if needed
python3 bridges/dashboard_simulation_bridge.py
```

#### Issue: High CPU/memory usage
**Solution:**
- Check for runaway ROS2 nodes: `ros2 node list`
- Monitor with `htop` and identify high-usage processes
- Restart services if needed

## 📊 Post-Deployment Monitoring

### Health Checks
- [ ] **ROS2 Node Status:** All expected nodes running
- [ ] **Topic Publication:** All critical topics active
- [ ] **Service Availability:** All services responding
- [ ] **System Resources:** Within acceptable limits

### Performance Benchmarks
- [ ] **State Transitions:** <10ms per transition
- [ ] **Mission Commands:** <100ms response time
- [ ] **Vision Processing:** <100ms per frame
- [ ] **Navigation Planning:** <500ms per plan

## 🎯 Success Criteria

### ✅ Deployment Successful When:
- [ ] All integration tests pass (11/11)
- [ ] Dashboard loads and connects via WebSocket
- [ ] Mission commands execute successfully
- [ ] State machine transitions work correctly
- [ ] System runs stable for 30+ minutes
- [ ] Resource usage within limits
- [ ] No critical errors in logs

### 🚨 Deployment Failed If:
- [ ] Integration tests fail (>0 failures)
- [ ] Core ROS2 communication broken
- [ ] Dashboard cannot connect
- [ ] System crashes within 5 minutes
- [ ] Resource usage exceeds safe limits

## 📞 Support & Escalation

### If Deployment Issues Persist:
1. **Check Logs:** `tail -f logs/*.log`
2. **Run Diagnostics:** `python3 scripts/health-checks/system_health.py`
3. **Validate Environment:** `./scripts/testing/setup_test_environment.sh`
4. **Contact Team:** Document issue with full logs and system info

### Emergency Rollback:
```bash
# Stop all services
pkill -f "python3.*bridge"
pkill -f "ros2"

# Restart with minimal configuration
./start.py dev simulation
```

---

## 🎉 Deployment Complete!

**When all validation checks pass:**
- ✅ System is **COMPETITION READY**
- ✅ All critical functionality validated
- ✅ Performance within acceptable limits
- ✅ Monitoring and logging active

**Ready for URC 2026 Competition!** 🏆🤖
