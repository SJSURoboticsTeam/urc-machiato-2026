.. _arm_guide:

============
ARM Team Guide
============

Welcome to the ARM Team! You own the rover's ability to **physically interact with the world**. From collecting samples to typing on keyboards, the arm is the rover's "hand" - and you're responsible for making sure it works perfectly every time.

.. image:: architecture/diagrams/low/09_hardware_interface_architecture.png
   :alt: Hardware Interface Architecture (ARM Focus)
   :align: center
   :width: 80%

What You Own
============

**Arm Control & Manipulation Systems:**
- Autonomous keyboard typing (URC Equipment Service challenge)
- Sample collection and manipulation
- Arm motion planning and control
- Computer vision for typing alignment
- Hardware interface to arm motors/servos
- Calibration and registration systems

**Key Files:**
- ``src/autonomy/perception/autonomous_typing/`` - ARM control system
- ``src/autonomy/control/hardware_interface/`` - Hardware interfaces
- ``missions/autonomous_keyboard_mission.py`` - Keyboard mission
- ``missions/sample_collection_mission.py`` - Sample collection

Your Day-to-Day
===============

**ARM Health Checks:**
```bash
# Check arm status
ros2 topic echo /arm/status

# Monitor joint positions
ros2 topic echo /arm/joint_states

# Check typing alignment
ros2 topic echo /typing/alignment_status

# Monitor sample collection
ros2 topic echo /sample_collection/status
```

**Common Issues You Fix:**
- Arm calibration drift over time
- Typing accuracy problems (missed keys)
- Sample collection failures (can't grasp objects)
- Hardware interface communication errors
- Motion planning failures around obstacles

Week 1: Getting Started
=======================

**Day 1: Understand ARM Concepts**
1. Read the Big Picture guide (focus on perception and control layers)
2. Study forward/inverse kinematics basics
3. Run ``./start.py dev simulation`` to see arm in Gazebo
4. Execute ``ros2 topic echo /arm/joint_states`` to see arm telemetry

**Day 2: Learn ARM Monitoring**
```bash
# Monitor arm systems
ros2 topic echo /arm/status
ros2 topic echo /arm/joint_states
ros2 topic echo /arm/effort

# Check typing systems
ros2 topic echo /typing/camera_alignment
ros2 topic echo /typing/keyboard_detection

# Monitor sample collection
ros2 topic echo /sample_collection/gripper_status
ros2 topic echo /sample_collection/sample_detected
```

**Day 3: Test ARM Systems**
```bash
# Run ARM unit tests
python -m pytest tests/unit/test_arm_control.py -v

# Test vision-ARM integration
python -m pytest tests/integration/test_vision_control_integration.py -v

# Test autonomous typing
python -m pytest tests/integration/test_autonomous_typing.py -v
```

**Day 4-5: Your First ARM Fix**
1. Calibrate the arm system (see calibration section below)
2. Test a simple arm movement
3. Identify and fix an alignment or control issue
4. Document your calibration procedure

ARM Debugging Tools
===================

**ROS2 ARM Diagnostics:**
```bash
# Check arm node status
ros2 node list | grep arm
ros2 node info /arm_controller

# Monitor joint states and efforts
ros2 topic echo /arm/joint_states
ros2 topic echo /arm/effort

# Check transform tree
ros2 run tf2_tools view_frames.py
ros2 run tf2_tools echo.py base_link tool_link
```

**Typing System Debugging:**
```bash
# Monitor keyboard detection
ros2 topic echo /typing/keyboard_pose
ros2 topic echo /typing/key_positions

# Check alignment accuracy
ros2 topic echo /typing/alignment_error
ros2 topic echo /typing/calibration_status

# Debug typing execution
ros2 topic echo /typing/typing_progress
ros2 topic echo /typing/current_key
```

**Sample Collection Debugging:**
```bash
# Monitor gripper status
ros2 topic echo /sample_collection/gripper_state
ros2 topic echo /sample_collection/force_sensor

# Check sample detection
ros2 topic echo /sample_collection/sample_pose
ros2 topic echo /sample_collection/grasp_planning

# Debug collection sequence
ros2 topic echo /sample_collection/sequence_status
```

Key ARM Components
==================

**1. ARM Controller** (``src/autonomy/perception/autonomous_typing/arm_controller.py``)
- High-level arm motion coordination
- Trajectory planning and execution
- Safety monitoring and emergency stops
- Integration with vision systems

**2. Typing Executor** (``src/autonomy/perception/autonomous_typing/typing_executor.py``)
- Keyboard interaction logic
- Key press sequencing and timing
- Error recovery for missed keys
- Integration with computer vision

**3. Keyboard Localization** (``src/autonomy/perception/autonomous_typing/keyboard_localization.py``)
- AR marker detection for keyboard finding
- Keyboard layout recognition
- Coordinate system transformations
- Confidence scoring for detection quality

**4. ARUco Detection** (``src/autonomy/perception/autonomous_typing/aruco_detection/``)
- Fiducial marker detection and tracking
- Pose estimation from markers
- Multi-marker configurations
- Robustness to lighting/angle changes

**5. Hardware Interface** (``src/autonomy/control/hardware_interface/``)
- Low-level motor control commands
- Joint position/velocity feedback
- Safety limits and current monitoring
- CAN bus communication with motor controllers

**6. Sample Collection** (``missions/sample_collection_mission.py``)
- Sample detection and approach planning
- Gripper control and grasping logic
- Sample handling and storage
- Mission-specific execution logic

Common ARM Issues & Solutions
=============================

**Calibration Drift:**
```python
# Check calibration quality
calibration_error = measure_calibration_error()
if calibration_error > MAX_CALIBRATION_ERROR:
    log_calibration_drift()
    trigger_recalibration()

# Perform hand-eye calibration
def recalibrate_arm():
    collect_calibration_data()
    compute_transforms()
    update_configuration()
    validate_calibration()
```

**Typing Accuracy Problems:**
```python
# Check alignment quality
alignment_error = get_alignment_error()
if alignment_error > TYPING_ACCURACY_THRESHOLD:
    refine_keyboard_pose()
    adjust_typing_trajectory()
    slow_down_movement()

# Handle missed keys
if key_press_failed():
    reposition_arm()
    retry_key_press()
    update_keyboard_model()
```

**Gripper Failures:**
```python
# Check gripper force feedback
force_reading = get_gripper_force()
if force_reading > MAX_GRIP_FORCE:
    emergency_release()
    log_gripper_stuck()

# Validate grasping
if not object_grasped():
    adjust_gripper_position()
    retry_grasp()
    check_gripper_health()
```

**Motion Planning Failures:**
```python
# Check for collisions
if trajectory_has_collisions():
    replan_trajectory()
    reduce_velocity()
    use_conservative_path()

# Handle singularities
if near_singularity():
    switch_to_joint_space_planning()
    avoid_problematic_configurations()
```

ARM Testing Strategy
====================

**Unit Tests:**
```bash
# Test individual ARM components
pytest tests/unit/test_arm_controller.py
pytest tests/unit/test_typing_executor.py
pytest tests/unit/test_keyboard_localization.py
pytest tests/unit/test_aruco_detection.py
```

**Integration Tests:**
```bash
# Test ARM system integration
pytest tests/integration/test_arm_control.py
pytest tests/integration/test_vision_control_integration.py
pytest tests/integration/test_autonomous_typing_integration.py
pytest tests/integration/test_sample_collection_integration.py
```

**Hardware Tests:**
```bash
# Test on real arm hardware
pytest tests/hardware/test_arm_hardware.py
pytest tests/hardware/test_typing_hardware.py
pytest tests/hardware/test_gripper_hardware.py
```

**Calibration Tests:**
```bash
# Test calibration procedures
pytest tests/hardware/test_arm_calibration.py
pytest tests/hardware/test_camera_calibration.py
pytest tests/hardware/test_hand_eye_calibration.py
```

ARM Performance Metrics
=======================

**Key Metrics to Monitor:**
- **Typing Accuracy**: >95% key press success rate
- **Calibration Stability**: <1mm drift over 1 hour operation
- **Motion Planning Time**: <500ms for typical movements
- **Gripper Success Rate**: >90% for sample collection
- **Position Accuracy**: <2mm RMS at tool center point

**Monitoring Commands:**
```bash
# ARM performance dashboard
watch -n 1 "ros2 topic echo /arm/metrics"

# Typing accuracy monitoring
ros2 topic echo /typing/accuracy_metrics

# Calibration health
ros2 topic echo /calibration/health
```

ARM Calibration Procedures
==========================

**Hand-Eye Calibration:**
```python
def perform_hand_eye_calibration():
    # Collect calibration data
    poses = collect_calibration_poses()
    images = collect_calibration_images()

    # Solve hand-eye problem
    transform = solve_hand_eye_calibration(poses, images)

    # Validate calibration
    error = validate_calibration(transform)
    if error < MAX_CALIBRATION_ERROR:
        save_calibration(transform)
    else:
        log_calibration_failure()
```

**Keyboard Registration:**
```python
def register_keyboard():
    # Detect keyboard markers
    markers = detect_keyboard_markers()

    # Estimate keyboard pose
    keyboard_pose = estimate_keyboard_pose(markers)

    # Validate registration
    accuracy = validate_keyboard_registration(keyboard_pose)
    if accuracy > MIN_REGISTRATION_ACCURACY:
        save_keyboard_registration(keyboard_pose)
```

**Joint Zero Calibration:**
```python
def calibrate_joint_zeros():
    # Move to known positions
    move_to_calibration_positions()

    # Record encoder values
    zero_positions = record_encoder_values()

    # Update joint limits
    update_joint_limits(zero_positions)

    # Validate calibration
    validate_joint_calibration()
```

ARM Safety Considerations
=========================

**Critical Safety Systems:**
- Emergency stop on arm collision
- Force limiting to prevent damage
- Software position limits
- Current monitoring for motor stalls
- Human presence detection

**Safety-Triggered Behaviors:**
```python
# ARM safety monitor
def check_arm_safety():
    if joint_current > MAX_SAFE_CURRENT:
        trigger_emergency_stop("Motor overload")

    if tool_force > MAX_SAFE_FORCE:
        trigger_emergency_stop("Excessive contact force")

    if human_detected_near_arm():
        pause_arm_movement()
        require_manual_confirmation()
```

Development Workflow
====================

**Making ARM Changes:**
1. **Calibrate before testing**: ARM calibration affects everything
2. **Test in simulation first**: Protect expensive hardware
3. **Monitor forces carefully**: Don't break things on hardware
4. **Update calibration procedures**: If changing kinematics
5. **Test with real objects**: Simulation ≠ reality

**ARM Code Review Checklist:**
- [ ] Calibration procedures updated
- [ ] Safety limits verified
- [ ] Force monitoring implemented
- [ ] Error handling for hardware failures
- [ ] Simulation testing completed
- [ ] Hardware validation planned
- [ ] Documentation updated

Getting Help
============

**ARM-Specific Resources:**
- Robot kinematics textbooks
- ROS2 MoveIt documentation
- Computer vision calibration guides
- AR marker detection libraries

**Team Resources:**
- ARM calibration procedures in ``docs/calibration/``
- Hardware interface documentation
- Test objects and fixtures
- Emergency arm procedures

**When You're Stuck:**
1. Recalibrate the system: See calibration procedures above
2. Check transforms: ``ros2 run tf2_tools view_frames.py``
3. Monitor joint states: ``ros2 topic echo /arm/joint_states``
4. Test in isolation: ``python tools/test_arm_isolation.py``
5. Review logs: ``tail -f log/arm_*.log``
6. Ask in #arm-team channel

Remember: The arm is the rover's most precise and expensive component. Take your time, calibrate carefully, and test thoroughly. A well-calibrated arm makes the difference between mission success and failure!
