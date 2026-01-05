.. _slam_nav_guide:

===================
SLAM/NAV Team Guide
===================

Welcome to the SLAM/NAV Team! You own the rover's ability to know **where it is** and **how to get where it's going**. Without good navigation, the rover is just an expensive rock. With great navigation, it's an autonomous Mars explorer.

.. image:: architecture/diagrams/high/04_system_component_architecture.png
   :alt: System Component Architecture (Navigation Focus)
   :align: center
   :width: 80%

What You Own
============

**Navigation & Mapping Systems:**
- Real-time localization and mapping (SLAM)
- Path planning and obstacle avoidance
- GPS/GNSS processing and fusion
- Motion control and trajectory execution
- Terrain intelligence and traversability analysis

**Key Files:**
- ``src/autonomy/core/navigation/`` - Navigation stack
- ``src/autonomy/perception/slam/`` - SLAM implementation
- ``src/autonomy/core/terrain_intelligence/`` - Terrain analysis
- ``config/rover.yaml`` - Navigation parameters

Your Day-to-Day
===============

**Navigation Health Checks:**
```bash
# Check navigation status
ros2 topic echo /navigation/status

# Monitor position accuracy
ros2 topic echo /navigation/odom

# View current path
ros2 topic echo /navigation/current_path

# Check SLAM map quality
ros2 topic echo /slam/map_metadata
```

**Common Issues You Fix:**
- Localization drift in GPS-denied environments
- Path planning failures around obstacles
- Motion control oscillations or instability
- Terrain classification errors
- SLAM loop closure failures

Week 1: Getting Started
=======================

**Day 1: Understand Navigation Concepts**
1. Read the Big Picture guide (focus on autonomy layer)
2. Study GPS vs SLAM vs Visual Odometry
3. Run ``./start.py dev simulation`` to see navigation in action
4. Execute ``ros2 topic echo /navigation/goal`` to see navigation commands

**Day 2: Learn Navigation Monitoring**
```bash
# Monitor navigation stack
ros2 topic echo /navigation/status
ros2 topic echo /navigation/odom
ros2 topic echo /navigation/cmd_vel

# Check SLAM performance
ros2 topic echo /slam/pose
ros2 topic echo /slam/map

# Monitor GPS quality
ros2 topic echo /gps/fix
ros2 topic echo /gps/quality
```

**Day 3: Test Navigation Systems**
```bash
# Run navigation unit tests
python -m pytest tests/unit/navigation/ -v

# Test SLAM integration
python -m pytest tests/integration/test_navigation_comprehensive.py -v

# Test in Gazebo simulation
./start.py dev simulation
# Then send navigation goals via dashboard
```

**Day 4-5: Your First Navigation Fix**
1. Create a simple navigation test scenario
2. Identify and fix a navigation issue
3. Test your changes in simulation
4. Document your findings

Navigation Debugging Tools
==========================

**ROS2 Navigation Diagnostics:**
```bash
# Check navigation node status
ros2 node list | grep navigation
ros2 node info /navigation_node

# Monitor transform tree
ros2 run tf2_tools view_frames.py
ros2 run tf2_tools echo.py map odom

# Check navigation parameters
ros2 param list /navigation_node
ros2 param get /navigation_node max_vel_x
```

**SLAM Debugging:**
```bash
# Monitor SLAM performance
ros2 topic echo /slam/pose
ros2 topic echo /slam/features
ros2 topic echo /slam/constraints

# Visualize SLAM map
ros2 run nav2_map_server map_saver_cli -f /tmp/slam_map

# Check SLAM configuration
ros2 param list /slam_node
```

**Path Planning Analysis:**
```bash
# View planned path
ros2 topic echo /navigation/global_plan
ros2 topic echo /navigation/local_plan

# Monitor planner status
ros2 topic echo /navigation/planner_status

# Check costmaps
ros2 topic echo /navigation/global_costmap/costmap
ros2 topic echo /navigation/local_costmap/costmap
```

Key Navigation Components
=========================

**1. Navigation Node** (``src/autonomy/core/navigation/navigation_node.py``)
- Main navigation coordinator
- Integrates planning, control, and localization
- Handles navigation action requests
- Monitors navigation health

**2. Path Planner** (``src/autonomy/core/navigation/path_planner.py``)
- Global path planning using A* or similar algorithms
- Local trajectory optimization
- Obstacle avoidance planning
- Terrain-aware path generation

**3. Motion Controller** (``src/autonomy/core/navigation/motion_controller.py``)
- Converts path plans to velocity commands
- Implements PID control loops
- Handles trajectory following
- Manages acceleration/deceleration

**4. GNSS Processor** (``src/autonomy/core/navigation/gnss_processor.py``)
- GPS/GNSS data processing and filtering
- Coordinate system transformations
- Accuracy estimation and quality metrics
- Integration with IMU data

**5. SLAM System** (``src/autonomy/perception/slam/slam_node.py``)
- Real-time mapping and localization
- Feature extraction from camera/depth data
- Loop closure detection
- Map optimization and maintenance

**6. Terrain Intelligence** (``src/autonomy/core/terrain_intelligence/``)
- Terrain classification and analysis
- Traversability assessment
- Slope and roughness calculation
- Hazard detection

Common Navigation Issues & Solutions
====================================

**Localization Drift:**
```python
# Check GPS quality first
gps_quality = get_gps_quality()
if gps_quality.hdop > 2.0:  # Poor GPS accuracy
    increase_slam_weight()
    enable_visual_odometry_fallback()

# Reset SLAM if drift is too large
if pose_uncertainty > MAX_UNCERTAINTY:
    reset_slam_pose()
    perform_loop_closure()
```

**Path Planning Failures:**
```python
# Check if goal is reachable
if not is_goal_reachable(goal_pose):
    log_unreachable_goal(goal_pose)
    return False

# Relax planning constraints if needed
if planning_attempts > MAX_ATTEMPTS:
    increase_tolerance()
    reduce_safety_margin()
    replan_path()
```

**Motion Control Instability:**
```python
# Tune PID parameters based on error
error = calculate_tracking_error()
if abs(error) > ERROR_THRESHOLD:
    if error > 0:  # Overshooting
        increase_damping()
    else:  # Undershooting
        increase_gain()

    update_pid_parameters()
```

**Terrain Analysis Errors:**
```python
# Validate terrain data
terrain_data = get_terrain_patch()
if terrain_data.is_valid():
    traversability = analyze_traversability(terrain_data)
    update_costmap(traversability)
else:
    log_sensor_failure()
    use_conservative_defaults()
```

Navigation Testing Strategy
===========================

**Unit Tests:**
```bash
# Test individual navigation components
pytest tests/unit/navigation/test_path_planner.py
pytest tests/unit/navigation/test_motion_controller.py
pytest tests/unit/navigation/test_gnss_processor.py
pytest tests/unit/navigation/test_slam_node.py
```

**Integration Tests:**
```bash
# Test navigation stack integration
pytest tests/integration/test_navigation_comprehensive.py
pytest tests/integration/test_slam_navigation_integration.py
pytest tests/integration/test_terrain_navigation.py
```

**Simulation Tests:**
```bash
# Test in Gazebo environment
pytest tests/simulation/test_navigation_simulation.py
pytest tests/simulation/test_slam_simulation.py
pytest tests/simulation/test_terrain_simulation.py
```

**Hardware Tests:**
```bash
# Test on real rover
pytest tests/hardware/test_navigation_hardware.py
pytest tests/hardware/test_gps_hardware.py
pytest tests/hardware/test_slam_hardware.py
```

Navigation Performance Metrics
==============================

**Key Metrics to Monitor:**
- **Position Accuracy**: <0.5m RMS in GPS mode, <2.0m in SLAM-only mode
- **Path Following Error**: <0.2m lateral error, <0.1m longitudinal error
- **Planning Time**: <500ms for local plans, <2s for global plans
- **SLAM Drift Rate**: <1% per meter traveled
- **Terrain Classification Accuracy**: >90% for traversable terrain

**Monitoring Commands:**
```bash
# Navigation performance dashboard
watch -n 1 "ros2 topic echo /navigation/metrics"

# SLAM quality metrics
ros2 topic echo /slam/quality_metrics

# GPS accuracy monitoring
ros2 topic echo /gps/accuracy
```

Navigation Coordinate Systems
=============================

**Understanding Coordinate Frames:**
- **map**: Global coordinate frame (fixed reference)
- **odom**: Odometry estimate (drifts over time)
- **base_link**: Robot center point
- **camera_link**: Camera optical center
- **gps_link**: GPS antenna position

**Transform Chain:**
map → odom → base_link → camera_link

**Common Transform Issues:**
```python
# Check transform availability
tf_buffer.can_transform('map', 'base_link', rclpy.time.Time())

# Lookup latest transform
transform = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

# Handle transform exceptions
try:
    pose_in_map = tf2_geometry_msgs.do_transform_pose(pose_in_base, transform)
except TransformException as e:
    log_transform_error(e)
    use_last_known_pose()
```

Development Workflow
====================

**Making Navigation Changes:**
1. **Test in simulation first**: Never break navigation on hardware
2. **Monitor performance metrics**: Ensure no degradation
3. **Update coordinate transforms**: If changing reference frames
4. **Test edge cases**: GPS dropout, obstacle fields, rough terrain
5. **Document parameter changes**: Update configuration guides

**Navigation Code Review Checklist:**
- [ ] Coordinate frames properly handled
- [ ] Error handling for sensor failures implemented
- [ ] Performance metrics monitored
- [ ] Simulation testing completed
- [ ] Hardware validation planned
- [ ] Documentation updated
- [ ] Parameter ranges validated

Navigation Safety Considerations
================================

**Critical Safety Systems:**
- Emergency stop on navigation failure
- Fallback to manual control mode
- Speed limiting in uncertain terrain
- Obstacle detection overrides
- GPS quality monitoring

**Safety-Triggered Behaviors:**
```python
# Navigation safety monitor
def check_navigation_safety():
    if localization_uncertainty > MAX_SAFE_UNCERTAINTY:
        trigger_emergency_stop("Localization lost")

    if obstacle_distance < MIN_SAFE_DISTANCE:
        trigger_emergency_stop("Obstacle too close")

    if gps_quality_degraded():
        reduce_maximum_speed()
        enable_safety_mode()
```

Getting Help
============

**Navigation-Specific Resources:**
- ROS2 Navigation2 Documentation
- SLAM algorithm papers (RTAB-Map, ORB-SLAM)
- GPS/GNSS processing guides
- Terrain classification literature

**Team Resources:**
- Navigation architecture diagrams
- Simulation test worlds in ``simulation/worlds/``
- Navigation tuning tools in ``tools/navigation/``
- Performance profiling scripts

**When You're Stuck:**
1. Check transforms: ``ros2 run tf2_tools view_frames.py``
2. Visualize navigation: Use RViz with navigation plugins
3. Test in isolation: ``python tools/test_navigation_isolation.py``
4. Review logs: ``tail -f log/navigation_*.log``
5. Ask in #slam-nav-team channel

Remember: Navigation is the foundation of autonomy. When navigation works well, the rover seems almost intelligent. When it fails, nothing else matters!

