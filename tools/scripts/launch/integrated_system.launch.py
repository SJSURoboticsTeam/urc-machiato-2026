#!/usr/bin/env python3
"""
Integrated System Launch File - Full system with teleoperation and control systems

Launches the complete URC 2026 system including:
- Autonomy system (navigation, SLAM, mission control)
- Teleoperation frontend (submodule)
- Control systems (submodule)
- ROS bridge for WebSocket communication
- All necessary bridges and interfaces

This launch file demonstrates the integration of submodules for complete system testing.
"""

import os

from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description import LaunchDescription  # type: ignore
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate complete integrated system launch description"""

    # Get workspace paths
    # File is at: tools/scripts/launch/integrated_system.launch.py
    # dirname(abspath) -> tools/scripts/launch
    # dirname*2 -> tools/scripts
    # dirname*3 -> tools
    # dirname*4 -> urc-machiato-2026 (root)
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    teleoperation_path = os.path.join(workspace_root, "submodules", "teleoperation")
    control_systems_path = os.path.join(workspace_root, "submodules", "control-systems")

    # Define environment with PYTHONPATH
    # This ensures all nodes can find modules in /src, /missions, and /simulation
    # We append to existing PYTHONPATH to keep ROS2 libraries (rclpy, etc.)
    node_env = os.environ.copy()
    extra_python_paths = [
        workspace_root,
        os.path.join(workspace_root, "src"),
        os.path.join(workspace_root, "src/autonomy/core/state_management"),
        os.path.join(workspace_root, "src/autonomy/core/state_management/autonomy_state_machine"),
    ]
    
    current_pp = node_env.get("PYTHONPATH", "")
    node_env["PYTHONPATH"] = ":".join(extra_python_paths) + (f":{current_pp}" if current_pp else "")

    return LaunchDescription(
        [
            # ===========================================
            # SYSTEM STATUS
            # ===========================================
            ExecuteProcess(
                cmd=["echo", "=== URC 2026 Integrated System Launch ==="],
                output="screen",
                name="system_status",
            ),
            ExecuteProcess(
                cmd=["echo", f"Teleoperation submodule: {teleoperation_path}"],
                output="screen",
                name="teleoperation_status",
            ),
            ExecuteProcess(
                cmd=["echo", f"Control systems submodule: {control_systems_path}"],
                output="screen",
                name="control_systems_status",
            ),
            # ===========================================
            # BASIC ROS2 INFRASTRUCTURE
            # ===========================================
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "rosbridge_server",
                    "rosbridge_server",
                    "--ros-args",
                    "-p",
                    "port:=9090",
                ],
                output="screen",
                name="rosbridge_server",
            ),
            # ===========================================
            # MOCK TOPICS FOR INTEGRATION TESTING
            # ===========================================
            # GPS Data
            TimerAction(
                period=1.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "topic",
                            "pub",
                            "--once",
                            "/gps/fix",
                            "sensor_msgs/msg/NavSatFix",
                            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'gps'}, latitude: 35.0, longitude: -117.0, altitude: 100.0}",
                        ],
                        output="log",
                        name="gps_publisher",
                    )
                ],
            ),
            # IMU Data
            TimerAction(
                period=0.1,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "topic",
                            "pub",
                            "--once",
                            "/imu/data",
                            "sensor_msgs/msg/Imu",
                            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'imu'}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}",
                        ],
                        output="log",
                        name="imu_publisher",
                    )
                ],
            ),
            # Depth Camera
            TimerAction(
                period=0.5,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "topic",
                            "pub",
                            "--once",
                            "/camera/depth/image_raw",
                            "sensor_msgs/msg/Image",
                            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera_depth'}, height: 480, width: 640, encoding: '32FC1'}",
                        ],
                        output="log",
                        name="depth_publisher",
                    )
                ],
            ),
            # Cmd_vel for navigation
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/cmd_vel",
                    "geometry_msgs/msg/Twist",
                    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}",
                ],
                shell=True,  # Use shell to ensure YAML string is handled correctly
                output="log",
                name="cmd_vel_publisher",
            ),
            # ===========================================
            # TELEOPERATION TOPICS (Mock)
            # ===========================================
            TimerAction(
                period=0.2,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "topic",
                            "pub",
                            "--once",
                            "/teleoperation/joint_states",
                            "sensor_msgs/msg/JointState",
                            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'teleop'}, name: ['joint1'], position: [0.0]}",
                        ],
                        output="log",
                        name="joint_states_publisher",
                    )
                ],
            ),
            TimerAction(
                period=0.2,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "topic",
                            "pub",
                            "--once",
                            "/teleoperation/chassis_velocity",
                            "geometry_msgs/msg/TwistStamped",
                            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'teleop'}}",
                        ],
                        output="log",
                        name="chassis_velocity_publisher",
                    )
                ],
            ),
            TimerAction(
                period=0.2,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "topic",
                            "pub",
                            "--once",
                            "/teleoperation/motor_temperatures",
                            "std_msgs/msg/Float32MultiArray",
                            "{data: [25.0, 26.0, 24.5, 25.5]}",
                        ],
                        output="log",
                        name="motor_temp_publisher",
                    )
                ],
            ),
            TimerAction(
                period=0.2,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "topic",
                            "pub",
                            "--once",
                            "/teleoperation/system_status",
                            "sensor_msgs/msg/BatteryState",
                            "{voltage: 24.0, current: 2.0, charge: 95.0}",
                        ],
                        output="log",
                        name="system_status_publisher",
                    )
                ],
            ),
            # ===========================================
            # ADDITIONAL MOCK TOPICS FOR TESTING
            # ===========================================
            # Odometry for navigation testing
            TimerAction(
                period=0.1,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "topic",
                            "pub",
                            "--once",
                            "/odom",
                            "nav_msgs/msg/Odometry",
                            "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'odom'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}",
                        ],
                        output="log",
                        name="odom_publisher",
                    )
                ],
            ),
            # Map topic for navigation
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/map",
                    "nav_msgs/msg/OccupancyGrid",
                    "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, info: {width: 100, height: 100, resolution: 0.1}}",
                ],
                output="log",
                name="map_publisher",
            ),
            # Goal pose for navigation testing
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/goal_pose",
                    "geometry_msgs/msg/PoseStamped",
                    "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}",
                ],
                output="log",
                name="goal_pose_publisher",
            ),
            # Camera info for vision testing
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/camera/camera_info",
                    "sensor_msgs/msg/CameraInfo",
                    "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'camera'}, height: 480, width: 640, distortion_model: 'plumb_bob'}",
                ],
                output="log",
                name="camera_info_publisher",
            ),
            # ===========================================
            # TELEOPERATION FRONTEND
            # ===========================================
            # Note: Teleoperation frontend runs separately via npm/yarn
            # This is just for reference - actual startup is manual:
            # cd vendor/teleoperation && npm install && npm run dev
            ExecuteProcess(
                cmd=[
                    "echo",
                    'Teleoperation frontend: Run manually with "cd vendor/teleoperation && npm run dev"',
                ],
                output="screen",
                name="teleoperation_instruction",
            ),
            # ===========================================
            # CONTROL SYSTEMS INTEGRATION
            # ===========================================
            ExecuteProcess(
                cmd=[
                    "echo",
                    "Control systems: Integrated via control_systems_bridge in mission_system.launch.py",
                ],
                output="screen",
                name="control_systems_instruction",
            ),
            # ===========================================
            # SYSTEM MONITORING
            # ===========================================
            TimerAction(
                period=5.0,  # Wait for systems to start
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "topic", "list"],
                        output="screen",
                        name="topic_monitor",
                    )
                ],
            ),
            TimerAction(
                period=10.0,
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "node", "list"],
                        output="screen",
                        name="node_monitor",
                    )
                ],
            ),
            # ===========================================
            # SENSOR SIMULATOR & BRIDGES
            # ===========================================
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(workspace_root, "src/autonomy/perception/simulation/autonomy_simulation/sensor_simulator.py"),
                ],
                output="screen",
                name="sensor_simulator",
                env=node_env,
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(workspace_root, "src/bridges/communication_bridge.py"),
                ],
                output="screen",
                name="communication_bridge",
                env=node_env,
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(workspace_root, "src/bridges/ros2_mission_bridge.py"),
                ],
                output="screen",
                name="mission_bridge",
                env=node_env,
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(workspace_root, "src/bridges/slam_pose_publisher.py"),
                ],
                output="screen",
                name="slam_pose_publisher",
                env=node_env,
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(workspace_root, "src/bridges/map_data_publisher.py"),
                ],
                output="screen",
                name="map_data_publisher",
                env=node_env,
            ),
            # ===========================================
            # STATE MACHINE & NAVIGATION NODES
            # ===========================================
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(workspace_root, "src/autonomy/core/state_management/autonomy_state_machine/adaptive_state_machine.py"),
                ],
                output="screen",
                name="state_machine_director",
                env=node_env,
            ),
            ExecuteProcess(
                cmd=["python3", os.path.join(workspace_root, "tests/slam_nodes.py")],
                output="screen",
                name="slam_nodes",
                env=node_env,
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(workspace_root, "tests/navigation_service_node.py"),
                ],
                output="screen",
                name="navigation_service",
                env=node_env,
            ),
            # ===========================================
            # INTEGRATION VERIFICATION
            # ===========================================
            ExecuteProcess(
                cmd=["echo", "=== Integration Complete ==="],
                output="screen",
                name="integration_complete",
            ),
            ExecuteProcess(
                cmd=["echo", "Teleoperation: http://localhost:5173 (if started)"],
                output="screen",
                name="teleoperation_url",
            ),
            ExecuteProcess(
                cmd=["echo", "ROS Bridge: ws://localhost:9090"],
                output="screen",
                name="rosbridge_url",
            ),
        ]
    )
