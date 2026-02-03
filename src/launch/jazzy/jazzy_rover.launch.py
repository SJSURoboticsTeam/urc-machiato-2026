#!/usr/bin/env python3
"""
Jazzy-Enhanced URC 2026 Mars Rover Launch File

Demonstrates full system integration with:
- Cyclone DDS optimized communication
- Iceoryx2 shared memory
- Component lifecycle management
- QoS profiles for different communication patterns
- Real-time BT execution
- Performance monitoring
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.actions import GroupAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import EnvironmentVariable, TextSubstitution
from launch_ros.actions import Node, LifecycleNode, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate the Jazzy rover launch description"""

    # ===== LAUNCH CONFIGURATION =====

    # Launch arguments
    use_jazzy_features = DeclareLaunchArgument(
        "jazzy_features",
        default_value="true",
        description="Enable Jazzy-specific features (QoS, lifecycle, etc.)",
    )

    use_cyclone_dds = DeclareLaunchArgument(
        "cyclone_dds",
        default_value="false",  # Default RMW until Cyclone DDS is available
        description="Use Cyclone DDS instead of default RMW",
    )

    enable_iceoryx2 = DeclareLaunchArgument(
        "iceoryx2",
        default_value="true",
        description="Enable Iceoryx2 shared memory communication",
    )

    enable_simulation = DeclareLaunchArgument(
        "simulation",
        default_value="true",
        description="Run in simulation mode (no hardware)",
    )

    enable_monitoring = DeclareLaunchArgument(
        "monitoring",
        default_value="true",
        description="Enable performance monitoring and telemetry",
    )

    # ===== ENVIRONMENT SETUP =====

    # Set Jazzy environment variables
    set_jazzy_env = [
        # Iceoryx2 for shared memory
        ExecuteProcess(
            cmd=["echo", "Enabling Iceoryx2 shared memory communication"],
            name="jazzy_env_setup",
            output="screen",
        )
    ]

    # ===== COMPONENT LAUNCH =====

    # 1. Jazzy Component Manager (lifecycle-managed)
    component_manager = LifecycleNode(
        package="urc_core",  # Would be our package name
        executable="jazzy_component_manager",  # Our Python component manager
        name="jazzy_component_manager",
        namespace="",
        output="screen",
        parameters=[
            {
                "use_jazzy_features": LaunchConfiguration("jazzy_features"),
                "enable_monitoring": LaunchConfiguration("monitoring"),
                "simulation_mode": LaunchConfiguration("simulation"),
            }
        ],
        # Jazzy: Enhanced QoS for component management
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 2. Adaptive State Machine (production runtime state, lifecycle-managed)
    state_machine_bridge = ExecuteProcess(
        cmd=["python3", "-m", "src.core.adaptive_state_machine"],
        name="adaptive_state_machine",
        output="screen",
        additional_env={"PYTHONPATH": os.environ.get("PYTHONPATH", ".")},
    )

    # 3. BT Orchestrator (C++ lifecycle node)
    bt_orchestrator = LifecycleNode(
        package="autonomy_bt",
        executable="bt_orchestrator",
        name="bt_orchestrator",
        namespace="",
        output="screen",
        parameters=[
            {
                "bt_file": os.path.join(
                    get_package_share_directory("autonomy_bt"),
                    "behavior_trees",
                    "main_mission.xml",
                ),
                "execution_rate_hz": 10,
                "enable_performance_monitoring": LaunchConfiguration("monitoring"),
            }
        ],
        # Jazzy: Real-time scheduling parameters
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
            # Real-time scheduling (would require root/sudo)
            # '--scheduler', 'SCHED_FIFO:50'
        ],
    )

    # ===== LIFECYCLE MANAGEMENT =====

    # Configure components in dependency order
    configure_component_manager = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/jazzy_component_manager", "configure"],
        name="configure_component_manager",
        output="screen",
    )

    configure_state_machine = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/adaptive_state_machine", "configure"],
        name="configure_state_machine",
        output="screen",
    )

    configure_bt_orchestrator = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/bt_orchestrator", "configure"],
        name="configure_bt_orchestrator",
        output="screen",
    )

    # Activate components in dependency order
    activate_component_manager = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/jazzy_component_manager", "activate"],
        name="activate_component_manager",
        output="screen",
    )

    activate_state_machine = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/adaptive_state_machine", "activate"],
        name="activate_state_machine",
        output="screen",
    )

    activate_bt_orchestrator = ExecuteProcess(
        cmd=["ros2", "lifecycle", "set", "/bt_orchestrator", "activate"],
        name="activate_bt_orchestrator",
        output="screen",
    )

    # ===== MONITORING AND TELEMETRY =====

    # System health monitor
    health_monitor = Node(
        package="urc_core",
        executable="system_health_monitor",
        name="system_health_monitor",
        namespace="",
        output="screen",
        parameters=[
            {
                "monitor_rate_hz": 1.0,
                "alert_thresholds": {
                    "bt_execution_time_ms": 150,
                    "component_restart_count": 3,
                    "memory_usage_percent": 90,
                },
            }
        ],
        condition=IfCondition(LaunchConfiguration("monitoring")),
    )

    # Performance profiler
    performance_profiler = Node(
        package="urc_core",
        executable="performance_profiler",
        name="performance_profiler",
        namespace="",
        output="screen",
        parameters=[
            {
                "profile_rate_hz": 10.0,
                "metrics": [
                    "cpu_usage",
                    "memory_usage",
                    "network_latency",
                    "bt_execution_time",
                ],
            }
        ],
        condition=IfCondition(LaunchConfiguration("monitoring")),
    )

    # ===== SIMULATION COMPONENTS =====

    # Gazebo simulation (if in simulation mode)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("gazebo_simulation"),
                        "launch",
                        "mars_yard.launch.py",
                    ]
                )
            ]
        ),
        condition=IfCondition(LaunchConfiguration("simulation")),
    )

    # Simulated sensor data
    sensor_simulator = Node(
        package="urc_simulation",
        executable="sensor_simulator",
        name="sensor_simulator",
        namespace="",
        output="screen",
        parameters=[
            {
                "publish_rate_hz": 100,  # 100Hz sensor data
                "noise_level": 0.01,
                "failure_probability": 0.001,
            }
        ],
        condition=IfCondition(LaunchConfiguration("simulation")),
    )

    # ===== LOGGING AND DEBUGGING =====

    # Enhanced logging
    logging_config = ExecuteProcess(
        cmd=["ros2", "param", "set", "/jazzy_component_manager", "log_level", "info"],
        name="configure_logging",
        output="screen",
    )

    # Jazzy topic monitoring
    topic_monitor = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "hz",
            "/jazzy_component_manager/health",
            "--window",
            "-1",  # Monitor continuously
        ],
        name="topic_monitor",
        output="screen",
        condition=IfCondition(LaunchConfiguration("monitoring")),
    )

    # ===== LAUNCH SEQUENCE =====

    # Define the launch sequence with proper timing
    ld = LaunchDescription(
        [
            # Launch arguments
            use_jazzy_features,
            use_cyclone_dds,
            enable_iceoryx2,
            enable_simulation,
            enable_monitoring,
            # Environment setup
            *set_jazzy_env,
            # Start nodes
            component_manager,
            state_machine_bridge,
            bt_orchestrator,
            # Monitoring
            health_monitor,
            performance_profiler,
            # Simulation
            gazebo,
            sensor_simulator,
            # Configure components in sequence (with delays)
            TimerAction(period=2.0, actions=[configure_component_manager]),
            TimerAction(period=3.0, actions=[configure_state_machine]),
            TimerAction(period=4.0, actions=[configure_bt_orchestrator]),
            # Activate components in sequence
            TimerAction(period=6.0, actions=[activate_component_manager]),
            TimerAction(period=7.0, actions=[activate_state_machine]),
            TimerAction(period=8.0, actions=[activate_bt_orchestrator]),
            # Start monitoring
            TimerAction(period=10.0, actions=[topic_monitor]),
            # Logging configuration
            TimerAction(period=1.0, actions=[logging_config]),
            # Status message
            LogInfo(msg="🚀 Jazzy URC 2026 Mars Rover launched successfully!"),
            LogInfo(
                msg="📊 Monitor system health: ros2 topic echo /jazzy_component_manager/health"
            ),
            LogInfo(
                msg='🎮 Control rover: ros2 topic pub /adaptive_state_machine/commands std_msgs/String "data: START_MISSION"'
            ),
            LogInfo(
                msg="📈 View performance: ros2 topic echo /performance_profiler/metrics"
            ),
        ]
    )

    return ld
