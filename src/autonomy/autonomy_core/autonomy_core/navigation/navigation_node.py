#!/usr/bin/env python3
"""
Main navigation controller for URC 2026 rover.

Coordinates GNSS waypoint navigation, terrain-adaptive path planning,
AR tag precision approaches, obstacle avoidance, and mission progress tracking.
"""

import math
import os
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

# Intelligent ROS2 imports with fallbacks
from src.core.ros2_environment import get_ros2_environment_manager

# Get ROS2 environment for intelligent imports
ros2_env = get_ros2_environment_manager()
optimized_imports = ros2_env.get_optimized_imports()

# Import with fallbacks
rclpy = optimized_imports.get("rclpy")
NavigateToPose = optimized_imports.get("NavigateToPose")
PoseStamped = optimized_imports.get("PoseStamped")
Twist = optimized_imports.get("Twist")
Imu = optimized_imports.get("Imu")
NavSatFix = optimized_imports.get("NavSatFix")
String = optimized_imports.get("String")
Trigger = optimized_imports.get("Trigger")
LifecycleState = optimized_imports.get("LifecycleState")
TransitionCallbackReturn = optimized_imports.get("TransitionCallbackReturn")
# SLAM/odom types (for navigation-SLAM integration)
try:
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
except ImportError:
    PoseWithCovarianceStamped = None
    Odometry = None

# Use optimized node utilities and unified systems
from autonomy.core.node_utils import BaseURCNode
from src.core.simplified_component_registry import get_component_registry
from src.infrastructure.config import get_system_config

try:
    from autonomy_navigation.gnss_processor import GNSSProcessor
    from autonomy_navigation.motion_controller import MotionController
    from autonomy_navigation.path_planner import PathPlanner
except ImportError:
    # Fallback for local development
    from gnss_processor import GNSSProcessor
    from motion_controller import MotionController
    from path_planner import PathPlanner

sys.path.insert(0, os.path.dirname(__file__))

# Unified blackboard client and key constants
_workspace_src = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../../../../src")
)
if _workspace_src not in sys.path:
    sys.path.insert(0, _workspace_src)
try:
    from core.unified_blackboard_client import UnifiedBlackboardClient
    from core.blackboard_keys import BlackboardKeys

    UNIFIED_BLACKBOARD_AVAILABLE = True
except ImportError:
    UNIFIED_BLACKBOARD_AVAILABLE = False
    UnifiedBlackboardClient = None
    BlackboardKeys = None


@dataclass
class Waypoint:
    """Geographic waypoint with navigation metadata."""

    latitude: float
    longitude: float
    altitude: float = 0.0
    name: str = ""
    precision_required: bool = False


@dataclass
class NavigationGoal:
    """Navigation goal with approach constraints."""

    waypoint: Waypoint
    approach_tolerance: float = 1.0  # meters
    orientation_required: bool = False
    target_heading: float = 0.0  # radians


class NavigationNode(BaseURCNode):
    """
    Main navigation controller coordinating all navigation subsystems.
    Now with Lifecycle management and optimized sensor integration.
    """

    def __init__(self) -> None:
        """Initialize navigation node with intelligent ROS2 handling."""
        super().__init__("navigation_node")

        # Performance and monitoring
        self.profiler = get_profiler()
        self.memory_monitor = get_memory_monitor()

        # Set memory baseline
        self.memory_monitor.set_baseline()

        # Get component registry for advanced component management
        self.component_registry = get_component_registry()

        # Register component loaders for heavy dependencies
        # These will be loaded lazily when first accessed
        self._component_loaders = {
            "gnss_processor": self._load_gnss_processor,
            "path_planner": self._load_path_planner,
            "motion_controller": self._load_motion_controller,
            "terrain_analyzer": self._load_terrain_analyzer,
            "ml_analyzer": self._load_ml_analyzer,
        }

        # Subsystems - loaded lazily
        self.gnss_processor = None
        self.path_planner = None
        self.motion_controller = None
        self.terrain_analyzer = None
        self.ml_analyzer = None

        # Lazy loading methods for heavy components

    def _load_gnss_processor(self):
        """Lazy load GNSS processor."""
        try:
            from autonomy_navigation.gnss_processor import GNSSProcessor

            return GNSSProcessor()
        except ImportError:
            from gnss_processor import GNSSProcessor

            return GNSSProcessor()

    def _load_path_planner(self):
        """Lazy load path planner."""
        try:
            from autonomy_navigation.path_planner import PathPlanner

            return PathPlanner()
        except ImportError:
            from path_planner import PathPlanner

            return PathPlanner()

    def _load_motion_controller(self):
        """Lazy load motion controller."""
        try:
            from autonomy_navigation.motion_controller import MotionController

            return MotionController()
        except ImportError:
            from motion_controller import MotionController

            return MotionController()

    def _load_terrain_analyzer(self):
        """Lazy load terrain analyzer (heavy dependency)."""
        try:
            from autonomy.core.terrain_intelligence.terrain_analyzer import (
                TerrainAnalyzer,
            )

            return TerrainAnalyzer()
        except ImportError:
            logger.warning("Terrain analyzer not available")
            return None

    def _load_ml_analyzer(self):
        """Lazy load ML analyzer (very heavy dependency)."""
        try:
            from autonomy.core.terrain_intelligence.terrain_ml_analyzer import (
                TerrainMLAnalyzer,
            )

            return TerrainMLAnalyzer()
        except ImportError:
            logger.warning("ML analyzer not available")
            return None

        # Navigation state
        self.current_waypoint: Optional[Waypoint] = None
        self.current_path: List[Tuple[float, float]] = []
        self.current_pose = None
        self.last_imu: Optional[Imu] = None
        self.is_navigating = False
        self.control_timer = None
        # SLAM/odom integration: prefer SLAM pose when recent and confident, else odom
        self._slam_pose: Optional[Tuple[float, float, float]] = None
        self._slam_pose_stamp_ns: int = 0
        self._slam_confidence: float = 0.0
        self._odom_pose: Optional[Tuple[float, float, float]] = None
        self._odom_pose_stamp_ns: int = 0
        self._slam_pose_max_age_ns: int = int(1.0 * 1e9)  # 1s
        self._slam_confidence_min: float = 0.3
        # Costmap-driven replan: obstacles (x, y, radius), rate limit
        self._obstacle_list: List[Tuple[float, float, float]] = []
        self._last_replan_time: float = 0.0
        self._max_replan_interval_sec: float = 2.0
        self._replan_timer = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handle transition to configured state with lazy loading."""
        self.logger.info("Configuring Navigation Subsystems...")

        # Compatibility fix for legacy interface_factory usage
        self.interface_factory = self

        # Load core subsystems lazily (only when needed)
        # GNSS processor loaded on first GPS message
        # Path planner loaded on first navigation request
        # Motion controller loaded on first movement command

        # Setup interfaces (lightweight - no heavy dependencies)
        self._setup_interfaces()
        self._setup_processing_pipeline()

        # Log memory usage after configuration
        memory_info = self.get_memory_usage()
        self.logger.info(
            f"Navigation node configured. Memory: {memory_info['total_memory_mb']:.1f}MB "
            f"(+{memory_info['memory_delta_mb']:.1f}MB)"
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handle transition to active state."""
        self.logger.info("Activating Navigation Control...")

        # Start control loop timer
        self.control_timer = self.create_timer(
            1.0 / self.params.update_rate, self._control_loop
        )
        # Replan timer: periodic costmap-driven replan (max rate limited)
        self._replan_timer = self.create_timer(1.0, self._try_replan)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handle transition to inactive state."""
        self.logger.info("Deactivating Navigation Control...")
        self.stop_navigation()
        if self.control_timer:
            self.control_timer.cancel()
        if self._replan_timer:
            self._replan_timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handle transition to unconfigured state."""
        self.logger.info("Cleaning up Navigation...")
        self.is_navigating = False
        return TransitionCallbackReturn.SUCCESS

    def _setup_interfaces(self) -> None:
        """Setup ROS2 interfaces with automatic registration."""
        # Subscribers
        self.interface_factory.create_subscriber(
            NavSatFix, "/gnss/fix", self._gnss_callback
        )
        self.interface_factory.create_subscriber(Imu, "/imu/data", self._imu_callback)
        # SLAM and odom for unified pose (fallback when GPS poor)
        if PoseWithCovarianceStamped is not None:
            self.interface_factory.create_subscriber(
                PoseWithCovarianceStamped, "slam/pose", self._slam_pose_callback
            )
        if Odometry is not None:
            self.interface_factory.create_subscriber(
                Odometry, "/odom", self._odom_callback
            )
        # Costmap/obstacles for dynamic replanning (optional)
        try:
            from nav_msgs.msg import OccupancyGrid

            self.interface_factory.create_subscriber(
                OccupancyGrid, "/costmap", self._costmap_callback
            )
        except ImportError:
            pass  # No OccupancyGrid: replan uses empty obstacle list

        # Publishers - Updated to use Twist Mux autonomy topic
        self.cmd_vel_pub = self.interface_factory.create_publisher(
            Twist, "/cmd_vel/autonomy"
        )
        self.waypoint_pub = self.interface_factory.create_publisher(
            PoseStamped, "/navigation/current_waypoint"
        )

        # Services
        self.interface_factory.create_service(
            Trigger, "/navigation/stop", self._stop_navigation_callback
        )

        # Action server
        self.navigate_action_server = self.interface_factory.create_action_server(
            NavigateToPose, "/navigate_to_pose", self._navigate_to_pose_callback
        )

        # Control timer
        self.interface_factory.create_timer(
            1.0 / self.params.update_rate, self._control_loop, "control"
        )

        # Initialize unified blackboard client for navigation state
        if UNIFIED_BLACKBOARD_AVAILABLE:
            try:
                self.blackboard = UnifiedBlackboardClient(self)
                self.logger.info("Unified blackboard client initialized for navigation")
            except Exception as e:
                self.logger.warning(f"Failed to initialize blackboard client: {e}")
                self.blackboard = None
        else:
            self.blackboard = None
            self.logger.warning(
                "Unified blackboard not available - navigation state won't be written to blackboard"
            )

    def _setup_processing_pipeline(self) -> None:
        """Setup data processing pipeline."""
        self.navigation_pipeline = (
            MessagePipeline(self.logger, self.trace_data)
            .add_step(self._validate_navigation_data, "validation")
            .add_step(self._process_sensor_fusion, "sensor_fusion")
            .add_step(self._update_navigation_state, "state_update")
        )

    def _validate_navigation_data(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate navigation input data."""
        required_fields = ["gnss", "imu", "goal"]
        for field in required_fields:
            if field not in data:
                raise ValueError(f"Missing required field: {field}")
        return data

    def _process_sensor_fusion(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Process sensor fusion for navigation."""
        # Sensor fusion logic
        fused_pose = self.gnss_processor.fuse_sensors(data["gnss"], data["imu"])
        data["fused_pose"] = fused_pose
        return data

    def _update_navigation_state(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Update navigation state based on processed data."""
        if self.is_navigating and self.current_path:
            # Update progress along path
            progress = self._calculate_path_progress(data["fused_pose"])
            data["progress"] = progress

            # Update unified blackboard with navigation state
            if self.blackboard and BlackboardKeys:
                # Calculate distance to target
                if self.current_waypoint and "fused_pose" in data:
                    # Simplified distance calculation
                    distance = math.sqrt(
                        (self.current_waypoint.latitude - data["fused_pose"][0]) ** 2
                        + (self.current_waypoint.longitude - data["fused_pose"][1]) ** 2
                    )
                    self.blackboard.set(BlackboardKeys.DISTANCE_TO_TARGET, distance)

                # Update path clear status (simplified - check if path exists)
                self.blackboard.set(
                    BlackboardKeys.PATH_CLEAR, len(self.current_path) > 0
                )
        return data

    def _gnss_callback(self, msg: NavSatFix) -> None:
        """Handle GNSS data updates with lazy loading."""
        self.trace_data("gnss_received", msg)

        # Lazy load GNSS processor on first GPS message
        if self.gnss_processor is None:
            try:
                if "gnss_processor" in self._component_loaders:
                    self.gnss_processor = self._component_loaders["gnss_processor"]()
                    self.logger.info("GNSS processor loaded on-demand")
                else:
                    # Fallback to registry
                    self.gnss_processor = self.component_registry.get_component(
                        "gnss_processor"
                    )
            except Exception as e:
                self.logger.error(f"Failed to load GNSS processor: {e}")
                return

        # Process GNSS data through pipeline
        pipeline_data = {
            "gnss": msg,
            "imu": self.last_imu,
            "goal": self.current_waypoint,
        }
        result = self.navigation_pipeline.process(pipeline_data)

        if isinstance(result, Failure):
            self.logger.error("GNSS processing failed", error=result.error)
        else:
            self.logger.debug("GNSS data processed successfully")

    def _imu_callback(self, msg: Imu) -> None:
        """Handle IMU data updates."""
        self.last_imu = msg
        self.trace_data("imu_received", msg)

    def _slam_pose_callback(self, msg) -> None:
        """Handle SLAM pose updates for unified pose and fallback."""
        self._slam_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        stamp = msg.header.stamp
        self._slam_pose_stamp_ns = stamp.sec * (10**9) + stamp.nanosec
        # Covariance-based confidence (trace of position block)
        cov = getattr(msg.pose, "covariance", None)
        if cov and len(cov) >= 9:
            self._slam_confidence = 1.0 / (1.0 + (cov[0] + cov[7] + cov[14]) / 3.0)
        else:
            self._slam_confidence = 0.8
        try:
            from ..perception.sensor_health import get_sensor_health_tracker

            get_sensor_health_tracker().update("slam_pose", valid=True)
        except ImportError:
            try:
                from autonomy_core.perception.sensor_health import (
                    get_sensor_health_tracker,
                )

                get_sensor_health_tracker().update("slam_pose", valid=True)
            except ImportError:
                pass
        if self.blackboard and BlackboardKeys:
            self.blackboard.set(BlackboardKeys.SLAM_CONFIDENCE, self._slam_confidence)
        self._update_current_pose_from_unified()

    def _odom_callback(self, msg) -> None:
        """Handle odometry updates for fallback when SLAM is lost."""
        self._odom_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        stamp = msg.header.stamp
        self._odom_pose_stamp_ns = stamp.sec * (10**9) + stamp.nanosec
        try:
            from ..perception.sensor_health import get_sensor_health_tracker

            get_sensor_health_tracker().update("odom", valid=True)
        except ImportError:
            try:
                from autonomy_core.perception.sensor_health import (
                    get_sensor_health_tracker,
                )

                get_sensor_health_tracker().update("odom", valid=True)
            except ImportError:
                pass
        self._update_current_pose_from_unified()

    def _costmap_callback(self, msg) -> None:
        """Convert OccupancyGrid to obstacle list (x, y, radius) for path planner."""
        try:
            from nav_msgs.msg import OccupancyGrid
        except ImportError:
            return
        res = getattr(msg.info, "resolution", 0.5)
        if res <= 0:
            res = 0.5
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        w = msg.info.width
        h = msg.info.height
        obstacles: List[Tuple[float, float, float]] = []
        occupied_threshold = 80
        for i in range(min(w * h, len(msg.data))):
            if msg.data[i] > occupied_threshold:
                gx = i % w
                gy = i // w
                wx = ox + (gx + 0.5) * res
                wy = oy + (gy + 0.5) * res
                obstacles.append((wx, wy, res * 0.6))
        self._obstacle_list = obstacles

    def _try_replan(self) -> None:
        """Rate-limited replan using current costmap/obstacles; replace current path."""
        if (
            not self.is_navigating
            or not self.current_pose
            or not self.current_waypoint
            or self.path_planner is None
        ):
            return
        now = time.time()
        if now - self._last_replan_time < self._max_replan_interval_sec:
            return
        self._last_replan_time = now
        try:
            self.path_planner.update_costmap(self._obstacle_list)
            goal_xy = (self.current_waypoint.latitude, self.current_waypoint.longitude)
            new_path = self.path_planner.plan_path(
                self.current_pose, goal_xy, self.params.node_specific_params
            )
            if new_path:
                self.current_path = new_path
                self.logger.debug("Replanned path (%d points)", len(new_path))
        except Exception as e:
            self.logger.debug("Replan skipped: %s", e)

    def _get_unified_pose(self) -> Optional[Tuple[float, float]]:
        """Return best available (x, y) in map frame: prefer SLAM when recent and confident, else odom (fallback fusion)."""
        now_ns = self.get_clock().now().nanoseconds
        # Prefer SLAM if recent and confident
        if (
            self._slam_pose is not None
            and (now_ns - self._slam_pose_stamp_ns) <= self._slam_pose_max_age_ns
        ):
            if self._slam_confidence >= self._slam_confidence_min:
                return (self._slam_pose[0], self._slam_pose[1])
        # Fallback to odom when SLAM is lost or stale (one valid source drives degraded pose)
        if (
            self._odom_pose is not None
            and (now_ns - self._odom_pose_stamp_ns) <= self._slam_pose_max_age_ns
        ):
            return (self._odom_pose[0], self._odom_pose[1])
        return None

    def _update_current_pose_from_unified(self) -> None:
        """Update current_pose from unified pose (SLAM or odom) for path planning and goal check."""
        unified = self._get_unified_pose()
        if unified is not None:
            self.current_pose = unified

    def _navigate_to_pose_callback(self, goal_handle) -> NavigateToPose.Result:
        """Handle navigation action requests."""
        self.profiler.start_timer("navigate_to_pose")

        try:
            # Validate operation requirements
            validation = self.validate_operation(
                "navigate_to_pose",
                required_state="idle",
                required_interfaces={"subscribers": 2},  # GNSS and IMU
            )

            if isinstance(validation, Failure):
                self.logger.error(
                    "Navigation validation failed", error=validation.error
                )
                goal_handle.abort()
                return NavigateToPose.Result()

            # Extract goal and start navigation
            goal = goal_handle.request.target_pose
            result = self.start_navigation_to_pose(goal)

            if isinstance(result, Failure):
                self.logger.error("Navigation start failed", error=result.error)
                goal_handle.abort()
                return NavigateToPose.Result()

            goal_handle.succeed()

            # End profiling
            duration = self.profiler.end_timer("navigate_to_pose")
            self.get_logger().debug(".3f")

            return NavigateToPose.Result()
        except Exception as e:
            self.logger.error(f"Navigation failed: {e}")
            goal_handle.abort()
            return NavigateToPose.Result()

    def _stop_navigation_callback(self, request, response) -> Trigger.Response:
        """Handle navigation stop requests."""
        self.stop_navigation()
        response.success = True
        response.message = "Navigation stopped"
        return response

    def _control_loop(self) -> None:
        """Main navigation control loop."""
        # Keep current_pose updated from SLAM/odom for path following
        self._update_current_pose_from_unified()
        if not self.is_navigating:
            return

        try:
            # Memory monitoring
            mem_usage = self.memory_monitor.get_memory_usage()
            mem_pressure = self.memory_monitor.check_memory_pressure()

            if mem_pressure in ["HIGH", "CRITICAL"]:
                self.get_logger().warn(".1f")

            # Lazy load motion controller on first movement command
            if self.motion_controller is None:
                try:
                    if "motion_controller" in self._component_loaders:
                        self.motion_controller = self._component_loaders[
                            "motion_controller"
                        ]()
                        self.logger.info("Motion controller loaded on-demand")
                    else:
                        # Fallback to registry
                        self.motion_controller = self.component_registry.get_component(
                            "motion_controller"
                        )
                except Exception as e:
                    self.logger.error(f"Failed to load motion controller: {e}")
                    return

            # Generate velocity commands
            cmd_vel = self.motion_controller.compute_velocity_commands(
                self.current_path, self.current_pose
            )

            # Publish command
            self.cmd_vel_pub.publish(cmd_vel)
            self.trace_data("velocity_command", cmd_vel)

            # Check if goal reached
            if self._is_goal_reached():
                self._complete_navigation()

        except Exception as e:
            self.logger.error("Control loop error", error=e)
            self.transition_to("error", f"Control loop failed: {str(e)}")

    def start_navigation_to_pose(self, target_pose: PoseStamped) -> OperationResult:
        """Start navigation to target pose with validation."""
        try:
            # Convert to waypoint
            waypoint = self._pose_to_waypoint(target_pose)

            # Validate waypoint
            validation = self._validate_waypoint(waypoint)
            if isinstance(validation, Failure):
                return validation

            # Lazy load path planner on first navigation request
            if self.path_planner is None:
                try:
                    if "path_planner" in self._component_loaders:
                        self.path_planner = self._component_loaders["path_planner"]()
                        self.logger.info("Path planner loaded on-demand")
                    else:
                        # Fallback to registry
                        self.path_planner = self.component_registry.get_component(
                            "path_planner"
                        )
                except Exception as e:
                    self.logger.error(f"Failed to load path planner: {e}")
                    return failure(
                        ProcessingError("component_loading", "path_planner_failed"),
                        operation="start_navigation",
                    )

            # Plan path (use unified pose so SLAM/odom drives start point)
            self._update_current_pose_from_unified()
            path_result = self.path_planner.plan_path(
                self.current_pose, waypoint, self.params.node_specific_params
            )

            if not path_result:
                return failure(
                    ProcessingError("path_planning", "no_valid_path_found"),
                    operation="start_navigation",
                )

            # Start navigation
            self.current_waypoint = waypoint
            self.current_path = path_result
            self.is_navigating = True
            self.transition_to("navigating", f"Goal: {waypoint}")

            self.logger.info(
                "Navigation started",
                waypoint=waypoint.name,
                path_length=len(self.current_path),
            )

            return success(True, "start_navigation")

        except Exception as e:
            return failure(
                ProcessingError("start_navigation", f"unexpected_error: {str(e)}"),
                operation="start_navigation",
            )

    def stop_navigation(self) -> None:
        """Stop current navigation."""
        if self.is_navigating:
            self.is_navigating = False
            self.current_path = []
            self.transition_to("idle", "Navigation stopped")

            # Update unified blackboard
            if self.blackboard and BlackboardKeys:
                self.blackboard.set(BlackboardKeys.NAVIGATION_STATUS, "idle")
                self.blackboard.set(BlackboardKeys.PATH_CLEAR, True)

            # Publish zero velocity to stop
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

            self.logger.info("Navigation stopped")

    def _is_goal_reached(self) -> bool:
        """Check if current goal has been reached."""
        if not self.current_waypoint or not self.current_pose:
            return False

        distance = math.sqrt(
            (self.current_waypoint.latitude - self.current_pose[0]) ** 2
            + (self.current_waypoint.longitude - self.current_pose[1]) ** 2
        )

        return distance < self.params.node_specific_params["waypoint_tolerance"]

    def _complete_navigation(self) -> None:
        """Handle successful navigation completion."""
        self.logger.info(
            "Navigation goal reached",
            waypoint=self.current_waypoint.name if self.current_waypoint else "unknown",
        )

        self.is_navigating = False
        self.transition_to("completed", "Goal reached")

    def _pose_to_waypoint(self, pose: PoseStamped) -> Waypoint:
        """Convert PoseStamped to Waypoint."""
        # Simplified conversion - would need proper coordinate transformation
        return Waypoint(
            latitude=pose.pose.position.x,  # Simplified
            longitude=pose.pose.position.y,  # Simplified
            altitude=pose.pose.position.z,
            name=f"waypoint_{int(pose.header.stamp.sec)}",
        )

    def _validate_waypoint(self, waypoint: Waypoint) -> OperationResult:
        """Validate waypoint parameters."""
        if not (-90 <= waypoint.latitude <= 90):
            return failure(
                ValidationError(
                    "latitude", waypoint.latitude, "must be between -90 and 90"
                )
            )

        if not (-180 <= waypoint.longitude <= 180):
            return failure(
                ValidationError(
                    "longitude", waypoint.longitude, "must be between -180 and 180"
                )
            )

        return success(waypoint)

    def _calculate_path_progress(self, current_pose) -> float:
        """Calculate progress along current path."""
        if not self.current_path:
            return 0.0

        # Simplified progress calculation
        return min(1.0, len(self.current_path) * 0.1)  # Placeholder

    def set_navigation_goal(self, goal: NavigationGoal) -> bool:
        """Set a new navigation goal"""
        if not self.current_position:
            self.get_logger().error("No current position available")
            return False

        self.current_goal = goal
        self.goal_start_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.get_logger().info(f"Set navigation goal: {goal.waypoint.name}")
        return True

    def check_goal_timeout(self) -> bool:
        """Check if current goal has timed out"""
        if not self.goal_start_time:
            return False

        elapsed = self.get_clock().now().seconds_nanoseconds()[0] - self.goal_start_time
        return elapsed > self.goal_timeout

    def publish_velocity_commands(self, linear_vel: float, angular_vel: float):
        """Publish velocity commands to motion controller"""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_publisher.publish(twist)

    def publish_current_waypoint(self):
        """Publish current waypoint information"""
        if self.current_goal:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.current_goal.waypoint.latitude
            pose.pose.position.y = self.current_goal.waypoint.longitude
            pose.pose.position.z = self.current_goal.waypoint.altitude
            self.current_waypoint_publisher.publish(pose)

    def stop_motion(self):
        """Stop all motion"""
        twist = Twist()  # Zero velocities
        self.cmd_vel_publisher.publish(twist)

    def status_callback(self):
        """Publish navigation status"""
        status_msg = String()
        status_msg.data = f"Navigation: {self.current_state.value}"
        if self.current_goal:
            status_msg.data += f" | Goal: {self.current_goal.waypoint.name}"
        if self.current_position:
            status_msg.data += ".2f"
        self.status_publisher.publish(status_msg)

    def calculate_distance_bearing(
        self, pos1: Tuple[float, float, float], pos2: Tuple[float, float, float]
    ) -> Tuple[float, float]:
        """Calculate distance and bearing between two positions"""
        # Simplified calculation - would use proper geodesy in production
        lat1, lon1, alt1 = pos1
        lat2, lon2, alt2 = pos2

        # Convert to radians
        lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
        lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)

        # Haversine distance
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = 6371000 * c  # Earth radius in meters

        # Bearing calculation
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(
            lat2_rad
        ) * math.cos(dlon)
        bearing = math.atan2(y, x)

        return distance, bearing

    def extract_heading_from_imu(self, imu_msg: Imu) -> float:
        """Extract heading from IMU quaternion"""
        # Simplified quaternion to euler conversion
        # In production, would use proper conversion library
        q = imu_msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        heading = math.atan2(siny_cosp, cosy_cosp)
        return heading


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    navigation_node = NavigationNode()

    try:
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
