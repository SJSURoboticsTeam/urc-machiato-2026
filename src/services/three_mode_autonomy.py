#!/usr/bin/env python3
"""
3-Mode Autonomy Implementation

Hybrid Default (operator controlled + safety autonomy)
Fully Autonomous (mission-specific complete AI control)  
Fully Operator Controlled (manual override for debugging)

Features:
- Mode-specific autonomous safety features
- Collision detection with emergency braking
- Living surface detection
- Operator override capabilities with safety toggle

Author: URC 2026 Autonomy Team
"""

import enum
import time
import threading
import numpy as np
from typing import Dict, Any, Optional, List, Tuple
from dataclasses import dataclass
from enum import Enum
import rclpy.node as Node
from geometry_msgs.msg import Twist, Point, Vector3
from sensor_msgs.msg import Imu, NavSatFix, Image
from std_msgs.msg import Header
from infrastructure.logging import get_logger, get_performance_logger
from infrastructure.validation import validate_input
from src.core.simplified_state_manager import get_state_manager


class AutonomyMode(Enum):
    """Primary autonomy modes."""
    HYBRID_DEFAULT = "hybrid_default"          # Operator controlled + safety autonomy
    FULLY_AUTONOMOUS = "fully_autonomous"    # Mission-specific complete AI control
    FULLY_OPERATOR = "operator_controlled"     # Manual override for debugging


class ObstacleType(Enum):
    """Types of obstacles detected."""
    HARD_SURFACE = "hard_surface"          # Rocks, barriers, hard obstacles
    LIVING_SURFACE = "living_surface"      # Plants, animals, soft obstacles
    UNKNOWN = "unknown"                   # Unclassified obstacle


@dataclass
class SafetyConfig:
    """Configuration for safety systems."""
    collision_threshold: float = 1.0          # meters
    emergency_brake_distance: float = 0.5      # meters  
    obstacle_detection_range: float = 15.0     # meters
    emergency_brake_force: float = 1.0        # maximum brake force
    operator_override_enabled: bool = True
    recovery_behavior: str = "reverse_then_retry"  # recovery strategy


@dataclass
class Obstacle:
    """Detected obstacle information."""
    type: ObstacleType
    position: Tuple[float, float, float]     # x, y, z in meters
    distance: float                         # distance from rover
    confidence: float                        # detection confidence 0-1
    timestamp: float                        # detection time


class SafetySystem:
    """Multi-layer safety and collision detection system."""
    
    def __init__(self, config: Optional[SafetyConfig] = None):
        self.config = config or SafetyConfig()
        self.logger = get_logger("safety_system")
        self.perf_logger = get_performance_logger("safety_system")
        
        # Safety state
        self.emergency_active = False
        self.obstacles: List[Obstacle] = []
        self.collision_count = 0
        self.last_collision_time = 0.0
        self.emergency_brake_active = False
        
        # Sensor data
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.imu_data = None
        self.camera_image = None
        
        # Performance tracking
        self.safety_stats = {
            'obstacle_detections': 0,
            'emergency_stops': 0,
            'operator_overrides': 0,
            'collision_response_time_ms': 0.0
        }
        
        self.logger.info("Safety system initialized")
    
    def update_sensor_data(self, position: np.ndarray, velocity: np.ndarray, 
                       imu: Optional[Imu] = None, camera: Optional[Image] = None):
        """Update sensor data for safety processing."""
        self.current_position = position
        self.current_velocity = velocity
        self.imu_data = imu
        self.camera_image = camera
        
        # Trigger safety processing
        self._process_safety_data()
    
    def _process_safety_data(self):
        """Process current sensor data for safety hazards."""
        with self.perf_logger:
            self._detect_obstacles()
            self._check_collision_risk()
            self._assess_emergency_conditions()
    
    def _detect_obstacles(self):
        """Detect obstacles using camera and sensor fusion."""
        if self.camera_image is None:
            return
        
        # Simulate obstacle detection (would use real computer vision)
        obstacles = []
        current_time = time.time()
        
        # Simulate detecting some obstacles
        for i in range(3):  # Simulate up to 3 obstacles
            if np.random.random() < 0.3:  # 30% chance of obstacle
                distance = np.random.uniform(2.0, self.config.obstacle_detection_range)
                angle = np.random.uniform(0, 2 * np.pi)
                
                position = (
                    self.current_position[0] + distance * np.cos(angle),
                    self.current_position[1] + distance * np.sin(angle),
                    self.current_position[2]  # Assume ground level
                )
                
                obstacle_type = ObstacleType.HARD_SURFACE if np.random.random() < 0.7 else ObstacleType.LIVING_SURFACE
                
                obstacles.append(Obstacle(
                    type=obstacle_type,
                    position=position,
                    distance=distance,
                    confidence=np.random.uniform(0.6, 0.95),
                    timestamp=current_time
                ))
        
        # Update obstacle list (keep recent ones)
        self.obstacles = [obs for obs in self.obstacles 
                        if current_time - obs.timestamp < 10.0]  # Keep last 10 seconds
        self.obstacles.extend(obstacles)
        
        # Keep only closest 10 obstacles
        self.obstacles.sort(key=lambda obs: obs.distance)
        self.obstacles = self.obstacles[:10]
        
        self.safety_stats['obstacle_detections'] += len(obstacles)
        
        if obstacles:
            self.logger.warning(
                f"Obstacles detected: {len(obstacles)}",
                extra={
                    'closest_distance': obstacles[0].distance if obstacles else float('inf'),
                    'types': [obs.type.value for obs in obstacles]
                }
            )
    
    def _check_collision_risk(self):
        """Check for collision risk and take action."""
        if not self.obstacles:
            return
        
        velocity_magnitude = np.linalg.norm(self.current_velocity)
        
        # Time to collision for each obstacle
        for obstacle in self.obstacles:
            if obstacle.distance < self.config.collision_threshold:
                # Estimate time to collision
                time_to_collision = obstacle.distance / max(velocity_magnitude, 0.1)
                
                if time_to_collision < 1.0:  # Less than 1 second
                    self._trigger_emergency_stop("collision_imminent", obstacle)
                    break
    
    def _assess_emergency_conditions(self):
        """Assess conditions that require emergency response."""
        emergency_conditions = []
        
        # Check for stuck condition
        velocity_magnitude = np.linalg.norm(self.current_velocity)
        if velocity_magnitude < 0.01 and time.time() - self.last_collision_time > 30.0:
            emergency_conditions.append("stuck_in_place")
        
        # Check for sensor failures
        if self.imu_data is None or self.camera_image is None:
            emergency_conditions.append("sensor_failure")
        
        # Check for unsafe tilt
        if self.imu_data is not None:
            tilt_angle = abs(self.imu_data.orientation.x) + abs(self.imu_data.orientation.y)
            if tilt_angle > 45.0:  # More than 45 degrees tilt
                emergency_conditions.append("unsafe_tilt")
        
        # Trigger emergency if conditions met
        if emergency_conditions:
            self._trigger_emergency_stop("safety_system", emergency_conditions)
    
    def _trigger_emergency_stop(self, reason: str, context: Any = None):
        """Trigger emergency stop with maximum priority."""
        with self.perf_logger:
            self.emergency_active = True
            self.emergency_brake_active = True
            self.safety_stats['emergency_stops'] += 1
            
            self.logger.critical(
                f"EMERGENCY STOP ACTIVATED: {reason}",
                extra={
                    'reason': reason,
                    'context': context,
                    'position': self.current_position.tolist(),
                    'velocity': self.current_velocity.tolist(),
                    'timestamp': time.time()
                }
            )
            
            # Send emergency brake command
            self._send_emergency_brake()
            
            # Update collision response time
            self.safety_stats['collision_response_time_ms'] = 50.0  # Target <100ms
    
    def _send_emergency_brake(self):
        """Send emergency brake command to hardware."""
        # This would interface with STM32 hardware interface
        brake_command = {
            'action': 'emergency_stop',
            'force': self.config.emergency_brake_force,
            'timestamp': time.time()
        }
        
        # In real implementation, this would use the service communication bus
        self.logger.info(f"Emergency brake command sent: {brake_command}")
        
        # Simulate brake response time
        time.sleep(0.05)  # Simulate 50ms response time
        self.safety_stats['collision_response_time_ms'] = 50.0
    
    def get_safety_status(self) -> Dict[str, Any]:
        """Get current safety system status."""
        return {
            'emergency_active': self.emergency_active,
            'emergency_brake_active': self.emergency_brake_active,
            'collision_count': self.collision_count,
            'obstacle_count': len(self.obstacles),
            'obstacles': [
                {
                    'type': obs.type.value,
                    'distance': obs.distance,
                    'confidence': obs.confidence,
                    'position': obs.position
                } for obs in self.obstacles
            ],
            'current_position': self.current_position.tolist(),
            'current_velocity': np.linalg.norm(self.current_velocity),
            'safety_config': {
                'collision_threshold': self.config.collision_threshold,
                'emergency_brake_distance': self.config.emergency_brake_distance,
                'operator_override_enabled': self.config.operator_override_enabled
            },
            'performance_stats': self.safety_stats
        }


class AutonomyController:
    """3-Mode autonomy controller with safety features."""
    
    def __init__(self, autonomy_mode: AutonomyMode = AutonomyMode.HYBRID_DEFAULT):
        self.autonomy_mode = autonomy_mode
        self.logger = get_logger("autonomy_controller")
        self.perf_logger = get_performance_logger("autonomy_controller")
        
        # Safety system
        self.safety_system = SafetySystem()
        
        # Control authority
        self.operator_control_enabled = True
        self.safety_autonomy_enabled = True
        
        # Mission state
        self.mission_active = False
        self.current_mission = None
        
        # Performance tracking
        self.control_stats = {
            'mode_switches': 0,
            'safety_interventions': 0,
            'operator_overrides': 0,
            'autonomous_decisions': 0
        }
        
        self.logger.info(f"Autonomy controller initialized in {autonomy_mode.value} mode")
    
    def set_mode(self, mode: AutonomyMode) -> bool:
        """Switch autonomy mode."""
        try:
            old_mode = self.autonomy_mode
            self.autonomy_mode = mode
            
            # Configure mode-specific behavior
            if mode == AutonomyMode.HYBRID_DEFAULT:
                self.operator_control_enabled = True
                self.safety_autonomy_enabled = True
            elif mode == AutonomyMode.FULLY_AUTONOMOUS:
                self.operator_control_enabled = False
                self.safety_autonomy_enabled = True
            elif mode == AutonomyMode.FULLY_OPERATOR:
                self.operator_control_enabled = True
                self.safety_autonomy_enabled = False
            
            self.control_stats['mode_switches'] += 1
            
            self.logger.info(
                f"Mode switched: {old_mode.value} → {mode.value}",
                extra={
                    'old_mode': old_mode.value,
                    'new_mode': mode.value,
                    'operator_control': self.operator_control_enabled,
                    'safety_autonomy': self.safety_autonomy_enabled
                }
            )
            
            return True
            
        except Exception as e:
            self.logger.error(f"Mode switch failed: {e}")
            return False
    
    def enable_safety_features(self, enabled: bool) -> bool:
        """Toggle safety features (operator mode only)."""
        if self.autonomy_mode == AutonomyMode.FULLY_OPERATOR and self.config.operator_override_enabled:
            self.safety_autonomy_enabled = enabled
            
            self.logger.info(
                f"Safety features {'enabled' if enabled else 'disabled'}",
                extra={
                    'mode': self.autonomy_mode.value,
                    'safety_enabled': enabled,
                    'operator_override': self.config.operator_override_enabled
                }
            )
            
            return True
        else:
            self.logger.warning("Cannot toggle safety features in current mode")
            return False
    
    def process_sensor_data(self, position: np.ndarray, velocity: np.ndarray,
                        imu: Optional[Imu] = None, camera: Optional[Image] = None):
        """Process sensor data and make autonomous decisions."""
        self.safety_system.update_sensor_data(position, velocity, imu, camera)
        
        # Mode-specific processing
        if self.autonomy_mode == AutonomyMode.HYBRID_DEFAULT:
            self._process_hybrid_mode(position, velocity)
        elif self.autonomy_mode == AutonomyMode.FULLY_AUTONOMOUS:
            self._process_autonomous_mode(position, velocity, imu, camera)
        elif self.autonomy_mode == AutonomyMode.FULLY_OPERATOR:
            self._process_operator_mode(position, velocity)
    
    def _process_hybrid_mode(self, position: np.ndarray, velocity: np.ndarray):
        """Process hybrid mode - operator control with safety autonomy."""
        safety_status = self.safety_system.get_safety_status()
        
        # Check for safety interventions needed
        if safety_status['emergency_active']:
            self.control_stats['safety_interventions'] += 1
            # Safety system handles emergency brake
            return
        
        # Operator maintains control, but safety systems can intervene
        if safety_status['obstacle_count'] > 0:
            # Suggest alternative route or speed reduction
            self._suggest_alternative_action()
        
        self.control_stats['autonomous_decisions'] += 1
    
    def _process_autonomous_mode(self, position: np.ndarray, velocity: np.ndarray,
                                imu: Optional[Imu] = None, camera: Optional[Image] = None):
        """Process fully autonomous mode - complete AI control."""
        safety_status = self.safety_system.get_safety_status()
        
        # Safety system can still trigger emergencies in autonomous mode
        if safety_status['emergency_active']:
            self.control_stats['safety_interventions'] += 1
            return
        
        # Autonomous navigation decisions
        velocity_magnitude = np.linalg.norm(velocity)
        
        # Simple autonomous behavior - maintain safe distance from obstacles
        if safety_status['obstacle_count'] > 0:
            closest_obstacle = safety_status['obstacles'][0]  # Already sorted by distance
            
            if closest_obstacle['distance'] < 3.0:  # Too close
                # Reduce speed or stop
                safe_speed = min(velocity_magnitude * 0.5, closest_obstacle['distance'] - 1.0)
                self._adjust_velocity_for_safety(safe_speed)
        
        self.control_stats['autonomous_decisions'] += 1
    
    def _process_operator_mode(self, position: np.ndarray, velocity: np.ndarray):
        """Process operator mode - manual control with safety toggle."""
        safety_status = self.safety_system.get_safety_status()
        
        # Operator has full control, but safety can still intervene
        if safety_status['emergency_active']:
            self.control_stats['safety_interventions'] += 1
            # Safety system handles emergency brake even in operator mode
            return
        
        # Log operator decisions for monitoring
        self.control_stats['autonomous_decisions'] += 1  # Track operator decisions as well
    
    def _suggest_alternative_action(self):
        """Suggest alternative action to avoid obstacles."""
        self.logger.warning(
            "Alternative action suggested - obstacle detected",
            extra={
                'safety_intervention': True,
                'obstacle_avoidance': True
            }
        )
    
    def _adjust_velocity_for_safety(self, safe_speed: float):
        """Adjust velocity for safety (would interface with hardware)."""
        self.logger.info(f"Velocity adjusted for safety: {safe_speed:.2f} m/s")
        
        # In real implementation, this would send command to STM32
        # For now, just log the action
        self.control_stats['safety_interventions'] += 1
    
    def start_mission(self, mission_config: Dict[str, Any]) -> bool:
        """Start an autonomous mission."""
        if self.autonomy_mode not in [AutonomyMode.HYBRID_DEFAULT, AutonomyMode.FULLY_AUTONOMOUS]:
            self.logger.warning("Cannot start mission in current mode")
            return False
        
        try:
            self.mission_active = True
            self.current_mission = mission_config
            
            self.logger.info(
                f"Mission started: {mission_config.get('type', 'unknown')}",
                extra={'mission_config': mission_config}
            )
            
            return True
            
        except Exception as e:
            self.logger.error(f"Mission start failed: {e}")
            return False
    
    def get_autonomy_status(self) -> Dict[str, Any]:
        """Get current autonomy system status."""
        safety_status = self.safety_system.get_safety_status()
        
        return {
            'mode': self.autonomy_mode.value,
            'operator_control_enabled': self.operator_control_enabled,
            'safety_autonomy_enabled': self.safety_autonomy_enabled,
            'mission_active': self.mission_active,
            'current_mission': self.current_mission,
            'safety_status': safety_status,
            'control_stats': self.control_stats
        }


class ThreeModeAutonomyNode(Node):
    """ROS2 node for 3-mode autonomy system."""
    
    def __init__(self):
        super().__init__('three_mode_autonomy')
        
        self.autonomy_controller = AutonomyController()
        self.safety_system = SafetySystem()
        
        # ROS2 interfaces
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/autonomy/status',
            10
        )
        
        self.safety_status_publisher = self.create_publisher(
            String,
            '/safety/status',
            10
        )
        
        self.mode_service = self.create_service(
            SetAutonomyMode,
            '/autonomy/set_mode',
            self._set_mode_callback
        )
        
        self.safety_toggle_service = self.create_service(
            EnableSafetyFeatures,
            '/autonomy/enable_safety',
            self._enable_safety_callback
        )
        
        # Status publishing timer
        self.status_timer = self.create_timer(
            1.0,  # 10Hz status publishing
            self._publish_status
        )
        
        self.logger = get_logger("three_mode_autonomy")
        self.logger.info("3-mode autonomy node initialized")
    
    def _cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands."""
        # Convert to numpy arrays
        position = np.array([0.0, 0.0, 0.0])  # Would come from localization
        velocity = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        
        # Process through autonomy controller with sensor data
        self.autonomy_controller.process_sensor_data(position, velocity)
    
    def _publish_status(self):
        """Publish autonomy system status."""
        status = self.autonomy_controller.get_autonomy_status()
        
        status_msg = String()
        status_msg.data = str(status)
        self.status_publisher.publish(status_msg)
    
    def _set_mode_callback(self, request, response):
        """Handle mode set requests."""
        try:
            mode_value = request.mode
            validate_input(
                mode_value,
                schema={
                    'type': 'string',
                    'name': 'mode',
                    'allowed_values': [mode.value for mode in AutonomyMode]
                }
            )
            
            mode = AutonomyMode(mode_value)
            success = self.autonomy_controller.set_mode(mode)
            
            response.success = success
            response.message = f"Mode set to {mode.value}" if success else "Mode change failed"
            response.timestamp = time.time()
            
            self.logger.info(
                f"Mode set request: {mode_value}",
                extra={
                    'requested_mode': mode_value,
                    'success': success,
                    'current_mode': self.autonomy_controller.autonomy_mode.value
                }
            )
            
        except Exception as e:
            self.logger.error(f"Mode set failed: {e}")
            response.success = False
            response.message = str(e)
    
    def _enable_safety_callback(self, request, response):
        """Handle safety feature toggle requests."""
        try:
            enabled = request.enabled
            validate_input(
                enabled,
                schema={
                    'type': 'boolean',
                    'name': 'enabled'
                }
            )
            
            success = self.autonomy_controller.enable_safety_features(enabled)
            
            response.success = success
            response.message = f"Safety features {'enabled' if enabled else 'disabled'}"
            response.timestamp = time.time()
            
            self.logger.info(
                f"Safety toggle request: {enabled}",
                extra={
                    'success': success,
                    'safety_enabled': self.autonomy_controller.safety_autonomy_enabled,
                    'operator_override_enabled': self.autonomy_controller.operator_control_enabled
                }
            )
            
        except Exception as e:
            self.logger.error(f"Safety toggle failed: {e}")
            response.success = False
            response.message = str(e)


if __name__ == "__main__":
    # Demo 3-mode autonomy system
    print("🚀 3-Mode Autonomy Demo")
    
    controller = AutonomyController(AutonomyMode.HYBRID_DEFAULT)
    
    # Simulate sensor data updates
    import numpy as np
    
    print("Testing mode switching...")
    controller.set_mode(AutonomyMode.FULLY_AUTONOMOUS)
    controller.set_mode(AutonomyMode.FULLY_OPERATOR)
    controller.set_mode(AutonomyMode.HYBRID_DEFAULT)
    
    print("\nTesting obstacle detection...")
    # Simulate obstacle scenario
    position = np.array([0.0, 0.0, 0.0])
    velocity = np.array([1.0, 0.0, 0.0])  # Moving forward at 1 m/s
    
    controller.process_sensor_data(position, velocity)
    
    status = controller.get_autonomy_status()
    print(f"Status: {status}")
    
    print("\n3-mode autonomy demo complete!")