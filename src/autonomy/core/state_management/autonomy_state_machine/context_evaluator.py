#!/usr/bin/env python3
"""
Context Evaluator - Monitors system context for adaptive state machine

Evaluates real-time system conditions including battery, mission status,
communication health, system performance, and environmental safety.
"""

import time
from collections import deque
from typing import Any, Dict, List, Optional

import psutil
import rclpy
from autonomy_interfaces.msg import ContextState, NavigationStatus, SystemState
from autonomy_interfaces.srv import GetSystemState
from rclpy.node import Node


class ContextEvaluator:
    """
    Evaluates and monitors system context for adaptive decision making.

    Continuously monitors:
    - Battery levels and power status
    - Mission execution progress
    - Communication health and latency
    - System performance (CPU, memory, temperature)
    - Environmental conditions and safety
    """

    def __init__(self, node: Node) -> None:
        """Initialize context evaluator with ROS2 node reference."""
        self.node = node
        self.logger = node.get_logger()

        # Context thresholds
        self.thresholds = {
            "battery_critical": 10.0,  # %
            "battery_warning": 20.0,  # %
            "obstacle_critical": 0.3,  # meters
            "obstacle_warning": 1.0,  # meters
            "comm_timeout": 10.0,  # seconds
            "cpu_warning": 80.0,  # %
            "memory_warning": 90.0,  # %
            "temperature_warning": 70.0,  # Celsius
            "temperature_critical": 85.0,  # Celsius
        }

        # Context history for pattern analysis
        self.context_history: deque = deque(maxlen=100)
        self.last_evaluation_time: float = time.time()

        # Cached values to avoid redundant queries
        self._cached_battery: Optional[float] = None
        self._cached_mission_status: Optional[Dict[str, Any]] = None
        self._cache_timestamp: float = 0
        self._cache_timeout: float = 0.5  # 500ms cache

        # ROS2 subscriptions for real-time data
        self._setup_subscriptions()

        self.logger.info("Context Evaluator initialized")

    def _setup_subscriptions(self) -> None:
        """Set up ROS2 subscriptions for context data."""
        # System state updates
        self.system_state_sub = self.node.create_subscription(
            SystemState, "/state_machine/system_state", self._system_state_callback, 10
        )

        # Navigation status for obstacle detection
        self.nav_status_sub = self.node.create_subscription(
            NavigationStatus, "/navigation/status", self._navigation_status_callback, 10
        )

        # Mission status updates
        self.mission_status_sub = self.node.create_subscription(
            SystemState, "/mission/status", self._mission_status_callback, 10
        )

    def evaluate_system_context(self) -> ContextState:
        """
        Evaluate complete system context.

        Returns:
            ContextState: Current system context snapshot
        """
        context = ContextState()
        context.timestamp = self.node.get_clock().now().to_msg()

        # Evaluate all context components
        context.battery_level = self._evaluate_battery()
        context.battery_voltage = self._evaluate_battery_voltage()
        context.battery_critical = (
            context.battery_level < self.thresholds["battery_critical"]
        )
        context.battery_warning = (
            context.battery_level < self.thresholds["battery_warning"]
        )

        # Mission context
        mission_data = self._evaluate_mission_status()
        context.mission_type = mission_data.get("type", "NONE")
        context.mission_status = mission_data.get("status", "UNKNOWN")
        context.mission_progress = mission_data.get("progress", 0.0)
        context.mission_time_remaining = mission_data.get("time_remaining", 0.0)

        # Communication context
        comm_data = self._evaluate_communication()
        context.communication_active = comm_data["active"]
        context.communication_latency = comm_data["latency"]
        context.communication_quality = comm_data["quality"]

        # System performance
        perf_data = self._evaluate_system_performance()
        context.cpu_usage = perf_data["cpu"]
        context.memory_usage = perf_data["memory"]
        context.temperature = perf_data["temperature"]

        # Environmental context
        env_data = self._evaluate_environment()
        context.obstacle_detected = env_data["obstacle_detected"]
        context.obstacle_distance = env_data["obstacle_distance"]
        context.terrain_difficulty = env_data["terrain_difficulty"]
        context.weather_adverse = env_data["weather_adverse"]

        # Safety context
        safety_data = self._evaluate_safety()
        context.safety_active = safety_data["active"]
        context.safety_reason = safety_data["reason"]

        # Store in history for pattern analysis
        self.context_history.append({"context": context, "timestamp": time.time()})

        return context

    def _evaluate_battery(self) -> float:
        """Evaluate battery level (0-100)."""
        try:
            # Check cache first
            if (
                self._cached_battery is not None
                and (time.time() - self._cache_timestamp) < self._cache_timeout
            ):
                return self._cached_battery

            # Fallback to system monitoring (if available)
            # In real implementation, this would read from hardware interface
            battery_level = 75.0  # Placeholder - replace with actual battery monitoring

            self._cached_battery = battery_level
            self._cache_timestamp = time.time()
            return battery_level

        except Exception as e:
            self.logger.warning(f"Battery evaluation failed: {e}")
            return 50.0  # Safe default

    def _evaluate_battery_voltage(self) -> float:
        """Evaluate battery voltage."""
        # Placeholder - implement actual voltage monitoring
        return 24.0  # Typical LiPo voltage

    def _evaluate_mission_status(self) -> Dict[str, Any]:
        """Evaluate current mission status."""
        try:
            # Check cache first
            if (
                self._cached_mission_status is not None
                and (time.time() - self._cache_timestamp) < self._cache_timeout
            ):
                return self._cached_mission_status

            # Default mission status
            mission_data = {
                "type": "NONE",
                "status": "IDLE",
                "progress": 0.0,
                "time_remaining": 0.0,
            }

            # Try to get from cached mission status
            if hasattr(self, "_mission_status"):
                mission_data.update(self._mission_status)

            self._cached_mission_status = mission_data
            return mission_data

        except Exception as e:
            self.logger.warning(f"Mission status evaluation failed: {e}")
            return {
                "type": "UNKNOWN",
                "status": "ERROR",
                "progress": 0.0,
                "time_remaining": 0.0,
            }

    def _evaluate_communication(self) -> Dict[str, Any]:
        """Evaluate communication health."""
        try:
            # Check ROS2 communication health
            # This is a simplified implementation
            comm_active = True  # ROS2 is running
            latency = 0.1  # Estimated latency
            quality = 95  # Estimated quality percentage

            return {"active": comm_active, "latency": latency, "quality": quality}

        except Exception as e:
            self.logger.warning(f"Communication evaluation failed: {e}")
            return {"active": False, "latency": 999.0, "quality": 0}

    def _evaluate_system_performance(self) -> Dict[str, float]:
        """Evaluate system performance metrics."""
        try:
            # CPU usage
            cpu_percent = psutil.cpu_percent(interval=0.1)

            # Memory usage
            memory = psutil.virtual_memory()
            memory_percent = memory.percent

            # Temperature (if available)
            try:
                temps = psutil.sensors_temperatures()
                cpu_temp = 0.0
                if "coretemp" in temps:
                    cpu_temp = temps["coretemp"][0].current
                elif "cpu_thermal" in temps:
                    cpu_temp = temps["cpu_thermal"][0].current
                else:
                    cpu_temp = 45.0  # Safe default
            except:
                cpu_temp = 45.0  # Safe default

            return {
                "cpu": cpu_percent,
                "memory": memory_percent,
                "temperature": cpu_temp,
            }

        except Exception as e:
            self.logger.warning(f"System performance evaluation failed: {e}")
            return {"cpu": 50.0, "memory": 50.0, "temperature": 45.0}

    def _evaluate_environment(self) -> Dict[str, Any]:
        """Evaluate environmental conditions."""
        try:
            # Default safe values
            env_data = {
                "obstacle_detected": False,
                "obstacle_distance": 10.0,  # No obstacle
                "terrain_difficulty": 0.2,  # Easy terrain
                "weather_adverse": False,
            }

            # Update with real sensor data if available
            # This would integrate with navigation/obstacle detection systems

            return env_data

        except Exception as e:
            self.logger.warning(f"Environment evaluation failed: {e}")
            return {
                "obstacle_detected": True,  # Conservative default
                "obstacle_distance": 0.5,
                "terrain_difficulty": 0.8,
                "weather_adverse": True,
            }

    def _evaluate_safety(self) -> Dict[str, Any]:
        """Evaluate safety system status."""
        try:
            # Check if any safety conditions are active
            # This would integrate with the safety manager
            safety_active = False
            safety_reason = "NONE"

            # Get current values directly (avoid recursion)
            battery_level = self._evaluate_battery()
            comm_status = self._evaluate_communication()
            env_data = self._evaluate_environment()

            # Check various safety conditions
            if battery_level < self.thresholds["battery_critical"]:
                safety_active = True
                safety_reason = "BATTERY_CRITICAL"
            elif (
                env_data["obstacle_detected"]
                and env_data["obstacle_distance"] < self.thresholds["obstacle_critical"]
            ):
                safety_active = True
                safety_reason = "OBSTACLE_CRITICAL"
            elif not comm_status["active"]:
                safety_active = True
                safety_reason = "COMMUNICATION_LOSS"

            return {"active": safety_active, "reason": safety_reason}

        except Exception as e:
            self.logger.warning(f"Safety evaluation failed: {e}")
            return {"active": True, "reason": "EVALUATION_FAILED"}

    def get_context_patterns(self) -> Dict[str, Any]:
        """Analyze context patterns for predictive adaptation."""
        if len(self.context_history) < 10:
            return {}

        try:
            # Analyze recent context patterns
            recent_contexts = list(self.context_history)[-20:]  # Last 20 readings

            patterns = {
                "battery_trend": self._analyze_battery_trend(recent_contexts),
                "performance_trend": self._analyze_performance_trend(recent_contexts),
                "safety_frequency": self._analyze_safety_frequency(recent_contexts),
                "adaptation_effectiveness": self._analyze_adaptation_effectiveness(
                    recent_contexts
                ),
            }

            return patterns

        except Exception as e:
            self.logger.warning(f"Pattern analysis failed: {e}")
            return {}

    def _analyze_battery_trend(self, contexts: List[Dict[str, Any]]) -> str:
        """Analyze battery usage trends."""
        if len(contexts) < 5:
            return "UNKNOWN"

        battery_levels = [ctx["context"].battery_level for ctx in contexts]
        trend = self._calculate_trend(battery_levels)

        if trend < -2.0:  # Dropping fast
            return "RAPID_DECLINE"
        elif trend < -0.5:  # Dropping
            return "DECLINING"
        elif trend > 0.5:  # Rising
            return "RECOVERING"
        else:
            return "STABLE"

    def _analyze_performance_trend(self, contexts: List[Dict[str, Any]]) -> str:
        """Analyze system performance trends."""
        cpu_usage = [ctx["context"].cpu_usage for ctx in contexts]
        cpu_trend = self._calculate_trend(cpu_usage)

        if cpu_trend > 5.0:
            return "DEGRADING"
        elif cpu_trend < -5.0:
            return "IMPROVING"
        else:
            return "STABLE"

    def _analyze_safety_frequency(self, contexts: List[Dict[str, Any]]) -> float:
        """Calculate frequency of safety activations."""
        safety_count = sum(1 for ctx in contexts if ctx["context"].safety_active)
        return safety_count / len(contexts)

    def _analyze_adaptation_effectiveness(
        self, contexts: List[Dict[str, Any]]
    ) -> float:
        """Analyze how well adaptations are working."""
        # This would correlate adaptation actions with improved outcomes
        # Placeholder implementation
        return 0.8  # 80% effectiveness

    def _calculate_trend(self, values: List[float]) -> float:
        """Calculate linear trend in values."""
        if len(values) < 2:
            return 0.0

        n = len(values)
        x = list(range(n))
        y = values

        # Simple linear regression slope
        sum_x = sum(x)
        sum_y = sum(y)
        sum_xy = sum(xi * yi for xi, yi in zip(x, y))
        sum_xx = sum(xi * xi for xi in x)

        slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x)
        return slope

    # ROS2 Callbacks
    def _system_state_callback(self, msg: SystemState) -> None:
        """Handle system state updates."""
        # Update cached system state
        pass

    def _navigation_status_callback(self, msg: NavigationStatus) -> None:
        """Handle navigation status updates."""
        # Update obstacle detection status
        pass

    def _mission_status_callback(self, msg: SystemState) -> None:
        """Handle mission status updates."""
        # Update cached mission status
        self._mission_status = {
            "type": getattr(msg, "mission_type", "UNKNOWN"),
            "status": getattr(msg, "mission_status", "UNKNOWN"),
            "progress": getattr(msg, "mission_progress", 0.0),
            "time_remaining": getattr(msg, "time_remaining", 0.0),
        }
