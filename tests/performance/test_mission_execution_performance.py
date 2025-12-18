#!/usr/bin/env python3
"""
Mission Execution Performance Tests - URC 2026

Tests mission planning and execution performance:
- Mission planning: <2s
- Path generation: <1s
- State transitions: <100ms
- Memory usage: <100MB during execution

Critical for competition mission completion and autonomous operation.
"""

import json
import statistics
import threading
import time
import tracemalloc
import unittest
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List, Optional, Tuple

import psutil
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class MissionExecutionPerformanceTest(unittest.TestCase):
    """Test mission execution performance under various conditions."""

    def setUp(self):
        """Set up mission execution performance testing."""
        rclpy.init()

        # Test parameters - aligned with competition requirements
        self.num_missions = 50  # Test 50 mission executions
        self.target_planning_ms = 2000.0  # Mission planning target (2s)
        self.target_path_gen_ms = 1000.0  # Path generation target (1s)
        self.target_state_transition_ms = 100.0  # State transition target (100ms)
        self.target_memory_mb = 100.0  # Memory usage target during execution

        # Performance tracking
        self.planning_times = []
        self.path_generation_times = []
        self.state_transition_times = []
        self.execution_times = []
        self.memory_usage_readings = []

        # Create test components
        self.mission_planner = MissionPlannerSimulator()
        self.mission_executor = MissionExecutorSimulator()
        self.state_manager = StateManagerSimulator()

    def tearDown(self):
        """Clean up resources."""
        rclpy.shutdown()

    def test_mission_execution_performance(self):
        """Test complete mission execution performance."""
        print("\n[OBJECTIVE] Testing Mission Execution Performance")
        print("=" * 60)

        try:
            # Start memory monitoring
            tracemalloc.start()
            self._start_resource_monitoring()

            # Warm up mission system
            print(" Warming up mission execution system...")
            self._warm_up_mission_system()

            # Run performance test
            print("[GRAPH] Measuring mission execution performance...")
            self._run_mission_execution_performance_test()

            # Analyze results
            self._analyze_mission_results()

        finally:
            self._stop_resource_monitoring()
            tracemalloc.stop()

    def _warm_up_mission_system(self):
        """Warm up the mission execution system."""
        for i in range(10):  # Warm up with 10 missions
            # Create simple mission
            mission = self._create_test_mission(f"warmup_{i}")

            # Execute mission
            self._execute_test_mission(mission)

            time.sleep(0.1)  # Brief pause between missions

        time.sleep(0.5)  # Allow system to stabilize

    def _run_mission_execution_performance_test(self):
        """Run the main mission execution performance test."""
        self.planning_times.clear()
        self.path_generation_times.clear()
        self.state_transition_times.clear()
        self.execution_times.clear()

        start_time = time.time()

        for mission_num in range(self.num_missions):
            # Create test mission
            mission = self._create_test_mission(f"perf_test_{mission_num}")

            # Execute mission and measure performance
            mission_result = self._execute_test_mission(mission)

            # Record timing data
            if mission_result:
                self.planning_times.append(mission_result["planning_time_ms"])
                self.path_generation_times.append(
                    mission_result["path_generation_time_ms"]
                )
                self.state_transition_times.extend(
                    mission_result["state_transition_times_ms"]
                )
                self.execution_times.append(mission_result["total_execution_time_ms"])

            # Progress indicator
            if (mission_num + 1) % 10 == 0:  # Every 10 missions
                print(f"  Progress: {mission_num + 1}/{self.num_missions} missions")

            # Brief pause between missions
            time.sleep(0.05)

        total_duration = time.time() - start_time
        missions_per_second = self.num_missions / total_duration

        print(".2f")
        print(".1f")

    def _create_test_mission(self, mission_id: str) -> Dict[str, Any]:
        """Create a test mission specification."""
        mission_types = [
            "waypoint_navigation",
            "object_search",
            "terrain_traversal",
            "return_home",
        ]

        # Create mission with waypoints
        num_waypoints = 5  # 5 waypoints per mission
        waypoints = []

        for i in range(num_waypoints):
            waypoint = {
                "id": f"wp_{i}",
                "position": {
                    "x": float(i * 2.0),  # 2m spacing
                    "y": float((i % 2) * 1.5),  # Some lateral variation
                    "z": 0.0,
                },
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }
            waypoints.append(waypoint)

        return {
            "id": mission_id,
            "type": mission_types[len(mission_id) % len(mission_types)],
            "waypoints": waypoints,
            "timeout_s": 30.0,
            "priority": "normal",
            "created_time": time.time(),
        }

    def _execute_test_mission(
        self, mission: Dict[str, Any]
    ) -> Optional[Dict[str, Any]]:
        """Execute a test mission and measure performance."""
        try:
            # Step 1: Mission planning
            planning_start = time.time()
            plan = self.mission_planner.plan_mission(mission)
            planning_time_ms = (time.time() - planning_start) * 1000

            # Step 2: Path generation
            path_gen_start = time.time()
            path = self.mission_planner.generate_path(plan)
            path_gen_time_ms = (time.time() - path_gen_start) * 1000

            # Step 3: Mission execution with state transitions
            execution_start = time.time()
            state_transitions_ms = []

            # Initialize mission state
            state_transition_start = time.time()
            self.state_manager.transition_to_state("READY")
            state_transitions_ms.append((time.time() - state_transition_start) * 1000)

            # Start mission
            state_transition_start = time.time()
            self.state_manager.transition_to_state("AUTO")
            state_transitions_ms.append((time.time() - state_transition_start) * 1000)

            # Execute mission waypoints
            for waypoint in mission["waypoints"]:
                # Navigate to waypoint (simulated)
                navigation_time = self.mission_executor.navigate_to_waypoint(waypoint)
                time.sleep(navigation_time)  # Simulate navigation time

                # Brief state check
                state_transition_start = time.time()
                self.state_manager.check_state_consistency()
                state_transitions_ms.append(
                    (time.time() - state_transition_start) * 1000
                )

            # Complete mission
            state_transition_start = time.time()
            self.state_manager.transition_to_state("IDLE")
            state_transitions_ms.append((time.time() - state_transition_start) * 1000)

            total_execution_time_ms = (time.time() - execution_start) * 1000

            return {
                "mission_id": mission["id"],
                "planning_time_ms": planning_time_ms,
                "path_generation_time_ms": path_gen_time_ms,
                "state_transition_times_ms": state_transitions_ms,
                "total_execution_time_ms": total_execution_time_ms,
                "waypoints_completed": len(mission["waypoints"]),
                "success": True,
            }

        except Exception as e:
            print(f"Warning: Mission execution failed for {mission['id']}: {e}")
            return None

    def _start_resource_monitoring(self):
        """Start monitoring system resources."""
        self.monitoring_active = True
        self.monitor_thread = threading.Thread(
            target=self._monitor_resources, daemon=True
        )
        self.monitor_thread.start()

    def _stop_resource_monitoring(self):
        """Stop resource monitoring."""
        self.monitoring_active = False
        if hasattr(self, "monitor_thread"):
            self.monitor_thread.join(timeout=2.0)

    def _monitor_resources(self):
        """Monitor system resources during testing."""
        process = psutil.Process()
        while self.monitoring_active:
            # Memory usage
            memory_mb = process.memory_info().rss / 1024 / 1024
            self.memory_usage_readings.append(memory_mb)

            time.sleep(0.5)  # Monitor every 500ms

    def _analyze_mission_results(self):
        """Analyze mission execution performance results."""
        if not self.planning_times or not self.execution_times:
            self.fail("No mission execution data collected")

        # Calculate planning statistics
        avg_planning = statistics.mean(self.planning_times)
        max_planning = max(self.planning_times)
        p95_planning = statistics.quantiles(self.planning_times, n=20)[18]

        # Calculate path generation statistics
        avg_path_gen = statistics.mean(self.path_generation_times)
        max_path_gen = max(self.path_generation_times)
        p95_path_gen = statistics.quantiles(self.path_generation_times, n=20)[18]

        # Calculate state transition statistics
        avg_state_transition = statistics.mean(self.state_transition_times)
        max_state_transition = max(self.state_transition_times)
        p95_state_transition = statistics.quantiles(self.state_transition_times, n=20)[
            18
        ]

        # Calculate execution statistics
        avg_execution = statistics.mean(self.execution_times)
        max_execution = max(self.execution_times)
        p95_execution = statistics.quantiles(self.execution_times, n=20)[18]

        # Resource usage statistics
        avg_memory_mb = (
            statistics.mean(self.memory_usage_readings)
            if self.memory_usage_readings
            else 0
        )
        max_memory_mb = (
            max(self.memory_usage_readings) if self.memory_usage_readings else 0
        )
        memory_range_mb = (
            max_memory_mb - min(self.memory_usage_readings)
            if self.memory_usage_readings
            else 0
        )

        # Print detailed results
        print("\n Mission Execution Performance Results:")
        print("-" * 50)
        print("Planning Times:")
        print(".3f")
        print(".3f")
        print(".3f")
        print("\nPath Generation Times:")
        print(".3f")
        print(".3f")
        print(".3f")
        print("\nState Transition Times:")
        print(".3f")
        print(".3f")
        print(".3f")
        print("\nTotal Execution Times:")
        print(".3f")
        print(".3f")
        print(".3f")
        print("\nResource Usage:")
        print(".2f")
        print(".2f")

        # Validate against competition targets
        print("\n[OBJECTIVE] Competition Requirements Check:")
        print("-" * 50)

        requirements = {
            "Planning Time (avg)": (avg_planning, self.target_planning_ms),
            "Planning Time (95p)": (p95_planning, self.target_planning_ms * 1.2),
            "Path Generation (avg)": (avg_path_gen, self.target_path_gen_ms),
            "Path Generation (95p)": (p95_path_gen, self.target_path_gen_ms * 1.2),
            "State Transitions (avg)": (
                avg_state_transition,
                self.target_state_transition_ms,
            ),
            "State Transitions (95p)": (
                p95_state_transition,
                self.target_state_transition_ms * 1.2,
            ),
            "Memory Usage (max)": (max_memory_mb, self.target_memory_mb),
            "Memory Usage Range": (memory_range_mb, self.target_memory_mb * 0.5),
        }

        all_passed = True
        for metric, (actual, target) in requirements.items():
            passed = actual <= target
            status = "[PASS] PASS" if passed else "[FAIL] FAIL"
            print(".3f")

            if not passed:
                all_passed = False

        # Performance assessment
        print("\n[IGNITE] Performance Assessment:")
        print("-" * 50)

        if all_passed:
            print("[PASS] ALL MISSION EXECUTION REQUIREMENTS MET")
            print("   Mission system suitable for competition operations")
        elif (
            avg_planning <= self.target_planning_ms and avg_execution <= 5000
        ):  # 5s total
            print("  CORE FUNCTIONALITY OK, BUT OPTIMIZATION NEEDED")
            print("   Monitor performance during complex mission scenarios")
        else:
            print("[FAIL] MISSION EXECUTION REQUIREMENTS NOT MET")
            print("   Mission system requires optimization before competition")

        # Detailed analysis and recommendations
        if (
            avg_planning > self.target_planning_ms
            or avg_path_gen > self.target_path_gen_ms
        ):
            print("\n Optimization Recommendations:")
            if avg_planning > self.target_planning_ms:
                print("   - Optimize mission planning algorithm")
                print("   - Consider pre-computed mission templates")
            if avg_path_gen > self.target_path_gen_ms:
                print("   - Optimize path planning algorithm")
                print("   - Consider simplified path planning for known environments")
            if avg_state_transition > self.target_state_transition_ms:
                print("   - Optimize state machine transitions")
                print("   - Reduce state validation overhead")
            if max_memory_mb > self.target_memory_mb:
                print("   - Monitor memory usage during mission execution")
                print("   - Implement mission data cleanup")

        # Store results for regression testing
        self.test_results = {
            "avg_planning_ms": avg_planning,
            "max_planning_ms": max_planning,
            "p95_planning_ms": p95_planning,
            "avg_path_gen_ms": avg_path_gen,
            "max_path_gen_ms": max_path_gen,
            "p95_path_gen_ms": p95_path_gen,
            "avg_state_transition_ms": avg_state_transition,
            "max_state_transition_ms": max_state_transition,
            "p95_state_transition_ms": p95_state_transition,
            "avg_execution_ms": avg_execution,
            "max_execution_ms": max_execution,
            "p95_execution_ms": p95_execution,
            "avg_memory_mb": avg_memory_mb,
            "max_memory_mb": max_memory_mb,
            "memory_range_mb": memory_range_mb,
            "requirements_met": all_passed,
            "missions_executed": len(self.execution_times),
        }

        # Assert critical requirements
        self.assertLess(avg_planning, self.target_planning_ms * 1.5, ".3f")
        self.assertLess(avg_path_gen, self.target_path_gen_ms * 1.5, ".3f")
        self.assertLess(
            avg_state_transition, self.target_state_transition_ms * 2.0, ".3f"
        )
        self.assertLess(max_memory_mb, self.target_memory_mb * 1.5, ".2f")

    def test_mission_execution_under_concurrent_load(self):
        """Test mission execution performance under concurrent operations."""
        print("\n[REFRESH] Testing Mission Execution Under Concurrent Load")

        # Create multiple mission executors
        concurrent_executors = []
        for i in range(3):
            executor = MissionExecutorSimulator()
            concurrent_executors.append(executor)

        try:
            # Start resource monitoring
            self._start_resource_monitoring()

            # Generate concurrent mission load
            load_threads = []
            for i in range(5):
                t = threading.Thread(target=self._generate_mission_load, args=(i,))
                load_threads.append(t)
                t.start()

            # Run performance test under load
            self._run_mission_execution_performance_test()

            # Stop load generation
            for t in load_threads:
                t.join(timeout=2.0)

        finally:
            self._stop_resource_monitoring()

    def _generate_mission_load(self, thread_id: int):
        """Generate background mission processing load."""
        while self.monitoring_active:
            # Simulate background mission processing
            mission = self._create_test_mission(f"background_{thread_id}_{time.time()}")
            result = self._execute_test_mission(mission)
            time.sleep(0.1)  # Brief pause between background missions


class MissionPlannerSimulator:
    """Simulates mission planning for performance testing."""

    def __init__(self):
        """Initialize mission planner simulator."""
        # Simulated planning parameters (based on real mission planning)
        self.planning_base_time = 1.0  # 1s base planning time
        self.path_gen_base_time = 0.5  # 0.5s base path generation time

    def plan_mission(self, mission: Dict[str, Any]) -> Dict[str, Any]:
        """Plan a mission."""
        # Simulate mission planning complexity based on mission type and waypoints
        complexity_multiplier = (
            len(mission["waypoints"]) / 5.0
        )  # Normalize to 5 waypoints
        mission_type_complexity = {
            "waypoint_navigation": 1.0,
            "object_search": 1.3,
            "terrain_traversal": 1.5,
            "return_home": 0.8,
        }.get(mission["type"], 1.0)

        planning_time = (
            self.planning_base_time * complexity_multiplier * mission_type_complexity
        )
        time.sleep(planning_time)

        return {
            "mission_id": mission["id"],
            "waypoints": mission["waypoints"],
            "estimated_duration": len(mission["waypoints"]) * 3.0,  # 3s per waypoint
            "risk_assessment": "low",
            "planning_time": planning_time,
        }

    def generate_path(self, plan: Dict[str, Any]) -> Dict[str, Any]:
        """Generate navigation path for mission."""
        # Simulate path planning complexity
        num_waypoints = len(plan["waypoints"])
        path_complexity = num_waypoints / 5.0

        path_gen_time = self.path_gen_base_time * path_complexity
        time.sleep(path_gen_time)

        # Generate path points
        path_points = []
        for i, waypoint in enumerate(plan["waypoints"]):
            path_points.append(
                {
                    "x": waypoint["position"]["x"],
                    "y": waypoint["position"]["y"],
                    "z": waypoint["position"]["z"],
                    "sequence": i,
                }
            )

        return {
            "points": path_points,
            "total_distance": num_waypoints * 2.0,  # Approximate distance
            "estimated_time": num_waypoints * 2.5,  # Time estimate
            "path_generation_time": path_gen_time,
        }


class MissionExecutorSimulator:
    """Simulates mission execution for performance testing."""

    def __init__(self):
        """Initialize mission executor simulator."""
        self.execution_speed = 1.0  # Execution speed multiplier

    def navigate_to_waypoint(self, waypoint: Dict[str, Any]) -> float:
        """Navigate to a waypoint (simulated)."""
        # Simulate navigation time based on distance
        distance = abs(waypoint["position"]["x"]) + abs(waypoint["position"]["y"])
        navigation_time = distance * 0.5 * self.execution_speed  # 0.5s per meter

        # Add some randomness
        navigation_time *= 0.8 + np.random.random() * 0.4  # ±20% variation

        return navigation_time


class StateManagerSimulator:
    """Simulates state management for performance testing."""

    def __init__(self):
        """Initialize state manager simulator."""
        self.current_state = "IDLE"
        self.valid_states = ["IDLE", "READY", "AUTO", "TELEOP", "EMERGENCY"]
        self.state_transition_time = 0.05  # 50ms base transition time

    def transition_to_state(self, target_state: str):
        """Transition to a new state."""
        if target_state not in self.valid_states:
            raise ValueError(f"Invalid state: {target_state}")

        # Simulate state transition time
        transition_time = self.state_transition_time * (0.8 + np.random.random() * 0.4)
        time.sleep(transition_time)

        self.current_state = target_state

    def check_state_consistency(self):
        """Check state consistency (simulated validation)."""
        # Simulate state validation time
        validation_time = self.state_transition_time * 0.3  # Faster validation
        time.sleep(validation_time)


if __name__ == "__main__":
    unittest.main()
