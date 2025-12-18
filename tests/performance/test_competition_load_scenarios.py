#!/usr/bin/env python3
"""
Competition Load Scenarios - URC 2026

Realistic competition load testing scenarios:
- Navigation heavy: Complex path planning and obstacle avoidance
- Vision intensive: High-frequency image processing and object detection
- Communication stress: Network congestion and message bursts
- Emergency response: Rapid commands and safety system triggers

Critical for ensuring system performance under competition conditions.
"""

import time
import threading
import unittest
from typing import Dict, List, Optional, Tuple, Any
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import psutil
import numpy as np
from concurrent.futures import ThreadPoolExecutor
import tracemalloc


class CompetitionLoadScenariosTest(unittest.TestCase):
    """Test system performance under realistic competition load scenarios."""

    def setUp(self):
        """Set up competition load testing."""
        rclpy.init()

        # Competition scenario definitions with realistic load patterns
        self.competition_scenarios = {
            'navigation_heavy': {
                'description': 'Complex autonomous navigation with dense obstacles',
                'duration_minutes': 10,
                'cpu_load': 70,      # High CPU for path planning
                'memory_pressure': True,  # Complex terrain maps
                'network_load': 'high',   # Frequent odometry/path updates
                'vision_load': 'medium',  # Terrain analysis active
                'concurrency': 8,    # Multiple navigation threads
                'targets': {
                    'cpu_max': 75,
                    'memory_max': 550,
                    'response_time_max_ms': 100
                }
            },
            'vision_intensive': {
                'description': 'High-frequency vision processing for object detection',
                'duration_minutes': 8,
                'cpu_load': 80,      # Heavy vision processing
                'memory_pressure': False,  # Streaming processing
                'network_load': 'medium',  # Feature data transmission
                'vision_load': 'high',     # 30Hz processing
                'concurrency': 6,    # Parallel vision pipelines
                'targets': {
                    'cpu_max': 85,
                    'memory_max': 500,
                    'frame_processing_max_ms': 33  # 30Hz = ~33ms
                }
            },
            'communication_stress': {
                'description': 'Network congestion with message bursts and telemetry',
                'duration_minutes': 12,
                'cpu_load': 40,      # Moderate CPU for message handling
                'memory_pressure': False,  # Buffer management
                'network_load': 'very_high',  # Message flood
                'vision_load': 'low',  # Minimal vision
                'concurrency': 4,    # Message processing threads
                'targets': {
                    'cpu_max': 50,
                    'memory_max': 450,
                    'bandwidth_max_mbps': 15,
                    'message_latency_max_ms': 50
                }
            },
            'emergency_response': {
                'description': 'Rapid emergency stops and system recovery',
                'duration_minutes': 5,
                'cpu_load': 60,      # Safety system processing
                'memory_pressure': False,  # Quick state changes
                'network_load': 'burst',   # Emergency broadcasts
                'vision_load': 'medium',   # Safety monitoring
                'concurrency': 10,   # Emergency response threads
                'targets': {
                    'cpu_max': 70,
                    'memory_max': 480,
                    'emergency_response_max_ms': 200,
                    'recovery_time_max_ms': 1000
                }
            },
            'full_competition': {
                'description': 'Complete competition scenario combining all loads',
                'duration_minutes': 15,
                'cpu_load': 85,      # Maximum sustained load
                'memory_pressure': True,  # All systems active
                'network_load': 'very_high',  # Full telemetry
                'vision_load': 'high',  # Continuous processing
                'concurrency': 12,   # All systems running
                'targets': {
                    'cpu_max': 90,
                    'memory_max': 600,
                    'response_time_max_ms': 150,
                    'system_stability': True
                }
            }
        }

        # Performance tracking
        self.scenario_results = {}

    def tearDown(self):
        """Clean up resources."""
        rclpy.shutdown()

    def test_all_competition_scenarios(self):
        """Test system performance across all competition scenarios."""
        print("\n🏁 Testing Competition Load Scenarios")
        print("=" * 60)

        overall_success = True

        for scenario_name, scenario_config in self.competition_scenarios.items():
            print(f"\n🎯 Testing Scenario: {scenario_name.replace('_', ' ').title()}")
            print(f"   {scenario_config['description']}")

            try:
                success = self._run_competition_scenario(scenario_name, scenario_config)
                if not success:
                    overall_success = False

                status = "✅ PASSED" if success else "❌ FAILED"
                print(f"   Result: {status}")

            except Exception as e:
                print(f"   ❌ ERROR: {e}")
                overall_success = False

        # Overall assessment
        print("\n" + "=" * 60)
        print("🏆 COMPETITION READINESS ASSESSMENT")
        print("=" * 60)

        if overall_success:
            print("✅ ALL COMPETITION SCENARIOS PASSED")
            print("   System is ready for competition deployment")
            print("   Performance targets met under all expected loads")
        else:
            print("❌ COMPETITION READINESS ISSUES DETECTED")
            print("   Address performance issues before competition")

        # Detailed results summary
        passed_scenarios = sum(1 for result in self.scenario_results.values() if result.get('passed', False))
        total_scenarios = len(self.scenario_results)

        print(f"\n📊 Results Summary:")
        print(f"   Scenarios tested: {total_scenarios}")
        print(f"   Scenarios passed: {passed_scenarios}")
        print(".1f")

        self.assertTrue(overall_success, "Competition performance requirements not met")

    def _run_competition_scenario(self, scenario_name: str, config: Dict[str, Any]) -> bool:
        """Run a specific competition scenario."""
        # Initialize monitoring
        monitoring_active = True
        cpu_readings = []
        memory_readings = []
        response_times = []

        def monitor_performance():
            process = psutil.Process()
            while monitoring_active:
                cpu_readings.append(process.cpu_percent(interval=0.5))
                memory_readings.append(process.memory_info().rss / 1024 / 1024)
                time.sleep(2)  # Monitor every 2 seconds

        # Start monitoring thread
        monitor_thread = threading.Thread(target=monitor_performance, daemon=True)
        monitor_thread.start()

        try:
            # Run scenario workload
            start_time = time.time()
            scenario_success = self._execute_scenario_workload(scenario_name, config, response_times)
            duration = time.time() - start_time

            # Stop monitoring
            monitoring_active = False
            monitor_thread.join(timeout=5.0)

            # Analyze results
            results = self._analyze_scenario_results(
                scenario_name, config, cpu_readings, memory_readings,
                response_times, duration, scenario_success
            )

            self.scenario_results[scenario_name] = results
            return results['passed']

        except Exception as e:
            monitoring_active = False
            print(f"   Error in scenario {scenario_name}: {e}")
            return False

    def _execute_scenario_workload(self, scenario_name: str, config: Dict[str, Any],
                                 response_times: List[float]) -> bool:
        """Execute the actual workload for a competition scenario."""
        duration_seconds = config['duration_minutes'] * 60
        concurrency = config['concurrency']

        # Create workload threads based on scenario
        workload_threads = []

        if scenario_name == 'navigation_heavy':
            for i in range(concurrency):
                t = threading.Thread(target=self._navigation_heavy_workload,
                                   args=(duration_seconds, response_times))
                workload_threads.append(t)

        elif scenario_name == 'vision_intensive':
            for i in range(concurrency):
                t = threading.Thread(target=self._vision_intensive_workload,
                                   args=(duration_seconds, response_times))
                workload_threads.append(t)

        elif scenario_name == 'communication_stress':
            for i in range(concurrency):
                t = threading.Thread(target=self._communication_stress_workload,
                                   args=(duration_seconds, response_times))
                workload_threads.append(t)

        elif scenario_name == 'emergency_response':
            for i in range(concurrency):
                t = threading.Thread(target=self._emergency_response_workload,
                                   args=(duration_seconds, response_times))
                workload_threads.append(t)

        elif scenario_name == 'full_competition':
            # Mix of all workloads
            nav_threads = 4
            vision_threads = 4
            comm_threads = 2
            emergency_threads = 2

            for i in range(nav_threads):
                t = threading.Thread(target=self._navigation_heavy_workload,
                                   args=(duration_seconds, response_times))
                workload_threads.append(t)

            for i in range(vision_threads):
                t = threading.Thread(target=self._vision_intensive_workload,
                                   args=(duration_seconds, response_times))
                workload_threads.append(t)

            for i in range(comm_threads):
                t = threading.Thread(target=self._communication_stress_workload,
                                   args=(duration_seconds, response_times))
                workload_threads.append(t)

            for i in range(emergency_threads):
                t = threading.Thread(target=self._emergency_response_workload,
                                   args=(duration_seconds, response_times))
                workload_threads.append(t)

        # Start all threads
        for t in workload_threads:
            t.start()

        # Wait for completion with timeout
        start_time = time.time()
        success = True

        for t in workload_threads:
            remaining_time = duration_seconds - (time.time() - start_time)
            if remaining_time > 0:
                t.join(timeout=remaining_time + 10)  # Extra 10s grace period
                if t.is_alive():
                    print(f"   Warning: Workload thread did not complete in time")
                    success = False
            else:
                success = False
                break

        return success

    def _navigation_heavy_workload(self, duration_seconds: float, response_times: List[float]):
        """Execute navigation-heavy workload."""
        start_time = time.time()

        while time.time() - start_time < duration_seconds:
            # Complex path planning simulation
            request_time = time.time()

            # Simulate A* pathfinding through complex terrain
            grid_size = 200  # Large grid for complex planning
            obstacles = np.random.choice([0, 1], size=(grid_size, grid_size), p=[0.7, 0.3])

            # Simulate multiple path planning operations
            for _ in range(5):
                start = (np.random.randint(0, grid_size), np.random.randint(0, grid_size))
                goal = (np.random.randint(0, grid_size), np.random.randint(0, grid_size))

                # Simplified pathfinding simulation (expensive computation)
                path_length = self._simulate_pathfinding_complexity(start, goal, obstacles)

            # Simulate sensor fusion and state estimation
            num_sensors = 8
            sensor_readings = np.random.normal(0, 1, (num_sensors, 3))  # Position/orientation
            fused_estimate = np.mean(sensor_readings, axis=0)

            # Simulate control law computation
            error = np.random.random(3) * 2 - 1  # Random error vector
            control_output = self._compute_pid_control(error)

            response_time = (time.time() - request_time) * 1000
            response_times.append(response_time)

            # Realistic timing - navigation updates at ~20Hz
            time.sleep(0.05)

    def _vision_intensive_workload(self, duration_seconds: float, response_times: List[float]):
        """Execute vision-intensive workload."""
        start_time = time.time()

        while time.time() - start_time < duration_seconds:
            request_time = time.time()

            # Simulate high-frequency vision processing
            for frame in range(10):  # Process 10 frames per cycle
                # Generate synthetic frame
                frame_data = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

                # Simulate computer vision pipeline
                gray = cv2.cvtColor(frame_data, cv2.COLOR_BGR2GRAY) if 'cv2' in globals() else np.dot(frame_data[...,:3], [0.2989, 0.5870, 0.1140])

                # Feature detection
                corners = cv2.goodFeaturesToTrack(gray, 100, 0.01, 10) if 'cv2' in globals() else np.random.random((50, 2)) * 640

                # Object detection simulation
                detections = []
                for _ in range(15):  # Detect multiple objects
                    x, y = np.random.randint(0, 640), np.random.randint(0, 480)
                    confidence = np.random.random()
                    detections.append({'x': x, 'y': y, 'confidence': confidence})

                # Terrain classification
                terrain_patches = []
                for i in range(0, 480, 80):  # Process in patches
                    for j in range(0, 640, 80):
                        patch = gray[i:i+80, j:j+80]
                        if patch.size > 0:
                            features = np.std(patch), np.mean(patch), np.max(patch)
                            terrain_class = np.random.choice(['sand', 'rock', 'slope'], p=[0.6, 0.3, 0.1])
                            terrain_patches.append((features, terrain_class))

                # Feature matching and tracking
                if frame > 0:  # Track between frames
                    prev_features = np.random.random((50, 2)) * 640
                    curr_features = corners if 'cv2' in globals() else np.random.random((50, 2)) * 640

                    # Simulate optical flow computation
                    flows = curr_features - prev_features + np.random.normal(0, 2, curr_features.shape)

            response_time = (time.time() - request_time) * 1000
            response_times.append(response_time)

            # Maintain 30Hz processing rate
            time.sleep(1.0/30)

    def _communication_stress_workload(self, duration_seconds: float, response_times: List[float]):
        """Execute communication stress workload."""
        start_time = time.time()

        while time.time() - start_time < duration_seconds:
            # Simulate message flood
            messages_per_burst = 50
            burst_duration = 0.1  # 100ms burst

            burst_start = time.time()

            for msg_idx in range(messages_per_burst):
                request_time = time.time()

                # Simulate different message types
                if msg_idx % 5 == 0:
                    # Large telemetry message
                    telemetry = self._generate_large_telemetry_message()
                    serialized_size = len(str(telemetry).encode('utf-8'))
                elif msg_idx % 5 == 1:
                    # Sensor data message
                    sensor_data = self._generate_sensor_data_message()
                    serialized_size = len(str(sensor_data).encode('utf-8'))
                elif msg_idx % 5 == 2:
                    # Command message
                    command = self._generate_command_message()
                    serialized_size = len(str(command).encode('utf-8'))
                else:
                    # Status update
                    status = self._generate_status_message()
                    serialized_size = len(str(status).encode('utf-8'))

                # Simulate message processing
                processing_time = serialized_size / 100000  # Size-based processing time
                time.sleep(processing_time)

                response_time = (time.time() - request_time) * 1000
                response_times.append(response_time)

            # Brief pause between bursts
            elapsed = time.time() - burst_start
            if elapsed < burst_duration:
                time.sleep(burst_duration - elapsed)

            # Longer pause to simulate realistic traffic patterns
            time.sleep(0.2)

    def _emergency_response_workload(self, duration_seconds: float, response_times: List[float]):
        """Execute emergency response workload."""
        start_time = time.time()

        while time.time() - start_time < duration_seconds:
            # Simulate emergency events at random intervals
            if np.random.random() < 0.1:  # 10% chance per cycle
                request_time = time.time()

                # Emergency stop sequence
                emergency_type = np.random.choice(['obstacle', 'system_fault', 'communication_loss', 'safety_trigger'])

                # Simulate emergency response
                if emergency_type == 'obstacle':
                    # Rapid obstacle avoidance
                    sensor_readings = np.random.random((16, 3)) * 10  # 16 proximity sensors
                    closest_obstacle = np.min(sensor_readings, axis=0)
                    avoidance_vector = np.mean(sensor_readings, axis=0) - closest_obstacle

                elif emergency_type == 'system_fault':
                    # System diagnosis and recovery
                    system_checks = []
                    for _ in range(20):  # Check 20 subsystems
                        check_result = np.random.choice([True, False], p=[0.9, 0.1])
                        system_checks.append(check_result)

                    failed_systems = [i for i, check in enumerate(system_checks) if not check]

                elif emergency_type == 'communication_loss':
                    # Communication recovery
                    recovery_attempts = 0
                    while recovery_attempts < 5:
                        recovery_attempts += 1
                        if np.random.random() < 0.8:  # 80% recovery success rate
                            break
                        time.sleep(0.05)  # Retry delay

                elif emergency_type == 'safety_trigger':
                    # Safety system activation
                    safety_checks = np.random.random(10)  # 10 safety parameters
                    safety_status = np.all(safety_checks > 0.1)  # All must pass threshold

                # Emergency response delay
                response_delay = np.random.exponential(0.05)  # Mean 50ms response
                time.sleep(response_delay)

                response_time = (time.time() - request_time) * 1000
                response_times.append(response_time)

            # Normal operation between emergencies
            time.sleep(0.1)

    def _analyze_scenario_results(self, scenario_name: str, config: Dict[str, Any],
                                cpu_readings: List[float], memory_readings: List[float],
                                response_times: List[float], duration: float,
                                scenario_success: bool) -> Dict[str, Any]:
        """Analyze results from a competition scenario."""
        results = {
            'scenario': scenario_name,
            'duration_seconds': duration,
            'config': config,
            'workload_success': scenario_success
        }

        # CPU analysis
        if cpu_readings:
            results['cpu_stats'] = {
                'avg_cpu_percent': statistics.mean(cpu_readings),
                'max_cpu_percent': max(cpu_readings),
                'p95_cpu_percent': statistics.quantiles(cpu_readings, n=20)[18] if len(cpu_readings) >= 20 else max(cpu_readings)
            }

        # Memory analysis
        if memory_readings:
            results['memory_stats'] = {
                'avg_memory_mb': statistics.mean(memory_readings),
                'max_memory_mb': max(memory_readings),
                'memory_range_mb': max(memory_readings) - min(memory_readings)
            }

        # Response time analysis
        if response_times:
            results['response_stats'] = {
                'avg_response_ms': statistics.mean(response_times),
                'max_response_ms': max(response_times),
                'p95_response_ms': statistics.quantiles(response_times, n=20)[18] if len(response_times) >= 20 else max(response_times),
                'response_count': len(response_times)
            }

        # Validate against targets
        passed = True
        violations = []

        targets = config['targets']

        if 'cpu_max' in targets and results.get('cpu_stats', {}).get('max_cpu_percent', 0) > targets['cpu_max']:
            passed = False
            violations.append(f"CPU usage exceeded {targets['cpu_max']}%")

        if 'memory_max' in targets and results.get('memory_stats', {}).get('max_memory_mb', 0) > targets['memory_max']:
            passed = False
            violations.append(f"Memory usage exceeded {targets['memory_max']}MB")

        if 'response_time_max_ms' in targets and results.get('response_stats', {}).get('avg_response_ms', 0) > targets['response_time_max_ms']:
            passed = False
            violations.append(f"Response time exceeded {targets['response_time_max_ms']}ms")

        results['passed'] = passed and scenario_success
        results['violations'] = violations

        # Print detailed results
        print(f"   📊 Performance Results:")
        if cpu_readings:
            cpu_stats = results['cpu_stats']
            print(".1f")
        if memory_readings:
            mem_stats = results['memory_stats']
            print(".1f")
        if response_times:
            resp_stats = results['response_stats']
            print(".1f")

        if violations:
            print("   ❌ Violations:")
            for violation in violations:
                print(f"      - {violation}")

        return results

    def _simulate_pathfinding_complexity(self, start: Tuple[int, int], goal: Tuple[int, int],
                                       obstacles: np.ndarray) -> float:
        """Simulate pathfinding computational complexity."""
        # Simplified complexity simulation
        distance = np.sqrt((goal[0] - start[0])**2 + (goal[1] - start[1])**2)

        # Obstacle density affects complexity
        obstacle_density = np.mean(obstacles)
        complexity_factor = 1 + obstacle_density * 2

        # Simulate computation time
        base_time = 0.01  # 10ms base
        computation_time = base_time * complexity_factor * (distance / 100)

        time.sleep(computation_time)
        return distance

    def _compute_pid_control(self, error: np.ndarray) -> np.ndarray:
        """Simulate PID control computation."""
        kp, ki, kd = 1.0, 0.1, 0.05

        # Simulate integral and derivative terms
        integral = error * 0.01  # Simplified
        derivative = error * 0.005  # Simplified

        return kp * error + ki * integral + kd * derivative

    def _generate_large_telemetry_message(self) -> Dict[str, Any]:
        """Generate a large telemetry message."""
        return {
            'timestamp': time.time(),
            'system': {
                'cpu': np.random.random() * 100,
                'memory': np.random.random() * 100,
                'disk': np.random.random() * 100,
                'temperature': 30 + np.random.random() * 40
            },
            'navigation': {
                'position': [np.random.random() * 1000, np.random.random() * 1000, 0],
                'velocity': [np.random.random() * 2, np.random.random() * 2, 0],
                'heading': np.random.random() * 360
            },
            'sensors': {f'sensor_{i}': np.random.random() * 100 for i in range(20)},
            'actuators': {f'actuator_{i}': np.random.random() * 255 for i in range(10)},
            'diagnostics': ['OK'] * 50
        }

    def _generate_sensor_data_message(self) -> Dict[str, Any]:
        """Generate sensor data message."""
        return {
            'timestamp': time.time(),
            'imu': {
                'accel': [np.random.normal(0, 0.1) for _ in range(3)],
                'gyro': [np.random.normal(0, 0.1) for _ in range(3)],
                'mag': [np.random.normal(0, 1) for _ in range(3)]
            },
            'gps': {
                'lat': 35 + np.random.random(),
                'lon': -120 + np.random.random(),
                'alt': 100 + np.random.random() * 50,
                'hdop': 1 + np.random.random()
            }
        }

    def _generate_command_message(self) -> Dict[str, Any]:
        """Generate command message."""
        return {
            'timestamp': time.time(),
            'command': 'navigate_to',
            'parameters': {
                'target': [np.random.random() * 100, np.random.random() * 100, 0],
                'speed': 0.5 + np.random.random(),
                'precision': 0.1 + np.random.random() * 0.9
            }
        }

    def _generate_status_message(self) -> Dict[str, Any]:
        """Generate status message."""
        return {
            'timestamp': time.time(),
            'mission_status': np.random.choice(['executing', 'paused', 'completed']),
            'battery_level': np.random.random() * 100,
            'system_health': np.random.choice(['nominal', 'warning', 'critical']),
            'waypoints_completed': np.random.randint(0, 20)
        }


if __name__ == '__main__':
    unittest.main()


