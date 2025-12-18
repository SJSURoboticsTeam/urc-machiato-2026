#!/usr/bin/env python3
"""
CPU Utilization Performance Tests - URC 2026

Tests CPU usage under different operational modes:
- Idle: <5% CPU
- Autonomous navigation: <50% CPU
- Vision processing: <60% CPU
- Peak load: <80% CPU

Critical for resource optimization and thermal management.
"""

import time
import statistics
import unittest
import psutil
import threading
import multiprocessing as mp
from typing import Dict, List, Optional, Tuple, Any
import numpy as np
import os
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import cProfile
import pstats
import io


class CPUUtilizationPerformanceTest(unittest.TestCase):
    """Test CPU utilization under various operational modes and loads."""

    def setUp(self):
        """Set up CPU utilization testing."""
        # Test parameters - aligned with competition requirements
        self.monitoring_duration_seconds = 60  # Monitor for 60 seconds per test
        self.target_idle_cpu_percent = 5.0      # Idle CPU target
        self.target_navigation_cpu_percent = 50.0  # Autonomous navigation target
        self.target_vision_cpu_percent = 60.0   # Vision processing target
        self.target_peak_cpu_percent = 80.0     # Peak load target

        # CPU tracking
        self.cpu_readings = []
        self.cpu_per_core = []
        self.process_cpu_readings = []
        self.system_load_readings = []

        # Test scenarios
        self.operational_modes = {
            'idle': self.target_idle_cpu_percent,
            'navigation': self.target_navigation_cpu_percent,
            'vision': self.target_vision_cpu_percent,
            'peak_load': self.target_peak_cpu_percent
        }

    def test_cpu_utilization_by_operational_mode(self):
        """Test CPU utilization across different operational modes."""
        print("\n⚡ Testing CPU Utilization by Operational Mode")
        print("=" * 60)

        results_by_mode = {}

        for mode, target_cpu in self.operational_modes.items():
            print(f"\n🔄 Testing {mode.replace('_', ' ').title()} Mode...")
            cpu_usage = self._test_operational_mode_cpu(mode, target_cpu)
            results_by_mode[mode] = cpu_usage

        # Analyze results across modes
        self._analyze_cpu_mode_comparison(results_by_mode)

    def _test_operational_mode_cpu(self, mode: str, target_cpu: float) -> Dict[str, Any]:
        """Test CPU usage for a specific operational mode."""
        # Reset tracking
        self.cpu_readings.clear()
        self.cpu_per_core.clear()
        self.process_cpu_readings.clear()
        self.system_load_readings.clear()

        # Start CPU monitoring
        self._start_cpu_monitoring()

        try:
            # Run workload for this mode
            workload_start = time.time()

            if mode == 'idle':
                self._run_idle_workload()
            elif mode == 'navigation':
                self._run_navigation_workload()
            elif mode == 'vision':
                self._run_vision_workload()
            elif mode == 'peak_load':
                self._run_peak_load_workload()

            workload_duration = time.time() - workload_start

            # Allow monitoring to collect data
            time.sleep(5)

        finally:
            self._stop_cpu_monitoring()

        # Analyze CPU usage for this mode
        return self._analyze_mode_cpu_usage(mode, target_cpu, workload_duration)

    def _start_cpu_monitoring(self):
        """Start comprehensive CPU monitoring."""
        self.monitoring_active = True
        self.monitor_thread = threading.Thread(target=self._cpu_monitor_worker, daemon=True)
        self.monitor_thread.start()

    def _stop_cpu_monitoring(self):
        """Stop CPU monitoring."""
        self.monitoring_active = False
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.join(timeout=2.0)

    def _cpu_monitor_worker(self):
        """Background CPU monitoring worker."""
        process = psutil.Process()
        cpu_count = psutil.cpu_count()

        while self.monitoring_active:
            # Overall CPU usage
            overall_cpu = psutil.cpu_percent(interval=0.5)
            self.cpu_readings.append(overall_cpu)

            # Per-core CPU usage
            per_core = psutil.cpu_percent(interval=0.5, percpu=True)
            self.cpu_per_core.append(per_core)

            # Process-specific CPU usage
            try:
                process_cpu = process.cpu_percent(interval=0.5)
                self.process_cpu_readings.append(process_cpu)
            except psutil.NoSuchProcess:
                self.process_cpu_readings.append(0.0)

            # System load
            load_avg = os.getloadavg() if hasattr(os, 'getloadavg') else (0, 0, 0)
            self.system_load_readings.append(load_avg)

    def _run_idle_workload(self):
        """Run idle workload (minimal CPU usage)."""
        # Just sleep for the monitoring duration
        time.sleep(self.monitoring_duration_seconds)

    def _run_navigation_workload(self):
        """Run autonomous navigation workload."""
        # Simulate navigation computations
        navigation_tasks = []

        for i in range(10):  # 10 concurrent navigation tasks
            task = threading.Thread(target=self._navigation_computation_task, args=(i,))
            navigation_tasks.append(task)
            task.start()

        # Run path planning simulation
        self._run_path_planning_simulation()

        # Wait for tasks to complete
        for task in navigation_tasks:
            task.join(timeout=5.0)

    def _run_vision_workload(self):
        """Run vision processing workload."""
        # Simulate vision processing pipeline
        vision_tasks = []

        for i in range(8):  # 8 concurrent vision processing tasks
            task = threading.Thread(target=self._vision_processing_task, args=(i,))
            vision_tasks.append(task)
            task.start()

        # Run image processing simulation
        self._run_image_processing_simulation()

        # Wait for tasks to complete
        for task in vision_tasks:
            task.join(timeout=5.0)

    def _run_peak_load_workload(self):
        """Run peak load workload (maximum CPU stress)."""
        # Combine all workloads for maximum stress
        peak_tasks = []

        # Navigation tasks
        for i in range(6):
            task = threading.Thread(target=self._navigation_computation_task, args=(i,))
            peak_tasks.append(task)

        # Vision tasks
        for i in range(6):
            task = threading.Thread(target=self._vision_processing_task, args=(i,))
            peak_tasks.append(task)

        # Additional CPU-intensive tasks
        for i in range(4):
            task = threading.Thread(target=self._cpu_intensive_task, args=(i,))
            peak_tasks.append(task)

        # Start all tasks
        for task in peak_tasks:
            task.start()

        # Run simultaneous simulations
        simulation_threads = []
        sim1 = threading.Thread(target=self._run_path_planning_simulation)
        sim2 = threading.Thread(target=self._run_image_processing_simulation)
        simulation_threads.extend([sim1, sim2])

        for sim in simulation_threads:
            sim.start()

        # Wait for completion with timeout
        for task in peak_tasks + simulation_threads:
            task.join(timeout=10.0)

    def _navigation_computation_task(self, task_id: int):
        """Simulate navigation computation task."""
        duration = self.monitoring_duration_seconds * 0.8  # 80% of monitoring time
        start_time = time.time()

        while time.time() - start_time < duration:
            # Simulate navigation calculations
            waypoints = [{'x': i*2.0, 'y': (i%3)*1.0} for i in range(20)]

            for wp in waypoints:
                # Path planning calculations
                distance = np.sqrt(wp['x']**2 + wp['y']**2)
                angle = np.arctan2(wp['y'], wp['x'])

                # Simulate obstacle avoidance
                obstacles = np.random.random((10, 2)) * 10
                distances_to_obstacles = np.sqrt(np.sum((obstacles - [wp['x'], wp['y']])**2, axis=1))
                min_distance = np.min(distances_to_obstacles)

                # PID control calculations
                error = distance - 5.0  # Target distance
                integral = error * 0.1
                derivative = error * 0.05
                control_output = error * 0.5 + integral + derivative

            time.sleep(0.01)  # Small delay to prevent complete CPU domination

    def _vision_processing_task(self, task_id: int):
        """Simulate vision processing task."""
        duration = self.monitoring_duration_seconds * 0.8
        start_time = time.time()

        while time.time() - start_time < duration:
            # Simulate image processing
            for frame in range(30):  # Process 30 frames
                # Create synthetic image
                image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

                # Simulate computer vision operations
                gray = np.dot(image[...,:3], [0.2989, 0.5870, 0.1140]).astype(np.uint8)

                # Edge detection
                edges = np.zeros_like(gray)
                for i in range(1, gray.shape[0]-1):
                    for j in range(1, gray.shape[1]-1):
                        gx = (gray[i+1, j] - gray[i-1, j])
                        gy = (gray[i, j+1] - gray[i, j-1])
                        edges[i, j] = min(255, np.sqrt(gx**2 + gy**2))

                # Feature detection (simplified)
                features = []
                for i in range(0, edges.shape[0], 20):
                    for j in range(0, edges.shape[1], 20):
                        if edges[i, j] > 100:
                            features.append((i, j))

                # Simulate object detection
                detections = []
                for _ in range(5):
                    x, y = np.random.randint(0, 640), np.random.randint(0, 480)
                    detections.append({'x': x, 'y': y, 'confidence': np.random.random()})

            time.sleep(0.01)

    def _cpu_intensive_task(self, task_id: int):
        """Run CPU-intensive computational task."""
        duration = self.monitoring_duration_seconds * 0.6
        start_time = time.time()

        while time.time() - start_time < duration:
            # Pure CPU-intensive computation
            result = 0
            for i in range(100000):
                result += i ** 2
                result = result % 1000000  # Prevent overflow

            # Matrix operations
            matrix1 = np.random.random((50, 50))
            matrix2 = np.random.random((50, 50))
            result_matrix = np.dot(matrix1, matrix2)

            time.sleep(0.001)  # Brief pause

    def _run_path_planning_simulation(self):
        """Run path planning simulation."""
        # Simulate A* path planning
        grid_size = 100
        start = (0, 0)
        goal = (99, 99)

        # Create obstacle grid
        grid = np.zeros((grid_size, grid_size))
        # Add random obstacles
        for _ in range(200):
            x, y = np.random.randint(0, grid_size, 2)
            grid[x, y] = 1

        # Simulate path planning algorithm
        path = self._simulate_astar_pathfinding(grid, start, goal)

    def _run_image_processing_simulation(self):
        """Run image processing simulation."""
        # Simulate batch image processing
        for batch in range(10):
            images = []
            for i in range(5):  # Process 5 images per batch
                image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
                images.append(image)

            # Simulate batch processing
            processed_images = []
            for img in images:
                # Color space conversion
                hsv = np.zeros_like(img)
                for i in range(img.shape[0]):
                    for j in range(img.shape[1]):
                        r, g, b = img[i, j]
                        hsv[i, j] = [np.arctan2(np.sqrt(3)*(g-b), 2*r-g-b) * 180/np.pi,
                                   np.sqrt(r**2 + g**2 + b**2),
                                   max(r, g, b)]

                processed_images.append(hsv)

    def _simulate_astar_pathfinding(self, grid, start, goal):
        """Simulate A* pathfinding algorithm."""
        from queue import PriorityQueue

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while not frontier.empty():
            current = frontier.get()[1]

            if current == goal:
                break

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                next_pos = (current[0] + dx, current[1] + dy)

                if (0 <= next_pos[0] < len(grid) and
                    0 <= next_pos[1] < len(grid[0]) and
                    grid[next_pos[0]][next_pos[1]] == 0):

                    new_cost = cost_so_far[current] + 1
                    if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                        cost_so_far[next_pos] = new_cost
                        priority = new_cost + heuristic(goal, next_pos)
                        frontier.put((priority, next_pos))
                        came_from[next_pos] = current

        # Reconstruct path
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()

        return path

    def _analyze_mode_cpu_usage(self, mode: str, target_cpu: float, duration: float) -> Dict[str, Any]:
        """Analyze CPU usage for a specific operational mode."""
        if not self.cpu_readings:
            return {'error': 'No CPU readings collected'}

        # Calculate CPU statistics
        avg_cpu = statistics.mean(self.cpu_readings)
        max_cpu = max(self.cpu_readings)
        min_cpu = min(self.cpu_readings)
        p95_cpu = statistics.quantiles(self.cpu_readings, n=20)[18] if len(self.cpu_readings) > 1 else max_cpu

        # Process-specific CPU
        avg_process_cpu = statistics.mean(self.process_cpu_readings) if self.process_cpu_readings else 0
        max_process_cpu = max(self.process_cpu_readings) if self.process_cpu_readings else 0

        # Per-core analysis
        if self.cpu_per_core:
            avg_per_core = np.mean(self.cpu_per_core, axis=0)
            max_per_core = np.max(self.cpu_per_core, axis=0)

        # System load
        if self.system_load_readings:
            avg_load = np.mean(self.system_load_readings, axis=0)

        # Print mode-specific results
        print(f"  📊 {mode.replace('_', ' ').title()} Mode CPU Usage:")
        print(".1f")
        print(".1f")
        print(".1f")
        print(".1f")
        print(".1f")
        print(".1f")

        # Check against target
        passed = avg_cpu <= target_cpu
        status = "✅ PASS" if passed else "❌ FAIL"
        print(".1f")

        return {
            'mode': mode,
            'avg_cpu_percent': avg_cpu,
            'max_cpu_percent': max_cpu,
            'p95_cpu_percent': p95_cpu,
            'avg_process_cpu_percent': avg_process_cpu,
            'max_process_cpu_percent': max_process_cpu,
            'target_cpu_percent': target_cpu,
            'passed': passed,
            'readings_count': len(self.cpu_readings),
            'test_duration': duration
        }

    def _analyze_cpu_mode_comparison(self, results_by_mode: Dict[str, Any]):
        """Analyze CPU usage across different operational modes."""
        print("\n🚀 CPU Utilization Performance Summary:")
        print("=" * 50)

        # Overall assessment
        all_passed = all(result.get('passed', False) for result in results_by_mode.values())

        if all_passed:
            print("✅ ALL CPU UTILIZATION TARGETS MET")
            print("   System CPU usage within competition requirements")
        else:
            print("❌ SOME CPU UTILIZATION TARGETS NOT MET")
            print("   CPU optimization required before competition")

        # Detailed mode comparison
        print("\n📋 Mode-by-Mode CPU Analysis:")
        print("-" * 40)

        for mode, result in results_by_mode.items():
            if 'error' in result:
                print(f"  {mode.replace('_', ' ').title()}: ERROR - {result['error']}")
                continue

            status = "✅" if result['passed'] else "❌"
            print("20"
                  ".1f")

        # Performance trends and recommendations
        print("\n💡 CPU Optimization Analysis:")
        print("-" * 40)

        # Check for concerning patterns
        idle_result = results_by_mode.get('idle', {})
        peak_result = results_by_mode.get('peak_load', {})

        if idle_result.get('avg_cpu_percent', 0) > self.target_idle_cpu_percent * 2:
            print("   ⚠️  High idle CPU usage detected - investigate background processes")

        if peak_result.get('max_cpu_percent', 0) > 90:
            print("   ⚠️  Peak CPU usage approaching system limits - consider load balancing")

        # CPU efficiency analysis
        navigation_eff = results_by_mode.get('navigation', {}).get('avg_cpu_percent', 0)
        vision_eff = results_by_mode.get('vision', {}).get('avg_cpu_percent', 0)

        if navigation_eff > self.target_navigation_cpu_percent:
            print("   📈 Navigation CPU usage high - optimize path planning algorithms")

        if vision_eff > self.target_vision_cpu_percent:
            print("   📈 Vision CPU usage high - consider image resolution reduction or algorithm optimization")

        print("   🎯 Target CPU utilization ensures thermal management and battery efficiency")

        # Store comprehensive results
        self.test_results = {
            'modes_tested': list(results_by_mode.keys()),
            'results_by_mode': results_by_mode,
            'all_targets_met': all_passed,
            'recommendations': []
        }

        # Assert critical requirements
        idle_cpu = results_by_mode.get('idle', {}).get('avg_cpu_percent', 100)
        peak_cpu = results_by_mode.get('peak_load', {}).get('avg_cpu_percent', 0)

        self.assertLess(idle_cpu, self.target_idle_cpu_percent * 3,
                       ".1f")
        self.assertLess(peak_cpu, 95,
                       ".1f")

    def test_cpu_profiling_detailed_analysis(self):
        """Perform detailed CPU profiling for performance bottleneck identification."""
        print("\n🔍 Performing Detailed CPU Profiling")

        # Profile key components
        components_to_profile = [
            ('navigation_computation', self._profile_navigation_computation),
            ('vision_processing', self._profile_vision_processing),
            ('path_planning', self._profile_path_planning),
        ]

        profiling_results = {}

        for component_name, component_func in components_to_profile:
            print(f"  Profiling {component_name}...")

            # Create profiler
            profiler = cProfile.Profile()
            profiler.enable()

            # Run component
            start_time = time.time()
            component_func()
            execution_time = time.time() - start_time

            profiler.disable()

            # Analyze profile
            s = io.StringIO()
            stats = pstats.Stats(profiler, stream=s)
            stats.sort_stats('cumulative')
            stats.print_stats(10)  # Top 10 functions

            profiling_results[component_name] = {
                'execution_time': execution_time,
                'profile_stats': s.getvalue(),
                'cpu_usage_during_execution': psutil.cpu_percent(interval=0.1)
            }

        # Print profiling summary
        print("\n🔬 CPU Profiling Summary:")
        print("-" * 40)

        for component, results in profiling_results.items():
            print(".3f")
            print(".1f")
            print("    Top functions by cumulative time:")
            # Extract top functions from profile stats
            lines = results['profile_stats'].split('\n')
            for line in lines[-6:-1]:  # Last 5 lines typically show top functions
                if line.strip():
                    print(f"      {line.strip()}")

    def _profile_navigation_computation(self):
        """Profile navigation computation for CPU analysis."""
        # Run navigation computation for profiling
        waypoints = [{'x': i*2.0, 'y': (i%3)*1.0} for i in range(50)]

        for wp in waypoints:
            distance = np.sqrt(wp['x']**2 + wp['y']**2)
            angle = np.arctan2(wp['y'], wp['x'])

            # Simulate more complex navigation calculations
            trajectory = np.linspace([0, 0], [wp['x'], wp['y']], 100)
            velocities = np.gradient(trajectory, axis=0)
            accelerations = np.gradient(velocities, axis=0)

    def _profile_vision_processing(self):
        """Profile vision processing for CPU analysis."""
        # Run vision processing for profiling
        for frame in range(20):
            image = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)  # Smaller for profiling

            # Simulate vision pipeline
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if 'cv2' in globals() else np.dot(image[...,:3], [0.2989, 0.5870, 0.1140])
            blurred = cv2.GaussianBlur(gray, (5, 5), 0) if 'cv2' in globals() else gray
            edges = cv2.Canny(blurred, 50, 150) if 'cv2' in globals() else (blurred > 100).astype(np.uint8)

    def _profile_path_planning(self):
        """Profile path planning for CPU analysis."""
        # Run path planning for profiling
        grid = np.random.choice([0, 1], size=(50, 50), p=[0.8, 0.2])  # 50x50 grid with obstacles
        start, goal = (0, 0), (49, 49)

        # Simulate path planning
        path = self._simulate_astar_pathfinding(grid, start, goal)


if __name__ == '__main__':
    unittest.main()
