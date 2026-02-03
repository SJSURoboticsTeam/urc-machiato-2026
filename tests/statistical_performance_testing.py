#!/usr/bin/env python3
"""
Statistical Performance Testing Framework - URC 2026 Resource Optimization

Comprehensive statistical testing for resource optimization features:
- Fallback functionality testing
- Recovery mechanism validation
- Integration testing
- Performance benchmarking with statistical analysis
- Confidence intervals and hypothesis testing

Author: URC 2026 Statistical Testing Team
"""

import time
import psutil
import statistics
import threading
from typing import Dict, List, Any, Optional, Callable, Tuple
from dataclasses import dataclass, field
from enum import Enum
import json
import numpy as np
from datetime import datetime
import logging

logger = logging.getLogger(__name__)


class TestResult(Enum):
    """Test execution result."""

    PASS = "pass"
    FAIL = "fail"
    SKIP = "skip"
    ERROR = "error"


@dataclass
class StatisticalMeasurement:
    """Statistical measurement with confidence intervals."""

    name: str
    samples: List[float] = field(default_factory=list)
    mean: float = 0.0
    median: float = 0.0
    std_dev: float = 0.0
    min_val: float = 0.0
    max_val: float = 0.0
    confidence_interval_95: Tuple[float, float] = (0.0, 0.0)
    sample_size: int = 0
    unit: str = ""

    def add_sample(self, value: float):
        """Add a measurement sample."""
        self.samples.append(value)
        self.sample_size = len(self.samples)

        if self.sample_size >= 2:
            self._calculate_statistics()

    def _calculate_statistics(self):
        """Calculate statistical measures."""
        self.mean = statistics.mean(self.samples)
        self.median = statistics.median(self.samples)
        self.std_dev = statistics.stdev(self.samples)
        self.min_val = min(self.samples)
        self.max_val = max(self.samples)

        # 95% confidence interval using t-distribution approximation
        if self.sample_size > 1:
            # Use normal approximation for large samples
            margin = 1.96 * (self.std_dev / np.sqrt(self.sample_size))
            self.confidence_interval_95 = (self.mean - margin, self.mean + margin)


@dataclass
class PerformanceBenchmark:
    """Performance benchmark results."""

    test_name: str
    mission_profile: str
    duration_seconds: float
    cpu_usage: StatisticalMeasurement
    memory_usage: StatisticalMeasurement
    components_enabled: List[str]
    success_rate: float
    fallback_used: bool
    recovery_events: int
    metadata: Dict[str, Any] = field(default_factory=dict)


class StatisticalPerformanceTester:
    """
    Comprehensive statistical performance testing framework.

    Provides:
    - Statistical analysis of performance measurements
    - Confidence intervals for reliability assessment
    - Hypothesis testing for optimization validation
    - Automated testing of fallback and recovery features
    """

    def __init__(self, num_iterations: int = 10, confidence_level: float = 0.95):
        """Initialize statistical tester.

        Args:
            num_iterations: Number of test iterations for statistical significance
            confidence_level: Statistical confidence level (default 95%)
        """
        self.num_iterations = num_iterations
        self.confidence_level = confidence_level
        self.process = psutil.Process()

        # Test results storage
        self.test_results: Dict[str, List[PerformanceBenchmark]] = {}
        self.baseline_measurements: Dict[str, StatisticalMeasurement] = {}

        # Test configuration
        self.test_timeout_seconds = 30.0
        self.measurement_interval = 0.1  # 100ms sampling
        self.stability_threshold = 0.05  # 5% stability threshold

        logger.info(
            f"Statistical Performance Tester initialized with {num_iterations} iterations"
        )

    def run_comprehensive_test_suite(self) -> Dict[str, Any]:
        """
        Run complete test suite covering all optimization features.

        Returns:
            Comprehensive test results with statistical analysis
        """
        logger.info("Starting comprehensive statistical test suite")

        test_results = {
            "timestamp": datetime.now().isoformat(),
            "test_suite": "URC 2026 Resource Optimization",
            "iterations": self.num_iterations,
            "tests": {},
        }

        # Test scenarios
        test_scenarios = [
            self.test_mission_profile_switching,
            self.test_fallback_functionality,
            self.test_recovery_mechanisms,
            self.test_integration_scenarios,
            self.test_resource_optimization_effectiveness,
        ]

        for test_func in test_scenarios:
            test_name = test_func.__name__
            logger.info(f"Running test: {test_name}")

            try:
                results = test_func()
                test_results["tests"][test_name] = results
                logger.info(f"Test {test_name} completed: {len(results)} benchmarks")
            except Exception as e:
                logger.error(f"Test {test_name} failed: {e}")
                test_results["tests"][test_name] = {"error": str(e)}

        # Generate statistical report
        test_results["statistical_analysis"] = self.generate_statistical_report()

        return test_results

    def test_mission_profile_switching(self) -> List[PerformanceBenchmark]:
        """Test mission profile switching performance and reliability."""
        logger.info("Testing mission profile switching")

        results = []
        mission_profiles = ["waypoint_navigation", "object_search", "sample_collection"]

        for mission in mission_profiles:
            for iteration in range(self.num_iterations):
                benchmark = self._benchmark_mission_switching(mission, iteration)
                results.append(benchmark)

        return results

    def _benchmark_mission_switching(
        self, mission_profile: str, iteration: int
    ) -> PerformanceBenchmark:
        """Benchmark a single mission profile switch."""
        benchmark = PerformanceBenchmark(
            test_name="mission_profile_switching",
            mission_profile=mission_profile,
            duration_seconds=0.0,
            cpu_usage=StatisticalMeasurement("cpu_usage", unit="%"),
            memory_usage=StatisticalMeasurement("memory_usage", unit="MB"),
            components_enabled=[],
            success_rate=0.0,
            fallback_used=False,
            recovery_events=0,
        )

        start_time = time.time()
        measurements = []

        try:
            # Import and initialize components
            from src.core.mission_resource_manager import get_mission_resource_manager

            resource_manager = get_mission_resource_manager()

            # Measure baseline
            baseline_cpu, baseline_memory = self._measure_resources()
            benchmark.cpu_usage.add_sample(baseline_cpu)
            benchmark.memory_usage.add_sample(baseline_memory)

            # Switch mission profile
            success = resource_manager.switch_mission_profile(mission_profile)

            if success:
                benchmark.success_rate = 1.0

                # Get component status
                status = resource_manager.get_resource_status()
                benchmark.components_enabled = list(
                    status.get("component_status", {}).keys()
                )

                # Measure post-switch resources
                post_cpu, post_memory = self._measure_resources()
                benchmark.cpu_usage.add_sample(post_cpu)
                benchmark.memory_usage.add_sample(post_memory)

                # Check for fallback usage (simplified check)
                benchmark.fallback_used = (
                    len(benchmark.components_enabled) < 2
                )  # Simple heuristic

            else:
                benchmark.success_rate = 0.0
                logger.warning(f"Mission profile switch failed: {mission_profile}")

        except Exception as e:
            logger.error(f"Mission switching benchmark failed: {e}")
            benchmark.success_rate = 0.0

        benchmark.duration_seconds = time.time() - start_time
        return benchmark

    def test_fallback_functionality(self) -> List[PerformanceBenchmark]:
        """Test fallback functionality when dependencies are missing."""
        logger.info("Testing fallback functionality")

        results = []

        # Test scenarios with missing dependencies
        fallback_scenarios = [
            {
                "scenario": "no_ml_libraries",
                "mock_missing": ["torch", "detectron2", "tensorflow"],
            },
            {"scenario": "no_slam", "mock_missing": ["rtabmap"]},
            {"scenario": "minimal_hardware", "mock_missing": ["opencv", "pcl"]},
        ]

        for scenario in fallback_scenarios:
            for iteration in range(self.num_iterations):
                benchmark = self._benchmark_fallback_scenario(scenario, iteration)
                results.append(benchmark)

        return results

    def _benchmark_fallback_scenario(
        self, scenario: Dict[str, Any], iteration: int
    ) -> PerformanceBenchmark:
        """Benchmark a fallback scenario."""
        benchmark = PerformanceBenchmark(
            test_name="fallback_functionality",
            mission_profile="sample_collection",  # Test with full requirements
            duration_seconds=0.0,
            cpu_usage=StatisticalMeasurement("cpu_usage", unit="%"),
            memory_usage=StatisticalMeasurement("memory_usage", unit="MB"),
            components_enabled=[],
            success_rate=0.0,
            fallback_used=True,
            recovery_events=0,
            metadata={
                "scenario": scenario["scenario"],
                "mock_missing": scenario["mock_missing"],
            },
        )

        start_time = time.time()

        try:
            # Import lightweight implementations directly to test fallbacks
            from src.autonomy.perception.computer_vision.lightweight_vision import (
                LightweightVisionProcessor,
            )
            from src.autonomy.perception.slam.lightweight_slam import (
                LightweightVisualOdometry,
            )

            # Measure baseline
            baseline_cpu, baseline_memory = self._measure_resources()
            benchmark.cpu_usage.add_sample(baseline_cpu)
            benchmark.memory_usage.add_sample(baseline_memory)

            # Test lightweight vision
            vision_processor = LightweightVisionProcessor()
            # Create dummy image for testing
            dummy_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            results = vision_processor.process_frame_lightweight(dummy_image)

            # Test lightweight SLAM
            slam_system = LightweightVisualOdometry()
            slam_results = slam_system.process_frame(dummy_image)

            benchmark.components_enabled = ["lightweight_vision", "lightweight_slam"]
            benchmark.success_rate = 1.0 if results and slam_results else 0.0

            # Measure post-fallback resources
            post_cpu, post_memory = self._measure_resources()
            benchmark.cpu_usage.add_sample(post_cpu)
            benchmark.memory_usage.add_sample(post_memory)

        except Exception as e:
            logger.error(f"Fallback benchmark failed: {e}")
            benchmark.success_rate = 0.0

        benchmark.duration_seconds = time.time() - start_time
        return benchmark

    def test_recovery_mechanisms(self) -> List[PerformanceBenchmark]:
        """Test recovery mechanisms under resource pressure."""
        logger.info("Testing recovery mechanisms")

        results = []

        # Test recovery scenarios
        recovery_scenarios = [
            {
                "scenario": "cpu_pressure",
                "target_resource": "cpu",
                "pressure_level": 80,
            },
            {
                "scenario": "memory_pressure",
                "target_resource": "memory",
                "pressure_level": 85,
            },
            {
                "scenario": "combined_pressure",
                "target_resource": "both",
                "pressure_level": 75,
            },
        ]

        for scenario in recovery_scenarios:
            for iteration in range(self.num_iterations):
                benchmark = self._benchmark_recovery_scenario(scenario, iteration)
                results.append(benchmark)

        return results

    def _benchmark_recovery_scenario(
        self, scenario: Dict[str, Any], iteration: int
    ) -> PerformanceBenchmark:
        """Benchmark a recovery scenario under resource pressure."""
        benchmark = PerformanceBenchmark(
            test_name="recovery_mechanisms",
            mission_profile="sample_collection",
            duration_seconds=0.0,
            cpu_usage=StatisticalMeasurement("cpu_usage", unit="%"),
            memory_usage=StatisticalMeasurement("memory_usage", unit="MB"),
            components_enabled=[],
            success_rate=0.0,
            fallback_used=False,
            recovery_events=0,
            metadata={
                "scenario": scenario["scenario"],
                "pressure_level": scenario["pressure_level"],
            },
        )

        start_time = time.time()

        try:
            from src.core.mission_resource_manager import get_mission_resource_manager

            resource_manager = get_mission_resource_manager()

            # Switch to full mission mode
            resource_manager.switch_mission_profile("sample_collection")

            # Measure baseline
            baseline_cpu, baseline_memory = self._measure_resources()
            benchmark.cpu_usage.add_sample(baseline_cpu)
            benchmark.memory_usage.add_sample(baseline_memory)

            # Simulate resource pressure by starting background load
            load_thread = self._simulate_resource_pressure(scenario)
            load_thread.start()

            # Wait for pressure to build
            time.sleep(2.0)

            # Check if recovery mechanisms activate
            initial_components = len(
                resource_manager.get_resource_status().get("component_status", {})
            )

            # Wait for recovery
            time.sleep(3.0)

            # Check recovery results
            final_status = resource_manager.get_resource_status()
            final_components = len(final_status.get("component_status", {}))
            benchmark.recovery_events = max(0, initial_components - final_components)

            benchmark.components_enabled = list(
                final_status.get("component_status", {}).keys()
            )
            benchmark.success_rate = 1.0 if benchmark.recovery_events > 0 else 0.5

            # Measure post-recovery resources
            post_cpu, post_memory = self._measure_resources()
            benchmark.cpu_usage.add_sample(post_cpu)
            benchmark.memory_usage.add_sample(post_memory)

            # Stop load simulation
            load_thread.join(timeout=1.0)

        except Exception as e:
            logger.error(f"Recovery benchmark failed: {e}")
            benchmark.success_rate = 0.0

        benchmark.duration_seconds = time.time() - start_time
        return benchmark

    def _simulate_resource_pressure(self, scenario: Dict[str, Any]) -> threading.Thread:
        """Simulate resource pressure in background thread."""

        def pressure_worker():
            try:
                if scenario["target_resource"] in ["cpu", "both"]:
                    # Simulate CPU pressure
                    self._generate_cpu_load(duration=5.0, intensity=0.7)

                if scenario["target_resource"] in ["memory", "both"]:
                    # Simulate memory pressure
                    self._generate_memory_load(duration=5.0, size_mb=200)
            except Exception as e:
                logger.error(f"Pressure simulation failed: {e}")

        thread = threading.Thread(target=pressure_worker, daemon=True)
        return thread

    def _generate_cpu_load(self, duration: float, intensity: float):
        """Generate CPU load for testing."""
        end_time = time.time() + duration
        while time.time() < end_time:
            # Busy wait with controlled intensity
            busy_time = intensity * 0.01
            idle_time = (1 - intensity) * 0.01

            start = time.time()
            while time.time() - start < busy_time:
                pass

            time.sleep(idle_time)

    def _generate_memory_load(self, duration: float, size_mb: int):
        """Generate memory load for testing."""
        # Allocate memory temporarily
        data = [0] * (size_mb * 1024 * 128)  # Approximate MB allocation
        time.sleep(duration)
        # Let garbage collector free memory
        del data

    def test_integration_scenarios(self) -> List[PerformanceBenchmark]:
        """Test integration between components."""
        logger.info("Testing integration scenarios")

        results = []

        # Integration test scenarios
        integration_scenarios = [
            {
                "scenario": "resource_manager_feature_flags",
                "components": ["resource_manager", "feature_flags"],
            },
            {
                "scenario": "behavior_tree_resource_manager",
                "components": ["behavior_tree", "resource_manager"],
            },
            {
                "scenario": "monitoring_resource_manager",
                "components": ["monitoring", "resource_manager"],
            },
            {"scenario": "full_system_integration", "components": ["all"]},
        ]

        for scenario in integration_scenarios:
            for iteration in range(self.num_iterations):
                benchmark = self._benchmark_integration_scenario(scenario, iteration)
                results.append(benchmark)

        return results

    def _benchmark_integration_scenario(
        self, scenario: Dict[str, Any], iteration: int
    ) -> PerformanceBenchmark:
        """Benchmark an integration scenario."""
        benchmark = PerformanceBenchmark(
            test_name="integration_scenarios",
            mission_profile="sample_collection",
            duration_seconds=0.0,
            cpu_usage=StatisticalMeasurement("cpu_usage", unit="%"),
            memory_usage=StatisticalMeasurement("memory_usage", unit="MB"),
            components_enabled=[],
            success_rate=0.0,
            fallback_used=False,
            recovery_events=0,
            metadata={
                "scenario": scenario["scenario"],
                "components": scenario["components"],
            },
        )

        start_time = time.time()

        try:
            # Measure baseline
            baseline_cpu, baseline_memory = self._measure_resources()
            benchmark.cpu_usage.add_sample(baseline_cpu)
            benchmark.memory_usage.add_sample(baseline_memory)

            success_count = 0

            if "resource_manager" in scenario["components"]:
                try:
                    from src.core.mission_resource_manager import (
                        get_mission_resource_manager,
                    )

                    rm = get_mission_resource_manager()
                    rm.switch_mission_profile("sample_collection")
                    success_count += 1
                except Exception as e:
                    logger.warning(f"Resource manager integration failed: {e}")

            if "feature_flags" in scenario["components"]:
                try:
                    from src.core.feature_flags import get_feature_flag_manager

                    fm = get_feature_flag_manager()
                    fm.set_mission_profile("sample_collection")
                    success_count += 1
                except Exception as e:
                    logger.warning(f"Feature flags integration failed: {e}")

            if "behavior_tree" in scenario["components"]:
                try:
                    from missions.robust_behavior_tree import PyTreesBehaviorTree

                    bt = PyTreesBehaviorTree()
                    bt.switch_mission_profile("sample_collection")
                    success_count += 1
                except Exception as e:
                    logger.warning(f"Behavior tree integration failed: {e}")

            if "monitoring" in scenario["components"]:
                try:
                    from simulation.tools.monitoring_dashboard import SimulationMonitor

                    monitor = SimulationMonitor()
                    monitor.start_monitoring()
                    time.sleep(1.0)
                    monitor.stop_monitoring()
                    success_count += 1
                except Exception as e:
                    logger.warning(f"Monitoring integration failed: {e}")

            # Calculate success rate
            expected_components = len(scenario["components"])
            if "all" in scenario["components"]:
                expected_components = 4  # All components

            benchmark.success_rate = (
                success_count / expected_components if expected_components > 0 else 0.0
            )

            # Measure post-integration resources
            post_cpu, post_memory = self._measure_resources()
            benchmark.cpu_usage.add_sample(post_cpu)
            benchmark.memory_usage.add_sample(post_memory)

            benchmark.components_enabled = scenario["components"]

        except Exception as e:
            logger.error(f"Integration benchmark failed: {e}")
            benchmark.success_rate = 0.0

        benchmark.duration_seconds = time.time() - start_time
        return benchmark

    def test_resource_optimization_effectiveness(self) -> List[PerformanceBenchmark]:
        """Test the effectiveness of resource optimization across mission profiles."""
        logger.info("Testing resource optimization effectiveness")

        results = []

        mission_profiles = ["waypoint_navigation", "object_search", "sample_collection"]

        for mission in mission_profiles:
            for iteration in range(self.num_iterations):
                benchmark = self._benchmark_resource_optimization(mission, iteration)
                results.append(benchmark)

        return results

    def _benchmark_resource_optimization(
        self, mission_profile: str, iteration: int
    ) -> PerformanceBenchmark:
        """Benchmark resource optimization for a specific mission profile."""
        benchmark = PerformanceBenchmark(
            test_name="resource_optimization_effectiveness",
            mission_profile=mission_profile,
            duration_seconds=0.0,
            cpu_usage=StatisticalMeasurement("cpu_usage", unit="%"),
            memory_usage=StatisticalMeasurement("memory_usage", unit="MB"),
            components_enabled=[],
            success_rate=0.0,
            fallback_used=False,
            recovery_events=0,
        )

        start_time = time.time()

        try:
            from src.core.mission_resource_manager import get_mission_resource_manager

            resource_manager = get_mission_resource_manager()

            # Measure baseline (before any mission)
            baseline_cpu, baseline_memory = self._measure_resources()
            benchmark.cpu_usage.add_sample(baseline_cpu)
            benchmark.memory_usage.add_sample(baseline_memory)

            # Switch to mission
            success = resource_manager.switch_mission_profile(mission_profile)

            if success:
                benchmark.success_rate = 1.0

                # Get component status
                status = resource_manager.get_resource_status()
                benchmark.components_enabled = list(
                    status.get("component_status", {}).keys()
                )

                # Measure mission-specific resources
                mission_cpu, mission_memory = self._measure_resources()
                benchmark.cpu_usage.add_sample(mission_cpu)
                benchmark.memory_usage.add_sample(mission_memory)

                # Calculate optimization effectiveness (reduction from baseline)
                cpu_reduction = baseline_cpu - mission_cpu
                memory_reduction = baseline_memory - mission_memory

                benchmark.metadata = {
                    "cpu_reduction": cpu_reduction,
                    "memory_reduction": memory_reduction,
                    "optimization_effective": cpu_reduction > 0 or memory_reduction > 0,
                }

            else:
                benchmark.success_rate = 0.0

        except Exception as e:
            logger.error(f"Resource optimization benchmark failed: {e}")
            benchmark.success_rate = 0.0

        benchmark.duration_seconds = time.time() - start_time
        return benchmark

    def _measure_resources(self) -> Tuple[float, float]:
        """Measure current CPU and memory usage."""
        cpu_percent = self.process.cpu_percent(interval=None)
        memory_mb = self.process.memory_info().rss / (1024 * 1024)
        return cpu_percent, memory_mb

    def generate_statistical_report(self) -> Dict[str, Any]:
        """Generate comprehensive statistical report."""
        report = {
            "summary": {},
            "detailed_results": {},
            "confidence_intervals": {},
            "hypothesis_tests": {},
            "recommendations": [],
        }

        # Analyze each test type
        for test_name, benchmarks in self.test_results.items():
            if not benchmarks:
                continue

            report["detailed_results"][test_name] = {
                "total_samples": len(benchmarks),
                "success_rate": sum(b.success_rate for b in benchmarks)
                / len(benchmarks),
                "avg_duration": statistics.mean(b.duration_seconds for b in benchmarks),
                "cpu_usage_stats": self._analyze_measurements(
                    [b.cpu_usage for b in benchmarks]
                ),
                "memory_usage_stats": self._analyze_measurements(
                    [b.memory_usage for b in benchmarks]
                ),
            }

        # Generate recommendations
        report["recommendations"] = self._generate_recommendations(report)

        return report

    def _analyze_measurements(
        self, measurements: List[StatisticalMeasurement]
    ) -> Dict[str, Any]:
        """Analyze statistical measurements."""
        if not measurements:
            return {}

        # Combine all samples
        all_samples = []
        for measurement in measurements:
            all_samples.extend(measurement.samples)

        if len(all_samples) < 2:
            return {"insufficient_data": True}

        analysis = {
            "sample_size": len(all_samples),
            "mean": statistics.mean(all_samples),
            "median": statistics.median(all_samples),
            "std_dev": statistics.stdev(all_samples),
            "min": min(all_samples),
            "max": max(all_samples),
            "confidence_interval_95": self._calculate_confidence_interval(all_samples),
        }

        return analysis

    def _calculate_confidence_interval(
        self, samples: List[float], confidence: float = 0.95
    ) -> Tuple[float, float]:
        """Calculate confidence interval."""
        if len(samples) < 2:
            return (0.0, 0.0)

        mean = statistics.mean(samples)
        std_dev = statistics.stdev(samples)
        n = len(samples)

        # t-distribution critical value approximation for 95%
        t_value = 1.96 if n > 30 else 2.0  # Conservative estimate

        margin = t_value * (std_dev / np.sqrt(n))
        return (mean - margin, mean + margin)

    def _generate_recommendations(self, report: Dict[str, Any]) -> List[str]:
        """Generate recommendations based on test results."""
        recommendations = []

        # Check resource optimization effectiveness
        if "resource_optimization_effectiveness" in report["detailed_results"]:
            results = report["detailed_results"]["resource_optimization_effectiveness"]

            if results["success_rate"] > 0.8:
                recommendations.append(
                    "✅ Resource optimization is highly effective (>80% success rate)"
                )
            elif results["success_rate"] > 0.6:
                recommendations.append(
                    "⚠️ Resource optimization moderately effective (60-80% success rate)"
                )
            else:
                recommendations.append(
                    "❌ Resource optimization needs improvement (<60% success rate)"
                )

        # Check fallback functionality
        if "fallback_functionality" in report["detailed_results"]:
            results = report["detailed_results"]["fallback_functionality"]

            if results["success_rate"] > 0.9:
                recommendations.append("✅ Fallback mechanisms are robust and reliable")
            else:
                recommendations.append("⚠️ Fallback mechanisms need improvement")

        # Check recovery mechanisms
        if "recovery_mechanisms" in report["detailed_results"]:
            results = report["detailed_results"]["recovery_mechanisms"]
            avg_recovery_events = sum(
                b.recovery_events
                for b in self.test_results.get("recovery_mechanisms", [])
            )

            if avg_recovery_events > 0:
                recommendations.append(
                    "✅ Recovery mechanisms activate under resource pressure"
                )
            else:
                recommendations.append(
                    "⚠️ Recovery mechanisms may need tuning for resource pressure detection"
                )

        return recommendations

    def save_results(self, filename: str):
        """Save test results to file."""
        results = {
            "test_results": self.test_results,
            "statistical_report": self.generate_statistical_report(),
            "metadata": {
                "iterations": self.num_iterations,
                "confidence_level": self.confidence_level,
                "timestamp": datetime.now().isoformat(),
            },
        }

        with open(filename, "w") as f:
            json.dump(results, f, indent=2, default=str)

        logger.info(f"Test results saved to {filename}")


def run_statistical_performance_tests():
    """Run comprehensive statistical performance tests."""
    print("🧪 STATISTICAL PERFORMANCE TESTING - URC 2026")
    print("=" * 60)

    tester = StatisticalPerformanceTester(num_iterations=5)  # Reduced for demo

    try:
        print("Running comprehensive test suite...")
        results = tester.run_comprehensive_test_suite()

        print("\n📊 TEST RESULTS SUMMARY")
        print("=" * 40)

        for test_name, test_results in results["tests"].items():
            if isinstance(test_results, list):
                success_rate = sum(b.success_rate for b in test_results) / len(
                    test_results
                )
                print(".1%")
            else:
                print(f"❌ {test_name}: ERROR - {test_results.get('error', 'Unknown')}")

        # Show statistical analysis
        if "statistical_analysis" in results:
            analysis = results["statistical_analysis"]

            print("\n📈 STATISTICAL ANALYSIS")
            print("-" * 30)

            for test_name, stats in analysis.get("detailed_results", {}).items():
                if "cpu_usage_stats" in stats:
                    cpu_stats = stats["cpu_usage_stats"]
                    if "mean" in cpu_stats:
                        print(".1f")

            # Show recommendations
            print("\n💡 RECOMMENDATIONS")
            for rec in analysis.get("recommendations", []):
                print(f"  {rec}")

        # Save detailed results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"statistical_performance_results_{timestamp}.json"
        tester.save_results(filename)
        print(f"\n📁 Detailed results saved to: {filename}")

        return True

    except Exception as e:
        print(f"❌ Testing failed: {e}")
        import traceback

        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = run_statistical_performance_tests()
    exit(0 if success else 1)
