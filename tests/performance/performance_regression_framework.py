#!/usr/bin/env python3
"""
Performance Regression Framework - URC 2026

Establishes performance baselines and detects regressions:
- Baseline establishment for all performance metrics
- Automated regression detection (>5% degradation)
- Performance trend analysis and reporting
- CI/CD integration for continuous performance monitoring

Critical for maintaining competition performance over development lifecycle.
"""

import json
import os
import statistics
import time
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


class PerformanceBaseline:
    """Performance baseline data and statistics."""

    def __init__(self, test_name: str, metric_name: str):
        self.test_name = test_name
        self.metric_name = metric_name
        self.values: List[float] = []
        self.timestamps: List[datetime] = []
        self.baseline_stats: Optional[Dict[str, float]] = None

    def add_measurement(self, value: float, timestamp: Optional[datetime] = None):
        """Add a performance measurement."""
        if timestamp is None:
            timestamp = datetime.now()

        self.values.append(value)
        self.timestamps.append(timestamp)

    def calculate_baseline_stats(self) -> Dict[str, float]:
        """Calculate baseline statistics from measurements."""
        if len(self.values) < 3:
            # Need minimum samples for reliable baseline
            return {}

        stats = {
            "mean": statistics.mean(self.values),
            "median": statistics.median(self.values),
            "std_dev": statistics.stdev(self.values) if len(self.values) > 1 else 0,
            "min": min(self.values),
            "max": max(self.values),
            "p95": statistics.quantiles(self.values, n=20)[18]
            if len(self.values) >= 20
            else max(self.values),
            "sample_count": len(self.values),
            "last_updated": self.timestamps[-1].isoformat()
            if self.timestamps
            else None,
        }

        self.baseline_stats = stats
        return stats

    def is_regression(
        self, current_value: float, threshold_percent: float = 5.0
    ) -> Tuple[bool, float]:
        """Check if current value represents a performance regression."""
        if not self.baseline_stats:
            return False, 0.0  # No baseline to compare against

        baseline_mean = self.baseline_stats["mean"]

        # For most metrics, higher values indicate worse performance
        # (latency, memory usage, CPU usage, etc.)
        if self.metric_name in [
            "latency",
            "memory_usage",
            "cpu_usage",
            "power_consumption",
            "bandwidth_usage",
            "startup_time",
            "recovery_time",
        ]:
            degradation_percent = (
                (current_value - baseline_mean) / baseline_mean
            ) * 100
            is_regression = degradation_percent > threshold_percent
        else:
            # For metrics where lower values are worse (throughput, FPS, etc.)
            degradation_percent = (
                (baseline_mean - current_value) / baseline_mean
            ) * 100
            is_regression = degradation_percent > threshold_percent

        return is_regression, degradation_percent

    def get_trend_analysis(self) -> Dict[str, Any]:
        """Analyze performance trends over time."""
        if len(self.values) < 5:
            return {"trend": "insufficient_data"}

        # Calculate linear trend
        x = np.arange(len(self.values))
        y = np.array(self.values)

        slope, intercept = np.polyfit(x, y, 1)
        trend_direction = "improving" if slope < 0 else "degrading"
        trend_magnitude = (
            abs(slope) / statistics.mean(self.values) * 100
        )  # Percent per measurement

        # Calculate recent vs old performance
        midpoint = len(self.values) // 2
        old_mean = statistics.mean(self.values[:midpoint])
        new_mean = statistics.mean(self.values[midpoint:])

        if self.metric_name in ["latency", "memory_usage", "cpu_usage"]:
            recent_change = ((new_mean - old_mean) / old_mean) * 100
        else:
            recent_change = ((old_mean - new_mean) / old_mean) * 100

        return {
            "trend_direction": trend_direction,
            "trend_slope": slope,
            "trend_magnitude_percent": trend_magnitude,
            "recent_change_percent": recent_change,
            "data_points": len(self.values),
        }


class PerformanceRegressionFramework:
    """Framework for performance baseline management and regression detection."""

    def __init__(self, baseline_dir: str = "performance_baselines"):
        self.baseline_dir = Path(baseline_dir)
        self.baseline_dir.mkdir(exist_ok=True)
        self.baselines: Dict[str, PerformanceBaseline] = {}
        self.load_existing_baselines()

    def load_existing_baselines(self):
        """Load existing baseline data from disk."""
        for baseline_file in self.baseline_dir.glob("*.json"):
            try:
                with open(baseline_file, "r") as f:
                    data = json.load(f)

                test_name = data["test_name"]
                metric_name = data["metric_name"]

                baseline = PerformanceBaseline(test_name, metric_name)

                # Load historical measurements
                for measurement in data.get("measurements", []):
                    value = measurement["value"]
                    timestamp_str = measurement.get("timestamp")
                    timestamp = (
                        datetime.fromisoformat(timestamp_str) if timestamp_str else None
                    )
                    baseline.add_measurement(value, timestamp)

                # Load baseline statistics
                if "baseline_stats" in data:
                    baseline.baseline_stats = data["baseline_stats"]

                key = f"{test_name}.{metric_name}"
                self.baselines[key] = baseline

            except Exception as e:
                print(f"Warning: Failed to load baseline from {baseline_file}: {e}")

    def establish_baseline(
        self, test_name: str, metric_name: str, measurements: List[float]
    ) -> PerformanceBaseline:
        """Establish a performance baseline from measurements."""
        key = f"{test_name}.{metric_name}"

        if key not in self.baselines:
            self.baselines[key] = PerformanceBaseline(test_name, metric_name)

        baseline = self.baselines[key]

        # Add measurements
        for value in measurements:
            baseline.add_measurement(value)

        # Calculate baseline statistics
        baseline.calculate_baseline_stats()

        # Save to disk
        self._save_baseline(baseline)

        return baseline

    def record_measurement(self, test_name: str, metric_name: str, value: float):
        """Record a single performance measurement."""
        key = f"{test_name}.{metric_name}"

        if key not in self.baselines:
            self.baselines[key] = PerformanceBaseline(test_name, metric_name)

        baseline = self.baselines[key]
        baseline.add_measurement(value)

        # Update baseline stats periodically (every 10 measurements)
        if len(baseline.values) % 10 == 0:
            baseline.calculate_baseline_stats()
            self._save_baseline(baseline)

    def detect_regression(
        self,
        test_name: str,
        metric_name: str,
        current_value: float,
        threshold_percent: float = 5.0,
    ) -> Dict[str, Any]:
        """Detect performance regression for a metric."""
        key = f"{test_name}.{metric_name}"

        if key not in self.baselines:
            return {
                "regression_detected": False,
                "reason": "no_baseline",
                "baseline_mean": None,
                "current_value": current_value,
                "degradation_percent": 0.0,
            }

        baseline = self.baselines[key]

        if not baseline.baseline_stats:
            baseline.calculate_baseline_stats()

        if not baseline.baseline_stats:
            return {
                "regression_detected": False,
                "reason": "insufficient_baseline_data",
                "baseline_mean": None,
                "current_value": current_value,
                "degradation_percent": 0.0,
            }

        is_regression, degradation_percent = baseline.is_regression(
            current_value, threshold_percent
        )

        return {
            "regression_detected": is_regression,
            "baseline_mean": baseline.baseline_stats["mean"],
            "baseline_std_dev": baseline.baseline_stats["std_dev"],
            "current_value": current_value,
            "degradation_percent": degradation_percent,
            "threshold_percent": threshold_percent,
            "sample_count": baseline.baseline_stats["sample_count"],
        }

    def get_performance_trends(
        self, test_name: Optional[str] = None, metric_name: Optional[str] = None
    ) -> Dict[str, Any]:
        """Get performance trend analysis for specified tests/metrics."""
        trends = {}

        for key, baseline in self.baselines.items():
            if test_name and not key.startswith(f"{test_name}."):
                continue
            if metric_name and not key.endswith(f".{metric_name}"):
                continue

            trend_analysis = baseline.get_trend_analysis()
            trends[key] = {
                "baseline_stats": baseline.baseline_stats,
                "trend_analysis": trend_analysis,
                "latest_value": baseline.values[-1] if baseline.values else None,
            }

        return trends

    def generate_regression_report(
        self, current_results: Dict[str, Any], threshold_percent: float = 5.0
    ) -> Dict[str, Any]:
        """Generate comprehensive regression report."""
        report = {
            "timestamp": datetime.now().isoformat(),
            "regressions_detected": [],
            "improvements_found": [],
            "no_baseline_metrics": [],
            "summary": {},
        }

        total_checks = 0
        regression_count = 0
        improvement_count = 0

        # Check each metric in current results
        for test_name, test_results in current_results.items():
            if not isinstance(test_results, dict):
                continue

            for metric_name, current_value in test_results.items():
                if not isinstance(current_value, (int, float)):
                    continue

                total_checks += 1

                regression_result = self.detect_regression(
                    test_name, metric_name, current_value, threshold_percent
                )

                if regression_result["regression_detected"]:
                    regression_count += 1
                    report["regressions_detected"].append(
                        {
                            "test": test_name,
                            "metric": metric_name,
                            "details": regression_result,
                        }
                    )
                elif regression_result["reason"] == "no_baseline":
                    report["no_baseline_metrics"].append(
                        {
                            "test": test_name,
                            "metric": metric_name,
                            "current_value": current_value,
                        }
                    )
                elif regression_result["degradation_percent"] < -threshold_percent:
                    # Significant improvement
                    improvement_count += 1
                    report["improvements_found"].append(
                        {
                            "test": test_name,
                            "metric": metric_name,
                            "details": regression_result,
                        }
                    )

        # Generate summary
        report["summary"] = {
            "total_metrics_checked": total_checks,
            "regressions_detected": regression_count,
            "improvements_found": improvement_count,
            "no_baseline_metrics": len(report["no_baseline_metrics"]),
            "pass_rate": ((total_checks - regression_count) / total_checks * 100)
            if total_checks > 0
            else 100,
            "threshold_percent": threshold_percent,
        }

        return report

    def export_baselines(self, output_file: str):
        """Export all baseline data to a file."""
        export_data = {"export_timestamp": datetime.now().isoformat(), "baselines": {}}

        for key, baseline in self.baselines.items():
            export_data["baselines"][key] = {
                "test_name": baseline.test_name,
                "metric_name": baseline.metric_name,
                "measurements": [
                    {"value": v, "timestamp": t.isoformat()}
                    for v, t in zip(baseline.values, baseline.timestamps)
                ],
                "baseline_stats": baseline.baseline_stats,
            }

        with open(output_file, "w") as f:
            json.dump(export_data, f, indent=2, default=str)

        print(f"[GRAPH] Exported {len(self.baselines)} baselines to {output_file}")

    def _save_baseline(self, baseline: PerformanceBaseline):
        """Save baseline data to disk."""
        filename = f"{baseline.test_name}_{baseline.metric_name}_baseline.json"
        filepath = self.baseline_dir / filename

        data = {
            "test_name": baseline.test_name,
            "metric_name": baseline.metric_name,
            "measurements": [
                {"value": v, "timestamp": t.isoformat()}
                for v, t in zip(baseline.values, baseline.timestamps)
            ],
            "baseline_stats": baseline.baseline_stats,
        }

        with open(filepath, "w") as f:
            json.dump(data, f, indent=2, default=str)


class PerformanceRegressionTest:
    """Test class for performance regression detection."""

    def __init__(self):
        self.framework = PerformanceRegressionFramework()

    def run_baseline_establishment(self):
        """Run baseline establishment for all performance tests."""
        print("[GRAPH] Establishing Performance Baselines")
        print("=" * 50)

        # Import and run performance tests to establish baselines
        performance_tests = [
            ("control_loop_latency", "test_control_loop_latency"),
            ("vision_pipeline", "test_vision_pipeline_performance"),
            ("terrain_analysis", "test_terrain_analysis_performance"),
            ("mission_execution", "test_mission_execution_performance"),
            ("memory_usage", "test_memory_usage_trends"),
            ("cpu_utilization", "test_cpu_utilization_performance"),
            ("network_bandwidth", "test_network_bandwidth_performance"),
        ]

        for test_category, test_module in performance_tests:
            try:
                print(f"\n[REFRESH] Establishing baseline for {test_category}...")

                # Import the test module dynamically
                module_name = f"tests.performance.{test_module}"
                # For baseline establishment, we'd run the actual tests
                # This is a simplified version - in practice, you'd run the full test suite

                # Simulate baseline measurements (replace with actual test runs)
                simulated_measurements = self._simulate_baseline_measurements(
                    test_category
                )

                for metric_name, measurements in simulated_measurements.items():
                    baseline = self.framework.establish_baseline(
                        test_category, metric_name, measurements
                    )
                    print(
                        f"  [PASS] {metric_name}: {len(measurements)} samples, baseline mean: {baseline.baseline_stats['mean']:.3f}"
                    )

            except Exception as e:
                print(f"  [FAIL] Failed to establish baseline for {test_category}: {e}")

        print(
            f"\n Established baselines for {len(self.framework.baselines)} performance metrics"
        )

    def run_regression_detection(self):
        """Run regression detection against established baselines."""
        print("\n[MAGNIFY] Running Performance Regression Detection")
        print("=" * 50)

        # Simulate current test results (replace with actual test runs)
        current_results = self._simulate_current_results()

        # Generate regression report
        report = self.framework.generate_regression_report(current_results)

        # Print report summary
        print(f"Performance Regression Report - {report['timestamp']}")
        print("-" * 50)
        print(f"Total metrics checked: {report['summary']['total_metrics_checked']}")
        print(f"Regressions detected: {report['summary']['regressions_detected']}")
        print(f"Improvements found: {report['summary']['improvements_found']}")
        print(f"No baseline metrics: {report['summary']['no_baseline_metrics']}")
        print(".1f")

        # Report regressions
        if report["regressions_detected"]:
            print("\n[FAIL] PERFORMANCE REGRESSIONS DETECTED:")
            print("-" * 40)
            for regression in report["regressions_detected"]:
                details = regression["details"]
                print("5.1f" ".3f")

        # Report improvements
        if report["improvements_found"]:
            print("\n[PASS] PERFORMANCE IMPROVEMENTS DETECTED:")
            print("-" * 40)
            for improvement in report["improvements_found"]:
                details = improvement["details"]
                print("5.1f" ".3f")

        # Overall assessment
        if report["summary"]["regressions_detected"] > 0:
            print("\n PERFORMANCE REGRESSION ALERT")
            print("   Address performance regressions before deployment")
        elif report["summary"]["pass_rate"] >= 95:
            print("\n[PASS] PERFORMANCE WITHIN ACCEPTABLE RANGE")
            print("   No critical regressions detected")
        else:
            print("\n  PERFORMANCE MONITORING REQUIRED")
            print("   Some metrics lack baseline data")

        return report

    def _simulate_baseline_measurements(
        self, test_category: str
    ) -> Dict[str, List[float]]:
        """Simulate baseline measurements for testing (replace with actual test runs)."""
        # These are simulated measurements - in practice, you'd run the actual performance tests
        # multiple times to establish reliable baselines

        if test_category == "control_loop_latency":
            return {
                "avg_latency_ms": [45, 47, 46, 48, 44, 46, 45, 47, 46, 45],
                "max_latency_ms": [85, 88, 82, 90, 86, 84, 87, 89, 83, 85],
            }
        elif test_category == "vision_pipeline":
            return {
                "avg_frame_time_ms": [55, 58, 56, 57, 54, 56, 55, 58, 56, 55],
                "avg_memory_mb": [180, 185, 182, 188, 178, 183, 181, 186, 184, 182],
            }
        elif test_category == "terrain_analysis":
            return {
                "avg_classification_ms": [45, 47, 46, 44, 48, 46, 45, 47, 46, 45],
                "avg_traversability_ms": [28, 29, 27, 30, 28, 29, 27, 28, 29, 27],
            }
        elif test_category == "mission_execution":
            return {
                "avg_planning_ms": [
                    1800,
                    1850,
                    1750,
                    1900,
                    1800,
                    1850,
                    1750,
                    1800,
                    1850,
                    1800,
                ],
                "avg_execution_ms": [
                    4500,
                    4600,
                    4400,
                    4700,
                    4500,
                    4600,
                    4400,
                    4500,
                    4600,
                    4500,
                ],
            }
        elif test_category == "memory_usage":
            return {
                "memory_growth_mb_per_hour": [
                    0.8,
                    0.9,
                    0.7,
                    1.0,
                    0.8,
                    0.9,
                    0.7,
                    0.8,
                    0.9,
                    0.8,
                ],
                "peak_memory_mb": [580, 585, 575, 590, 580, 585, 575, 580, 585, 580],
            }
        elif test_category == "cpu_utilization":
            return {
                "idle_cpu_percent": [3, 4, 3, 5, 4, 3, 4, 3, 4, 3],
                "navigation_cpu_percent": [45, 47, 46, 48, 45, 47, 46, 45, 47, 46],
            }
        elif test_category == "network_bandwidth":
            return {
                "avg_bandwidth_mbps": [8, 9, 7, 10, 8, 9, 7, 8, 9, 8],
                "avg_compression_ratio": [
                    0.65,
                    0.67,
                    0.63,
                    0.68,
                    0.65,
                    0.67,
                    0.63,
                    0.65,
                    0.67,
                    0.65,
                ],
            }
        else:
            return {}

    def _simulate_current_results(self) -> Dict[str, Any]:
        """Simulate current test results for regression detection (replace with actual results)."""
        # These simulate current performance measurements with some intentional regressions
        # and improvements to test the regression detection system

        return {
            "control_loop_latency": {
                "avg_latency_ms": 52.0,  # Regression: was ~46ms baseline
                "max_latency_ms": 95.0,  # Regression: was ~85ms baseline
            },
            "vision_pipeline": {
                "avg_frame_time_ms": 58.0,  # Slight regression: was ~56ms baseline
                "avg_memory_mb": 175.0,  # Improvement: was ~182MB baseline
            },
            "terrain_analysis": {
                "avg_classification_ms": 43.0,  # Improvement: was ~46ms baseline
                "avg_traversability_ms": 26.0,  # Improvement: was ~28ms baseline
            },
            "mission_execution": {
                "avg_planning_ms": 1950.0,  # Regression: was ~1800ms baseline
                "avg_execution_ms": 4550.0,  # Slight regression: was ~4500ms baseline
            },
            "memory_usage": {
                "memory_growth_mb_per_hour": 1.2,  # Regression: was ~0.8MB/hr baseline
                "peak_memory_mb": 585.0,  # No significant change
            },
            "cpu_utilization": {
                "idle_cpu_percent": 4.0,  # No significant change
                "navigation_cpu_percent": 44.0,  # Slight improvement: was ~46% baseline
            },
            "network_bandwidth": {
                "avg_bandwidth_mbps": 7.5,  # Improvement: was ~8MBps baseline
                "avg_compression_ratio": 0.62,  # Improvement: was ~0.65 baseline
            },
        }


def run_performance_regression_tests():
    """Run the complete performance regression testing suite."""
    print("[IGNITE] URC 2026 Performance Regression Testing Framework")
    print("=" * 60)

    framework = PerformanceRegressionTest()

    try:
        # Establish baselines
        framework.run_baseline_establishment()

        # Run regression detection
        report = framework.run_regression_detection()

        # Export comprehensive baseline data
        export_file = f"performance_baselines_export_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        framework.framework.export_baselines(export_file)

        # Generate trend analysis
        trends = framework.framework.get_performance_trends()
        print(f"\n Performance Trends Analysis:")
        print("-" * 40)

        improving_trends = 0
        degrading_trends = 0

        for metric_key, trend_data in trends.items():
            trend_info = trend_data["trend_analysis"]
            if trend_info["trend"] == "insufficient_data":
                continue

            direction = trend_info["trend_direction"]
            magnitude = trend_info["trend_magnitude_percent"]

            if direction == "improving":
                improving_trends += 1
            elif direction == "degrading":
                degrading_trends += 1

            if magnitude > 1.0:  # Only show significant trends
                status = "" if direction == "improving" else ""
                print(".1f")

        print(
            f"\nTrend Summary: {improving_trends} improving, {degrading_trends} degrading"
        )

        return report["summary"]["regressions_detected"] == 0

    except Exception as e:
        print(f"[FAIL] Performance regression testing failed: {e}")
        return False


if __name__ == "__main__":
    success = run_performance_regression_tests()
    exit(0 if success else 1)
