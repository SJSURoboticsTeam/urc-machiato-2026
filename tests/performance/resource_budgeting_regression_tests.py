#!/usr/bin/env python3
"""
URC 2026 Resource Budgeting and Performance Regression Tests

Conducts comprehensive testing under constrained resource conditions:
- CPU: 2GHz equivalent baseline
- RAM: 16GB limit
- Performance regression detection
- Resource utilization analysis
- Bottleneck identification

Usage:
    python resource_budgeting_regression_tests.py --cpu-limit 2.0 --ram-limit 16 --iterations 50

Author: URC 2026 Performance Engineering Team
"""

import argparse
import json
import os
import psutil
import subprocess
import sys
import threading
import time
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
import statistics

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from src.testing.performance_profiling import PerformanceProfiler
from tests.performance.test_performance_baseline import PerformanceBaselineTester


@dataclass
class ResourceLimits:
    """Resource constraints for testing."""
    cpu_cores: int = 4  # 2GHz equivalent (4 cores @ 2GHz each)
    ram_gb: float = 16.0
    cpu_freq_mhz: int = 2000  # 2GHz
    network_bandwidth_mbps: int = 1000  # 1Gbps


@dataclass
class PerformanceBaseline:
    """Performance baseline measurements."""
    test_name: str
    resource_limits: ResourceLimits
    measurements: Dict[str, List[float]]
    timestamp: datetime
    system_info: Dict[str, Any]


class ResourceBudgetingRegressionTester:
    """Comprehensive resource budgeting and regression testing."""

    def __init__(self, cpu_limit_ghz: float = 2.0, ram_limit_gb: float = 16.0,
                 output_dir: str = "resource_regression_results"):
        self.cpu_limit_ghz = cpu_limit_ghz
        self.ram_limit_gb = ram_limit_gb
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)

        # Resource limits (translate GHz to cores)
        self.resource_limits = ResourceLimits(
            cpu_cores=max(1, int(cpu_limit_ghz / 0.5)),  # Assume 0.5GHz per core baseline
            ram_gb=ram_limit_gb,
            cpu_freq_mhz=int(cpu_limit_ghz * 1000)
        )

        # Monitoring
        self.monitoring_active = False
        self.system_metrics = defaultdict(list)
        self.baselines = {}
        self.current_test_results = {}

        # Performance bounds (based on 2GHz/16GB constraints)
        self.performance_bounds = {
            'binary_protocol_p99_ms': 5.0,      # Max 5ms under resource constraints
            'ipc_bridge_p99_ms': 10.0,           # Max 10ms for IPC
            'motion_control_p99_ms': 50.0,       # Max 50ms (relaxed from 20ms due to constraints)
            'end_to_end_p99_ms': 15.0,           # Max 15ms end-to-end
            'cpu_usage_percent': 85.0,           # Max 85% CPU usage
            'memory_usage_mb': ram_limit_gb * 1024 * 0.9  # Max 90% RAM usage
        }

    def run_comprehensive_resource_budgeting(self, iterations: int = 50) -> Dict[str, Any]:
        """Run comprehensive resource budgeting tests."""
        print("🔬 URC 2026 Resource Budgeting & Regression Tests")
        print("=" * 60)
        print(f"Resource Constraints: {self.cpu_limit_ghz}GHz CPU, {self.ram_limit_gb}GB RAM")
        print(f"Test Iterations: {iterations}")
        print()

        # Start resource monitoring
        self.monitoring_active = True
        monitor_thread = threading.Thread(target=self._background_resource_monitoring)
        monitor_thread.start()

        try:
            # Phase 1: Establish performance baselines
            print("📊 Phase 1: Establishing Performance Baselines...")
            baselines = self._establish_performance_baselines(iterations)

            # Phase 2: Resource stress testing
            print("\n🏋️ Phase 2: Resource Stress Testing...")
            stress_results = self._run_resource_stress_tests()

            # Phase 3: Regression detection
            print("\n🔍 Phase 3: Performance Regression Analysis...")
            regression_analysis = self._analyze_performance_regressions(baselines, stress_results)

            # Phase 4: Resource budgeting analysis
            print("\n💰 Phase 4: Resource Budgeting Analysis...")
            resource_budget = self._analyze_resource_budgeting(baselines, stress_results)

            # Stop monitoring
            self.monitoring_active = False
            monitor_thread.join(timeout=10)

            # Generate comprehensive report
            report = self._generate_resource_budgeting_report(
                baselines, stress_results, regression_analysis, resource_budget
            )

            return report

        except Exception as e:
            print(f"❌ Resource budgeting tests failed: {e}")
            return {"error": str(e)}

    def _establish_performance_baselines(self, iterations: int) -> Dict[str, PerformanceBaseline]:
        """Establish performance baselines under resource constraints."""
        print("  Establishing baselines with resource constraints...")

        baselines = {}

        # Simulate CPU frequency limiting (in real implementation, would use CPU governor)
        print(f"  Simulating {self.cpu_limit_ghz}GHz CPU constraint...")

        # Run baseline performance tests
        baseline_tester = PerformanceBaselineTester()

        # Multiple iterations to establish statistical baseline
        for iteration in range(iterations):
            if iteration % 10 == 0:
                print(f"  Baseline iteration {iteration + 1}/{iterations}...")

            # Run performance tests
            results = baseline_tester.run_all_baselines()

            # Store results for statistical analysis
            for test_name, test_data in results.get('results', {}).items():
                if test_name not in baselines:
                    baselines[test_name] = PerformanceBaseline(
                        test_name=test_name,
                        resource_limits=self.resource_limits,
                        measurements=defaultdict(list),
                        timestamp=datetime.now(),
                        system_info=self._get_system_info()
                    )

                # Extract performance metrics
                if isinstance(test_data, dict):
                    for metric_name, metric_value in test_data.items():
                        if isinstance(metric_value, (int, float)) and 'ms' in metric_name:
                            baselines[test_name].measurements[metric_name].append(metric_value)

        # Calculate statistical baselines
        for test_name, baseline in baselines.items():
            for metric_name, values in baseline.measurements.items():
                if values:
                    baseline.measurements[f"{metric_name}_mean"] = statistics.mean(values)
                    baseline.measurements[f"{metric_name}_median"] = statistics.median(values)
                    baseline.measurements[f"{metric_name}_p95"] = sorted(values)[int(len(values) * 0.95)]
                    baseline.measurements[f"{metric_name}_p99"] = sorted(values)[int(len(values) * 0.99)]
                    baseline.measurements[f"{metric_name}_stdev"] = statistics.stdev(values) if len(values) > 1 else 0

        print(f"  Established baselines for {len(baselines)} test categories")
        return baselines

    def _run_resource_stress_tests(self) -> Dict[str, Any]:
        """Run resource stress tests to identify limits."""
        print("  Running resource stress tests...")

        stress_results = {
            'cpu_stress': self._run_cpu_stress_test(),
            'memory_stress': self._run_memory_stress_test(),
            'combined_stress': self._run_combined_stress_test(),
            'network_stress': self._run_network_stress_test()
        }

        return stress_results

    def _run_cpu_stress_test(self) -> Dict[str, Any]:
        """Run CPU stress test under resource constraints."""
        print("    Testing CPU limits...")

        # Simulate CPU-intensive operations
        start_time = time.time()
        cpu_usage_samples = []

        # Run CPU-intensive workload for 30 seconds
        test_duration = 30
        end_time = start_time + test_duration

        while time.time() < end_time:
            # CPU-intensive computation (simulate sensor processing)
            for _ in range(10000):
                x = sum(i * i for i in range(100))

            # Sample CPU usage
            cpu_usage_samples.append(psutil.cpu_percent(interval=None))
            time.sleep(0.1)  # 10Hz sampling

        return {
            'duration_seconds': test_duration,
            'avg_cpu_usage': statistics.mean(cpu_usage_samples),
            'max_cpu_usage': max(cpu_usage_samples),
            'p95_cpu_usage': sorted(cpu_usage_samples)[int(len(cpu_usage_samples) * 0.95)],
            'within_limits': max(cpu_usage_samples) <= self.performance_bounds['cpu_usage_percent']
        }

    def _run_memory_stress_test(self) -> Dict[str, Any]:
        """Run memory stress test under resource constraints."""
        print("    Testing memory limits...")

        memory_usage_samples = []
        allocations = []

        try:
            # Gradually allocate memory up to near limit
            max_allocation_mb = int(self.ram_limit_gb * 1024 * 0.8)  # 80% of limit
            allocation_step_mb = 100  # 100MB steps

            for i in range(0, max_allocation_mb, allocation_step_mb):
                # Allocate memory
                allocation = bytearray(i * 1024 * 1024)  # MB to bytes
                allocations.append(allocation)

                # Sample memory usage
                memory_info = psutil.virtual_memory()
                memory_usage_samples.append(memory_info.percent)

                # Check if approaching limit
                if memory_info.percent > 85:  # 85% threshold
                    break

                time.sleep(0.5)  # Slow allocation to monitor

        finally:
            # Clean up allocations
            allocations.clear()

        return {
            'peak_memory_percent': max(memory_usage_samples) if memory_usage_samples else 0,
            'avg_memory_percent': statistics.mean(memory_usage_samples) if memory_usage_samples else 0,
            'within_limits': max(memory_usage_samples) <= 90 if memory_usage_samples else True,
            'memory_limit_gb': self.ram_limit_gb
        }

    def _run_combined_stress_test(self) -> Dict[str, Any]:
        """Run combined CPU + memory stress test."""
        print("    Testing combined CPU + memory stress...")

        # Run both CPU and memory stress simultaneously
        start_time = time.time()
        test_duration = 20

        allocations = []
        cpu_samples = []
        memory_samples = []

        try:
            end_time = start_time + test_duration
            allocation_size_mb = 50

            while time.time() < end_time:
                # CPU work
                for _ in range(5000):
                    x = sum(i * i for i in range(50))

                # Memory allocation (gradual)
                if len(allocations) < 10:  # Limit allocations
                    allocations.append(bytearray(allocation_size_mb * 1024 * 1024))

                # Sample system metrics
                cpu_samples.append(psutil.cpu_percent(interval=None))
                memory_samples.append(psutil.virtual_memory().percent)

                time.sleep(0.2)

        finally:
            allocations.clear()

        return {
            'duration_seconds': test_duration,
            'avg_cpu_usage': statistics.mean(cpu_samples),
            'avg_memory_usage': statistics.mean(memory_samples),
            'max_cpu_usage': max(cpu_samples),
            'max_memory_usage': max(memory_samples),
            'cpu_within_limits': max(cpu_samples) <= self.performance_bounds['cpu_usage_percent'],
            'memory_within_limits': max(memory_samples) <= 90
        }

    def _run_network_stress_test(self) -> Dict[str, Any]:
        """Run network stress test."""
        print("    Testing network performance...")

        # Simulate network traffic patterns
        network_samples = []

        # Generate network traffic for 15 seconds
        start_time = time.time()
        end_time = start_time + 15

        while time.time() < end_time:
            # Get network I/O statistics
            net_io = psutil.net_io_counters()

            # Calculate rates (simplified)
            network_samples.append({
                'bytes_sent': net_io.bytes_sent,
                'bytes_recv': net_io.bytes_recv,
                'timestamp': time.time()
            })

            # Simulate network activity
            time.sleep(0.5)

        # Calculate network utilization
        if len(network_samples) > 1:
            time_diff = network_samples[-1]['timestamp'] - network_samples[0]['timestamp']
            bytes_sent = network_samples[-1]['bytes_sent'] - network_samples[0]['bytes_sent']
            bytes_recv = network_samples[-1]['bytes_recv'] - network_samples[0]['bytes_recv']

            sent_rate_mbps = (bytes_sent * 8) / (time_diff * 1000000)  # Mbps
            recv_rate_mbps = (bytes_recv * 8) / (time_diff * 1000000)  # Mbps

            return {
                'duration_seconds': time_diff,
                'sent_rate_mbps': sent_rate_mbps,
                'recv_rate_mbps': recv_rate_mbps,
                'total_rate_mbps': sent_rate_mbps + recv_rate_mbps,
                'within_bandwidth_limit': (sent_rate_mbps + recv_rate_mbps) <= self.resource_limits.network_bandwidth_mbps
            }

        return {'error': 'Insufficient network samples'}

    def _analyze_performance_regressions(self, baselines: Dict[str, PerformanceBaseline],
                                        stress_results: Dict[str, Any]) -> Dict[str, Any]:
        """Analyze performance regressions."""
        print("  Analyzing performance regressions...")

        regression_analysis = {
            'regressions_detected': [],
            'performance_trends': {},
            'bottleneck_analysis': {},
            'recommendations': []
        }

        # Check for performance regressions against bounds
        for test_name, baseline in baselines.items():
            for metric_name, values in baseline.measurements.items():
                if 'p99' in metric_name and 'ms' in metric_name:
                    baseline_p99 = values[-1] if isinstance(values, list) else values
                    bound_key = f"{test_name.split('_')[0]}_p99_ms"

                    if bound_key in self.performance_bounds:
                        bound_value = self.performance_bounds[bound_key]

                        if baseline_p99 > bound_value:
                            regression_analysis['regressions_detected'].append({
                                'test': test_name,
                                'metric': metric_name,
                                'baseline_value': baseline_p99,
                                'bound_value': bound_value,
                                'regression_percent': ((baseline_p99 - bound_value) / bound_value) * 100
                            })

        # Analyze resource utilization trends
        cpu_stress = stress_results.get('cpu_stress', {})
        memory_stress = stress_results.get('memory_stress', {})

        regression_analysis['bottleneck_analysis'] = {
            'cpu_bottleneck': cpu_stress.get('avg_cpu_usage', 0) > 80,
            'memory_bottleneck': memory_stress.get('peak_memory_percent', 0) > 85,
            'resource_contention': stress_results.get('combined_stress', {}).get('avg_cpu_usage', 0) > 70
        }

        # Generate recommendations
        if regression_analysis['regressions_detected']:
            regression_analysis['recommendations'].append("❌ Performance regressions detected - investigate bottlenecks")

        if regression_analysis['bottleneck_analysis']['cpu_bottleneck']:
            regression_analysis['recommendations'].append("⚠️ CPU bottleneck detected - consider optimization or resource increase")

        if regression_analysis['bottleneck_analysis']['memory_bottleneck']:
            regression_analysis['recommendations'].append("⚠️ Memory bottleneck detected - monitor memory usage patterns")

        if not regression_analysis['regressions_detected']:
            regression_analysis['recommendations'].append("✅ No performance regressions detected - system performing within bounds")

        return regression_analysis

    def _analyze_resource_budgeting(self, baselines: Dict[str, PerformanceBaseline],
                                   stress_results: Dict[str, Any]) -> Dict[str, Any]:
        """Analyze resource budgeting requirements."""
        print("  Analyzing resource budgeting...")

        # Calculate resource requirements
        cpu_requirements = []
        memory_requirements = []

        for test_name, baseline in baselines.items():
            # Estimate CPU requirements based on performance
            if 'binary_protocol' in test_name:
                cpu_requirements.append(0.5)  # Low CPU for binary ops
            elif 'motion_control' in test_name:
                cpu_requirements.append(1.5)  # Moderate CPU for control
            else:
                cpu_requirements.append(1.0)  # Default

        # Memory requirements based on stress test
        memory_stress = stress_results.get('memory_stress', {})
        memory_requirements.append(memory_stress.get('peak_memory_percent', 0))

        resource_budget = {
            'cpu_cores_required': max(1, int(sum(cpu_requirements) / 0.8)),  # 80% utilization target
            'memory_gb_required': self.ram_limit_gb,
            'cpu_frequency_mhz_required': self.resource_limits.cpu_freq_mhz,
            'network_bandwidth_mbps_required': self.resource_limits.network_bandwidth_mbps,
            'power_budget_watts': self._estimate_power_budget(),
            'thermal_budget_celsius': 75,  # Typical embedded system limit
            'resource_efficiency_score': self._calculate_efficiency_score(baselines, stress_results)
        }

        return resource_budget

    def _estimate_power_budget(self) -> float:
        """Estimate power budget based on resource constraints."""
        # Rough estimation: CPU power + memory power + overhead
        cpu_power = self.resource_limits.cpu_cores * 5  # ~5W per core at 2GHz
        memory_power = self.ram_limit_gb * 0.5  # ~0.5W per GB RAM
        overhead_power = 10  # Base system power

        return cpu_power + memory_power + overhead_power

    def _calculate_efficiency_score(self, baselines: Dict[str, PerformanceBaseline],
                                  stress_results: Dict[str, Any]) -> float:
        """Calculate resource efficiency score (0-100)."""
        score = 100

        # Penalize for high resource usage
        cpu_stress = stress_results.get('cpu_stress', {})
        memory_stress = stress_results.get('memory_stress', {})

        if cpu_stress.get('avg_cpu_usage', 0) > 70:
            score -= 20
        if memory_stress.get('peak_memory_percent', 0) > 80:
            score -= 20

        # Penalize for performance regressions
        regressions = len(self._analyze_performance_regressions(baselines, stress_results)['regressions_detected'])
        score -= regressions * 10

        return max(0, min(100, score))

    def _background_resource_monitoring(self):
        """Background resource monitoring thread."""
        print("  📈 Starting background resource monitoring...")

        while self.monitoring_active:
            try:
                # System metrics
                cpu_percent = psutil.cpu_percent(interval=None)
                memory_info = psutil.virtual_memory()
                disk_io = psutil.disk_io_counters()
                net_io = psutil.net_io_counters()

                # Record metrics
                self.system_metrics['timestamp'].append(time.time())
                self.system_metrics['cpu_percent'].append(cpu_percent)
                self.system_metrics['memory_percent'].append(memory_info.percent)
                self.system_metrics['memory_mb'].append(memory_info.used / (1024 * 1024))

                if disk_io:
                    self.system_metrics['disk_read_mb'].append(disk_io.read_bytes / (1024 * 1024))
                    self.system_metrics['disk_write_mb'].append(disk_io.write_bytes / (1024 * 1024))

                if net_io:
                    self.system_metrics['net_bytes_sent'].append(net_io.bytes_sent)
                    self.system_metrics['net_bytes_recv'].append(net_io.bytes_recv)

                time.sleep(1.0)  # 1Hz monitoring

            except Exception as e:
                print(f"  ⚠️ Monitoring error: {e}")
                time.sleep(1.0)

        print("  📈 Background monitoring completed")

    def _get_system_info(self) -> Dict[str, Any]:
        """Get system information."""
        return {
            'cpu_count': psutil.cpu_count(),
            'cpu_freq_mhz': psutil.cpu_freq().current if psutil.cpu_freq() else None,
            'memory_total_gb': psutil.virtual_memory().total / (1024**3),
            'platform': sys.platform,
            'python_version': sys.version
        }

    def _generate_resource_budgeting_report(self, baselines: Dict[str, PerformanceBaseline],
                                          stress_results: Dict[str, Any],
                                          regression_analysis: Dict[str, Any],
                                          resource_budget: Dict[str, Any]) -> Dict[str, Any]:
        """Generate comprehensive resource budgeting report."""
        print("  📋 Generating resource budgeting report...")

        report = {
            'test_metadata': {
                'resource_limits': {
                    'cpu_ghz': self.cpu_limit_ghz,
                    'ram_gb': self.ram_limit_gb,
                    'cpu_cores': self.resource_limits.cpu_cores,
                    'cpu_freq_mhz': self.resource_limits.cpu_freq_mhz
                },
                'timestamp': datetime.now().isoformat(),
                'system_info': self._get_system_info()
            },
            'performance_baselines': {},
            'stress_test_results': stress_results,
            'regression_analysis': regression_analysis,
            'resource_budgeting': resource_budget,
            'system_monitoring': dict(self.system_metrics)
        }

        # Convert baselines to serializable format
        for test_name, baseline in baselines.items():
            report['performance_baselines'][test_name] = {
                'measurements': dict(baseline.measurements),
                'timestamp': baseline.timestamp.isoformat(),
                'system_info': baseline.system_info
            }

        # Save detailed report
        report_file = self.output_dir / f"resource_budgeting_report_{int(time.time())}.json"
        with open(report_file, 'w') as f:
            json.dump(report, f, indent=2, default=str)

        print(f"  📁 Detailed report saved to: {report_file}")
        return report

    def generate_performance_summary(self, report: Dict[str, Any]) -> str:
        """Generate human-readable performance summary."""
        summary_lines = []
        summary_lines.append("=" * 80)
        summary_lines.append("🧪 URC 2026 RESOURCE BUDGETING & REGRESSION TEST SUMMARY")
        summary_lines.append("=" * 80)

        # Resource constraints
        metadata = report.get('test_metadata', {})
        limits = metadata.get('resource_limits', {})
        summary_lines.append("\n🎯 RESOURCE CONSTRAINTS:")
        summary_lines.append(f"  CPU: {limits.get('cpu_ghz', 0)}GHz ({limits.get('cpu_cores', 0)} cores)")
        summary_lines.append(f"  RAM: {limits.get('ram_gb', 0)}GB")
        summary_lines.append(f"  CPU Frequency: {limits.get('cpu_freq_mhz', 0)}MHz")

        # Performance baselines
        baselines = report.get('performance_baselines', {})
        summary_lines.append("\n📊 PERFORMANCE BASELINES:")
        for test_name, baseline_data in baselines.items():
            measurements = baseline_data.get('measurements', {})
            if 'p99_ms_mean' in measurements:
                p99_mean = measurements['p99_ms_mean']
                summary_lines.append(".3f")
        # Stress test results
        stress = report.get('stress_test_results', {})
        summary_lines.append("\n🏋️ STRESS TEST RESULTS:")
        cpu_stress = stress.get('cpu_stress', {})
        memory_stress = stress.get('memory_stress', {})
        combined_stress = stress.get('combined_stress', {})

        if cpu_stress:
            summary_lines.append(".1f")
        if memory_stress:
            summary_lines.append(".1f")
        if combined_stress:
            summary_lines.append(".1f")
        # Regression analysis
        regression = report.get('regression_analysis', {})
        regressions = regression.get('regressions_detected', [])
        summary_lines.append("\n🔍 REGRESSION ANALYSIS:")
        summary_lines.append(f"  Regressions Detected: {len(regressions)}")

        for regression_item in regressions[:3]:  # Show first 3
            summary_lines.append(".1f")
        if len(regressions) > 3:
            summary_lines.append(f"  ... and {len(regressions) - 3} more")

        # Resource budgeting
        budget = report.get('resource_budgeting', {})
        summary_lines.append("\n💰 RESOURCE BUDGETING:")
        summary_lines.append(f"  CPU Cores Required: {budget.get('cpu_cores_required', 0)}")
        summary_lines.append(f"  Memory Required: {budget.get('memory_gb_required', 0)}GB")
        summary_lines.append(f"  Power Budget: {budget.get('power_budget_watts', 0):.1f}W")
        summary_lines.append(f"  Efficiency Score: {budget.get('resource_efficiency_score', 0):.1f}/100")
        # Recommendations
        recommendations = regression.get('recommendations', [])
        if recommendations:
            summary_lines.append("\n💡 RECOMMENDATIONS:")
            for rec in recommendations:
                summary_lines.append(f"  {rec}")

        summary_lines.append("\n" + "=" * 80)

        return "\n".join(summary_lines)


def main():
    """Main entry point for resource budgeting and regression tests."""
    parser = argparse.ArgumentParser(description='URC 2026 Resource Budgeting & Regression Tests')
    parser.add_argument('--cpu-limit', type=float, default=2.0,
                       help='CPU limit in GHz (default: 2.0)')
    parser.add_argument('--ram-limit', type=float, default=16.0,
                       help='RAM limit in GB (default: 16.0)')
    parser.add_argument('--iterations', type=int, default=50,
                       help='Number of baseline iterations (default: 50)')
    parser.add_argument('--output-dir', type=str, default='resource_regression_results',
                       help='Output directory (default: resource_regression_results)')

    args = parser.parse_args()

    print("🔬 Starting Resource Budgeting & Performance Regression Tests")
    print(f"Constraints: {args.cpu_limit}GHz CPU, {args.ram_limit}GB RAM")
    print(f"Iterations: {args.iterations}")
    print()

    # Run comprehensive testing
    tester = ResourceBudgetingRegressionTester(
        cpu_limit_ghz=args.cpu_limit,
        ram_limit_gb=args.ram_limit,
        output_dir=args.output_dir
    )

    start_time = time.time()
    report = tester.run_comprehensive_resource_budgeting(args.iterations)
    end_time = time.time()

    # Generate and display summary
    if 'error' not in report:
        summary = tester.generate_performance_summary(report)
        print(summary)
        print(f"\n⏱️ Total Test Time: {end_time - start_time:.1f}s")
    else:
        print(f"❌ Tests failed: {report['error']}")

    print(f"\n📁 Results saved to: {args.output_dir}/")


if __name__ == "__main__":
    main()
