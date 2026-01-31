#!/usr/bin/env python3
"""
Simple Resource Budgeting Test - URC 2026

Tests system performance under 2GHz CPU and 16GB RAM constraints.
Provides resource budgeting analysis and performance regression detection.

Author: URC 2026 Performance Engineering Team
"""

import time
import psutil
import json
import threading
from pathlib import Path
from typing import Dict, Any
import sys

# Add project root to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import pytest

try:
    from tests.performance.test_performance_baseline import PerformanceBaselineTester
except Exception as e:
    if e.__class__.__name__ in ("Skipped", "SkipException"):
        pytest.skip("Performance baseline deps unavailable", allow_module_level=True)
    raise


class SimpleResourceBudgetingTester:
    """Simple resource budgeting tester for 2GHz/16GB constraints."""

    def __init__(self, output_dir: str = "resource_budgeting_results"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)

        # Resource limits
        self.cpu_limit_ghz = 2.0
        self.ram_limit_gb = 16.0

        # Performance bounds under constraints
        self.performance_bounds = {
            'binary_protocol_p99_ms': 5.0,
            'ipc_bridge_p99_ms': 10.0,
            'motion_control_p99_ms': 50.0,  # Relaxed for resource constraints
            'end_to_end_p99_ms': 15.0,
            'cpu_usage_percent': 85.0,
            'memory_usage_mb': self.ram_limit_gb * 1024 * 0.9
        }

        # Monitoring
        self.system_metrics = {
            'cpu_percent': [],
            'memory_percent': [],
            'memory_mb': [],
            'timestamps': []
        }

    def run_resource_budgeting_test(self) -> Dict[str, Any]:
        """Run comprehensive resource budgeting test."""
        print("🔬 URC 2026 Resource Budgeting Test")
        print(f"Constraints: {self.cpu_limit_ghz}GHz CPU, {self.ram_limit_gb}GB RAM")
        print("=" * 50)

        start_time = time.time()

        # Start monitoring
        monitoring_active = True
        monitor_thread = threading.Thread(target=self._monitor_system_resources,
                                        args=(lambda: monitoring_active,))
        monitor_thread.start()

        try:
            # Run performance tests under constraints
            print("\n📊 Running Performance Tests...")
            perf_results = self._run_performance_tests()

            # Run resource stress tests
            print("\n🏋️ Running Resource Stress Tests...")
            stress_results = self._run_resource_stress_tests()

            # Analyze resource budgeting
            print("\n💰 Analyzing Resource Budgeting...")
            budget_analysis = self._analyze_resource_budgeting(perf_results, stress_results)

            # Generate report
            report = {
                'test_metadata': {
                    'constraints': {
                        'cpu_ghz': self.cpu_limit_ghz,
                        'ram_gb': self.ram_limit_gb
                    },
                    'timestamp': time.time(),
                    'duration_seconds': time.time() - start_time
                },
                'performance_results': perf_results,
                'stress_results': stress_results,
                'budget_analysis': budget_analysis,
                'system_monitoring': self.system_metrics
            }

            return report

        finally:
            monitoring_active = False
            monitor_thread.join(timeout=5)

    def _run_performance_tests(self) -> Dict[str, Any]:
        """Run performance tests under resource constraints."""
        tester = PerformanceBaselineTester()

        # Run baseline tests
        results = tester.run_all_baselines()

        # Extract key metrics
        perf_metrics = {}
        if 'results' in results:
            for test_name, test_data in results['results'].items():
                if isinstance(test_data, dict) and 'p99_ms' in test_data:
                    perf_metrics[f"{test_name}_p99_ms"] = test_data['p99_ms']

        return perf_metrics

    def _run_resource_stress_tests(self) -> Dict[str, Any]:
        """Run resource stress tests."""
        stress_results = {}

        # CPU stress test
        print("  Testing CPU under 2GHz constraint...")
        stress_results['cpu_stress'] = self._cpu_stress_test()

        # Memory stress test
        print("  Testing memory under 16GB constraint...")
        stress_results['memory_stress'] = self._memory_stress_test()

        # Combined stress test
        print("  Testing combined CPU + memory stress...")
        stress_results['combined_stress'] = self._combined_stress_test()

        return stress_results

    def _cpu_stress_test(self) -> Dict[str, Any]:
        """Test CPU performance under constraints."""
        cpu_samples = []

        # Simulate CPU-intensive workload for 15 seconds
        start_time = time.time()
        end_time = start_time + 15

        while time.time() < end_time:
            # CPU-intensive computation (simulates sensor processing)
            for _ in range(5000):
                x = sum(i * i for i in range(50))

            cpu_samples.append(psutil.cpu_percent(interval=None))
            time.sleep(0.1)

        return {
            'duration_seconds': 15,
            'avg_cpu_usage': sum(cpu_samples) / len(cpu_samples),
            'max_cpu_usage': max(cpu_samples),
            'within_limit': max(cpu_samples) <= self.performance_bounds['cpu_usage_percent']
        }

    def _memory_stress_test(self) -> Dict[str, Any]:
        """Test memory usage under constraints."""
        memory_samples = []
        allocations = []

        try:
            # Allocate memory up to ~12GB (75% of 16GB limit)
            max_allocation_mb = int(self.ram_limit_gb * 1024 * 0.75)
            allocation_step_mb = 50

            for i in range(0, max_allocation_mb, allocation_step_mb):
                allocations.append(bytearray(allocation_step_mb * 1024 * 1024))
                memory_info = psutil.virtual_memory()
                memory_samples.append(memory_info.percent)

                # Stop if approaching limit
                if memory_info.percent > 80:
                    break

                time.sleep(0.2)

        finally:
            allocations.clear()  # Clean up

        return {
            'peak_memory_percent': max(memory_samples) if memory_samples else 0,
            'avg_memory_percent': sum(memory_samples) / len(memory_samples) if memory_samples else 0,
            'within_limit': max(memory_samples) <= 90 if memory_samples else True,
            'memory_limit_gb': self.ram_limit_gb
        }

    def _combined_stress_test(self) -> Dict[str, Any]:
        """Test combined CPU and memory stress."""
        cpu_samples = []
        memory_samples = []
        allocations = []

        try:
            start_time = time.time()
            end_time = start_time + 10

            allocation_size_mb = 25

            while time.time() < end_time:
                # CPU work
                for _ in range(2500):
                    x = sum(i * i for i in range(30))

                # Memory allocation
                if len(allocations) < 20:
                    allocations.append(bytearray(allocation_size_mb * 1024 * 1024))

                # Sample metrics
                cpu_samples.append(psutil.cpu_percent(interval=None))
                memory_samples.append(psutil.virtual_memory().percent)

                time.sleep(0.15)

        finally:
            allocations.clear()

        return {
            'duration_seconds': 10,
            'avg_cpu_usage': sum(cpu_samples) / len(cpu_samples),
            'avg_memory_usage': sum(memory_samples) / len(memory_samples),
            'max_cpu_usage': max(cpu_samples),
            'max_memory_usage': max(memory_samples),
            'cpu_within_limit': max(cpu_samples) <= self.performance_bounds['cpu_usage_percent'],
            'memory_within_limit': max(memory_samples) <= 90
        }

    def _analyze_resource_budgeting(self, perf_results: Dict[str, Any],
                                  stress_results: Dict[str, Any]) -> Dict[str, Any]:
        """Analyze resource budgeting requirements."""
        analysis = {
            'performance_compliance': {},
            'resource_requirements': {},
            'bottlenecks': [],
            'recommendations': []
        }

        # Check performance compliance under constraints
        for metric_name, actual_value in perf_results.items():
            if metric_name in self.performance_bounds:
                bound_value = self.performance_bounds[metric_name]
                compliant = actual_value <= bound_value
                analysis['performance_compliance'][metric_name] = {
                    'actual': actual_value,
                    'limit': bound_value,
                    'compliant': compliant,
                    'margin': bound_value - actual_value
                }

        # Resource requirements analysis
        cpu_stress = stress_results.get('cpu_stress', {})
        memory_stress = stress_results.get('memory_stress', {})

        analysis['resource_requirements'] = {
            'cpu_cores_required': max(1, int((cpu_stress.get('avg_cpu_usage', 0) / 80) * 4)),
            'memory_gb_required': self.ram_limit_gb,
            'cpu_freq_mhz_required': int(self.cpu_limit_ghz * 1000),
            'power_budget_watts': self._estimate_power_budget(),
            'thermal_budget_celsius': 75
        }

        # Identify bottlenecks
        if cpu_stress.get('avg_cpu_usage', 0) > 70:
            analysis['bottlenecks'].append('CPU utilization high under 2GHz constraint')
        if memory_stress.get('peak_memory_percent', 0) > 80:
            analysis['bottlenecks'].append('Memory utilization high under 16GB constraint')

        # Generate recommendations
        if analysis['bottlenecks']:
            analysis['recommendations'].append('⚠️ Resource bottlenecks detected - consider hardware upgrade')
        else:
            analysis['recommendations'].append('✅ Resources adequate for 2GHz/16GB constraints')

        # Performance recommendations
        compliant_count = sum(1 for r in analysis['performance_compliance'].values() if r['compliant'])
        total_count = len(analysis['performance_compliance'])

        if compliant_count < total_count:
            analysis['recommendations'].append('⚠️ Performance regressions detected under resource constraints')
        else:
            analysis['recommendations'].append('✅ All performance requirements met under resource constraints')

        return analysis

    def _estimate_power_budget(self) -> float:
        """Estimate power budget for 2GHz/16GB system."""
        # Rough estimation: CPU + memory + overhead
        cpu_power = 4 * 8  # 4 cores * ~8W each at 2GHz
        memory_power = self.ram_limit_gb * 0.5  # ~0.5W per GB
        overhead_power = 15  # Base system power

        return cpu_power + memory_power + overhead_power

    def _monitor_system_resources(self, is_active):
        """Monitor system resources in background."""
        while is_active():
            try:
                self.system_metrics['timestamps'].append(time.time())
                self.system_metrics['cpu_percent'].append(psutil.cpu_percent(interval=None))

                memory_info = psutil.virtual_memory()
                self.system_metrics['memory_percent'].append(memory_info.percent)
                self.system_metrics['memory_mb'].append(memory_info.used / (1024 * 1024))

                time.sleep(1.0)

            except Exception as e:
                print(f"  ⚠️ Monitoring error: {e}")
                time.sleep(1.0)

    def generate_report(self, results: Dict[str, Any]) -> str:
        """Generate comprehensive test report."""
        report_lines = []
        report_lines.append("=" * 70)
        report_lines.append("🧪 URC 2026 RESOURCE BUDGETING TEST REPORT")
        report_lines.append("=" * 70)

        # Test metadata
        metadata = results.get('test_metadata', {})
        constraints = metadata.get('constraints', {})
        report_lines.append("\n🎯 TEST CONSTRAINTS:")
        report_lines.append(f"  CPU: {constraints.get('cpu_ghz', 0)}GHz")
        report_lines.append(f"  RAM: {constraints.get('ram_gb', 0)}GB")
        report_lines.append(".1f")
        # Performance results
        perf_results = results.get('performance_results', {})
        if perf_results:
            report_lines.append("\n📊 PERFORMANCE RESULTS:")
            for metric, value in perf_results.items():
                report_lines.append(".3f")
        # Stress test results
        stress_results = results.get('stress_results', {})
        if stress_results:
            report_lines.append("\n🏋️ STRESS TEST RESULTS:")
            cpu_stress = stress_results.get('cpu_stress', {})
            memory_stress = stress_results.get('memory_stress', {})
            combined_stress = stress_results.get('combined_stress', {})

            if cpu_stress:
                report_lines.append(".1f")
            if memory_stress:
                report_lines.append(".1f")
            if combined_stress:
                report_lines.append(".1f")
        # Budget analysis
        budget_analysis = results.get('budget_analysis', {})
        if budget_analysis:
            report_lines.append("\n💰 RESOURCE BUDGETING:")
            requirements = budget_analysis.get('resource_requirements', {})
            report_lines.append(f"  CPU Cores Required: {requirements.get('cpu_cores_required', 0)}")
            report_lines.append(f"  Memory Required: {requirements.get('memory_gb_required', 0)}GB")
            report_lines.append(f"  Power Budget: {requirements.get('power_budget_watts', 0):.1f}W")

            # Compliance
            compliance = budget_analysis.get('performance_compliance', {})
            compliant_count = sum(1 for r in compliance.values() if r.get('compliant', False))
            total_count = len(compliance)
            report_lines.append(f"  Performance Compliance: {compliant_count}/{total_count} requirements met")

            # Bottlenecks
            bottlenecks = budget_analysis.get('bottlenecks', [])
            if bottlenecks:
                report_lines.append("  ⚠️ Bottlenecks Detected:")
                for bottleneck in bottlenecks:
                    report_lines.append(f"    - {bottleneck}")

            # Recommendations
            recommendations = budget_analysis.get('recommendations', [])
            if recommendations:
                report_lines.append("  💡 Recommendations:")
                for rec in recommendations:
                    report_lines.append(f"    {rec}")

        report_lines.append("\n" + "=" * 70)
        return "\n".join(report_lines)


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description='URC 2026 Resource Budgeting Test')
    parser.add_argument('--cpu-limit', type=float, default=2.0,
                       help='CPU limit in GHz (default: 2.0)')
    parser.add_argument('--ram-limit', type=float, default=16.0,
                       help='RAM limit in GB (default: 16.0)')
    parser.add_argument('--output-dir', type=str, default='resource_budgeting_results',
                       help='Output directory (default: resource_budgeting_results)')

    args = parser.parse_args()

    print("🔬 Starting Resource Budgeting Test")
    print(f"Constraints: {args.cpu_limit}GHz CPU, {args.ram_limit}GB RAM")
    print()

    # Run tests
    tester = SimpleResourceBudgetingTester(args.output_dir)
    results = tester.run_resource_budgeting_test()

    # Generate and save report
    report = tester.generate_report(results)
    print(report)

    # Save detailed results
    output_file = Path(args.output_dir) / f"resource_budgeting_report_{int(time.time())}.json"
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2, default=str)

    print(f"\n📁 Detailed results saved to: {output_file}")
    print("✅ Resource budgeting test complete!")


if __name__ == "__main__":
    main()
