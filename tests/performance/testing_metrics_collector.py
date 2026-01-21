#!/usr/bin/env python3
"""
URC 2026 Testing Metrics Collector

Automatically collects performance metrics during test execution:
- Performance benchmarks (latency, throughput)
- Resource utilization (CPU, memory, network)
- Test results and pass/fail status
- System health monitoring
- Regression detection

Integrates with pytest and unittest frameworks.

Usage:
    # During testing
    python -m pytest --metrics-collector tests/ -v

    # Or run standalone
    python testing_metrics_collector.py --run-tests

Author: URC 2026 Testing and Metrics Team
"""

import argparse
import atexit
import functools
import json
import os
import psutil
import sys
import threading
import time
import unittest
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Any, Optional, Callable
import inspect
import traceback

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from testing_metrics_dashboard import metrics_store
from system_integration_monitor import start_system_integration_monitoring, stop_system_integration_monitoring


class TestMetricsCollector:
    """Collects metrics during test execution."""

    def __init__(self, output_dir: str = "./test_metrics"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)

        self.test_results = []
        self.current_test = None
        self.test_start_time = None
        self.monitoring_active = False
        self.monitor_thread = None

        # Register cleanup
        atexit.register(self.save_results)

    def start_collection(self):
        """Start metrics collection."""
        self.monitoring_active = True
        self.monitor_thread = threading.Thread(target=self._background_monitoring, daemon=True)
        self.monitor_thread.start()

        # Start system integration monitoring
        self.system_monitor = start_system_integration_monitoring()

        # Monkey patch unittest for automatic collection
        original_run = unittest.TestCase.run
        def patched_run(self, result=None):
            test_name = f"{self.__class__.__name__}.{self._testMethodName}"
            collector.on_test_start(test_name)
            try:
                result = original_run(self, result)
                collector.on_test_end(test_name, 'PASS' if result.wasSuccessful() else 'FAIL')
                return result
            except Exception as e:
                collector.on_test_end(test_name, 'ERROR', str(e))
                raise

        unittest.TestCase.run = patched_run

    def stop_collection(self):
        """Stop metrics collection."""
        self.monitoring_active = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=5)

        # Stop system integration monitoring
        stop_system_integration_monitoring()

    def on_test_start(self, test_name: str):
        """Called when a test starts."""
        self.current_test = test_name
        self.test_start_time = time.time()

        metrics_store.update_metric('tests', 'active_test', 1)
        metrics_store.update_metric('tests', f'{test_name}_start', time.time())

        print(f"🧪 Starting test: {test_name}")

    def on_test_end(self, test_name: str, status: str, error: Optional[str] = None):
        """Called when a test ends."""
        if self.test_start_time:
            duration = time.time() - self.test_start_time

            self.test_results.append({
                'test_name': test_name,
                'status': status,
                'duration': duration,
                'error': error,
                'timestamp': time.time()
            })

            # Update metrics
            metrics_store.update_metric('tests', 'test_duration', duration)
            metrics_store.update_metric('tests', f'{test_name}_duration', duration)
            metrics_store.update_metric('tests', f'{test_name}_status', 1 if status == 'PASS' else 0)

            print(f"{'✅' if status == 'PASS' else '❌'} Test {test_name}: {status} ({duration:.2f}s)")

        self.current_test = None
        self.test_start_time = None
        metrics_store.update_metric('tests', 'active_test', 0)

    def collect_performance_metric(self, category: str, metric: str, value: float,
                                 test_context: Optional[str] = None):
        """Collect a performance metric."""
        metrics_store.update_metric(category, metric, value)

        if test_context:
            metrics_store.update_metric(f"{category}_by_test", f"{test_context}_{metric}", value)

    def _background_monitoring(self):
        """Background system monitoring."""
        process = psutil.Process()

        while self.monitoring_active:
            try:
                # System metrics
                cpu_percent = psutil.cpu_percent(interval=None)
                memory_info = psutil.virtual_memory()
                memory_mb = memory_info.used / (1024 * 1024)

                # Network I/O (simplified)
                net_io = psutil.net_io_counters()
                if net_io:
                    net_bytes_total = net_io.bytes_sent + net_io.bytes_recv

                # Update metrics store
                metrics_store.update_metric('system', 'cpu_percent', cpu_percent)
                metrics_store.update_metric('system', 'memory_mb', memory_mb)
                metrics_store.update_metric('system', 'memory_percent', memory_info.percent)

                if net_io:
                    metrics_store.update_metric('system', 'net_bytes_total', net_bytes_total)

                # Context-specific metrics
                if self.current_test:
                    metrics_store.update_metric('system_by_test',
                                              f"{self.current_test}_cpu", cpu_percent)
                    metrics_store.update_metric('system_by_test',
                                              f"{self.current_test}_memory", memory_mb)

                time.sleep(1.0)  # 1Hz monitoring

            except Exception as e:
                print(f"⚠️ Monitoring error: {e}")
                time.sleep(1.0)

    def save_results(self):
        """Save collected results."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        # Save test results
        test_results_file = self.output_dir / f"test_results_{timestamp}.json"
        with open(test_results_file, 'w') as f:
            json.dump({
                'collection_timestamp': timestamp,
                'test_results': self.test_results,
                'summary': {
                    'total_tests': len(self.test_results),
                    'passed': len([t for t in self.test_results if t['status'] == 'PASS']),
                    'failed': len([t for t in self.test_results if t['status'] == 'FAIL']),
                    'errors': len([t for t in self.test_results if t['status'] == 'ERROR']),
                    'total_duration': sum(t['duration'] for t in self.test_results)
                }
            }, f, indent=2)

        # Save metrics snapshot
        metrics_file = self.output_dir / f"metrics_snapshot_{timestamp}.json"
        dashboard_data = metrics_store.get_dashboard_data()
        with open(metrics_file, 'w') as f:
            json.dump(dashboard_data, f, indent=2, default=str)

        print(f"💾 Results saved to {self.output_dir}")
        print(f"  📄 Test results: {test_results_file}")
        print(f"  📊 Metrics: {metrics_file}")


# Global collector instance
collector = TestMetricsCollector()


def performance_test(func: Callable) -> Callable:
    """Decorator to automatically collect performance metrics for tests."""
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        test_name = f"{func.__module__}.{func.__name__}"
        collector.on_test_start(test_name)

        start_time = time.time()
        start_cpu = psutil.cpu_percent(interval=None)
        start_memory = psutil.virtual_memory().used

        try:
            result = func(*args, **kwargs)

            # Collect performance metrics
            end_time = time.time()
            end_cpu = psutil.cpu_percent(interval=None)
            end_memory = psutil.virtual_memory().used

            duration = end_time - start_time
            cpu_delta = end_cpu - start_cpu
            memory_delta = end_memory - start_memory

            collector.collect_performance_metric('performance', 'test_duration', duration, test_name)
            collector.collect_performance_metric('performance', 'cpu_usage', cpu_delta, test_name)
            collector.collect_performance_metric('performance', 'memory_delta_mb', memory_delta / (1024*1024), test_name)

            collector.on_test_end(test_name, 'PASS')
            return result

        except Exception as e:
            collector.on_test_end(test_name, 'ERROR', str(e))
            raise

    return wrapper


def benchmark_operation(operation_name: str, category: str = 'benchmarks'):
    """Decorator to benchmark specific operations."""
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            start_time = time.perf_counter_ns()

            try:
                result = func(*args, **kwargs)
                end_time = time.perf_counter_ns()

                duration_ns = end_time - start_time
                duration_ms = duration_ns / 1_000_000

                collector.collect_performance_metric(category, f"{operation_name}_latency_ms", duration_ms)
                collector.collect_performance_metric(category, f"{operation_name}_latency_ns", duration_ns)

                return result
            except Exception as e:
                # Still record the failed operation
                collector.collect_performance_metric(category, f"{operation_name}_error", 1)
                raise

        return wrapper
    return decorator


def run_test_suite_with_metrics():
    """Run the test suite with metrics collection."""
    print("🚀 Running URC 2026 Test Suite with Metrics Collection")
    print("=" * 60)

    # Start metrics collection
    collector.start_collection()

    try:
        # Import and run tests
        import subprocess
        import sys

        # Run pytest with our collection
        cmd = [sys.executable, '-m', 'pytest', 'tests/', '-v', '--tb=short']
        result = subprocess.run(cmd, capture_output=True, text=True)

        print("Test Output:")
        print(result.stdout)
        if result.stderr:
            print("Errors:")
            print(result.stderr)

        print(f"\nExit code: {result.returncode}")

    finally:
        # Stop collection and save results
        collector.stop_collection()
        collector.save_results()

        print("\n📊 Test Summary:")
        dashboard_data = metrics_store.get_dashboard_data()
        summary = dashboard_data['summary']
        print(f"  Total Metrics: {summary['total_metrics']}")
        print(f"  Categories: {summary['total_categories']}")
        print(f"  Active Alerts: {summary['active_alerts']}")


def demonstrate_metrics_collection():
    """Demonstrate metrics collection with sample tests."""
    print("🎭 Demonstrating Metrics Collection")
    print("=" * 40)

    collector.start_collection()

    @performance_test
    def sample_performance_test():
        """Sample performance test."""
        import time
        time.sleep(0.1)  # Simulate work
        return "success"

    @benchmark_operation("matrix_multiplication", "math_operations")
    def matrix_multiply():
        """Sample benchmarked operation."""
        import numpy as np
        a = np.random.rand(50, 50)
        b = np.random.rand(50, 50)
        return np.dot(a, b)

    # Run sample tests
    print("Running sample tests...")
    result1 = sample_performance_test()
    result2 = matrix_multiply()

    print(f"Test 1 result: {result1}")
    print("Matrix multiplication completed")

    # Simulate some system metrics
    for i in range(10):
        collector.collect_performance_metric('demo', 'counter', i)
        collector.collect_performance_metric('demo', 'random_value', i * 2.5 + (i % 3))
        time.sleep(0.1)

    collector.stop_collection()
    collector.save_results()

    print("✅ Demonstration complete")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='URC 2026 Testing Metrics Collector')
    parser.add_argument('--run-tests', action='store_true',
                       help='Run the full test suite with metrics collection')
    parser.add_argument('--demonstrate', action='store_true',
                       help='Run demonstration with sample metrics')
    parser.add_argument('--dashboard', action='store_true',
                       help='Start metrics dashboard after collection')
    parser.add_argument('--output-dir', type=str, default='./test_metrics',
                       help='Output directory for metrics (default: ./test_metrics)')

    args = parser.parse_args()

    if args.run_tests:
        run_test_suite_with_metrics()

        if args.dashboard:
            print("\n🚀 Starting metrics dashboard...")
            from testing_metrics_dashboard import run_server
            run_server(8080)

    elif args.demonstrate:
        demonstrate_metrics_collection()

        if args.dashboard:
            print("\n🚀 Starting metrics dashboard...")
            from testing_metrics_dashboard import run_server
            run_server(8080)

    else:
        print("URC 2026 Testing Metrics Collector")
        print("Usage:")
        print("  python testing_metrics_collector.py --run-tests        # Run full test suite")
        print("  python testing_metrics_collector.py --demonstrate      # Run demo")
        print("  python testing_metrics_collector.py --dashboard        # Start dashboard only")
        print()
        print("Add --dashboard to any command to start the web dashboard after collection")


if __name__ == "__main__":
    main()
