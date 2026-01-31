# URC 2026 Testing Infrastructure Implementation Plan (Complete)

---

## 📊 PHASE 4: PERFORMANCE DEGRADATION TESTING FRAMEWORK

### 4.1 Current Issues Identified

**Problems:**
- No systematic performance degradation testing
- Missing resource exhaustion simulation
- No baseline performance tracking
- Inadequate bottleneck identification

### 4.2 Implementation Steps

#### Step 4.1: Performance Testing Framework
<tool_call>write
<arg_key>content</arg_key>
<arg_value>#!/usr/bin/env python3
"""
Performance Degradation Testing Framework

Comprehensive performance testing that includes:
- Resource exhaustion testing
- Performance baseline tracking
- Bottleneck identification
- Degradation pattern analysis
- Automated performance regression detection
"""

import time
import psutil
import threading
import json
import logging
import statistics
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass, asdict
from enum import Enum
from pathlib import Path
import subprocess
import gc
import random

logger = logging.getLogger(__name__)

class ResourceType(Enum):
    """Types of resources to test."""
    CPU = "cpu"
    MEMORY = "memory"
    DISK_IO = "disk_io"
    NETWORK_IO = "network_io"
    THREADS = "threads"
    FILE_DESCRIPTORS = "file_descriptors"

class TestPattern(Enum):
    """Performance test patterns."""
    GRADUAL_LOAD = "gradual_load"
    SPIKE_LOAD = "spike_load"
    SUSTAINED_LOAD = "sustained_load"
    RESOURCE_EXHAUSTION = "resource_exhaustion"
    MEMORY_LEAK = "memory_leak"

@dataclass
class PerformanceBaseline:
    """Performance baseline data."""
    timestamp: float
    cpu_usage: float
    memory_usage: float
    disk_io_read: float
    disk_io_write: float
    network_io_sent: float
    network_io_recv: float
    thread_count: int
    fd_count: int
    response_time: float
    throughput: float

@dataclass
class PerformanceTest:
    """Performance test configuration."""
    name: str
    test_pattern: TestPattern
    target_resource: ResourceType
    duration: float
    intensity: float  # 0.0 to 1.0
    baseline: Optional[PerformanceBaseline] = None
    degradation_threshold: float = 0.2  # 20% degradation threshold

@dataclass
class PerformanceResult:
    """Performance test result."""
    test_name: str
    start_time: float
    end_time: float
    duration: float
    success: bool
    baseline: PerformanceBaseline
    peak_metrics: PerformanceBaseline
    average_metrics: PerformanceBaseline
    degradation_detected: bool
    degradation_percentage: float
    bottlenecks: List[str]
    error_details: List[str]

class PerformanceMonitor:
    """Real-time performance monitoring."""
    
    def __init__(self):
        self.logger = logging.getLogger(f"{__name__}.PerformanceMonitor")
        self.monitoring = False
        self.metrics_history = []
        self.start_time = 0
    
    def start_monitoring(self):
        """Start performance monitoring."""
        self.monitoring = True
        self.start_time = time.time()
        self.metrics_history = []
        
        def monitor_loop():
            while self.monitoring:
                try:
                    metrics = self._collect_metrics()
                    self.metrics_history.append(metrics)
                    time.sleep(0.1)  # 10Hz monitoring
                except Exception as e:
                    self.logger.error(f"Error collecting metrics: {e}")
        
        self.monitor_thread = threading.Thread(target=monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        self.logger.info("Performance monitoring started")
    
    def stop_monitoring(self) -> List[PerformanceBaseline]:
        """Stop monitoring and return collected metrics."""
        self.monitoring = False
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.join(timeout=1.0)
        
        self.logger.info(f"Performance monitoring stopped. Collected {len(self.metrics_history)} samples")
        return self.metrics_history
    
    def _collect_metrics(self) -> PerformanceBaseline:
        """Collect current system metrics."""
        # CPU metrics
        cpu_percent = psutil.cpu_percent(interval=None)
        
        # Memory metrics
        memory = psutil.virtual_memory()
        memory_percent = memory.percent
        
        # Disk I/O metrics
        disk_io = psutil.disk_io_counters()
        disk_read = disk_io.read_bytes if disk_io else 0
        disk_write = disk_io.write_bytes if disk_io else 0
        
        # Network I/O metrics
        network_io = psutil.net_io_counters()
        net_sent = network_io.bytes_sent if network_io else 0
        net_recv = network_io.bytes_recv if network_io else 0
        
        # Process metrics
        process = psutil.Process()
        thread_count = process.num_threads()
        
        try:
            fd_count = process.num_fds()
        except (AttributeError, psutil.AccessDenied):
            fd_count = 0
        
        return PerformanceBaseline(
            timestamp=time.time(),
            cpu_usage=cpu_percent,
            memory_usage=memory_percent,
            disk_io_read=disk_read,
            disk_io_write=disk_write,
            network_io_sent=net_sent,
            network_io_recv=net_recv,
            thread_count=thread_count,
            fd_count=fd_count,
            response_time=0.0,  # Will be filled by test
            throughput=0.0     # Will be filled by test
        )

class ResourceExhaustionSimulator:
    """Simulates resource exhaustion scenarios."""
    
    def __init__(self):
        self.logger = logging.getLogger(f"{__name__}.ResourceExhaustionSimulator")
        self.exhaustion_active = False
        self.exhaustion_threads = []
    
    def simulate_cpu_exhaustion(self, intensity: float, duration: float):
        """Simulate CPU exhaustion."""
        self.logger.info(f"Simulating CPU exhaustion at {intensity:.0%} for {duration}s")
        
        def cpu_load():
            end_time = time.time() + duration
            while time.time() < end_time and self.exhaustion_active:
                # CPU-intensive work
                _ = sum(i * i for i in range(1000))
                # Sleep based on intensity
                time.sleep((1.0 - intensity) * 0.01)
        
        self.exhaustion_active = True
        thread = threading.Thread(target=cpu_load, daemon=True)
        thread.start()
        self.exhaustion_threads.append(thread)
        
        return thread
    
    def simulate_memory_exhaustion(self, intensity: float, duration: float):
        """Simulate memory exhaustion."""
        self.logger.info(f"Simulating memory exhaustion at {intensity:.0%} for {duration}s")
        
        def memory_load():
            memory_blocks = []
            block_size = 1024 * 1024  # 1MB blocks
            target_blocks = int(psutil.virtual_memory().total * intensity / (block_size * 2))
            
            try:
                for i in range(target_blocks):
                    if not self.exhaustion_active:
                        break
                    # Allocate memory block
                    block = bytearray(block_size)
                    memory_blocks.append(block)
                    time.sleep(0.01)
                
                # Hold memory for duration
                time.sleep(duration)
                
            except MemoryError:
                self.logger.warning("Memory allocation failed (expected)")
            finally:
                # Clean up
                memory_blocks.clear()
                gc.collect()
        
        self.exhaustion_active = True
        thread = threading.Thread(target=memory_load, daemon=True)
        thread.start()
        self.exhaustion_threads.append(thread)
        
        return thread
    
    def simulate_disk_exhaustion(self, intensity: float, duration: float):
        """Simulate disk I/O exhaustion."""
        self.logger.info(f"Simulating disk I/O exhaustion at {intensity:.0%} for {duration}s")
        
        def disk_load():
            temp_dir = Path("/tmp/urc_performance_test")
            temp_dir.mkdir(exist_ok=True)
            
            try:
                end_time = time.time() + duration
                file_count = 0
                
                while time.time() < end_time and self.exhaustion_active:
                    # Write temporary file
                    temp_file = temp_dir / f"test_{file_count}.tmp"
                    with open(temp_file, 'wb') as f:
                        f.write(b'x' * (1024 * 1024))  # 1MB
                    
                    # Read it back
                    with open(temp_file, 'rb') as f:
                        f.read()
                    
                    # Clean up
                    temp_file.unlink()
                    file_count += 1
                    
                    # Rate limiting based on intensity
                    time.sleep((1.0 - intensity) * 0.1)
                    
            except Exception as e:
                self.logger.error(f"Disk load error: {e}")
            finally:
                # Clean up temp directory
                try:
                    import shutil
                    shutil.rmtree(temp_dir)
                except:
                    pass
        
        self.exhaustion_active = True
        thread = threading.Thread(target=disk_load, daemon=True)
        thread.start()
        self.exhaustion_threads.append(thread)
        
        return thread
    
    def stop_exhaustion(self):
        """Stop all resource exhaustion simulations."""
        self.logger.info("Stopping resource exhaustion simulations")
        self.exhaustion_active = False
        
        # Wait for threads to finish
        for thread in self.exhaustion_threads:
            if thread.is_alive():
                thread.join(timeout=2.0)
        
        self.exhaustion_threads.clear()

class PerformanceDegradationTester:
    """Main performance degradation testing framework."""
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """Initialize performance degradation tester."""
        self.logger = logging.getLogger(f"{__name__}.PerformanceDegradationTester")
        
        self.config = config or {}
        self.test_results: List[PerformanceResult] = []
        
        # Components
        self.monitor = PerformanceMonitor()
        self.simulator = ResourceExhaustionSimulator()
        
        # Baseline storage
        self.baseline_file = Path(self.config.get('baseline_file', 'performance_baseline.json'))
        self.baseline_data = self._load_baseline()
        
        self.logger.info("Performance degradation tester initialized")
    
    def _load_baseline(self) -> Optional[PerformanceBaseline]:
        """Load performance baseline from file."""
        try:
            if self.baseline_file.exists():
                with open(self.baseline_file, 'r') as f:
                    data = json.load(f)
                return PerformanceBaseline(**data)
        except Exception as e:
            self.logger.error(f"Error loading baseline: {e}")
        return None
    
    def _save_baseline(self, baseline: PerformanceBaseline):
        """Save performance baseline to file."""
        try:
            with open(self.baseline_file, 'w') as f:
                json.dump(asdict(baseline), f, indent=2)
            self.logger.info(f"Baseline saved to {self.baseline_file}")
        except Exception as e:
            self.logger.error(f"Error saving baseline: {e}")
    
    def establish_baseline(self, duration: float = 60.0) -> PerformanceBaseline:
        """Establish performance baseline."""
        self.logger.info(f"Establishing performance baseline for {duration}s")
        
        # Start monitoring
        self.monitor.start_monitoring()
        
        # Run normal workload
        start_time = time.time()
        response_times = []
        throughputs = []
        
        while time.time() - start_time < duration:
            # Simulate normal workload
            workload_start = time.time()
            
            # Simple computation task
            result = sum(i * i for i in range(1000))
            
            workload_time = time.time() - workload_start
            response_times.append(workload_time)
            throughputs.append(1.0 / workload_time if workload_time > 0 else 0)
            
            time.sleep(0.1)
        
        # Stop monitoring
        metrics = self.monitor.stop_monitoring()
        
        # Calculate baseline metrics
        if metrics:
            baseline = metrics[len(metrics) // 2]  # Use middle sample
            baseline.response_time = statistics.mean(response_times)
            baseline.throughput = statistics.mean(throughputs)
            
            self._save_baseline(baseline)
            self.baseline_data = baseline
            
            self.logger.info("Performance baseline established")
            return baseline
        
        raise RuntimeError("Failed to collect baseline metrics")
    
    def run_performance_test(self, test: PerformanceTest) -> PerformanceResult:
        """Run a performance degradation test."""
        self.logger.info(f"Running performance test: {test.name}")
        
        start_time = time.time()
        
        # Use provided baseline or stored baseline
        baseline = test.baseline or self.baseline_data
        if not baseline:
            raise RuntimeError("No baseline available. Run establish_baseline() first.")
        
        # Start monitoring
        self.monitor.start_monitoring()
        
        # Start resource exhaustion
        exhaustion_thread = None
        if test.target_resource == ResourceType.CPU:
            exhaustion_thread = self.simulator.simulate_cpu_exhaustion(test.intensity, test.duration)
        elif test.target_resource == ResourceType.MEMORY:
            exhaustion_thread = self.simulator.simulate_memory_exhaustion(test.intensity, test.duration)
        elif test.target_resource == ResourceType.DISK_IO:
            exhaustion_thread = self.simulator.simulate_disk_exhaustion(test.intensity, test.duration)
        
        # Run test workload
        response_times = []
        throughputs = []
        errors = []
        
        test_start = time.time()
        while time.time() - test_start < test.duration:
            try:
                workload_start = time.time()
                
                # Perform test workload
                if test.test_pattern == TestPattern.GRADUAL_LOAD:
                    # Gradually increase load
                    load_factor = min((time.time() - test_start) / test.duration, 1.0)
                    work_size = int(1000 * (1 + load_factor * 9))
                elif test.test_pattern == TestPattern.SPIKE_LOAD:
                    # Spike load pattern
                    if int(time.time() - test_start) % 10 < 2:
                        work_size = 5000
                    else:
                        work_size = 500
                else:
                    work_size = 1000
                
                result = sum(i * i for i in range(work_size))
                
                workload_time = time.time() - workload_start
                response_times.append(workload_time)
                throughputs.append(1.0 / workload_time if workload_time > 0 else 0)
                
                time.sleep(0.1)
                
            except Exception as e:
                errors.append(str(e))
        
        # Stop monitoring and exhaustion
        metrics = self.monitor.stop_monitoring()
        self.simulator.stop_exhaustion()
        
        end_time = time.time()
        
        # Calculate results
        if metrics:
            peak_metrics = max(metrics, key=lambda m: m.cpu_usage)
            avg_metrics = PerformanceBaseline(
                timestamp=statistics.mean([m.timestamp for m in metrics]),
                cpu_usage=statistics.mean([m.cpu_usage for m in metrics]),
                memory_usage=statistics.mean([m.memory_usage for m in metrics]),
                disk_io_read=statistics.mean([m.disk_io_read for m in metrics]),
                disk_io_write=statistics.mean([m.disk_io_write for m in metrics]),
                network_io_sent=statistics.mean([m.network_io_sent for m in metrics]),
                network_io_recv=statistics.mean([m.network_io_recv for m in metrics]),
                thread_count=statistics.mean([m.thread_count for m in metrics]),
                fd_count=statistics.mean([m.fd_count for m in metrics]),
                response_time=statistics.mean(response_times) if response_times else 0,
                throughput=statistics.mean(throughputs) if throughputs else 0
            )
            
            # Calculate degradation
            response_degradation = (avg_metrics.response_time - baseline.response_time) / baseline.response_time
            throughput_degradation = (baseline.throughput - avg_metrics.throughput) / baseline.throughput
            
            degradation_percentage = max(response_degradation, throughput_degradation) * 100
            degradation_detected = degradation_percentage > (test.degradation_threshold * 100)
            
            # Identify bottlenecks
            bottlenecks = []
            if avg_metrics.cpu_usage > 80:
                bottlenecks.append("CPU")
            if avg_metrics.memory_usage > 80:
                bottlenecks.append("Memory")
            if avg_metrics.disk_io_read > baseline.disk_io_read * 2 or avg_metrics.disk_io_write > baseline.disk_io_write * 2:
                bottlenecks.append("Disk I/O")
            
            result = PerformanceResult(
                test_name=test.name,
                start_time=start_time,
                end_time=end_time,
                duration=end_time - start_time,
                success=len(errors) == 0,
                baseline=baseline,
                peak_metrics=peak_metrics,
                average_metrics=avg_metrics,
                degradation_detected=degradation_detected,
                degradation_percentage=degradation_percentage,
                bottlenecks=bottlenecks,
                error_details=errors
            )
            
            self.test_results.append(result)
            return result
        
        raise RuntimeError("Failed to collect test metrics")
    
    def run_comprehensive_test_suite(self) -> List[PerformanceResult]:
        """Run comprehensive performance test suite."""
        self.logger.info("Running comprehensive performance test suite")
        
        # Define test scenarios
        tests = [
            PerformanceTest(
                name="cpu_gradual_load",
                test_pattern=TestPattern.GRADUAL_LOAD,
                target_resource=ResourceType.CPU,
                duration=120.0,
                intensity=0.7
            ),
            PerformanceTest(
                name="memory_exhaustion",
                test_pattern=TestPattern.RESOURCE_EXHAUSTION,
                target_resource=ResourceType.MEMORY,
                duration=60.0,
                intensity=0.8
            ),
            PerformanceTest(
                name="disk_io_spike",
                test_pattern=TestPattern.SPIKE_LOAD,
                target_resource=ResourceType.DISK_IO,
                duration=90.0,
                intensity=0.6
            ),
            PerformanceTest(
                name="sustained_load",
                test_pattern=TestPattern.SUSTAINED_LOAD,
                target_resource=ResourceType.CPU,
                duration=300.0,
                intensity=0.5
            )
        ]
        
        results = []
        for test in tests:
            try:
                result = self.run_performance_test(test)
                results.append(result)
                self.logger.info(f"Test '{test.name}' completed: {'PASS' if result.success else 'FAIL'}")
            except Exception as e:
                self.logger.error(f"Test '{test.name}' failed: {e}")
        
        return results
    
    def get_performance_report(self) -> Dict[str, Any]:
        """Generate comprehensive performance report."""
        if not self.test_results:
            return {"error": "No test results available"}
        
        # Calculate statistics
        total_tests = len(self.test_results)
        successful_tests = len([r for r in self.test_results if r.success])
        degradation_detected = len([r for r in self.test_results if r.degradation_detected])
        
        # Performance statistics
        degradations = [r.degradation_percentage for r in self.test_results]
        response_times = [r.average_metrics.response_time for r in self.test_results]
        throughputs = [r.average_metrics.throughput for r in self.test_results]
        
        # Bottleneck analysis
        bottleneck_counts = {}
        for result in self.test_results:
            for bottleneck in result.bottlenecks:
                bottleneck_counts[bottleneck] = bottleneck_counts.get(bottleneck, 0) + 1
        
        return {
            "summary": {
                "total_tests": total_tests,
                "successful_tests": successful_tests,
                "degradation_detected": degradation_detected,
                "success_rate": successful_tests / total_tests if total_tests > 0 else 0
            },
            "performance": {
                "avg_degradation": statistics.mean(degradations) if degradations else 0,
                "max_degradation": max(degradations) if degradations else 0,
                "avg_response_time": statistics.mean(response_times) if response_times else 0,
                "avg_throughput": statistics.mean(throughputs) if throughputs else 0
            },
            "bottlenecks": bottleneck_counts,
            "baseline": asdict(self.baseline_data) if self.baseline_data else None,
            "test_results": [asdict(result) for result in self.test_results]
        }
    
    def save_performance_report(self, filename: Optional[str] = None):
        """Save performance report to file."""
        if filename is None:
            timestamp = int(time.time())
            filename = f"performance_test_report_{timestamp}.json"
        
        report = self.get_performance_report()
        
        try:
            with open(filename, 'w') as f:
                json.dump(report, f, indent=2, default=str)
            
            self.logger.info(f"Performance report saved to {filename}")
            
        except Exception as e:
            self.logger.error(f"Error saving performance report: {e}")


# Convenience function
def create_performance_tester(config: Optional[Dict[str, Any]] = None) -> PerformanceDegradationTester:
    """Create performance degradation tester with configuration."""
    return PerformanceDegradationTester(config)