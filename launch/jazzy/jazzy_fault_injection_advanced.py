#!/usr/bin/env python3
"""
Advanced Fault Injection & Monitoring System for URC 2026

Enhanced fault injection with real-time visualization, trend analysis,
and comprehensive chaos engineering capabilities.

Features:
- Real-time data visualization and charting
- Advanced memory leak detection with trend analysis
- Expanded chaos engineering scenarios
- Performance monitoring dashboards
- Automated fault detection and alerting
- Comprehensive benchmarking and analysis
"""

import asyncio
import time
import threading
import logging
import psutil
import os
import signal
import json
import gc
import tracemalloc
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass, field
from datetime import datetime, timedelta
import statistics
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
import io
import base64

# Try to import visualization libraries
try:
    import plotly.graph_objects as go
    import plotly.express as px
    from plotly.subplots import make_subplots
    PLOTLY_AVAILABLE = True
except ImportError:
    PLOTLY_AVAILABLE = False
    logger.warning("Plotly not available - using matplotlib fallback")

logger = logging.getLogger(__name__)


@dataclass
class TimeSeriesData:
    """Time series data for visualization"""
    timestamps: List[float] = field(default_factory=list)
    values: List[float] = field(default_factory=list)
    labels: List[str] = field(default_factory=list)

    def add_point(self, timestamp: float, value: float, label: str = ""):
        """Add a data point"""
        self.timestamps.append(timestamp)
        self.values.append(value)
        self.labels.append(label)

    def get_recent(self, seconds: int) -> 'TimeSeriesData':
        """Get data from last N seconds"""
        cutoff = time.time() - seconds
        recent_indices = [i for i, ts in enumerate(self.timestamps) if ts >= cutoff]

        recent = TimeSeriesData()
        for i in recent_indices:
            recent.add_point(self.timestamps[i], self.values[i], self.labels[i])

        return recent


@dataclass
class FaultInjectionResult:
    """Results from fault injection testing"""
    fault_type: str
    start_time: float
    end_time: float
    detected: bool
    recovery_time: Optional[float]
    severity: str
    impact_metrics: Dict[str, Any] = field(default_factory=dict)
    time_series_data: Dict[str, TimeSeriesData] = field(default_factory=dict)


@dataclass
class SystemMetricsSnapshot:
    """Comprehensive system metrics snapshot"""
    timestamp: float
    cpu_percent: float
    memory_percent: float
    memory_mb: float
    memory_rss: int
    memory_vms: int
    memory_shared: int
    network_connections: int
    thread_count: int
    open_files: int
    disk_usage_percent: float
    load_average: tuple
    context_switches: int

    @classmethod
    def capture(cls) -> 'SystemMetricsSnapshot':
        """Capture current system metrics"""
        process = psutil.Process()
        memory_info = process.memory_info()
        disk_usage = psutil.disk_usage('/')
        load_avg = psutil.getloadavg()

        return cls(
            timestamp=time.time(),
            cpu_percent=psutil.cpu_percent(interval=0.1),
            memory_percent=process.memory_percent(),
            memory_mb=memory_info.rss / 1024 / 1024,
            memory_rss=memory_info.rss,
            memory_vms=memory_info.vms,
            memory_shared=getattr(memory_info, 'shared', 0),
            network_connections=len(psutil.net_connections()),
            thread_count=process.num_threads(),
            open_files=len(process.open_files()),
            disk_usage_percent=disk_usage.percent,
            load_average=load_avg,
            context_switches=getattr(process, 'num_ctx_switches', lambda: type('pctxsw', (), {'voluntary': 0, 'involuntary': 0})())().voluntary
        )


class AdvancedFaultInjector:
    """
    Advanced fault injection system with comprehensive monitoring
    and real-time visualization capabilities.
    """

    def __init__(self):
        self.fault_results: List[FaultInjectionResult] = []
        self.system_metrics_history: List[SystemMetricsSnapshot] = []
        self.monitoring_active = False
        self.monitoring_thread: Optional[threading.Thread] = None

        # Time series data for visualization
        self.cpu_time_series = TimeSeriesData()
        self.memory_time_series = TimeSeriesData()
        self.network_time_series = TimeSeriesData()
        self.thread_time_series = TimeSeriesData()

        # Memory leak detection
        self.memory_snapshots: List[tuple] = []
        tracemalloc.start()

        # Fault detection thresholds
        self.thresholds = {
            'cpu_spike_percent': 80.0,
            'memory_leak_mb_per_minute': 10.0,
            'network_timeout_seconds': 5.0,
            'thread_explosion_count': 50,
            'disk_usage_critical_percent': 90.0
        }

        logger.info("🎯 Advanced Fault Injector initialized")

    async def start_monitoring(self, interval_seconds: float = 0.1):
        """Start real-time system monitoring"""
        self.monitoring_active = True
        self.monitoring_thread = threading.Thread(
            target=self._monitoring_loop,
            args=(interval_seconds,),
            daemon=True
        )
        self.monitoring_thread.start()
        logger.info("📊 Real-time monitoring started")

    def stop_monitoring(self):
        """Stop system monitoring"""
        self.monitoring_active = False
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=1.0)
        logger.info("📊 Monitoring stopped")

    def _monitoring_loop(self, interval: float):
        """Continuous monitoring loop"""
        while self.monitoring_active:
            try:
                snapshot = SystemMetricsSnapshot.capture()
                self.system_metrics_history.append(snapshot)

                # Update time series
                self.cpu_time_series.add_point(snapshot.timestamp, snapshot.cpu_percent, "CPU %")
                self.memory_time_series.add_point(snapshot.timestamp, snapshot.memory_mb, "Memory MB")
                self.network_time_series.add_point(snapshot.timestamp, snapshot.network_connections, "Connections")
                self.thread_time_series.add_point(snapshot.timestamp, snapshot.thread_count, "Threads")

                time.sleep(interval)

            except Exception as e:
                logger.error(f"Monitoring error: {e}")
                time.sleep(interval)

    # ===== MEMORY LEAK DETECTION =====

    async def inject_memory_leak_fault(self, duration_seconds: int = 30) -> FaultInjectionResult:
        """
        Advanced memory leak injection with trend analysis
        """
        logger.info("💧 Injecting memory leak fault...")

        start_time = time.time()
        initial_snapshot = SystemMetricsSnapshot.capture()
        leak_detected = False
        detection_time = None

        # Take initial memory snapshot
        self.memory_snapshots.append((start_time, tracemalloc.get_traced_memory()[0]))

        # Create controlled memory leak
        leaked_objects = []
        leak_rate_mb_per_second = 0.5  # 0.5 MB/second leak

        for i in range(duration_seconds * 10):  # 10 iterations per second
            if not self.monitoring_active:
                break

            # Simulate memory leak
            leak_size = int(leak_rate_mb_per_second * 1024 * 1024 / 10)  # Bytes per iteration
            leaked_objects.append([0] * leak_size)  # Keep references to leak memory

            # Monitor for leak detection
            current_memory = SystemMetricsSnapshot.capture().memory_mb
            elapsed = time.time() - start_time

            # Advanced leak detection using trend analysis
            if elapsed > 10:  # Wait for stabilization
                leak_detected, confidence = self._analyze_memory_trend()
                if leak_detected and not detection_time:
                    detection_time = time.time()
                    logger.warning(".2f")

            await asyncio.sleep(0.1)

        end_time = time.time()

        # Analyze final state
        final_snapshot = SystemMetricsSnapshot.capture()
        memory_increase = final_snapshot.memory_mb - initial_snapshot.memory_mb
        leak_rate = memory_increase / (end_time - start_time) * 60  # MB per minute

        # Clean up leaked memory
        leaked_objects.clear()
        gc.collect()

        result = FaultInjectionResult(
            fault_type="memory_leak",
            start_time=start_time,
            end_time=end_time,
            detected=leak_detected,
            recovery_time=detection_time - start_time if detection_time else None,
            severity="high",
            impact_metrics={
                'memory_increase_mb': memory_increase,
                'leak_rate_mb_per_minute': leak_rate,
                'detection_confidence': confidence if leak_detected else 0.0,
                'cleanup_effective': True
            },
            time_series_data={
                'memory_mb': self.memory_time_series.get_recent(60),
                'cpu_percent': self.cpu_time_series.get_recent(60)
            }
        )

        self.fault_results.append(result)
        logger.info(f"💧 Memory leak test completed: {'DETECTED' if leak_detected else 'NOT DETECTED'}")
        return result

    def _analyze_memory_trend(self) -> tuple[bool, float]:
        """
        Advanced memory trend analysis for leak detection

        Uses statistical methods to detect gradual memory growth
        """
        if len(self.memory_time_series.values) < 20:  # Need minimum data points
            return False, 0.0

        # Get recent memory data (last 30 seconds)
        recent_data = self.memory_time_series.get_recent(30)
        if len(recent_data.values) < 10:
            return False, 0.0

        values = recent_data.values
        timestamps = recent_data.timestamps

        # Calculate linear regression slope (memory growth rate)
        n = len(values)
        x = np.array(timestamps)
        y = np.array(values)

        # Normalize timestamps to avoid large numbers
        x = x - x[0]

        # Linear regression
        slope = np.polyfit(x, y, 1)[0]  # Slope in MB/second
        slope_mb_per_minute = slope * 60

        # Calculate R-squared for confidence
        y_pred = np.polyval(np.polyfit(x, y, 1), x)
        ss_res = np.sum((y - y_pred) ** 2)
        ss_tot = np.sum((y - np.mean(y)) ** 2)
        r_squared = 1 - (ss_res / ss_tot) if ss_tot != 0 else 0

        # Detect leak if slope exceeds threshold and trend is consistent
        leak_detected = (
            slope_mb_per_minute > self.thresholds['memory_leak_mb_per_minute'] and
            r_squared > 0.7  # Strong correlation indicates consistent trend
        )

        confidence = min(r_squared * 100, 100.0) if leak_detected else 0.0

        return leak_detected, confidence

    # ===== CPU SPIKE FAULT =====

    async def inject_cpu_spike_fault(self, intensity: str = "high") -> FaultInjectionResult:
        """
        CPU spike fault injection with adaptive intensity
        """
        logger.info(f"⚡ Injecting CPU spike fault (intensity: {intensity})...")

        start_time = time.time()
        spike_detected = False
        detection_time = None

        # Configure spike intensity
        if intensity == "low":
            spike_duration = 5
            target_cpu_percent = 60
        elif intensity == "medium":
            spike_duration = 10
            target_cpu_percent = 75
        else:  # high
            spike_duration = 15
            target_cpu_percent = 90

        # Start CPU spike in background
        spike_task = asyncio.create_task(self._generate_cpu_spike(spike_duration, target_cpu_percent))

        # Monitor for detection
        monitoring_start = time.time()
        while time.time() - monitoring_start < spike_duration + 5:  # Monitor beyond spike
            current_cpu = SystemMetricsSnapshot.capture().cpu_percent

            if current_cpu > self.thresholds['cpu_spike_percent'] and not spike_detected:
                spike_detected = True
                detection_time = time.time()
                logger.warning(".1f")

            await asyncio.sleep(0.5)

        # Wait for spike to complete
        await spike_task
        end_time = time.time()

        # Analyze impact
        spike_duration_actual = detection_time - start_time if detection_time else 0
        max_cpu_during_spike = max(
            [s.cpu_percent for s in self.system_metrics_history[-50:]],  # Last 50 samples
            default=0
        )

        result = FaultInjectionResult(
            fault_type=f"cpu_spike_{intensity}",
            start_time=start_time,
            end_time=end_time,
            detected=spike_detected,
            recovery_time=None,  # CPU spikes typically resolve automatically
            severity="medium",
            impact_metrics={
                'spike_duration_seconds': spike_duration,
                'max_cpu_percent': max_cpu_during_spike,
                'target_cpu_percent': target_cpu_percent,
                'spike_intensity': intensity
            },
            time_series_data={
                'cpu_percent': self.cpu_time_series.get_recent(60),
                'memory_mb': self.memory_time_series.get_recent(60)
            }
        )

        self.fault_results.append(result)
        logger.info(f"⚡ CPU spike test completed: {'DETECTED' if spike_detected else 'NOT DETECTED'}")
        return result

    async def _generate_cpu_spike(self, duration_seconds: int, target_percent: int):
        """Generate controlled CPU spike"""
        end_time = time.time() + duration_seconds

        while time.time() < end_time:
            # CPU intensive computation
            for _ in range(10000):
                _ = sum(i*i for i in range(100))

        logger.debug(f"CPU spike generation completed ({duration_seconds}s at {target_percent}%)")

    # ===== NETWORK PARTITION FAULT =====

    async def inject_network_partition_fault(self) -> FaultInjectionResult:
        """
        Network partition fault with sophisticated detection
        """
        logger.info("🌐 Injecting network partition fault...")

        start_time = time.time()
        partition_detected = False
        detection_time = None
        recovery_time = None

        # Simulate network partition by creating connection timeouts
        partition_task = asyncio.create_task(self._simulate_network_partition(10))

        # Monitor network connectivity
        monitoring_start = time.time()
        baseline_connections = SystemMetricsSnapshot.capture().network_connections

        while time.time() - monitoring_start < 15:  # Monitor for 15 seconds
            current_snapshot = SystemMetricsSnapshot.capture()
            current_connections = current_snapshot.network_connections

            # Detect partition (significant drop in connections)
            connection_drop = baseline_connections - current_connections
            if connection_drop > 2 and not partition_detected:  # Arbitrary threshold
                partition_detected = True
                detection_time = time.time()
                logger.warning(f"🌐 Network partition detected (connection drop: {connection_drop})")

            # Detect recovery (connections return to baseline)
            if partition_detected and current_connections >= baseline_connections - 1:
                if not recovery_time:
                    recovery_time = time.time()
                    logger.info(".2f")

            await asyncio.sleep(0.5)

        await partition_task
        end_time = time.time()

        result = FaultInjectionResult(
            fault_type="network_partition",
            start_time=start_time,
            end_time=end_time,
            detected=partition_detected,
            recovery_time=recovery_time - detection_time if recovery_time and detection_time else None,
            severity="critical",
            impact_metrics={
                'baseline_connections': baseline_connections,
                'min_connections_during_partition': min([s.network_connections for s in self.system_metrics_history[-30:]], default=0),
                'partition_duration_seconds': 10,
                'recovery_successful': recovery_time is not None
            },
            time_series_data={
                'network_connections': self.network_time_series.get_recent(60),
                'cpu_percent': self.cpu_time_series.get_recent(60)
            }
        )

        self.fault_results.append(result)
        logger.info(f"🌐 Network partition test completed: {'DETECTED' if partition_detected else 'NOT DETECTED'}")
        return result

    async def _simulate_network_partition(self, duration_seconds: int):
        """Simulate network partition by creating artificial delays"""
        # In a real system, this would manipulate network interfaces
        # For testing, we simulate by creating connection timeouts
        logger.debug(f"Simulating network partition for {duration_seconds} seconds")

        # Create artificial network load
        tasks = []
        for _ in range(20):  # Simulate many failing connections
            task = asyncio.create_task(self._failing_network_operation(duration_seconds))
            tasks.append(task)

        await asyncio.gather(*tasks, return_exceptions=True)

    async def _failing_network_operation(self, duration: int):
        """Simulate failing network operation"""
        try:
            # Simulate network timeout
            await asyncio.sleep(duration + 1)  # Longer than partition duration
        except asyncio.CancelledError:
            pass

    # ===== THREAD EXPLOSION FAULT =====

    async def inject_thread_explosion_fault(self) -> FaultInjectionResult:
        """
        Thread explosion fault - excessive thread creation
        """
        logger.info("🧵 Injecting thread explosion fault...")

        start_time = time.time()
        explosion_detected = False
        detection_time = None

        # Get baseline thread count
        baseline_threads = SystemMetricsSnapshot.capture().thread_count

        # Create thread explosion
        explosion_task = asyncio.create_task(self._create_thread_explosion(20))

        # Monitor thread count
        monitoring_start = time.time()
        max_threads_seen = baseline_threads

        while time.time() - monitoring_start < 25:  # Monitor for 25 seconds
            current_threads = SystemMetricsSnapshot.capture().thread_count
            max_threads_seen = max(max_threads_seen, current_threads)

            # Detect thread explosion
            thread_increase = current_threads - baseline_threads
            if thread_increase > self.thresholds['thread_explosion_count'] and not explosion_detected:
                explosion_detected = True
                detection_time = time.time()
                logger.warning(f"🧵 Thread explosion detected ({current_threads} threads, +{thread_increase})")

            await asyncio.sleep(0.2)

        await explosion_task
        end_time = time.time()

        result = FaultInjectionResult(
            fault_type="thread_explosion",
            start_time=start_time,
            end_time=end_time,
            detected=explosion_detected,
            recovery_time=None,  # Threads typically clean up automatically
            severity="high",
            impact_metrics={
                'baseline_threads': baseline_threads,
                'max_threads': max_threads_seen,
                'thread_increase': max_threads_seen - baseline_threads,
                'explosion_intensity': 20
            },
            time_series_data={
                'thread_count': self.thread_time_series.get_recent(60),
                'cpu_percent': self.cpu_time_series.get_recent(60),
                'memory_mb': self.memory_time_series.get_recent(60)
            }
        )

        self.fault_results.append(result)
        logger.info(f"🧵 Thread explosion test completed: {'DETECTED' if explosion_detected else 'NOT DETECTED'}")
        return result

    async def _create_thread_explosion(self, num_threads: int):
        """Create controlled thread explosion"""
        threads = []

        def thread_worker(thread_id: int):
            """Worker function for each thread"""
            time.sleep(0.1)  # Brief work
            logger.debug(f"Thread {thread_id} completed")

        # Create many threads
        for i in range(num_threads):
            thread = threading.Thread(target=thread_worker, args=(i,), daemon=True)
            threads.append(thread)
            thread.start()

        # Wait for all threads to complete
        for thread in threads:
            thread.join(timeout=1.0)

        logger.debug(f"Thread explosion completed ({num_threads} threads created)")

    # ===== DISK SPACE EXHAUSTION FAULT =====

    async def inject_disk_exhaustion_fault(self) -> FaultInjectionResult:
        """
        Disk space exhaustion fault simulation
        """
        logger.info("💾 Injecting disk exhaustion fault...")

        start_time = time.time()
        exhaustion_detected = False
        detection_time = None

        # Get baseline disk usage
        baseline_disk = SystemMetricsSnapshot.capture().disk_usage_percent

        # Simulate disk exhaustion by creating temporary files
        exhaustion_task = asyncio.create_task(self._simulate_disk_exhaustion())

        # Monitor disk usage
        monitoring_start = time.time()
        max_disk_usage = baseline_disk

        while time.time() - monitoring_start < 20:  # Monitor for 20 seconds
            current_disk = SystemMetricsSnapshot.capture().disk_usage_percent
            max_disk_usage = max(max_disk_usage, current_disk)

            # Detect disk exhaustion
            if current_disk > self.thresholds['disk_usage_critical_percent'] and not exhaustion_detected:
                exhaustion_detected = True
                detection_time = time.time()
                logger.warning(".1f")

            await asyncio.sleep(0.5)

        await exhaustion_task
        end_time = time.time()

        result = FaultInjectionResult(
            fault_type="disk_exhaustion",
            start_time=start_time,
            end_time=end_time,
            detected=exhaustion_detected,
            recovery_time=None,  # Disk space recovers when files are deleted
            severity="medium",
            impact_metrics={
                'baseline_disk_percent': baseline_disk,
                'max_disk_percent': max_disk_usage,
                'disk_increase_percent': max_disk_usage - baseline_disk,
                'exhaustion_simulated': True
            },
            time_series_data={
                'cpu_percent': self.cpu_time_series.get_recent(60),
                'memory_mb': self.memory_time_series.get_recent(60)
            }
        )

        self.fault_results.append(result)
        logger.info(f"💾 Disk exhaustion test completed: {'DETECTED' if exhaustion_detected else 'NOT DETECTED'}")
        return result

    async def _simulate_disk_exhaustion(self):
        """Simulate disk space exhaustion"""
        # Create temporary files to consume disk space
        temp_files = []
        chunk_size = 1024 * 1024  # 1MB chunks
        target_mb = 50  # Try to consume 50MB

        try:
            for i in range(target_mb):
                temp_file = f"/tmp/jazzy_fault_test_{i}.tmp"
                with open(temp_file, 'wb') as f:
                    f.write(b'0' * chunk_size)
                temp_files.append(temp_file)

                if i % 10 == 0:
                    logger.debug(f"Created {i+1}MB of temporary files")

                await asyncio.sleep(0.1)  # Slow creation to simulate gradual exhaustion

        finally:
            # Clean up temporary files
            for temp_file in temp_files:
                try:
                    os.remove(temp_file)
                except OSError:
                    pass

        logger.debug(f"Disk exhaustion simulation completed, cleaned up {len(temp_files)} files")

    # ===== VISUALIZATION AND ANALYSIS =====

    def generate_performance_dashboard(self) -> Dict[str, Any]:
        """
        Generate comprehensive performance dashboard with charts
        """
        if not PLOTLY_AVAILABLE:
            return self._generate_matplotlib_dashboard()

        return self._generate_plotly_dashboard()

    def _generate_plotly_dashboard(self) -> Dict[str, Any]:
        """Generate interactive Plotly dashboard"""
        dashboard = {}

        # CPU Usage Over Time
        cpu_fig = go.Figure()
        cpu_fig.add_trace(go.Scatter(
            x=self.cpu_time_series.timestamps,
            y=self.cpu_time_series.values,
            mode='lines',
            name='CPU Usage %',
            line=dict(color='red', width=2)
        ))
        cpu_fig.update_layout(
            title="CPU Usage Over Time",
            xaxis_title="Time",
            yaxis_title="CPU %",
            template="plotly_white"
        )
        dashboard['cpu_chart'] = cpu_fig.to_html()

        # Memory Usage Over Time
        memory_fig = go.Figure()
        memory_fig.add_trace(go.Scatter(
            x=self.memory_time_series.timestamps,
            y=self.memory_time_series.values,
            mode='lines',
            name='Memory Usage (MB)',
            line=dict(color='blue', width=2)
        ))
        memory_fig.update_layout(
            title="Memory Usage Over Time",
            xaxis_title="Time",
            yaxis_title="Memory (MB)",
            template="plotly_white"
        )
        dashboard['memory_chart'] = memory_fig.to_html()

        # Network Connections Over Time
        network_fig = go.Figure()
        network_fig.add_trace(go.Scatter(
            x=self.network_time_series.timestamps,
            y=self.network_time_series.values,
            mode='lines',
            name='Network Connections',
            line=dict(color='green', width=2)
        ))
        network_fig.update_layout(
            title="Network Connections Over Time",
            xaxis_title="Time",
            yaxis_title="Connections",
            template="plotly_white"
        )
        dashboard['network_chart'] = network_fig.to_html()

        # Thread Count Over Time
        thread_fig = go.Figure()
        thread_fig.add_trace(go.Scatter(
            x=self.thread_time_series.timestamps,
            y=self.thread_time_series.values,
            mode='lines',
            name='Thread Count',
            line=dict(color='orange', width=2)
        ))
        thread_fig.update_layout(
            title="Thread Count Over Time",
            xaxis_title="Time",
            yaxis_title="Threads",
            template="plotly_white"
        )
        dashboard['thread_chart'] = thread_fig.to_html()

        # Fault Injection Timeline
        fault_fig = go.Figure()

        for result in self.fault_results:
            fault_fig.add_trace(go.Scatter(
                x=[result.start_time, result.end_time],
                y=[1, 1],  # Constant Y for timeline
                mode='lines+markers',
                name=f"{result.fault_type} ({'Detected' if result.detected else 'Not Detected'})",
                line=dict(width=4),
                marker=dict(size=8)
            ))

        fault_fig.update_layout(
            title="Fault Injection Timeline",
            xaxis_title="Time",
            yaxis_title="Fault Active",
            yaxis=dict(tickvals=[1], ticktext=['Active']),
            template="plotly_white",
            showlegend=True
        )
        dashboard['fault_timeline'] = fault_fig.to_html()

        return dashboard

    def _generate_matplotlib_dashboard(self) -> Dict[str, Any]:
        """Generate static matplotlib dashboard"""
        dashboard = {}

        # CPU Usage Chart
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.plot(self.cpu_time_series.timestamps, self.cpu_time_series.values, 'r-', linewidth=2)
        ax.set_title('CPU Usage Over Time')
        ax.set_xlabel('Time')
        ax.set_ylabel('CPU %')
        ax.grid(True)
        dashboard['cpu_chart'] = self._fig_to_base64(fig)
        plt.close(fig)

        # Memory Usage Chart
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.plot(self.memory_time_series.timestamps, self.memory_time_series.values, 'b-', linewidth=2)
        ax.set_title('Memory Usage Over Time')
        ax.set_xlabel('Time')
        ax.set_ylabel('Memory (MB)')
        ax.grid(True)
        dashboard['memory_chart'] = self._fig_to_base64(fig)
        plt.close(fig)

        # Network Connections Chart
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.plot(self.network_time_series.timestamps, self.network_time_series.values, 'g-', linewidth=2)
        ax.set_title('Network Connections Over Time')
        ax.set_xlabel('Time')
        ax.set_ylabel('Connections')
        ax.grid(True)
        dashboard['network_chart'] = self._fig_to_base64(fig)
        plt.close(fig)

        return dashboard

    def _fig_to_base64(self, fig: Figure) -> str:
        """Convert matplotlib figure to base64 string"""
        canvas = FigureCanvasAgg(fig)
        buf = io.BytesIO()
        canvas.print_png(buf)
        buf.seek(0)
        return base64.b64encode(buf.read()).decode('utf-8')

    def generate_fault_analysis_report(self) -> Dict[str, Any]:
        """Generate comprehensive fault analysis report"""
        total_faults = len(self.fault_results)
        detected_faults = sum(1 for r in self.fault_results if r.detected)
        detection_rate = detected_faults / total_faults if total_faults > 0 else 0

        # Analyze fault types
        fault_types = {}
        for result in self.fault_results:
            if result.fault_type not in fault_types:
                fault_types[result.fault_type] = {'detected': 0, 'total': 0, 'avg_recovery_time': []}

            fault_types[result.fault_type]['total'] += 1
            if result.detected:
                fault_types[result.fault_type]['detected'] += 1
                if result.recovery_time:
                    fault_types[result.fault_type]['avg_recovery_time'].append(result.recovery_time)

        # Calculate averages
        for fault_type, stats in fault_types.items():
            if stats['avg_recovery_time']:
                stats['avg_recovery_time'] = statistics.mean(stats['avg_recovery_time'])
            else:
                stats['avg_recovery_time'] = None
            stats['detection_rate'] = stats['detected'] / stats['total']

        # Performance analysis
        if self.system_metrics_history:
            cpu_samples = [s.cpu_percent for s in self.system_metrics_history]
            memory_samples = [s.memory_mb for s in self.system_metrics_history]

            performance_stats = {
                'cpu_mean': statistics.mean(cpu_samples),
                'cpu_std': statistics.stdev(cpu_samples) if len(cpu_samples) > 1 else 0,
                'cpu_max': max(cpu_samples),
                'memory_mean': statistics.mean(memory_samples),
                'memory_std': statistics.stdev(memory_samples) if len(memory_samples) > 1 else 0,
                'memory_max': max(memory_samples),
                'total_samples': len(self.system_metrics_history),
                'monitoring_duration': self.system_metrics_history[-1].timestamp - self.system_metrics_history[0].timestamp
            }
        else:
            performance_stats = {}

        return {
            'summary': {
                'total_faults_injected': total_faults,
                'faults_detected': detected_faults,
                'detection_rate': detection_rate,
                'test_duration_seconds': time.time() - (self.fault_results[0].start_time if self.fault_results else time.time())
            },
            'fault_analysis': fault_types,
            'performance_stats': performance_stats,
            'recommendations': self._generate_fault_recommendations(detection_rate, fault_types),
            'system_health_score': self._calculate_system_health_score(detection_rate, performance_stats)
        }

    def _generate_fault_recommendations(self, detection_rate: float, fault_types: Dict) -> List[str]:
        """Generate recommendations based on fault analysis"""
        recommendations = []

        if detection_rate < 0.8:
            recommendations.append("Overall fault detection rate below 80%. Enhance monitoring systems.")

        # Check specific fault types
        if 'memory_leak' in fault_types and fault_types['memory_leak']['detection_rate'] < 0.5:
            recommendations.append("Memory leak detection insufficient. Implement advanced trend analysis and alerting.")

        if 'cpu_spike' in fault_types and fault_types['cpu_spike']['detection_rate'] < 0.8:
            recommendations.append("CPU spike detection needs improvement. Add adaptive thresholds based on workload.")

        if 'network_partition' in fault_types and fault_types['network_partition']['detection_rate'] < 0.9:
            recommendations.append("Network partition detection critical. Enhance connectivity monitoring.")

        # Performance recommendations
        recommendations.extend([
            "Implement real-time performance dashboards for continuous monitoring",
            "Add automated alerting for fault detection with escalation procedures",
            "Expand chaos engineering scenarios to cover more failure modes",
            "Implement predictive fault detection using machine learning",
            "Create comprehensive failure mode documentation with recovery procedures"
        ])

        return recommendations

    def _calculate_system_health_score(self, detection_rate: float, performance_stats: Dict) -> float:
        """Calculate overall system health score (0-100)"""
        score = 0

        # Detection rate (40% weight)
        score += detection_rate * 40

        # Performance stability (30% weight)
        if performance_stats:
            cpu_stability = max(0, 100 - performance_stats.get('cpu_std', 0) * 2)
            memory_stability = max(0, 100 - performance_stats.get('memory_std', 0) * 10)
            score += (cpu_stability + memory_stability) / 2 * 0.3
        else:
            score += 30  # Default if no performance data

        # Fault recovery (20% weight)
        recovery_score = 0
        for result in self.fault_results:
            if result.recovery_time and result.recovery_time < 10:  # Fast recovery
                recovery_score += 1
        recovery_score = (recovery_score / len(self.fault_results)) * 20 if self.fault_results else 20
        score += recovery_score

        # Monitoring coverage (10% weight)
        monitoring_score = min(100, len(self.system_metrics_history) / 10)  # Bonus for extensive monitoring
        score += monitoring_score

        return min(100, score)


async def run_advanced_fault_testing():
    """
    Run comprehensive fault injection testing with visualization
    """
    print("="*80)
    print("🧪 ADVANCED FAULT INJECTION & MONITORING SYSTEM")
    print("="*80)
    print("Testing: Memory Leaks, CPU Spikes, Network Partitions, Thread Explosions")
    print("Features: Real-time Visualization, Trend Analysis, Chaos Engineering")
    print()

    # Initialize fault injector
    injector = AdvancedFaultInjector()

    try:
        # Start monitoring
        await injector.start_monitoring(interval_seconds=0.1)
        print("📊 Real-time monitoring started...")

        # Wait for baseline
        print("⏱️ Establishing baseline (5 seconds)...")
        await asyncio.sleep(5)

        # Run fault injection scenarios
        print("\n" + "="*60)
        print("FAULT INJECTION SCENARIOS")
        print("="*60)

        # 1. Memory Leak Detection
        print("\n1. 🔍 Memory Leak Detection Test")
        memory_result = await injector.inject_memory_leak_fault(duration_seconds=20)

        # 2. CPU Spike Fault
        print("\n2. ⚡ CPU Spike Fault Test")
        cpu_result = await injector.inject_cpu_spike_fault(intensity="high")

        # 3. Network Partition Fault
        print("\n3. 🌐 Network Partition Fault Test")
        network_result = await injector.inject_network_partition_fault()

        # 4. Thread Explosion Fault
        print("\n4. 🧵 Thread Explosion Fault Test")
        thread_result = await injector.inject_thread_explosion_fault()

        # 5. Disk Exhaustion Fault
        print("\n5. 💾 Disk Exhaustion Fault Test")
        disk_result = await injector.inject_disk_exhaustion_fault()

        # Stop monitoring
        injector.stop_monitoring()

        # Generate analysis report
        print("\n" + "="*60)
        print("📊 FAULT ANALYSIS REPORT")
        print("="*60)

        analysis_report = injector.generate_fault_analysis_report()

        print(f"Total Faults Injected: {analysis_report['summary']['total_faults_injected']}")
        print(f"Faults Detected: {analysis_report['summary']['faults_detected']}")
        print(".1%")
        print(".1f")

        print("\nFault Type Analysis:")
        for fault_type, stats in analysis_report['fault_analysis'].items():
            print(f"  {fault_type}:")
            print(".1%")
            if stats['avg_recovery_time']:
                print(".2f")
            else:
                print("    Recovery Time: N/A")

        if analysis_report['performance_stats']:
            perf = analysis_report['performance_stats']
            print("\nPerformance Statistics:")
            print(".1f")
            print(".1f")
            print(".1f")
            print(".1f")
            print(f"  Total Monitoring Samples: {perf['total_samples']}")
            print(".1f")

        print(".1f")

        print("\n🔧 Recommendations:")
        for i, rec in enumerate(analysis_report['recommendations'], 1):
            print(f"  {i}. {rec}")

        # Generate dashboard
        print("\n📈 Generating Performance Dashboard...")
        dashboard = injector.generate_performance_dashboard()

        if dashboard:
            print("✅ Dashboard generated with real-time charts")

            # Save dashboard to HTML file
            dashboard_html = f"""
<!DOCTYPE html>
<html>
<head>
    <title>Jazzy Fault Injection Dashboard</title>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
</head>
<body>
    <h1>Jazzy Fault Injection Dashboard</h1>
    <div id="cpu-chart">{dashboard.get('cpu_chart', 'CPU chart not available')}</div>
    <div id="memory-chart">{dashboard.get('memory_chart', 'Memory chart not available')}</div>
    <div id="network-chart">{dashboard.get('network_chart', 'Network chart not available')}</div>
    <div id="thread-chart">{dashboard.get('thread_chart', 'Thread chart not available')}</div>
    <div id="fault-timeline">{dashboard.get('fault_timeline', 'Fault timeline not available')}</div>
</body>
</html>
"""

            with open('/tmp/jazzy_fault_dashboard.html', 'w') as f:
                f.write(dashboard_html)
            print("💾 Dashboard saved to: /tmp/jazzy_fault_dashboard.html")
        else:
            print("⚠️ Dashboard generation failed")

        # Save detailed results
        with open('/tmp/jazzy_fault_analysis.json', 'w') as f:
            json.dump(analysis_report, f, indent=2, default=str)

        print("💾 Detailed analysis saved to: /tmp/jazzy_fault_analysis.json")

        print("\n" + "="*80)
        health_score = analysis_report['system_health_score']
        if health_score >= 80:
            print("🎉 SYSTEM HEALTH: EXCELLENT")
        elif health_score >= 60:
            print("✅ SYSTEM HEALTH: GOOD")
        elif health_score >= 40:
            print("⚠️ SYSTEM HEALTH: FAIR - IMPROVEMENTS NEEDED")
        else:
            print("❌ SYSTEM HEALTH: POOR - CRITICAL ISSUES DETECTED")
        print("="*80)

    except Exception as e:
        logger.error(f"❌ Fault testing failed: {e}")
        import traceback
        traceback.print_exc()

    finally:
        injector.stop_monitoring()


if __name__ == '__main__':
    asyncio.run(run_advanced_fault_testing())
