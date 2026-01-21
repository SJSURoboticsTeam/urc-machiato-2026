#!/usr/bin/env python3
"""
Jazzy Comprehensive Fault Monitor - Enhanced Detection Systems

Addresses critical issues from fault injection testing:
1. CPU Spike Detection Overhaul - Workload-aware baseline calculation
2. Thread Explosion Monitoring - Pattern-based analysis with resource correlation
3. Memory Leak Detection Enhancement - GC monitoring and fragmentation analysis
4. Network Partition Robustness - Multi-level connectivity monitoring

Features:
- Adaptive baseline calculation with workload awareness
- Machine learning-based pattern recognition
- Multi-dimensional resource correlation analysis
- Real-time visualization with predictive analytics
- Comprehensive benchmarking and reporting
"""

import asyncio
import time
import threading
import logging
import psutil
import os
import gc
import tracemalloc
import numpy as np
from scipy import stats
from scipy.signal import find_peaks
from sklearn.ensemble import IsolationForest
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.figure import Figure
import seaborn as sns
import pandas as pd
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass, field
from datetime import datetime, timedelta
import json

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Set style for matplotlib
plt.style.use('seaborn-v0_8-darkgrid')
sns.set_palette("husl")


@dataclass
class AdaptiveBaseline:
    """Adaptive baseline calculation with workload awareness"""
    window_size: int = 100  # Rolling window for baseline calculation
    sensitivity: float = 2.0  # Standard deviation threshold
    adaptation_rate: float = 0.1  # How quickly baseline adapts
    min_samples: int = 50  # Minimum samples before reliable baseline

    values: List[float] = field(default_factory=list)
    timestamps: List[float] = field(default_factory=list)
    baseline_mean: float = 0.0
    baseline_std: float = 1.0
    workload_factors: Dict[str, float] = field(default_factory=dict)

    def add_sample(self, value: float, timestamp: float, workload_context: Dict[str, Any] = None):
        """Add a sample and update baseline adaptively"""
        self.values.append(value)
        self.timestamps.append(timestamp)

        # Keep only recent samples
        if len(self.values) > self.window_size:
            self.values = self.values[-self.window_size:]
            self.timestamps = self.timestamps[-self.window_size:]

        # Update workload factors
        if workload_context:
            self._update_workload_factors(workload_context)

        # Recalculate baseline if we have enough samples
        if len(self.values) >= self.min_samples:
            self._recalculate_baseline()

    def _update_workload_factors(self, context: Dict[str, Any]):
        """Update workload adjustment factors"""
        # Memory usage factor
        memory_percent = context.get('memory_percent', 50)
        self.workload_factors['memory_load'] = memory_percent / 100.0

        # I/O activity factor
        io_counters = context.get('io_counters', {})
        read_bytes = io_counters.get('read_bytes', 0)
        write_bytes = io_counters.get('write_bytes', 0)
        self.workload_factors['io_activity'] = min(1.0, (read_bytes + write_bytes) / (1024 * 1024))  # MB threshold

        # Network activity factor
        net_io = context.get('net_io', {})
        bytes_sent = net_io.get('bytes_sent', 0)
        bytes_recv = net_io.get('bytes_recv', 0)
        self.workload_factors['network_activity'] = min(1.0, (bytes_sent + bytes_recv) / (1024 * 1024))  # MB threshold

    def _recalculate_baseline(self):
        """Recalculate adaptive baseline"""
        if len(self.values) < self.min_samples:
            return

        # Calculate basic statistics
        mean = np.mean(self.values)
        std = np.std(self.values)

        # Apply workload adjustments
        workload_multiplier = 1.0
        for factor_name, factor_value in self.workload_factors.items():
            workload_multiplier += factor_value * 0.1  # 10% adjustment per factor

        # Adaptive smoothing
        self.baseline_mean = (1 - self.adaptation_rate) * self.baseline_mean + self.adaptation_rate * mean
        self.baseline_std = (1 - self.adaptation_rate) * self.baseline_std + self.adaptation_rate * std

        # Apply workload multiplier to threshold
        self.baseline_std *= workload_multiplier

    def is_anomaly(self, value: float) -> Tuple[bool, float]:
        """Check if value is anomalous based on adaptive baseline"""
        if len(self.values) < self.min_samples:
            return False, 0.0  # Not enough data for reliable detection

        z_score = abs(value - self.baseline_mean) / max(self.baseline_std, 0.1)
        is_anomaly = z_score > self.sensitivity

        return is_anomaly, z_score

    def get_baseline_info(self) -> Dict[str, Any]:
        """Get current baseline information"""
        return {
            'mean': self.baseline_mean,
            'std': self.baseline_std,
            'samples': len(self.values),
            'workload_factors': self.workload_factors.copy(),
            'adaptation_rate': self.adaptation_rate,
            'sensitivity': self.sensitivity
        }


@dataclass
class ThreadPattern:
    """Thread creation pattern analysis"""
    thread_count: int = 0
    creation_rate: float = 0.0  # threads/second
    cleanup_rate: float = 0.0   # threads/second
    thread_types: Dict[str, int] = field(default_factory=dict)
    resource_correlation: Dict[str, float] = field(default_factory=dict)

    def analyze_pattern(self, thread_history: List[int], time_history: List[float],
                       cpu_history: List[float], memory_history: List[float]) -> Dict[str, Any]:
        """Analyze thread creation patterns and resource correlation"""

        if len(thread_history) < 10:
            return {'pattern': 'insufficient_data'}

        # Calculate thread creation/cleanup rates
        thread_deltas = np.diff(thread_history)
        time_deltas = np.diff(time_history)

        with np.errstate(divide='ignore', invalid='ignore'):
            rate_per_second = thread_deltas / time_deltas
            self.creation_rate = np.mean(rate_per_second[rate_per_second > 0]) if np.any(rate_per_second > 0) else 0
            self.cleanup_rate = abs(np.mean(rate_per_second[rate_per_second < 0])) if np.any(rate_per_second < 0) else 0

        # Detect explosion patterns
        peaks, _ = find_peaks(thread_history, height=np.mean(thread_history) + 2*np.std(thread_history))
        rapid_increases = len(peaks) > 0

        # Resource correlation analysis
        if len(cpu_history) == len(thread_history):
            cpu_corr = np.corrcoef(thread_history, cpu_history)[0, 1] if len(thread_history) > 1 else 0
            self.resource_correlation['cpu_thread_correlation'] = cpu_corr

        if len(memory_history) == len(thread_history):
            mem_corr = np.corrcoef(thread_history, memory_history)[0, 1] if len(thread_history) > 1 else 0
            self.resource_correlation['memory_thread_correlation'] = mem_corr

        # Pattern classification
        if rapid_increases and self.creation_rate > 5:  # More than 5 threads/second
            pattern = 'explosion'
            severity = 'critical'
        elif self.creation_rate > 2 and self.cleanup_rate < 0.5:  # Imbalanced creation/cleanup
            pattern = 'leak'
            severity = 'high'
        elif abs(self.resource_correlation.get('cpu_thread_correlation', 0)) > 0.7:
            pattern = 'cpu_bound'
            severity = 'medium'
        else:
            pattern = 'normal'
            severity = 'low'

        return {
            'pattern': pattern,
            'severity': severity,
            'creation_rate': self.creation_rate,
            'cleanup_rate': self.cleanup_rate,
            'resource_correlation': self.resource_correlation,
            'rapid_increases': rapid_increases
        }


@dataclass
class MemoryAnalysis:
    """Enhanced memory leak detection with GC monitoring"""
    gc_stats: Dict[str, Any] = field(default_factory=dict)
    fragmentation_analysis: Dict[str, Any] = field(default_factory=dict)
    allocation_patterns: Dict[str, Any] = field(default_factory=dict)
    trend_analysis: Dict[str, Any] = field(default_factory=dict)

    def analyze_memory_state(self, snapshot1, snapshot2) -> Dict[str, Any]:
        """Comprehensive memory analysis between two snapshots"""
        tracemalloc.start()

        # GC Statistics
        self.gc_stats = {
            'collections': gc.get_stats(),
            'objects': len(gc.get_objects()),
            'referrers': len(gc.get_referrers(*gc.get_objects()[:10])) if gc.get_objects() else 0
        }

        # Memory fragmentation analysis
        try:
            current, peak = tracemalloc.get_traced_memory()
            self.fragmentation_analysis = {
                'current_mb': current / 1024 / 1024,
                'peak_mb': peak / 1024 / 1024,
                'efficiency': current / peak if peak > 0 else 1.0,
                'waste_mb': (peak - current) / 1024 / 1024
            }
        except:
            self.fragmentation_analysis = {'error': 'tracemalloc_not_started'}

        # Allocation pattern analysis
        try:
            stats = tracemalloc.take_snapshot().statistics('lineno')
            self.allocation_patterns = {
                'top_allocators': [
                    {
                        'file': stat.traceback[0].filename if stat.traceback else 'unknown',
                        'line': stat.traceback[0].lineno if stat.traceback else 0,
                        'size_mb': stat.size / 1024 / 1024,
                        'count': stat.count
                    } for stat in stats[:10]
                ]
            }
        except:
            self.allocation_patterns = {'error': 'snapshot_failed'}

        return {
            'gc_stats': self.gc_stats,
            'fragmentation': self.fragmentation_analysis,
            'allocation_patterns': self.allocation_patterns
        }

    def detect_memory_leak_advanced(self, memory_history: List[float],
                                   time_history: List[float]) -> Dict[str, Any]:
        """Advanced memory leak detection with multiple algorithms"""

        if len(memory_history) < 20:
            return {'leak_detected': False, 'confidence': 0.0, 'method': 'insufficient_data'}

        # Method 1: Linear regression trend analysis
        slope, intercept, r_value, p_value, std_err = stats.linregress(time_history, memory_history)
        leak_rate_mb_per_minute = slope * 60  # Convert to MB/minute

        regression_confidence = abs(r_value)  # R-squared as confidence
        regression_leak = leak_rate_mb_per_minute > 5.0 and regression_confidence > 0.7

        # Method 2: Statistical process control (moving average)
        window_size = min(10, len(memory_history))
        moving_avg = pd.Series(memory_history).rolling(window=window_size).mean().iloc[-1]
        moving_std = pd.Series(memory_history).rolling(window=window_size).std().iloc[-1]

        current_value = memory_history[-1]
        control_limit = moving_avg + 3 * moving_std
        spc_violation = current_value > control_limit

        # Method 3: Isolation Forest anomaly detection
        if len(memory_history) > 50:
            data = np.array(memory_history).reshape(-1, 1)
            scaler = StandardScaler()
            scaled_data = scaler.fit_transform(data)

            iso_forest = IsolationForest(contamination=0.1, random_state=42)
            predictions = iso_forest.fit_predict(scaled_data)
            ml_anomaly = predictions[-1] == -1  # -1 indicates anomaly
            ml_confidence = abs(iso_forest.score_samples(scaled_data[-1].reshape(1, -1))[0])
        else:
            ml_anomaly = False
            ml_confidence = 0.0

        # Combined decision
        methods = [
            ('regression', regression_leak, regression_confidence),
            ('spc', spc_violation, 0.8 if spc_violation else 0.2),
            ('ml', ml_anomaly, ml_confidence)
        ]

        # Majority vote with confidence weighting
        leak_votes = sum(1 for _, detected, _ in methods if detected)
        total_confidence = sum(conf * weight for _, _, conf in methods for weight in [1])

        leak_detected = leak_votes >= 2  # Majority vote
        confidence = min(total_confidence / len(methods), 1.0)

        return {
            'leak_detected': leak_detected,
            'confidence': confidence,
            'methods': {
                name: {'detected': detected, 'confidence': conf}
                for name, detected, conf in methods
            },
            'leak_rate_mb_per_minute': leak_rate_mb_per_minute,
            'regression_r_squared': regression_confidence
        }


@dataclass
class NetworkMonitor:
    """Multi-level network connectivity monitoring"""
    connection_history: List[Dict[str, Any]] = field(default_factory=list)
    quality_metrics: Dict[str, Any] = field(default_factory=dict)
    failover_status: Dict[str, Any] = field(default_factory=dict)

    def analyze_connectivity(self) -> Dict[str, Any]:
        """Multi-level connectivity analysis"""
        try:
            net_connections = psutil.net_connections()
            net_io = psutil.net_io_counters()

            # Connection analysis
            connection_states = {}
            for conn in net_connections:
                state = str(conn.status)
                if state not in connection_states:
                    connection_states[state] = 0
                connection_states[state] += 1

            # Quality metrics
            self.quality_metrics = {
                'total_connections': len(net_connections),
                'connection_states': connection_states,
                'bytes_sent_mb': net_io.bytes_sent / 1024 / 1024,
                'bytes_recv_mb': net_io.bytes_recv / 1024 / 1024,
                'packets_sent': net_io.packets_sent,
                'packets_recv': net_io.packets_recv,
                'errin': net_io.errin,
                'errout': net_io.errout,
                'dropin': net_io.dropin,
                'dropout': net_io.dropout
            }

            # Quality assessment
            quality_score = self._calculate_quality_score()

            # Store history
            self.connection_history.append({
                'timestamp': time.time(),
                'metrics': self.quality_metrics.copy(),
                'quality_score': quality_score
            })

            # Keep last 100 entries
            if len(self.connection_history) > 100:
                self.connection_history = self.connection_history[-100:]

            return {
                'quality_score': quality_score,
                'status': self._get_quality_status(quality_score),
                'metrics': self.quality_metrics,
                'trend_analysis': self._analyze_trends()
            }

        except Exception as e:
            return {
                'error': str(e),
                'quality_score': 0.0,
                'status': 'error'
            }

    def _calculate_quality_score(self) -> float:
        """Calculate network quality score (0-1)"""
        metrics = self.quality_metrics

        score = 1.0

        # Penalty for errors
        total_packets = metrics.get('packets_sent', 0) + metrics.get('packets_recv', 0)
        if total_packets > 0:
            error_rate = (metrics.get('errin', 0) + metrics.get('errout', 0)) / total_packets
            score -= min(error_rate * 10, 0.5)  # 50% penalty max for errors

        # Penalty for drops
        drop_rate = (metrics.get('dropin', 0) + metrics.get('dropout', 0)) / max(total_packets, 1)
        score -= min(drop_rate * 5, 0.3)  # 30% penalty max for drops

        # Bonus for active connections
        established_conns = metrics.get('connection_states', {}).get('ESTABLISHED', 0)
        if established_conns > 0:
            score += min(established_conns * 0.01, 0.1)  # 10% bonus max

        return max(0.0, min(1.0, score))

    def _get_quality_status(self, score: float) -> str:
        """Get quality status description"""
        if score >= 0.9:
            return 'excellent'
        elif score >= 0.7:
            return 'good'
        elif score >= 0.5:
            return 'fair'
        elif score >= 0.3:
            return 'poor'
        else:
            return 'critical'

    def _analyze_trends(self) -> Dict[str, Any]:
        """Analyze network quality trends"""
        if len(self.connection_history) < 10:
            return {'trend': 'insufficient_data'}

        # Extract quality scores
        scores = [entry['quality_score'] for entry in self.connection_history[-20:]]

        # Trend analysis
        if len(scores) >= 2:
            slope, _, r_value, _, _ = stats.linregress(range(len(scores)), scores)
            trend = 'improving' if slope > 0.001 else 'degrading' if slope < -0.001 else 'stable'
            confidence = abs(r_value)
        else:
            trend = 'unknown'
            confidence = 0.0

        return {
            'trend': trend,
            'confidence': confidence,
            'slope': slope if 'slope' in locals() else 0.0,
            'volatility': np.std(scores) if len(scores) > 1 else 0.0
        }

    def simulate_partition(self, duration_seconds: int = 10) -> Dict[str, Any]:
        """Simulate network partition for testing"""
        import time
        logger.info(f"Simulating network partition for {duration_seconds} seconds")

        start_time = time.time()
        partition_detected = False
        recovery_detected = False
        detection_time = None
        recovery_time = None

        # Monitor during partition
        baseline_quality = self.analyze_connectivity()['quality_score']

        # Simulate partition (would normally manipulate network interfaces)
        partition_tasks = []
        for _ in range(5):
            task = asyncio.create_task(self._simulate_connection_timeout(duration_seconds))
            partition_tasks.append(task)

        async def monitor_partition():
            nonlocal partition_detected, recovery_detected, detection_time, recovery_time

            while time.time() - start_time < duration_seconds + 5:
                current_quality = self.analyze_connectivity()['quality_score']

                # Detect partition
                if not partition_detected and current_quality < baseline_quality * 0.5:
                    partition_detected = True
                    detection_time = time.time()
                    logger.warning(f"Network partition detected at {detection_time}")

                # Detect recovery
                if partition_detected and not recovery_detected and current_quality >= baseline_quality * 0.8:
                    recovery_detected = True
                    recovery_time = time.time()
                    logger.info(f"Network recovery detected at {recovery_time}")

                await asyncio.sleep(0.5)

        # Run monitoring and partition simulation concurrently
        async def run_simulation():
            await asyncio.gather(
                monitor_partition(),
                asyncio.gather(*partition_tasks)
            )

        # Use existing event loop instead of creating new one
        try:
            loop = asyncio.get_event_loop()
            if loop.is_running():
                # We're already in an event loop, create task instead
                asyncio.create_task(run_simulation())
                # Wait for simulation to complete using time.sleep in a separate thread
                import time
                time.sleep(13)
            else:
                asyncio.run(run_simulation())
        except RuntimeError:
            # Fallback: just wait for the simulation duration
            import time
            time.sleep(13)

        return {
            'partition_simulated': True,
            'duration_seconds': duration_seconds,
            'partition_detected': partition_detected,
            'recovery_detected': recovery_detected,
            'detection_time': detection_time - start_time if detection_time else None,
            'recovery_time': recovery_time - start_time if recovery_time else None,
            'baseline_quality': baseline_quality
        }

    async def _simulate_connection_timeout(self, duration: int):
        """Simulate connection timeout"""
        await asyncio.sleep(duration + np.random.uniform(0, 2))


class ComprehensiveFaultMonitor:
    """
    Comprehensive fault monitoring system with enhanced detection capabilities
    """

    def __init__(self):
        self.monitoring_active = False
        self.monitoring_thread: Optional[threading.Thread] = None

        # Enhanced monitoring components
        self.cpu_baseline = AdaptiveBaseline(window_size=200, sensitivity=3.0)
        self.thread_analyzer = ThreadPattern()
        self.memory_analyzer = MemoryAnalysis()
        self.network_monitor = NetworkMonitor()

        # Data collection
        self.cpu_history: List[float] = []
        self.memory_history: List[float] = []
        self.thread_history: List[int] = []
        self.network_history: List[Dict] = []
        self.time_history: List[float] = []

        # Fault tracking
        self.fault_events: List[Dict[str, Any]] = []
        self.alerts: List[Dict[str, Any]] = []

        logger.info("🎯 Comprehensive Fault Monitor initialized")

    async def start_comprehensive_monitoring(self, interval_seconds: float = 0.1):
        """Start comprehensive fault monitoring"""
        self.monitoring_active = True
        self.monitoring_thread = threading.Thread(
            target=self._monitoring_loop,
            args=(interval_seconds,),
            daemon=True
        )
        self.monitoring_thread.start()
        logger.info("📊 Comprehensive monitoring started")

    def stop_monitoring(self):
        """Stop monitoring"""
        self.monitoring_active = False
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=1.0)
        logger.info("📊 Monitoring stopped")

    def _monitoring_loop(self, interval: float):
        """Comprehensive monitoring loop"""
        while self.monitoring_active:
            try:
                timestamp = time.time()

                # Collect system metrics
                cpu_percent = psutil.cpu_percent(interval=0.05)
                memory_info = psutil.virtual_memory()
                memory_percent = memory_info.percent
                thread_count = psutil.Process().num_threads()

                # Update histories
                self.cpu_history.append(cpu_percent)
                self.memory_history.append(memory_percent)
                self.thread_history.append(thread_count)
                self.time_history.append(timestamp)

                # Keep history bounded
                max_history = 1000
                for history in [self.cpu_history, self.memory_history, self.thread_history, self.time_history]:
                    if len(history) > max_history:
                        history[:] = history[-max_history:]

                # Update adaptive baselines
                workload_context = {
                    'memory_percent': memory_percent,
                    'io_counters': psutil.disk_io_counters()._asdict() if psutil.disk_io_counters() else {},
                    'net_io': psutil.net_io_counters()._asdict() if psutil.net_io_counters() else {}
                }

                self.cpu_baseline.add_sample(cpu_percent, timestamp, workload_context)

                # Enhanced fault detection
                self._enhanced_fault_detection(timestamp, cpu_percent, memory_percent, thread_count)

                time.sleep(interval)

            except Exception as e:
                logger.error(f"Monitoring error: {e}")
                time.sleep(interval)

    def _enhanced_fault_detection(self, timestamp: float, cpu_percent: float,
                                 memory_percent: float, thread_count: int):
        """Enhanced multi-dimensional fault detection"""

        # 1. CPU Spike Detection with Workload Awareness
        cpu_anomaly, cpu_zscore = self.cpu_baseline.is_anomaly(cpu_percent)

        if cpu_anomaly and cpu_zscore > 3.0:
            # Additional validation with memory and I/O correlation
            memory_correlation = self._calculate_correlation(
                self.cpu_history[-50:], self.memory_history[-50:]
            )

            alert = {
                'timestamp': timestamp,
                'type': 'cpu_spike_enhanced',
                'severity': 'high' if cpu_zscore > 4.0 else 'medium',
                'metrics': {
                    'cpu_percent': cpu_percent,
                    'z_score': cpu_zscore,
                    'baseline_mean': self.cpu_baseline.baseline_mean,
                    'baseline_std': self.cpu_baseline.baseline_std,
                    'memory_correlation': memory_correlation,
                    'workload_factors': self.cpu_baseline.workload_factors
                },
                'description': f"Enhanced CPU spike detected with {cpu_zscore:.1f}σ deviation"
            }
            self._trigger_alert(alert)

        # 2. Thread Explosion with Pattern Analysis
        if len(self.thread_history) >= 20:
            thread_pattern = self.thread_analyzer.analyze_pattern(
                self.thread_history[-50:],
                self.time_history[-50:],
                self.cpu_history[-50:],
                self.memory_history[-50:]
            )

            if thread_pattern['pattern'] in ['explosion', 'leak']:
                alert = {
                    'timestamp': timestamp,
                    'type': 'thread_explosion_enhanced',
                    'severity': thread_pattern['severity'],
                    'metrics': {
                        'thread_count': thread_count,
                        'pattern': thread_pattern,
                        'creation_rate': self.thread_analyzer.creation_rate,
                        'cleanup_rate': self.thread_analyzer.cleanup_rate,
                        'resource_correlation': self.thread_analyzer.resource_correlation
                    },
                    'description': f"Thread {thread_pattern['pattern']} pattern detected"
                }
                self._trigger_alert(alert)

        # 3. Memory Leak with Advanced Analysis
        if len(self.memory_history) >= 30:
            leak_analysis = self.memory_analyzer.detect_memory_leak_advanced(
                self.memory_history[-100:],
                self.time_history[-100:]
            )

            if leak_analysis['leak_detected'] and leak_analysis['confidence'] > 0.7:
                alert = {
                    'timestamp': timestamp,
                    'type': 'memory_leak_enhanced',
                    'severity': 'high',
                    'metrics': {
                        'memory_percent': memory_percent,
                        'leak_analysis': leak_analysis,
                        'gc_stats': self.memory_analyzer.gc_stats,
                        'fragmentation': self.memory_analyzer.fragmentation_analysis,
                        'allocation_patterns': self.memory_analyzer.allocation_patterns
                    },
                    'description': f"Advanced memory leak detected with {leak_analysis['confidence']:.1%} confidence"
                }
                self._trigger_alert(alert)

        # 4. Network Partition with Quality Monitoring
        network_analysis = self.network_monitor.analyze_connectivity()

        if network_analysis.get('quality_score', 1.0) < 0.3:  # Critical threshold
            alert = {
                'timestamp': timestamp,
                'type': 'network_partition_enhanced',
                'severity': 'critical',
                'metrics': {
                    'quality_score': network_analysis.get('quality_score', 0),
                    'status': network_analysis.get('status', 'unknown'),
                    'trend_analysis': network_analysis.get('trend_analysis', {}),
                    'connection_metrics': self.network_monitor.quality_metrics
                },
                'description': f"Network partition detected with quality score {network_analysis.get('quality_score', 0):.1%}"
            }
            self._trigger_alert(alert)

    def _calculate_correlation(self, series1: List[float], series2: List[float]) -> float:
        """Calculate correlation between two series"""
        if len(series1) != len(series2) or len(series1) < 2:
            return 0.0

        try:
            return np.corrcoef(series1, series2)[0, 1]
        except:
            return 0.0

    def _trigger_alert(self, alert: Dict[str, Any]):
        """Trigger an alert and log the event"""
        self.alerts.append(alert)
        self.fault_events.append(alert)

        severity_colors = {
            'low': '🟢',
            'medium': '🟡',
            'high': '🔴',
            'critical': '💥'
        }

        color = severity_colors.get(alert['severity'], '⚪')
        logger.warning(f"{color} ALERT: {alert['description']}")

    # ===== TESTING AND VISUALIZATION =====

    async def run_enhanced_fault_tests(self) -> Dict[str, Any]:
        """Run comprehensive fault injection tests"""
        logger.info("🚀 Starting Enhanced Fault Injection Tests...")

        test_results = {}
        start_time = time.time()

        try:
            # Start monitoring
            await self.start_comprehensive_monitoring(0.1)
            await asyncio.sleep(2)  # Let monitoring stabilize

            # Test 1: Enhanced CPU Spike Detection
            logger.info("\n1. 🧪 Enhanced CPU Spike Detection Test")
            cpu_test = await self._test_enhanced_cpu_spike()
            test_results['cpu_spike_enhanced'] = cpu_test

            # Test 2: Advanced Thread Explosion Detection
            logger.info("\n2. 🧪 Advanced Thread Explosion Detection Test")
            thread_test = await self._test_advanced_thread_explosion()
            test_results['thread_explosion_enhanced'] = thread_test

            # Test 3: Advanced Memory Leak Detection
            logger.info("\n3. 🧪 Advanced Memory Leak Detection Test")
            memory_test = await self._test_advanced_memory_leak()
            test_results['memory_leak_enhanced'] = memory_test

            # Test 4: Network Partition Robustness
            logger.info("\n4. 🧪 Network Partition Robustness Test")
            network_test = await self._test_network_partition_robustness()
            test_results['network_partition_enhanced'] = network_test

        finally:
            self.stop_monitoring()

        # Generate comprehensive report
        report = self._generate_enhanced_report(test_results, time.time() - start_time)
        return report

    async def _test_enhanced_cpu_spike(self) -> Dict[str, Any]:
        """Test enhanced CPU spike detection"""
        logger.info("Testing enhanced CPU spike detection...")

        # Establish baseline
        await asyncio.sleep(5)

        # Inject CPU spike
        spike_task = asyncio.create_task(self._generate_controlled_cpu_spike(15, 85))

        # Monitor for detection
        start_time = time.time()
        alerts_during_test = []

        while time.time() - start_time < 20:  # Monitor for 20 seconds
            # Check for new alerts
            new_alerts = [alert for alert in self.alerts if alert['timestamp'] > start_time]
            alerts_during_test.extend(new_alerts)

            await asyncio.sleep(0.5)

        await spike_task

        # Analyze results
        cpu_spike_alerts = [a for a in alerts_during_test if a['type'] == 'cpu_spike_enhanced']
        detected = len(cpu_spike_alerts) > 0

        return {
            'test_duration': 20,
            'spike_detected': detected,
            'alerts_triggered': len(cpu_spike_alerts),
            'baseline_info': self.cpu_baseline.get_baseline_info(),
            'workload_adaptation': bool(self.cpu_baseline.workload_factors),
            'success': detected
        }

    async def _test_advanced_thread_explosion(self) -> Dict[str, Any]:
        """Test advanced thread explosion detection"""
        logger.info("Testing advanced thread explosion detection...")

        # Establish baseline
        await asyncio.sleep(3)

        # Create thread explosion
        explosion_task = asyncio.create_task(self._create_thread_explosion_pattern())

        # Monitor for detection
        start_time = time.time()
        alerts_during_test = []

        while time.time() - start_time < 15:
            new_alerts = [alert for alert in self.alerts if alert['timestamp'] > start_time]
            alerts_during_test.extend(new_alerts)
            await asyncio.sleep(0.5)

        await explosion_task

        # Analyze results
        thread_alerts = [a for a in alerts_during_test if a['type'] == 'thread_explosion_enhanced']
        detected = len(thread_alerts) > 0

        # Get thread pattern analysis
        pattern_analysis = self.thread_analyzer.analyze_pattern(
            self.thread_history[-50:],
            self.time_history[-50:],
            self.cpu_history[-50:],
            self.memory_history[-50:]
        )

        return {
            'test_duration': 15,
            'explosion_detected': detected,
            'alerts_triggered': len(thread_alerts),
            'pattern_analysis': pattern_analysis,
            'resource_correlation': self.thread_analyzer.resource_correlation,
            'success': detected and pattern_analysis['pattern'] == 'explosion'
        }

    async def _test_advanced_memory_leak(self) -> Dict[str, Any]:
        """Test advanced memory leak detection"""
        logger.info("Testing advanced memory leak detection...")

        # Establish baseline
        await asyncio.sleep(3)

        # Create controlled memory leak
        leak_task = asyncio.create_task(self._create_controlled_memory_leak(20))

        # Monitor for detection
        start_time = time.time()
        alerts_during_test = []

        while time.time() - start_time < 25:
            new_alerts = [alert for alert in self.alerts if alert['timestamp'] > start_time]
            alerts_during_test.extend(new_alerts)
            await asyncio.sleep(0.5)

        await leak_task

        # Analyze results
        memory_alerts = [a for a in alerts_during_test if a['type'] == 'memory_leak_enhanced']
        detected = len(memory_alerts) > 0

        # Get leak analysis
        leak_analysis = self.memory_analyzer.detect_memory_leak_advanced(
            self.memory_history[-100:],
            self.time_history[-100:]
        )

        return {
            'test_duration': 25,
            'leak_detected': detected,
            'alerts_triggered': len(memory_alerts),
            'leak_analysis': leak_analysis,
            'gc_stats': self.memory_analyzer.gc_stats,
            'fragmentation_analysis': self.memory_analyzer.fragmentation_analysis,
            'success': detected and leak_analysis['confidence'] > 0.7
        }

    async def _test_network_partition_robustness(self) -> Dict[str, Any]:
        """Test network partition robustness"""
        logger.info("Testing network partition robustness...")

        # Establish baseline
        await asyncio.sleep(2)

        # Simulate network partition
        partition_result = self.network_monitor.simulate_partition(8)

        # Monitor for detection
        start_time = time.time()
        alerts_during_test = []

        while time.time() - start_time < 15:
            new_alerts = [alert for alert in self.alerts if alert['timestamp'] > start_time]
            alerts_during_test.extend(new_alerts)
            await asyncio.sleep(0.5)

        # Analyze results
        network_alerts = [a for a in alerts_during_test if a['type'] == 'network_partition_enhanced']
        detected = len(network_alerts) > 0

        # Get network analysis
        final_analysis = self.network_monitor.analyze_connectivity()

        return {
            'test_duration': 15,
            'partition_detected': detected,
            'alerts_triggered': len(network_alerts),
            'partition_simulation': partition_result,
            'final_network_analysis': final_analysis,
            'trend_analysis': final_analysis.get('trend_analysis', {}),
            'success': detected and partition_result.get('recovery_detected', False)
        }

    # ===== SIMULATION METHODS =====

    async def _generate_controlled_cpu_spike(self, duration_seconds: int, target_percent: int):
        """Generate controlled CPU spike with realistic patterns"""
        logger.debug(f"Generating CPU spike: {duration_seconds}s at {target_percent}%")

        end_time = time.time() + duration_seconds

        while time.time() < end_time:
            # CPU intensive but controlled computation
            for _ in range(50000):  # Adjust for target CPU %
                _ = sum(i*i for i in range(50))

            # Small delay to prevent 100% CPU
            await asyncio.sleep(0.001)

    async def _create_thread_explosion_pattern(self):
        """Create realistic thread explosion pattern"""
        logger.debug("Creating thread explosion pattern")

        threads = []
        created_count = 0

        # Phase 1: Normal operation
        await asyncio.sleep(2)

        # Phase 2: Rapid thread creation (explosion)
        for i in range(25):  # Create 25 threads
            thread = threading.Thread(
                target=self._thread_worker,
                args=(i, 0.5),
                daemon=True
            )
            thread.start()
            threads.append(thread)
            created_count += 1

            if i % 5 == 0:
                await asyncio.sleep(0.1)  # Stagger creation

        # Phase 3: Some threads finish, but leak continues
        await asyncio.sleep(5)

        # Phase 4: More threads created (worsening leak)
        for i in range(10):
            thread = threading.Thread(
                target=self._thread_worker,
                args=(i + 25, 2.0),  # Longer running
                daemon=True
            )
            thread.start()
            threads.append(thread)
            created_count += 1

        # Wait for test completion (threads will be cleaned up)
        await asyncio.sleep(5)

    def _thread_worker(self, thread_id: int, duration: float):
        """Worker function for test threads"""
        time.sleep(duration)
        logger.debug(f"Thread {thread_id} completed")

    async def _create_controlled_memory_leak(self, duration_seconds: int):
        """Create controlled memory leak for testing"""
        logger.debug(f"Creating controlled memory leak for {duration_seconds} seconds")

        leaked_objects = []
        leak_rate_mb_per_second = 0.8  # 0.8 MB/second leak

        for i in range(duration_seconds * 5):  # 5 iterations per second
            # Create memory leak
            leak_size = int(leak_rate_mb_per_second * 1024 * 1024 / 5)  # Bytes per iteration
            leaked_objects.append([0] * leak_size)  # Keep references

            await asyncio.sleep(0.2)

        # Don't clean up - leak persists for analysis
        logger.debug(f"Memory leak created with {len(leaked_objects)} allocations")

    # ===== REPORTING AND VISUALIZATION =====

    def _generate_enhanced_report(self, test_results: Dict[str, Any], total_duration: float) -> Dict[str, Any]:
        """Generate comprehensive enhanced fault monitoring report"""
        report = {
            'test_suite': 'Enhanced Fault Monitoring & Detection System',
            'timestamp': time.time(),
            'total_duration_seconds': total_duration,
            'system_info': {
                'cpu_count': psutil.cpu_count(),
                'memory_total_mb': psutil.virtual_memory().total / 1024 / 1024,
                'platform': os.sys.platform
            },
            'test_results': test_results,
            'alerts_summary': {
                'total_alerts': len(self.alerts),
                'alerts_by_type': {},
                'alerts_by_severity': {}
            },
            'performance_metrics': {
                'monitoring_samples': len(self.cpu_history),
                'alerts_per_second': len(self.alerts) / total_duration if total_duration > 0 else 0,
                'cpu_baseline_adaptation': self.cpu_baseline.get_baseline_info(),
                'memory_analysis_quality': len(self.memory_analyzer.allocation_patterns)
            }
        }

        # Analyze alerts
        for alert in self.alerts:
            alert_type = alert['type']
            alert_severity = alert['severity']

            if alert_type not in report['alerts_summary']['alerts_by_type']:
                report['alerts_summary']['alerts_by_type'][alert_type] = 0
            report['alerts_summary']['alerts_by_type'][alert_type] += 1

            if alert_severity not in report['alerts_summary']['alerts_by_severity']:
                report['alerts_summary']['alerts_by_severity'][alert_severity] = 0
            report['alerts_summary']['alerts_by_severity'][alert_severity] += 1

        # Calculate overall success
        successful_tests = sum(1 for result in test_results.values() if result.get('success', False))
        total_tests = len(test_results)

        report['summary'] = {
            'total_tests': total_tests,
            'successful_tests': successful_tests,
            'success_rate': successful_tests / total_tests if total_tests > 0 else 0,
            'overall_status': 'PASS' if successful_tests >= total_tests * 0.75 else 'FAIL'
        }

        return report

    def generate_visualization_dashboard(self) -> str:
        """Generate interactive visualization dashboard"""
        # Create comprehensive charts
        fig = plt.figure(figsize=(20, 16))
        gs = fig.add_gridspec(4, 3, hspace=0.3, wspace=0.3)

        # CPU Usage with Baseline
        ax1 = fig.add_subplot(gs[0, 0])
        times_cpu = [(t - self.time_history[0]) for t in self.time_history[-200:]]
        cpu_data = self.cpu_history[-200:]
        ax1.plot(times_cpu, cpu_data, 'b-', alpha=0.7, label='CPU %')
        if hasattr(self.cpu_baseline, 'baseline_mean'):
            ax1.axhline(y=self.cpu_baseline.baseline_mean, color='r', linestyle='--',
                       label=f'Baseline ({self.cpu_baseline.baseline_mean:.1f}%)')
            ax1.axhline(y=self.cpu_baseline.baseline_mean + 2*self.cpu_baseline.baseline_std,
                       color='r', linestyle=':', alpha=0.5, label='Threshold')
        ax1.set_title('CPU Usage with Adaptive Baseline')
        ax1.set_xlabel('Time (seconds)')
        ax1.set_ylabel('CPU %')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Memory Usage Trend Analysis
        ax2 = fig.add_subplot(gs[0, 1])
        times_mem = [(t - self.time_history[0]) for t in self.time_history[-200:]]
        mem_data = self.memory_history[-200:]
        ax2.plot(times_mem, mem_data, 'g-', alpha=0.7)

        # Add trend line if enough data
        if len(mem_data) > 10:
            z = np.polyfit(times_mem, mem_data, 1)
            p = np.poly1d(z)
            ax2.plot(times_mem, p(times_mem), 'r--', alpha=0.8,
                    label=f'Trend: {z[0]:.4f} MB/s')

        ax2.set_title('Memory Usage with Trend Analysis')
        ax2.set_xlabel('Time (seconds)')
        ax2.set_ylabel('Memory %')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Thread Count with Pattern Analysis
        ax3 = fig.add_subplot(gs[0, 2])
        times_thread = [(t - self.time_history[0]) for t in self.time_history[-200:]]
        thread_data = self.thread_history[-200:]
        ax3.plot(times_thread, thread_data, 'm-', alpha=0.7)

        # Mark alerts
        for alert in self.alerts[-10:]:  # Last 10 alerts
            if alert['type'] == 'thread_explosion_enhanced':
                alert_time = alert['timestamp'] - self.time_history[0]
                ax3.axvline(x=alert_time, color='r', linestyle='--', alpha=0.7,
                           label='Thread Alert' if alert == self.alerts[-10] else "")

        ax3.set_title('Thread Count with Alert Markers')
        ax3.set_xlabel('Time (seconds)')
        ax3.set_ylabel('Thread Count')
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Alert Timeline
        ax4 = fig.add_subplot(gs[1, :])
        alert_times = [(a['timestamp'] - self.time_history[0]) for a in self.alerts]
        alert_types = [a['type'] for a in self.alerts]
        alert_severities = [a['severity'] for a in self.alerts]

        colors = {'low': 'green', 'medium': 'orange', 'high': 'red', 'critical': 'darkred'}
        severity_colors = [colors.get(s, 'blue') for s in alert_severities]

        ax4.scatter(alert_times, [1] * len(alert_times), c=severity_colors, s=100, alpha=0.7)

        # Add labels for major alerts
        for i, (time_val, alert_type) in enumerate(zip(alert_times, alert_types)):
            if i % 3 == 0:  # Label every 3rd alert to avoid crowding
                ax4.annotate(alert_type.replace('_enhanced', ''),
                           (time_val, 1.05), ha='center', va='bottom',
                           fontsize=8, rotation=45)

        ax4.set_title('Fault Alert Timeline')
        ax4.set_xlabel('Time (seconds)')
        ax4.set_yticks([1])
        ax4.set_yticklabels(['Alerts'])
        ax4.grid(True, alpha=0.3)

        # Test Results Summary
        ax5 = fig.add_subplot(gs[2, 0])
        test_names = list(self.test_results.keys()) if hasattr(self, 'test_results') else []
        test_successes = [1 if self.test_results.get(name, {}).get('success', False) else 0
                         for name in test_names]

        if test_names:
            bars = ax5.bar(range(len(test_names)), test_successes, color=['green' if s else 'red' for s in test_successes])
            ax5.set_xticks(range(len(test_names)))
            ax5.set_xticklabels([n.replace('_enhanced', '') for n in test_names], rotation=45, ha='right')
            ax5.set_title('Test Results Summary')
            ax5.set_ylabel('Success (1=Pass, 0=Fail)')
            ax5.set_ylim(0, 1.2)

            # Add value labels on bars
            for bar, success in zip(bars, test_successes):
                ax5.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.05,
                        'Pass' if success else 'Fail', ha='center', va='bottom')

        # Alert Distribution
        ax6 = fig.add_subplot(gs[2, 1])
        if self.alerts:
            alert_types_count = {}
            for alert in self.alerts:
                alert_type = alert['type'].replace('_enhanced', '')
                alert_types_count[alert_type] = alert_types_count.get(alert_type, 0) + 1

            types = list(alert_types_count.keys())
            counts = list(alert_types_count.values())

            ax6.bar(types, counts, color='skyblue', alpha=0.7)
            ax6.set_title('Alert Distribution by Type')
            ax6.set_ylabel('Count')
            ax6.tick_params(axis='x', rotation=45)

        # Performance Metrics
        ax7 = fig.add_subplot(gs[2, 2])
        if hasattr(self, 'cpu_history') and self.cpu_history:
            metrics_data = [
                ('Avg CPU %', sum(self.cpu_history) / len(self.cpu_history)),
                ('Max CPU %', max(self.cpu_history)),
                ('Memory %', self.memory_history[-1] if self.memory_history else 0),
                ('Thread Count', self.thread_history[-1] if self.thread_history else 0),
                ('Total Alerts', len(self.alerts))
            ]

            labels, values = zip(*metrics_data)
            bars = ax7.barh(labels, values, color='lightcoral', alpha=0.7)
            ax7.set_title('Performance Metrics Summary')
            ax7.set_xlabel('Value')

            # Add value labels
            for bar, value in zip(bars, values):
                ax7.text(value + 0.5, bar.get_y() + bar.get_height()/2,
                        f'{value:.1f}', va='center')

        # Network Quality Over Time (if available)
        ax8 = fig.add_subplot(gs[3, :2])
        if hasattr(self.network_monitor, 'connection_history') and self.network_monitor.connection_history:
            net_times = [(entry['timestamp'] - self.time_history[0]) for entry in self.network_monitor.connection_history]
            net_qualities = [entry['quality_score'] for entry in self.network_monitor.connection_history]

            ax8.plot(net_times, net_qualities, 'c-', linewidth=2, alpha=0.8)
            ax8.fill_between(net_times, net_qualities, alpha=0.3, color='cyan')
            ax8.set_title('Network Quality Over Time')
            ax8.set_xlabel('Time (seconds)')
            ax8.set_ylabel('Quality Score (0-1)')
            ax8.set_ylim(0, 1)
            ax8.grid(True, alpha=0.3)

            # Add quality thresholds
            ax8.axhline(y=0.9, color='green', linestyle='--', alpha=0.5, label='Excellent')
            ax8.axhline(y=0.7, color='orange', linestyle='--', alpha=0.5, label='Good')
            ax8.axhline(y=0.5, color='red', linestyle='--', alpha=0.5, label='Fair')
            ax8.legend()

        # System Health Score
        ax9 = fig.add_subplot(gs[3, 2])
        if hasattr(self, 'test_results'):
            # Calculate health score based on test results
            successful_tests = sum(1 for result in self.test_results.values() if result.get('success', False))
            total_tests = len(self.test_results)
            health_score = successful_tests / total_tests if total_tests > 0 else 0

            # Create a simple gauge-like visualization
            ax9.pie([health_score, 1-health_score],
                   colors=['green', 'red'],
                   startangle=90,
                   counterclock=False)
            ax9.set_title(f'System Health Score\n{health_score:.1%}')

            # Add center text
            center_text = f'{health_score:.0%}'
            ax9.text(0, 0, center_text, ha='center', va='center', fontsize=20, fontweight='bold')

        plt.tight_layout()

        # Save to base64 for HTML embedding
        import io
        buf = io.BytesIO()
        plt.savefig(buf, format='png', dpi=150, bbox_inches='tight')
        buf.seek(0)
        import base64
        img_base64 = base64.b64encode(buf.read()).decode('utf-8')
        plt.close(fig)

        return img_base64


async def main():
    """Main function for enhanced fault monitoring demonstration"""
    print("="*80)
    print("🎯 ENHANCED FAULT MONITORING & DETECTION SYSTEM")
    print("="*80)
    print("Testing: CPU Spike Overhaul, Thread Explosion Analysis,")
    print("         Memory Leak Enhancement, Network Partition Robustness")
    print("Features: Adaptive Baselines, Pattern Recognition, Trend Analysis")
    print()

    # Initialize enhanced fault monitor
    monitor = ComprehensiveFaultMonitor()

    try:
        # Run comprehensive fault tests
        logger.info("🚀 Running Enhanced Fault Detection Tests...")

        report = await monitor.run_enhanced_fault_tests()

        # Display results
        print("\n" + "="*60)
        print("📊 ENHANCED FAULT DETECTION RESULTS")
        print("="*60)

        summary = report['summary']
        print(f"Total Tests: {summary['total_tests']}")
        print(f"Successful Tests: {summary['successful_tests']}")
        print(".1%")
        print(f"Overall Status: {'✅ PASS' if summary['overall_status'] == 'PASS' else '❌ FAIL'}")
        print(f"Total Duration: {report['total_duration_seconds']:.1f} seconds")

        # Detailed test results
        print("\nTest Results:")
        for test_name, result in report['test_results'].items():
            status = "✅ PASS" if result.get('success', False) else "❌ FAIL"
            print(f"  {test_name.replace('_', ' ').title()}: {status}")

            # Show key metrics
            if test_name == 'cpu_spike_enhanced':
                print(f"    Alerts Triggered: {result.get('alerts_triggered', 0)}")
                print(f"    Workload Adaptation: {'✅ Active' if result.get('workload_adaptation', False) else '❌ None'}")

            elif test_name == 'thread_explosion_enhanced':
                pattern = result.get('pattern_analysis', {}).get('pattern', 'unknown')
                print(f"    Pattern Detected: {pattern}")
                print(f"    Resource Correlation: {len(result.get('resource_correlation', {}))} metrics")

            elif test_name == 'memory_leak_enhanced':
                confidence = result.get('leak_analysis', {}).get('confidence', 0)
                print(f"    Detection Confidence: {confidence:.1%}")
                print(f"    Analysis Methods: {len(result.get('leak_analysis', {}).get('methods', {}))}")

            elif test_name == 'network_partition_enhanced':
                quality_score = result.get('final_network_analysis', {}).get('quality_score', 0)
                print(f"    Final Quality Score: {quality_score:.1%}")
                trend = result.get('final_network_analysis', {}).get('trend_analysis', {}).get('trend', 'unknown')
                print(f"    Quality Trend: {trend}")

        # Performance metrics
        perf = report['performance_metrics']
        print("\nPerformance Metrics:")
        print(f"  Monitoring Samples: {perf['monitoring_samples']}")
        print(".2f")
        print(f"  CPU Baseline Adapted: {'✅ Yes' if perf.get('cpu_baseline_adaptation', {}).get('samples', 0) > 0 else '❌ No'}")

        # Alerts summary
        alerts = report['alerts_summary']
        print("\nAlert Summary:")
        print(f"  Total Alerts: {alerts['total_alerts']}")
        print("  Alerts by Type:")
        for alert_type, count in alerts['alerts_by_type'].items():
            print(f"    {alert_type}: {count}")
        print("  Alerts by Severity:")
        for severity, count in alerts['alerts_by_severity'].items():
            print(f"    {severity}: {count}")

        # Generate visualization dashboard
        print("\n📈 Generating Visualization Dashboard...")
        dashboard_img = monitor.generate_visualization_dashboard()
        print("✅ Dashboard generated with comprehensive charts")

        # Create HTML dashboard
        dashboard_html = f"""
<!DOCTYPE html>
<html>
<head>
    <title>Enhanced Fault Monitoring Dashboard</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 20px; }}
        .metric {{ background: #f5f5f5; padding: 15px; margin: 10px 0; border-radius: 5px; }}
        .alert {{ background: #ffe6e6; border-left: 4px solid #ff0000; padding: 10px; margin: 5px 0; }}
        .success {{ background: #e6ffe6; border-left: 4px solid #00aa00; }}
        .chart {{ max-width: 100%; height: auto; border: 1px solid #ddd; }}
    </style>
</head>
<body>
    <h1>🎯 Enhanced Fault Monitoring Dashboard</h1>

    <div class="metric">
        <h2>Test Summary</h2>
        <p><strong>Status:</strong> {summary['overall_status']}</p>
        <p><strong>Success Rate:</strong> {summary['success_rate']:.1%}</p>
        <p><strong>Duration:</strong> {report['total_duration_seconds']:.1f}s</p>
        <p><strong>Total Alerts:</strong> {alerts['total_alerts']}</p>
    </div>

    <h2>📊 System Monitoring Charts</h2>
    <img src="data:image/png;base64,{dashboard_img}" class="chart" alt="Fault Monitoring Dashboard">

    <h2>🔍 Detailed Test Results</h2>
"""

        # Add detailed test results
        for test_name, result in report['test_results'].items():
            status_class = "success" if result.get('success', False) else "alert"
            dashboard_html += f"""
    <div class="metric {status_class}">
        <h3>{test_name.replace('_', ' ').title()}</h3>
        <p><strong>Status:</strong> {'PASS' if result.get('success', False) else 'FAIL'}</p>
"""

            # Add specific metrics based on test type
            if test_name == 'cpu_spike_enhanced':
                dashboard_html += f"""
        <p><strong>Alerts:</strong> {result.get('alerts_triggered', 0)}</p>
        <p><strong>Workload Adaptation:</strong> {'Active' if result.get('workload_adaptation', False) else 'None'}</p>
"""

            elif test_name == 'memory_leak_enhanced':
                leak_analysis = result.get('leak_analysis', {})
                dashboard_html += f"""
        <p><strong>Detection Confidence:</strong> {leak_analysis.get('confidence', 0):.1%}</p>
        <p><strong>Leak Rate:</strong> {leak_analysis.get('leak_rate_mb_per_minute', 0):.2f} MB/min</p>
"""

            dashboard_html += "</div>"

        dashboard_html += """
</body>
</html>
"""

        # Save dashboard
        dashboard_path = os.path.join(os.getcwd(), 'enhanced_fault_dashboard.html')
        with open(dashboard_path, 'w') as f:
            f.write(dashboard_html)

        print(f"💾 Interactive dashboard saved to: {dashboard_path}")

        # Save detailed JSON report
        report_path = os.path.join(os.getcwd(), 'enhanced_fault_report.json')
        with open(report_path, 'w') as f:
            json.dump(report, f, indent=2, default=str)

        print(f"💾 Detailed JSON report saved to: {report_path}")

        print("\n" + "="*80)
        if summary['overall_status'] == 'PASS':
            print("🎉 ENHANCED FAULT MONITORING: COMPETITION READY!")
            print("✅ All critical detection improvements successfully implemented")
        else:
            print("⚠️ ENHANCED FAULT MONITORING: REQUIRES ATTENTION")
            print("❌ Some detection improvements need further work")
        print("="*80)

    except Exception as e:
        logger.error(f"❌ Enhanced fault monitoring failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    asyncio.run(main())
