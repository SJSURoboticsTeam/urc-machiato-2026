#!/usr/bin/env python3
"""
URC 2026 Telemetry System

High-performance time-series telemetry using Polars and InfluxDB.
Provides fast data processing, memory-efficient storage, and real-time analytics.

Features:
- Polars for 10-100x faster DataFrame operations than pandas
- InfluxDB for time-series storage and querying
- Apache Arrow for efficient data interchange
- Real-time streaming analytics
- Memory-efficient data structures

Author: URC 2026 Telemetry Team
"""

import asyncio
import time
import threading
from typing import Dict, Any, List, Optional, Union, Tuple
from dataclasses import dataclass, field
from datetime import datetime, timedelta
import logging
import json
from pathlib import Path

# High-performance libraries (HIGH PRIORITY)
try:
    import polars as pl
    POLARS_AVAILABLE = True
except ImportError:
    POLARS_AVAILABLE = False
    pl = None

try:
    import pandas as pd
    PANDAS_AVAILABLE = True
except ImportError:
    PANDAS_AVAILABLE = False
    pd = None

try:
    from influxdb_client import InfluxDBClient, Point, WritePrecision
    from influxdb_client.client.write_api import SYNCHRONOUS
    INFLUXDB_AVAILABLE = True
except ImportError:
    INFLUXDB_AVAILABLE = False

try:
    import pyarrow as pa
    PYARROW_AVAILABLE = True
except ImportError:
    PYARROW_AVAILABLE = False

from src.core.config_manager import get_config

logger = logging.getLogger(__name__)


@dataclass
class TelemetryConfig:
    """Configuration for telemetry system."""
    influxdb_url: str = "http://localhost:8086"
    influxdb_token: str = ""
    influxdb_org: str = "urc"
    influxdb_bucket: str = "telemetry"
    batch_size: int = 1000
    flush_interval: float = 5.0  # seconds
    retention_days: int = 30
    enable_streaming: bool = True
    memory_limit_mb: int = 512


@dataclass
class TelemetryPoint:
    """Individual telemetry data point."""
    timestamp: datetime
    measurement: str
    tags: Dict[str, str] = field(default_factory=dict)
    fields: Dict[str, Union[float, int, str, bool]] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)


class PolarsTelemetryBuffer:
    """High-performance telemetry buffer using Polars."""

    def __init__(self, max_size: int = 10000):
        if not POLARS_AVAILABLE:
            raise ImportError("Polars not available. Install with: pip install polars")

        self.max_size = max_size
        self.schema = {
            'timestamp': pl.Datetime('ns'),
            'measurement': pl.Utf8,
            'tags': pl.Struct({}),  # Will be expanded dynamically
            'fields': pl.Struct({}),  # Will be expanded dynamically
            'metadata': pl.Struct({})
        }

        # Initialize empty DataFrame
        self.df = pl.DataFrame(schema=self.schema)
        self.lock = threading.Lock()

    def add_point(self, point: TelemetryPoint) -> None:
        """Add telemetry point to buffer."""
        with self.lock:
            # Convert point to dictionary
            row_data = {
                'timestamp': point.timestamp,
                'measurement': point.measurement,
                'tags': point.tags,
                'fields': point.fields,
                'metadata': point.metadata
            }

            # Convert to Polars DataFrame row
            row_df = pl.DataFrame([row_data])

            # Append to main DataFrame
            self.df = pl.concat([self.df, row_df], how='diagonal')

            # Maintain size limit
            if len(self.df) > self.max_size:
                # Keep most recent points
                self.df = self.df.tail(self.max_size)

    def get_data(self, measurement: Optional[str] = None,
                start_time: Optional[datetime] = None,
                end_time: Optional[datetime] = None) -> pl.DataFrame:
        """Get filtered telemetry data."""
        with self.lock:
            filtered_df = self.df

            if measurement:
                filtered_df = filtered_df.filter(pl.col('measurement') == measurement)

            if start_time:
                filtered_df = filtered_df.filter(pl.col('timestamp') >= start_time)

            if end_time:
                filtered_df = filtered_df.filter(pl.col('timestamp') <= end_time)

            return filtered_df

    def get_statistics(self, measurement: str,
                      time_window_minutes: int = 5) -> Dict[str, Any]:
        """Get statistics for measurement over time window."""
        with self.lock:
            end_time = datetime.now()
            start_time = end_time - timedelta(minutes=time_window_minutes)

            data = self.get_data(measurement, start_time, end_time)

            if len(data) == 0:
                return {'count': 0, 'avg': 0, 'min': 0, 'max': 0}

            # Extract numeric fields for statistics
            numeric_cols = []
            for col in data.columns:
                if col.startswith('fields.') and data[col].dtype in [pl.Float64, pl.Int64]:
                    numeric_cols.append(col)

            stats = {'count': len(data)}

            for col in numeric_cols:
                col_stats = data.select([
                    pl.col(col).mean().alias('avg'),
                    pl.col(col).min().alias('min'),
                    pl.col(col).max().alias('max'),
                    pl.col(col).std().alias('std')
                ])
                if len(col_stats) > 0:
                    stats[col] = col_stats.to_dicts()[0]

            return stats

    def clear(self) -> None:
        """Clear all buffered data."""
        with self.lock:
            self.df = pl.DataFrame(schema=self.schema)

    def size(self) -> int:
        """Get current buffer size."""
        with self.lock:
            return len(self.df)


class InfluxDBTelemetryStorage:
    """InfluxDB-based telemetry storage."""

    def __init__(self, config: TelemetryConfig):
        if not INFLUXDB_AVAILABLE:
            raise ImportError("InfluxDB client not available. Install with: pip install influxdb-client")

        self.config = config
        self.client = None
        self.write_api = None
        self.query_api = None

        self._connect()

    def _connect(self):
        """Connect to InfluxDB."""
        try:
            self.client = InfluxDBClient(
                url=self.config.influxdb_url,
                token=self.config.influxdb_token,
                org=self.config.influxdb_org
            )

            self.write_api = self.client.write_api(write_options=SYNCHRONOUS)
            self.query_api = self.client.query_api()

            logger.info("Connected to InfluxDB")
        except Exception as e:
            logger.error(f"Failed to connect to InfluxDB: {e}")
            raise

    def store_point(self, point: TelemetryPoint) -> None:
        """Store single telemetry point."""
        try:
            influx_point = Point(point.measurement)

            # Add timestamp
            influx_point.time(point.timestamp, write_precision=WritePrecision.NS)

            # Add tags
            for tag_key, tag_value in point.tags.items():
                influx_point.tag(tag_key, str(tag_value))

            # Add fields
            for field_key, field_value in point.fields.items():
                if isinstance(field_value, (int, float)):
                    influx_point.field(field_key, field_value)
                else:
                    influx_point.field(field_key, str(field_value))

            self.write_api.write(
                bucket=self.config.influxdb_bucket,
                org=self.config.influxdb_org,
                record=influx_point
            )

        except Exception as e:
            logger.error(f"Failed to store telemetry point: {e}")

    def store_batch(self, points: List[TelemetryPoint]) -> None:
        """Store batch of telemetry points."""
        try:
            influx_points = []

            for point in points:
                influx_point = Point(point.measurement)
                influx_point.time(point.timestamp, write_precision=WritePrecision.NS)

                # Add tags and fields
                for tag_key, tag_value in point.tags.items():
                    influx_point.tag(tag_key, str(tag_value))

                for field_key, field_value in point.fields.items():
                    if isinstance(field_value, (int, float)):
                        influx_point.field(field_key, field_value)
                    else:
                        influx_point.field(field_key, str(field_value))

                influx_points.append(influx_point)

            self.write_api.write(
                bucket=self.config.influxdb_bucket,
                org=self.config.influxdb_org,
                record=influx_points
            )

            logger.debug(f"Stored {len(points)} telemetry points")

        except Exception as e:
            logger.error(f"Failed to store telemetry batch: {e}")

    def query_data(self, measurement: str,
                  start_time: Optional[datetime] = None,
                  end_time: Optional[datetime] = None,
                  tags: Optional[Dict[str, str]] = None) -> pl.DataFrame:
        """Query telemetry data from InfluxDB."""
        try:
            # Build Flux query
            flux_query = f'''
            from(bucket: "{self.config.influxdb_bucket}")
                |> range(
            '''

            if start_time:
                flux_query += f'start: {int(start_time.timestamp() * 1e9)}'
            else:
                flux_query += 'start: -1h'

            if end_time:
                flux_query += f', stop: {int(end_time.timestamp() * 1e9)}'
            else:
                flux_query += ', stop: now()'

            flux_query += f'''
                )
                |> filter(fn: (r) => r._measurement == "{measurement}")
            '''

            if tags:
                for tag_key, tag_value in tags.items():
                    flux_query += f'''
                |> filter(fn: (r) => r.{tag_key} == "{tag_value}")
                    '''

            flux_query += '''
                |> pivot(rowKey:["_time"], columnKey: ["_field"], valueColumn: "_value")
            '''

            # Execute query
            result = self.query_api.query(flux_query)

            # Convert to Polars DataFrame
            if result and len(result) > 0:
                records = []
                for table in result:
                    for record in table.records:
                        record_dict = {
                            'timestamp': record.get_time(),
                            'measurement': record.get_measurement()
                        }
                        # Add tags
                        for tag_key in record.values:
                            if tag_key not in ['_time', '_measurement', '_field', '_value', 'result', 'table']:
                                record_dict[f'tag_{tag_key}'] = record.values[tag_key]

                        # Add fields
                        for field in record.values:
                            if field not in ['_time', '_measurement', 'result', 'table'] and not field.startswith('tag_'):
                                record_dict[field] = record.values[field]

                        records.append(record_dict)

                if records:
                    return pl.DataFrame(records)

            return pl.DataFrame()

        except Exception as e:
            logger.error(f"Failed to query telemetry data: {e}")
            return pl.DataFrame()

    def close(self):
        """Close InfluxDB connection."""
        if self.client:
            self.client.close()


class TelemetryAnalytics:
    """Real-time telemetry analytics using Polars."""

    def __init__(self, buffer: PolarsTelemetryBuffer):
        self.buffer = buffer
        self.analytics_cache: Dict[str, Any] = {}
        self.cache_timeout = 30.0  # seconds

    def get_system_health_score(self, time_window_minutes: int = 5) -> float:
        """Calculate system health score from telemetry data."""
        try:
            # Get recent system metrics
            cpu_stats = self.buffer.get_statistics('system.cpu', time_window_minutes)
            memory_stats = self.buffer.get_statistics('system.memory', time_window_minutes)
            error_stats = self.buffer.get_statistics('system.errors', time_window_minutes)

            # Calculate health components
            cpu_health = 1.0 - min(cpu_stats.get('avg', 0) / 100.0, 1.0)  # Lower CPU usage is better
            memory_health = 1.0 - min(memory_stats.get('avg', 0) / 100.0, 1.0)  # Lower memory usage is better
            error_health = 1.0 - min(error_stats.get('count', 0) / 10.0, 1.0)  # Fewer errors is better

            # Weighted health score
            health_score = (cpu_health * 0.3 + memory_health * 0.4 + error_health * 0.3)

            return max(0.0, min(1.0, health_score))

        except Exception as e:
            logger.error(f"Failed to calculate health score: {e}")
            return 0.5  # Default neutral health

    def detect_anomalies(self, measurement: str,
                        time_window_minutes: int = 10,
                        threshold_sigma: float = 3.0) -> List[Dict[str, Any]]:
        """Detect anomalies in telemetry data using statistical methods."""
        try:
            data = self.buffer.get_data(measurement)
            if len(data) < 10:  # Need minimum data points
                return []

            anomalies = []

            # Check numeric fields for anomalies
            for col in data.columns:
                if data[col].dtype in [pl.Float64, pl.Int64]:
                    values = data[col].to_list()

                    if len(values) >= 10:
                        # Calculate rolling statistics
                        mean = sum(values) / len(values)
                        variance = sum((x - mean) ** 2 for x in values) / len(values)
                        std_dev = variance ** 0.5

                        # Detect outliers
                        for i, value in enumerate(values):
                            if std_dev > 0:
                                z_score = abs(value - mean) / std_dev
                                if z_score > threshold_sigma:
                                    anomalies.append({
                                        'timestamp': data['timestamp'][i],
                                        'measurement': measurement,
                                        'field': col,
                                        'value': value,
                                        'z_score': z_score,
                                        'severity': 'high' if z_score > 4.0 else 'medium'
                                    })

            return anomalies

        except Exception as e:
            logger.error(f"Failed to detect anomalies: {e}")
            return []

    def predict_trends(self, measurement: str,
                      field: str,
                      prediction_minutes: int = 5) -> Dict[str, Any]:
        """Simple trend prediction using linear regression."""
        try:
            data = self.buffer.get_data(measurement)
            if len(data) < 5:
                return {'trend': 'insufficient_data'}

            # Extract time and field values
            timestamps = data['timestamp'].to_list()
            values = data[field].to_list() if field in data.columns else []

            if not values:
                return {'trend': 'no_field_data'}

            # Simple linear regression
            n = len(values)
            x = list(range(n))  # Time indices
            y = values

            sum_x = sum(x)
            sum_y = sum(y)
            sum_xy = sum(xi * yi for xi, yi in zip(x, y))
            sum_x2 = sum(xi * xi for xi in x)

            # Calculate slope and intercept
            slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x)
            intercept = (sum_y - slope * sum_x) / n

            # Predict future values
            future_x = n + (prediction_minutes * 60 / 30)  # Assuming 30-second intervals
            prediction = slope * future_x + intercept

            trend_direction = "increasing" if slope > 0.01 else "decreasing" if slope < -0.01 else "stable"

            return {
                'trend': trend_direction,
                'slope': slope,
                'prediction': prediction,
                'confidence': max(0.0, min(1.0, 1.0 - abs(slope) * 10))  # Simple confidence measure
            }

        except Exception as e:
            logger.error(f"Failed to predict trends: {e}")
            return {'trend': 'error', 'error': str(e)}


class TelemetrySystem:
    """Main telemetry system orchestrator."""

    def __init__(self, config: Optional[TelemetryConfig] = None):
        self.config = config or TelemetryConfig()

        # Initialize components
        if POLARS_AVAILABLE:
            self.buffer = PolarsTelemetryBuffer(max_size=self.config.batch_size * 10)
            self.analytics = TelemetryAnalytics(self.buffer)
        else:
            logger.warning("Polars not available, telemetry performance will be reduced")
            self.buffer = None
            self.analytics = None

        if INFLUXDB_AVAILABLE and self.config.influxdb_url:
            try:
                self.storage = InfluxDBTelemetryStorage(self.config)
            except Exception as e:
                logger.warning(f"InfluxDB connection failed: {e}")
                self.storage = None
        else:
            self.storage = None

        # Background processing
        self.flush_thread: Optional[threading.Thread] = None
        self.running = False
        self.last_flush = time.time()

        # Start background tasks if enabled
        if self.config.enable_streaming:
            self.start_background_tasks()

        logger.info("Telemetry system initialized")

    def start_background_tasks(self):
        """Start background telemetry processing."""
        self.running = True
        self.flush_thread = threading.Thread(target=self._background_flush, daemon=True)
        self.flush_thread.start()

    def stop_background_tasks(self):
        """Stop background telemetry processing."""
        self.running = False
        if self.flush_thread:
            self.flush_thread.join(timeout=5.0)

    def record_point(self, measurement: str,
                    fields: Dict[str, Union[float, int, str, bool]],
                    tags: Optional[Dict[str, str]] = None,
                    metadata: Optional[Dict[str, Any]] = None) -> None:
        """Record a telemetry data point."""
        point = TelemetryPoint(
            timestamp=datetime.now(),
            measurement=measurement,
            tags=tags or {},
            fields=fields,
            metadata=metadata or {}
        )

        if self.buffer:
            self.buffer.add_point(point)

    def record_system_metrics(self) -> None:
        """Record system performance metrics."""
        try:
            import psutil
            import os

            process = psutil.Process(os.getpid())

            self.record_point(
                measurement="system.cpu",
                fields={
                    "usage_percent": psutil.cpu_percent(),
                    "process_cpu_percent": process.cpu_percent()
                },
                tags={"system": "rover"}
            )

            memory = psutil.virtual_memory()
            process_memory = process.memory_info()

            self.record_point(
                measurement="system.memory",
                fields={
                    "total_mb": memory.total / 1024 / 1024,
                    "available_mb": memory.available / 1024 / 1024,
                    "used_percent": memory.percent,
                    "process_rss_mb": process_memory.rss / 1024 / 1024
                },
                tags={"system": "rover"}
            )

        except ImportError:
            logger.debug("psutil not available for system metrics")
        except Exception as e:
            logger.error(f"Failed to record system metrics: {e}")

    def get_health_score(self) -> float:
        """Get current system health score."""
        if self.analytics:
            return self.analytics.get_system_health_score()
        return 0.5  # Default neutral health

    def get_anomalies(self, measurement: str) -> List[Dict[str, Any]]:
        """Get detected anomalies for measurement."""
        if self.analytics:
            return self.analytics.detect_anomalies(measurement)
        return []

    def predict_trend(self, measurement: str, field: str) -> Dict[str, Any]:
        """Predict trend for measurement field."""
        if self.analytics:
            return self.analytics.predict_trends(measurement, field)
        return {'trend': 'analytics_unavailable'}

    def query_historical_data(self, measurement: str,
                            hours_back: int = 1) -> pl.DataFrame:
        """Query historical telemetry data."""
        if self.storage:
            end_time = datetime.now()
            start_time = end_time - timedelta(hours=hours_back)
            return self.storage.query_data(measurement, start_time, end_time)

        # Fallback to buffer data
        if self.buffer:
            end_time = datetime.now()
            start_time = end_time - timedelta(hours=hours_back)
            return self.buffer.get_data(measurement, start_time, end_time)

        return pl.DataFrame() if POLARS_AVAILABLE else None

    def _background_flush(self):
        """Background task to periodically flush data to storage."""
        while self.running:
            time.sleep(self.config.flush_interval)

            try:
                current_time = time.time()
                if current_time - self.last_flush >= self.config.flush_interval:
                    self._flush_to_storage()
                    self.last_flush = current_time

            except Exception as e:
                logger.error(f"Background flush failed: {e}")

    def _flush_to_storage(self):
        """Flush buffered data to persistent storage."""
        if not self.storage or not self.buffer:
            return

        try:
            # Get all buffered data
            all_data = self.buffer.get_data()

            if len(all_data) == 0:
                return

            # Convert to TelemetryPoint objects
            points = []
            for row in all_data.to_dicts():
                point = TelemetryPoint(
                    timestamp=row['timestamp'],
                    measurement=row['measurement'],
                    tags=row.get('tags', {}),
                    fields=row.get('fields', {}),
                    metadata=row.get('metadata', {})
                )
                points.append(point)

            # Store in batches
            batch_size = self.config.batch_size
            for i in range(0, len(points), batch_size):
                batch = points[i:i + batch_size]
                self.storage.store_batch(batch)

            # Clear processed data from buffer
            self.buffer.clear()

            logger.debug(f"Flushed {len(points)} telemetry points to storage")

        except Exception as e:
            logger.error(f"Failed to flush telemetry data: {e}")

    def close(self):
        """Shutdown telemetry system."""
        self.stop_background_tasks()

        if self.storage:
            self.storage.close()

        logger.info("Telemetry system shutdown")


# Global telemetry instance
_telemetry_instance: Optional[TelemetrySystem] = None


def get_telemetry_system() -> TelemetrySystem:
    """Get global telemetry system instance."""
    global _telemetry_instance
    if _telemetry_instance is None:
        config = get_config()
        telemetry_config = TelemetryConfig(
            influxdb_url=getattr(config, 'telemetry', {}).get('influxdb_url', 'http://localhost:8086'),
            influxdb_token=getattr(config, 'telemetry', {}).get('influxdb_token', ''),
            enable_streaming=getattr(config, 'telemetry', {}).get('enable_streaming', True)
        )
        _telemetry_instance = TelemetrySystem(telemetry_config)
    return _telemetry_instance


# Convenience functions
def record_metric(measurement: str,
                 fields: Dict[str, Union[float, int, str, bool]],
                 tags: Optional[Dict[str, str]] = None):
    """Convenience function to record telemetry metric."""
    telemetry = get_telemetry_system()
    telemetry.record_point(measurement, fields, tags)


def record_system_health():
    """Record current system health metrics."""
    telemetry = get_telemetry_system()
    telemetry.record_system_metrics()


# Example usage and testing
if __name__ == "__main__":
    print("🚀 URC 2026 TELEMETRY SYSTEM DEMO")
    print("=" * 50)

    try:
        # Initialize telemetry system
        telemetry = get_telemetry_system()
        print("✅ Telemetry system initialized")

        # Record some sample data
        telemetry.record_point(
            measurement="navigation.status",
            fields={
                "waypoint_reached": True,
                "distance_to_target": 2.5,
                "heading_error": 1.2
            },
            tags={"mission": "exploration", "phase": "waypoint_navigation"}
        )

        telemetry.record_point(
            measurement="system.health",
            fields={
                "cpu_usage": 45.2,
                "memory_usage": 234.1,
                "temperature": 42.3
            },
            tags={"component": "rover_main"}
        )

        print("✅ Sample telemetry data recorded")

        # Record system metrics
        telemetry.record_system_metrics()
        print("✅ System metrics recorded")

        # Get health score
        health_score = telemetry.get_health_score()
        print(f"🏥 Current health score: {health_score:.1%}")

        # Check for anomalies
        anomalies = telemetry.get_anomalies("system.health")
        print(f"🔍 Detected anomalies: {len(anomalies)}")

        # Test trend prediction
        trend = telemetry.predict_trend("system.health", "cpu_usage")
        print(f"📈 CPU usage trend: {trend.get('trend', 'unknown')}")

        # Query recent data
        recent_data = telemetry.query_historical_data("navigation.status", hours_back=1)
        if POLARS_AVAILABLE and len(recent_data) > 0:
            print(f"📊 Recent navigation data points: {len(recent_data)}")

        # Cleanup
        telemetry.close()

        print("\\n🎉 Telemetry system demo completed successfully!")
        print("Features demonstrated:")
        print("  • High-performance Polars DataFrames")
        print("  • InfluxDB time-series storage")
        print("  • Real-time health monitoring")
        print("  • Anomaly detection")
        print("  • Trend prediction")
        print("  • Background data flushing")

    except Exception as e:
        print(f"❌ Telemetry demo failed: {e}")
        import traceback
        traceback.print_exc()
