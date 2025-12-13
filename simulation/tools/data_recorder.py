"""Data recording and playback for simulation.

Provides comprehensive data capture, storage, and playback capabilities
for simulation analysis and debugging.

Author: URC 2026 Autonomy Team
"""

import json
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np


class DataRecorder:
    """Records and plays back simulation data.

    Captures all simulation state for analysis, debugging, and replay.
    Supports multiple recording modes and data formats.
    """

    def __init__(self):
        """Initialize data recorder."""
        self.is_recording = False
        self.data: List[Dict[str, Any]] = []
        self.metadata: Dict[str, Any] = {}
        self.recording_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        # Recording configuration
        self.max_records = 10000  # Limit memory usage
        self.compress_data = True
        self.record_interval = 0.1  # 10Hz recording by default

        # Performance tracking
        self.records_per_second = 0.0
        self.last_record_time = 0.0
        self.record_count = 0

    def initialize(self, config: Dict[str, Any]):
        """Initialize recorder with configuration.

        Args:
            config: Recording configuration
        """
        self.max_records = config.get("max_records", 10000)
        self.compress_data = config.get("compress_data", True)
        self.record_interval = config.get("record_interval", 0.1)

        # Initialize metadata
        self.metadata = {
            "created_at": time.time(),
            "config": config,
            "version": "1.0.0",
            "data_format": "simulation_v1",
        }

    def start_recording(self):
        """Start recording simulation data."""
        if self.is_recording:
            return

        self.is_recording = True
        self.data.clear()
        self.record_count = 0
        self.last_record_time = time.time()

        # Start background recording thread if needed
        # For now, recording is done synchronously via record() calls

    def stop_recording(self):
        """Stop recording simulation data."""
        self.is_recording = False

    def record(self, simulation_state: Dict[str, Any]) -> bool:
        """Record a simulation state snapshot.

        Args:
            simulation_state: Current simulation state

        Returns:
            bool: True if recorded successfully
        """
        if not self.is_recording:
            return False

        current_time = time.time()

        # Check recording interval
        if current_time - self.last_record_time < self.record_interval:
            return False

        with self.lock:
            # Create record
            record = {
                "timestamp": current_time,
                "simulation_time": simulation_state.get("timestamp", 0.0),
                "step_count": simulation_state.get("step_count", 0),
                "data": simulation_state,
            }

            # Compress data if enabled
            if self.compress_data:
                record["data"] = self._compress_record(record["data"])

            # Add to data list
            self.data.append(record)

            # Maintain size limit
            if len(self.data) > self.max_records:
                # Remove oldest records (keep most recent)
                remove_count = len(self.data) - self.max_records
                self.data = self.data[remove_count:]

            # Update performance tracking
            self.record_count += 1
            self.last_record_time = current_time

            # Calculate records per second
            if self.record_count > 1:
                time_span = current_time - self.metadata["created_at"]
                if time_span > 0:
                    self.records_per_second = self.record_count / time_span

            return True

    def save_to_file(self, filepath: str, format: str = "json") -> bool:
        """Save recorded data to file.

        Args:
            filepath: Path to save file
            format: File format ("json", "csv", "binary")

        Returns:
            bool: True if saved successfully
        """
        try:
            path = Path(filepath)
            path.parent.mkdir(parents=True, exist_ok=True)

            if format == "json":
                self._save_json(path)
            elif format == "csv":
                self._save_csv(path)
            elif format == "binary":
                self._save_binary(path)
            else:
                raise ValueError(f"Unsupported format: {format}")

            return True

        except Exception as e:
            print(f"Failed to save data: {e}")
            return False

    def load_from_file(self, filepath: str) -> bool:
        """Load recorded data from file.

        Args:
            filepath: Path to data file

        Returns:
            bool: True if loaded successfully
        """
        try:
            path = Path(filepath)

            if path.suffix == ".json":
                self._load_json(path)
            elif path.suffix == ".csv":
                self._load_csv(path)
            elif path.suffix == ".bin":
                self._load_binary(path)
            else:
                # Try JSON as default
                self._load_json(path)

            return True

        except Exception as e:
            print(f"Failed to load data: {e}")
            return False

    def get_statistics(self) -> Dict[str, Any]:
        """Get recording statistics.

        Returns:
            Dict with recording statistics
        """
        if not self.data:
            return {"status": "no_data"}

        timestamps = [record["timestamp"] for record in self.data]

        return {
            "total_records": len(self.data),
            "recording_duration": max(timestamps) - min(timestamps) if timestamps else 0,
            "records_per_second": self.records_per_second,
            "data_size_mb": self._estimate_data_size() / (1024 * 1024),
            "memory_usage_mb": len(self.data) * 0.001,  # Rough estimate
            "compression_enabled": self.compress_data,
            "max_records_limit": self.max_records,
        }

    def clear(self):
        """Clear all recorded data."""
        with self.lock:
            self.data.clear()
            self.record_count = 0

    def get_record_at_time(self, simulation_time: float) -> Optional[Dict[str, Any]]:
        """Get record closest to specified simulation time.

        Args:
            simulation_time: Target simulation time

        Returns:
            Dict with closest record or None
        """
        if not self.data:
            return None

        # Find closest record
        closest_record = min(
            self.data,
            key=lambda r: abs(r["simulation_time"] - simulation_time)
        )

        return closest_record

    def playback(self, callback: callable, start_time: float = 0.0, end_time: Optional[float] = None):
        """Playback recorded data.

        Args:
            callback: Function to call for each record
            start_time: Start simulation time
            end_time: End simulation time (None for all)
        """
        for record in self.data:
            sim_time = record["simulation_time"]

            if sim_time < start_time:
                continue

            if end_time is not None and sim_time > end_time:
                break

            callback(record)

    def _compress_record(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Compress record data for storage efficiency.

        Args:
            data: Original record data

        Returns:
            Dict with compressed data
        """
        compressed: Dict[str, Any] = {}

        # Compress sensor data by rounding floating point values
        if "sensors" in data:
            compressed["sensors"] = {}
            for sensor_name, sensor_data in data["sensors"].items():
                compressed["sensors"][sensor_name] = self._compress_sensor_data(sensor_data)

        # Copy other data as-is
        for key, value in data.items():
            if key != "sensors":
                compressed[key] = value

        return compressed

    def _compress_sensor_data(self, sensor_data: Dict[str, Any]) -> Dict[str, Any]:
        """Compress individual sensor data.

        Args:
            sensor_data: Sensor data to compress

        Returns:
            Dict with compressed sensor data
        """
        compressed = {}

        for key, value in sensor_data.items():
            if isinstance(value, float):
                # Round to 4 decimal places for sensors
                compressed[key] = round(value, 4)
            elif isinstance(value, list) and all(isinstance(v, float) for v in value):
                # Round list of floats
                compressed[key] = [round(v, 4) for v in value]
            else:
                compressed[key] = value

        return compressed

    def _save_json(self, path: Path):
        """Save data in JSON format."""
        data = {
            "metadata": self.metadata,
            "statistics": self.get_statistics(),
            "records": self.data,
        }

        with open(path, "w") as f:
            json.dump(data, f, indent=2, default=str)

    def _save_csv(self, path: Path):
        """Save data in CSV format."""
        import csv

        if not self.data:
            return

        # Get all possible keys
        all_keys: set[str] = set()
        for record in self.data:
            all_keys.update(record.keys())
            if "data" in record:
                all_keys.update(record["data"].keys())

        # Write CSV
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=sorted(all_keys))
            writer.writeheader()

            for record in self.data:
                row: Dict[str, Any] = {}
                # Flatten nested data
                self._flatten_dict(record, row)
                writer.writerow(row)

    def _save_binary(self, path: Path):
        """Save data in binary format."""
        import pickle

        data = {
            "metadata": self.metadata,
            "statistics": self.get_statistics(),
            "records": self.data,
        }

        with open(path, "wb") as f:
            pickle.dump(data, f)

    def _load_json(self, path: Path):
        """Load data from JSON format."""
        with open(path, "r") as f:
            loaded = json.load(f)

        self.metadata = loaded.get("metadata", {})
        self.data = loaded.get("records", [])

    def _load_csv(self, path: Path):
        """Load data from CSV format."""
        import csv

        self.data = []
        with open(path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.data.append(row)

    def _load_binary(self, path: Path):
        """Load data from binary format."""
        import pickle

        with open(path, "rb") as f:
            loaded = pickle.load(f)

        self.metadata = loaded.get("metadata", {})
        self.data = loaded.get("records", [])

    def _flatten_dict(self, d: Dict[str, Any], result: Dict[str, Any], prefix: str = ""):
        """Flatten nested dictionary for CSV export.

        Args:
            d: Dictionary to flatten
            result: Result dictionary
            prefix: Key prefix for nested keys
        """
        for key, value in d.items():
            new_key = f"{prefix}{key}" if prefix else key

            if isinstance(value, dict):
                self._flatten_dict(value, result, f"{new_key}.")
            elif isinstance(value, list):
                result[new_key] = str(value)  # Convert lists to strings
            else:
                result[new_key] = value

    def _estimate_data_size(self) -> int:
        """Estimate data size in bytes.

        Returns:
            int: Estimated size in bytes
        """
        # Rough estimation based on record count and average record size
        avg_record_size = 1024  # Assume 1KB per record
        return len(self.data) * avg_record_size
