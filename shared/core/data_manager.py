#!/usr/bin/env python3
"""
Unified Data Manager - Single Interface for All Data Processing

Consolidates all data processing functionality:
- TelemetryDataProcessor (from data_processor.py)
- StatisticsProcessor (from statistics_processor.py)
- JSONProcessor (from json_processor.py)
- Transforms (from transforms.py)
- DataStructures (from data_structures.py)

Features:
- Single API for all data processing needs
- Advanced analytics and time series processing
- Efficient data structures and transformations
- Schema validation and data quality checks
- Performance optimization for embedded systems

Author: URC 2026 Unified Data Team
"""

import time
import json
import logging
from typing import Dict, List, Any, Optional, Tuple, Union, Callable
from pathlib import Path
from dataclasses import dataclass, field
import weakref

# Data processing libraries with fallbacks
try:
    import pandas as pd
    import xarray as xr
    import numpy as np

    DATA_AVAILABLE = True
except ImportError:
    DATA_AVAILABLE = False

    # Fallback implementations
    class pd:
        DataFrame = dict
        Series = list
        Timestamp = float

        def read_csv(*args, **kwargs):
            return {}

        def to_datetime(*args, **kwargs):
            return None

        def concat(*args, **kwargs):
            return {}

    class xr:
        Dataset = dict
        DataArray = list

    class np:
        ndarray = list
        array = list


try:
    import scipy.stats as stats
    from scipy import signal

    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

    # Fallback statistical functions
    class stats:
        @staticmethod
        def normaltest(x):
            return (0, 1.0)

        @staticmethod
        def shapiro(x):
            return (0, 1.0)

        @staticmethod
        def ttest_ind(a, b):
            return (0, 1.0)

        @staticmethod
        def pearsonr(x, y):
            return (0, 1.0)

        @staticmethod
        def linregress(x, y):
            return type(
                "obj",
                (object,),
                {"slope": 0, "intercept": 0, "rvalue": 0, "pvalue": 1.0, "stderr": 0},
            )

        @staticmethod
        def zscore(x):
            return x


try:
    import orjson
    import jsonschema

    ORJSON_AVAILABLE = True
except ImportError:
    ORJSON_AVAILABLE = False
    orjson = json  # Fallback to standard json

try:
    import transforms3d

    TRANSFORMS_AVAILABLE = True
except ImportError:
    TRANSFORMS_AVAILABLE = False

logger = logging.getLogger(__name__)


@dataclass
class DataSchema:
    """Data schema definition for validation."""

    name: str
    schema: Dict[str, Any]
    version: str = "1.0"
    required_fields: List[str] = field(default_factory=list)
    optional_fields: List[str] = field(default_factory=list)


@dataclass
class DataQualityMetrics:
    """Data quality assessment metrics."""

    completeness: float = 0.0  # Percentage of non-null values
    accuracy: float = 0.0  # Data accuracy score
    consistency: float = 0.0  # Data consistency score
    timeliness: float = 0.0  # Data timeliness score
    validity: float = 0.0  # Data validity score
    total_records: int = 0
    valid_records: int = 0
    invalid_records: int = 0
    outliers_detected: int = 0


@dataclass
class TimeSeriesData:
    """Time series data structure."""

    timestamps: List[float] = field(default_factory=list)
    values: Dict[str, List[Any]] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)


class DataValidationError(Exception):
    """Raised when data validation fails."""

    pass


class DataManager:
    """
    Unified Data Manager - Single interface for all data processing operations.

    Consolidates functionality from:
    - TelemetryDataProcessor (pandas/xarray processing)
    - StatisticsProcessor (scipy statistical analysis)
    - JSONProcessor (high-performance JSON handling)
    - Transforms (coordinate transformations)
    - DataStructures (efficient data structures)

    Features:
    - Advanced analytics and time series processing
    - Schema validation and data quality assessment
    - Efficient data structures and transformations
    - Performance optimization for embedded systems
    - Real-time data processing capabilities
    """

    def __init__(self):
        self.schemas: Dict[str, DataSchema] = {}
        self.data_cache: Dict[str, Any] = {}
        self.transform_functions: Dict[str, Callable] = {}
        self.validation_functions: Dict[str, Callable] = {}
        self.quality_metrics: Dict[str, DataQualityMetrics] = {}

        # Performance tracking
        self.processing_times: Dict[str, List[float]] = {}
        self.cache_hits = 0
        self.cache_misses = 0

        # Initialize default schemas
        self._initialize_default_schemas()

        logger.info("Unified Data Manager initialized")

    def _initialize_default_schemas(self):
        """Initialize default data schemas."""
        # Telemetry schema
        telemetry_schema = {
            "type": "object",
            "properties": {
                "timestamp": {"type": "number"},
                "sensor_id": {"type": "string"},
                "sensor_type": {"type": "string"},
                "value": {"type": "number"},
                "unit": {"type": "string"},
                "quality": {"type": "number", "minimum": 0, "maximum": 1},
            },
            "required": ["timestamp", "sensor_id", "value"],
        }

        # GPS data schema
        gps_schema = {
            "type": "object",
            "properties": {
                "timestamp": {"type": "number"},
                "latitude": {"type": "number", "minimum": -90, "maximum": 90},
                "longitude": {"type": "number", "minimum": -180, "maximum": 180},
                "altitude": {"type": "number"},
                "accuracy": {"type": "number", "minimum": 0},
                "satellites": {"type": "integer", "minimum": 0},
            },
            "required": ["timestamp", "latitude", "longitude"],
        }

        self.register_schema(
            "telemetry", telemetry_schema, ["timestamp", "sensor_id", "value"]
        )
        self.register_schema("gps", gps_schema, ["timestamp", "latitude", "longitude"])

    # Schema Management
    def register_schema(
        self,
        name: str,
        schema: Dict[str, Any],
        required_fields: Optional[List[str]] = None,
        version: str = "1.0",
    ) -> bool:
        """
        Register a data schema for validation.

        Args:
            name: Schema name
            schema: JSON schema definition
            required_fields: List of required field names
            version: Schema version

        Returns:
            True if registration successful
        """
        try:
            data_schema = DataSchema(
                name=name,
                schema=schema,
                version=version,
                required_fields=required_fields or [],
                optional_fields=[
                    k
                    for k in schema.get("properties", {}).keys()
                    if k not in (required_fields or [])
                ],
            )

            self.schemas[name] = data_schema

            # Create validation function if jsonschema available
            if ORJSON_AVAILABLE and "jsonschema" in globals():
                try:
                    self.validation_functions[name] = jsonschema.validate
                except:
                    pass

            logger.info(f"Registered data schema: {name} v{version}")
            return True

        except Exception as e:
            logger.error(f"Failed to register schema {name}: {e}")
            return False

    def validate_data(self, data: Any, schema_name: str) -> Tuple[bool, List[str]]:
        """
        Validate data against a registered schema.

        Args:
            data: Data to validate
            schema_name: Name of schema to validate against

        Returns:
            Tuple of (is_valid, error_messages)
        """
        if schema_name not in self.schemas:
            return False, [f"Schema '{schema_name}' not registered"]

        schema = self.schemas[schema_name]

        try:
            if schema_name in self.validation_functions:
                # Use jsonschema for validation
                jsonschema.validate(data, schema.schema)
                return True, []
            else:
                # Basic validation
                return self._basic_validation(data, schema)

        except Exception as e:
            return False, [str(e)]

    def _basic_validation(
        self, data: Any, schema: DataSchema
    ) -> Tuple[bool, List[str]]:
        """Basic data validation without jsonschema."""
        errors = []

        if not isinstance(data, dict):
            errors.append("Data must be a dictionary")
            return False, errors

        # Check required fields
        for field in schema.required_fields:
            if field not in data:
                errors.append(f"Missing required field: {field}")
            elif data[field] is None:
                errors.append(f"Required field is null: {field}")

        # Type validation for known fields
        properties = schema.schema.get("properties", {})
        for field, value in data.items():
            if field in properties and value is not None:
                field_def = properties[field]
                expected_type = field_def.get("type")

                if expected_type == "number" and not isinstance(value, (int, float)):
                    errors.append(f"Field '{field}' should be numeric")
                elif expected_type == "integer" and not isinstance(value, int):
                    errors.append(f"Field '{field}' should be integer")
                elif expected_type == "string" and not isinstance(value, str):
                    errors.append(f"Field '{field}' should be string")
                elif expected_type == "boolean" and not isinstance(value, bool):
                    errors.append(f"Field '{field}' should be boolean")

                # Range validation
                if expected_type in ["number", "integer"]:
                    min_val = field_def.get("minimum")
                    max_val = field_def.get("maximum")

                    if min_val is not None and value < min_val:
                        errors.append(f"Field '{field}' below minimum: {min_val}")
                    if max_val is not None and value > max_val:
                        errors.append(f"Field '{field}' above maximum: {max_val}")

        return len(errors) == 0, errors

    # Data Processing
    def process_telemetry_data(
        self,
        raw_data: Union[Dict[str, Any], List[Dict[str, Any]]],
        schema_name: str = "telemetry",
    ) -> Tuple[Any, DataQualityMetrics]:
        """
        Process telemetry data with validation and quality assessment.

        Args:
            raw_data: Raw telemetry data
            schema_name: Schema to validate against

        Returns:
            Tuple of (processed_data, quality_metrics)
        """
        start_time = time.time()

        # Ensure data is in list format
        if isinstance(raw_data, dict):
            data_list = [raw_data]
        else:
            data_list = raw_data

        # Validate data
        valid_data = []
        invalid_count = 0

        for item in data_list:
            is_valid, errors = self.validate_data(item, schema_name)
            if is_valid:
                valid_data.append(item)
            else:
                invalid_count += 1
                logger.warning(f"Invalid data item: {errors}")

        # Convert to DataFrame if pandas available
        if DATA_AVAILABLE and valid_data:
            try:
                df = pd.DataFrame(valid_data)
                processed_data = df
            except Exception as e:
                logger.warning(f"Failed to create DataFrame: {e}")
                processed_data = valid_data
        else:
            processed_data = valid_data

        # Calculate quality metrics
        quality = DataQualityMetrics(
            total_records=len(data_list),
            valid_records=len(valid_data),
            invalid_records=invalid_count,
            completeness=len(valid_data) / len(data_list) if data_list else 0,
            validity=len(valid_data) / len(data_list) if data_list else 0,
        )

        processing_time = time.time() - start_time
        self.processing_times.setdefault("telemetry", []).append(processing_time)

        return processed_data, quality

    def analyze_statistics(
        self, data: Any, analysis_type: str = "basic"
    ) -> Dict[str, Any]:
        """
        Perform statistical analysis on data.

        Args:
            data: Data to analyze (DataFrame, list, or array)
            analysis_type: Type of analysis to perform

        Returns:
            Dictionary of statistical results
        """
        start_time = time.time()

        results = {}

        try:
            # Convert data to numpy array
            if DATA_AVAILABLE and isinstance(data, pd.DataFrame):
                numeric_data = data.select_dtypes(include=[np.number]).values
            elif hasattr(data, "__iter__") and not isinstance(data, str):
                numeric_data = np.array(data)
            else:
                numeric_data = np.array([data])

            if numeric_data.size == 0:
                return {"error": "No numeric data to analyze"}

            # Basic statistics
            results["count"] = len(numeric_data.flatten())
            results["mean"] = float(np.mean(numeric_data))
            results["std"] = float(np.std(numeric_data))
            results["min"] = float(np.min(numeric_data))
            results["max"] = float(np.max(numeric_data))

            if SCIPY_AVAILABLE:
                # Advanced statistics
                if analysis_type == "distribution":
                    # Normality tests
                    if len(numeric_data) >= 3:
                        _, p_normal = stats.normaltest(numeric_data.flatten())
                        _, p_shapiro = stats.shapiro(
                            numeric_data.flatten()[:5000]
                        )  # Shapiro limited to 5000
                        results["normality_tests"] = {
                            "normaltest_p": float(p_normal),
                            "shapiro_p": float(p_shapiro),
                        }

                elif analysis_type == "correlation":
                    if numeric_data.shape[1] > 1:
                        corr_matrix = np.corrcoef(numeric_data.T)
                        results["correlation_matrix"] = corr_matrix.tolist()

                # Outlier detection
                z_scores = stats.zscore(numeric_data.flatten())
                outliers = np.abs(z_scores) > 3
                results["outliers"] = {
                    "count": int(np.sum(outliers)),
                    "percentage": float(np.sum(outliers) / len(outliers) * 100),
                }

        except Exception as e:
            results["error"] = str(e)
            logger.error(f"Statistical analysis failed: {e}")

        processing_time = time.time() - start_time
        self.processing_times.setdefault("statistics", []).append(processing_time)

        return results

    def transform_coordinates(
        self, data: Dict[str, Any], from_frame: str = "world", to_frame: str = "robot"
    ) -> Dict[str, Any]:
        """
        Transform coordinates between reference frames.

        Args:
            data: Coordinate data with x, y, z, roll, pitch, yaw
            from_frame: Source reference frame
            to_frame: Target reference frame

        Returns:
            Transformed coordinate data
        """
        if not TRANSFORMS_AVAILABLE:
            logger.warning("Transforms3d not available, returning original data")
            return data

        try:
            # Extract pose data
            x = data.get("x", 0.0)
            y = data.get("y", 0.0)
            z = data.get("z", 0.0)
            roll = data.get("roll", 0.0)
            pitch = data.get("pitch", 0.0)
            yaw = data.get("yaw", 0.0)

            # Create transformation matrix
            import transforms3d

            rotation_matrix = transforms3d.euler.euler2mat(roll, pitch, yaw)
            transform_matrix = transforms3d.affines.compose(
                [x, y, z], rotation_matrix, [1, 1, 1]
            )

            # For now, return the matrix - in a real implementation,
            # you'd apply different transformations based on frames
            return {
                **data,
                "transform_matrix": transform_matrix.tolist(),
                "transformed_at": time.time(),
            }

        except Exception as e:
            logger.error(f"Coordinate transformation failed: {e}")
            return data

    def process_time_series(
        self, data: Dict[str, Any], window_size: Optional[int] = None
    ) -> Dict[str, Any]:
        """
        Process time series data with filtering and analysis.

        Args:
            data: Time series data with timestamps and values
            window_size: Moving window size for analysis

        Returns:
            Processed time series data
        """
        try:
            timestamps = data.get("timestamps", [])
            values = data.get("values", {})

            if not timestamps or not values:
                return {"error": "Missing timestamps or values"}

            results = {
                "original_count": len(timestamps),
                "processed_at": time.time(),
                "metrics": {},
            }

            # Process each value series
            for key, value_list in values.items():
                if len(value_list) != len(timestamps):
                    continue

                value_array = np.array(value_list)

                # Basic metrics
                results["metrics"][key] = {
                    "mean": float(np.mean(value_array)),
                    "std": float(np.std(value_array)),
                    "min": float(np.min(value_array)),
                    "max": float(np.max(value_array)),
                }

                # Moving averages if window specified
                if window_size and len(value_array) > window_size:
                    if SCIPY_AVAILABLE:
                        try:
                            moving_avg = signal.savgol_filter(
                                value_array, window_size, min(3, window_size - 1)
                            )
                            results["metrics"][key][
                                "moving_average"
                            ] = moving_avg.tolist()
                        except:
                            pass

            return results

        except Exception as e:
            logger.error(f"Time series processing failed: {e}")
            return {"error": str(e)}

    # JSON Processing
    def json_dumps(self, data: Any, **kwargs) -> Union[str, bytes]:
        """
        High-performance JSON serialization.

        Args:
            data: Data to serialize
            **kwargs: Additional options

        Returns:
            JSON string or bytes
        """
        try:
            if ORJSON_AVAILABLE:
                return orjson.dumps(data, **kwargs).decode("utf-8")
            else:
                return json.dumps(data, **kwargs)
        except Exception as e:
            logger.error(f"JSON serialization failed: {e}")
            return "{}"

    def json_loads(self, data: Union[str, bytes], **kwargs) -> Any:
        """
        High-performance JSON deserialization.

        Args:
            data: JSON string or bytes to deserialize
            **kwargs: Additional options

        Returns:
            Deserialized data
        """
        try:
            if ORJSON_AVAILABLE:
                return orjson.loads(data, **kwargs)
            else:
                return json.loads(data, **kwargs)
        except Exception as e:
            logger.error(f"JSON deserialization failed: {e}")
            return {}

    # Data Structures
    def create_circular_buffer(self, max_size: int, data_type: type = float) -> List:
        """
        Create a circular buffer for efficient data storage.

        Args:
            max_size: Maximum buffer size
            data_type: Type of data to store

        Returns:
            Circular buffer implementation
        """

        class CircularBuffer:
            def __init__(self, size, dtype=float):
                self.size = size
                self.buffer = [dtype() for _ in range(size)]
                self.index = 0
                self.full = False

            def append(self, item):
                self.buffer[self.index] = item
                self.index = (self.index + 1) % self.size
                if self.index == 0:
                    self.full = True

            def get_all(self):
                if self.full:
                    return self.buffer[self.index :] + self.buffer[: self.index]
                else:
                    return self.buffer[: self.index]

            def get_last_n(self, n: int):
                all_data = self.get_all()
                return all_data[-n:] if len(all_data) >= n else all_data

        return CircularBuffer(max_size, data_type)

    def create_lru_cache(self, max_size: int = 100) -> Dict:
        """
        Create an LRU cache for efficient data access.

        Args:
            max_size: Maximum cache size

        Returns:
            LRU cache implementation
        """

        class LRUCache:
            def __init__(self, capacity):
                self.capacity = capacity
                self.cache = {}
                self.order = []

            def get(self, key):
                if key in self.cache:
                    self.order.remove(key)
                    self.order.append(key)
                    return self.cache[key]
                return None

            def put(self, key, value):
                if key in self.cache:
                    self.order.remove(key)
                elif len(self.cache) >= self.capacity:
                    oldest = self.order.pop(0)
                    del self.cache[oldest]

                self.cache[key] = value
                self.order.append(key)

        return LRUCache(max_size)

    # Caching and Performance
    def cache_data(self, key: str, data: Any, ttl_seconds: Optional[int] = None):
        """
        Cache data for performance optimization.

        Args:
            key: Cache key
            data: Data to cache
            ttl_seconds: Time to live in seconds
        """
        cache_entry = {"data": data, "timestamp": time.time(), "ttl": ttl_seconds}

        self.data_cache[key] = cache_entry

    def get_cached_data(self, key: str) -> Optional[Any]:
        """
        Retrieve cached data.

        Args:
            key: Cache key

        Returns:
            Cached data or None if not found/expired
        """
        if key not in self.data_cache:
            self.cache_misses += 1
            return None

        cache_entry = self.data_cache[key]

        # Check TTL
        if cache_entry["ttl"]:
            age = time.time() - cache_entry["timestamp"]
            if age > cache_entry["ttl"]:
                del self.data_cache[key]
                self.cache_misses += 1
                return None

        self.cache_hits += 1
        return cache_entry["data"]

    def get_performance_metrics(self) -> Dict[str, Any]:
        """Get data processing performance metrics."""
        return {
            "cache_stats": {
                "hits": self.cache_hits,
                "misses": self.cache_misses,
                "hit_rate": (
                    self.cache_hits / (self.cache_hits + self.cache_misses)
                    if (self.cache_hits + self.cache_misses) > 0
                    else 0
                ),
            },
            "processing_times": {
                operation: {
                    "avg_time": sum(times) / len(times),
                    "min_time": min(times),
                    "max_time": max(times),
                    "count": len(times),
                }
                for operation, times in self.processing_times.items()
            },
            "cache_size": len(self.data_cache),
            "schemas_registered": len(self.schemas),
            "quality_checks": len(self.quality_metrics),
        }

    def cleanup_cache(self, max_age_seconds: int = 3600):
        """
        Clean up expired cache entries.

        Args:
            max_age_seconds: Maximum age for cache entries
        """
        current_time = time.time()
        expired_keys = []

        for key, entry in self.data_cache.items():
            age = current_time - entry["timestamp"]
            if age > max_age_seconds:
                expired_keys.append(key)

        for key in expired_keys:
            del self.data_cache[key]

        if expired_keys:
            logger.info(f"Cleaned up {len(expired_keys)} expired cache entries")


# Global data manager instance
_data_manager = None


def get_data_manager() -> DataManager:
    """Get global data manager instance."""
    global _data_manager
    if _data_manager is None:
        _data_manager = DataManager()
    return _data_manager


# Convenience functions
def validate_data(data: Any, schema_name: str) -> Tuple[bool, List[str]]:
    """Validate data against schema."""
    return get_data_manager().validate_data(data, schema_name)


def process_telemetry(raw_data: Any) -> Tuple[Any, DataQualityMetrics]:
    """Process telemetry data."""
    return get_data_manager().process_telemetry_data(raw_data)


def analyze_statistics(data: Any, analysis_type: str = "basic") -> Dict[str, Any]:
    """Perform statistical analysis."""
    return get_data_manager().analyze_statistics(data, analysis_type)


def json_dumps(data: Any) -> str:
    """JSON serialization."""
    return get_data_manager().json_dumps(data)


def json_loads(data: str) -> Any:
    """JSON deserialization."""
    return get_data_manager().json_loads(data)


# Export key components
__all__ = [
    "DataManager",
    "DataSchema",
    "DataQualityMetrics",
    "TimeSeriesData",
    "DataValidationError",
    "get_data_manager",
    "validate_data",
    "process_telemetry",
    "analyze_statistics",
    "json_dumps",
    "json_loads",
]
