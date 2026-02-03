#!/usr/bin/env python3
"""
URC 2026 Test Data Factories - Faker + Factory Boy Implementation

Replaces 200+ lines of manual test data creation with professional
factories that generate realistic, consistent test data automatically.

Author: URC 2026 Testing Team
"""

from typing import Dict, List, Any, Optional
import numpy as np

# Import libraries with fallbacks
try:
    from faker import Faker
    from factory import Factory, LazyAttribute, LazyFunction, Sequence, SubFactory
    import factory

    FACTORY_AVAILABLE = True
except ImportError:
    FACTORY_AVAILABLE = False

    # Fallback implementations
    class Factory:
        pass

    class Faker:
        pass


# Initialize faker
fake = Faker()
fake.seed_instance(42)

# Initialize faker with deterministic seed for reproducible tests
fake = Faker()
fake.seed_instance(12345)  # Deterministic seed for consistent test data

if FACTORY_AVAILABLE:

    class URCBaseFactory(Factory):
        """Base factory with common URC testing utilities."""

        class Meta:
            abstract = True

        @classmethod
        def create_batch_dict(cls, size: int, **kwargs) -> List[Dict[str, Any]]:
            """Create batch of objects as dictionaries."""
            return [obj.__dict__ for obj in cls.create_batch(size, **kwargs)]

        @classmethod
        def build_batch_dict(cls, size: int, **kwargs) -> List[Dict[str, Any]]:
            """Build batch of objects as dictionaries (not saved)."""
            return [obj.__dict__ for obj in cls.build_batch(size, **kwargs)]

    # Rover State Factories
    class RoverStateFactory(URCBaseFactory):
        """Factory for rover state data."""

        class Meta:
            model = dict

        # Core state data
        battery_level = LazyFunction(
            lambda: fake.pyfloat(min_value=0.0, max_value=100.0, precision=2)
        )
        communication_status = LazyFunction(
            lambda: fake.random_element(elements=["connected", "degraded", "lost"])
        )
        system_health = LazyFunction(
            lambda: fake.random_element(
                elements=["nominal", "degraded", "critical", "emergency"]
            )
        )
        current_mode = LazyFunction(
            lambda: fake.random_element(
                elements=[
                    "BOOT",
                    "CALIBRATION",
                    "IDLE",
                    "TELEOPERATION",
                    "AUTONOMOUS",
                    "SAFETY",
                ]
            )
        )

        # Position data
        position_x = LazyFunction(
            lambda: fake.pyfloat(min_value=-1000, max_value=1000, precision=2)
        )
        position_y = LazyFunction(
            lambda: fake.pyfloat(min_value=-1000, max_value=1000, precision=2)
        )
        position_z = LazyFunction(
            lambda: fake.pyfloat(min_value=-10, max_value=10, precision=2)
        )

        # Orientation (quaternion)
        orientation_w = LazyFunction(
            lambda: fake.pyfloat(min_value=-1, max_value=1, precision=4)
        )
        orientation_x = LazyFunction(
            lambda: fake.pyfloat(min_value=-1, max_value=1, precision=4)
        )
        orientation_y = LazyFunction(
            lambda: fake.pyfloat(min_value=-1, max_value=1, precision=4)
        )
        orientation_z = LazyFunction(
            lambda: fake.pyfloat(min_value=-1, max_value=1, precision=4)
        )

        # Velocity data
        linear_velocity_x = LazyFunction(
            lambda: fake.pyfloat(min_value=-2, max_value=2, precision=3)
        )
        linear_velocity_y = LazyFunction(
            lambda: fake.pyfloat(min_value=-2, max_value=2, precision=3)
        )
        angular_velocity_z = LazyFunction(
            lambda: fake.pyfloat(min_value=-3, max_value=3, precision=3)
        )

        # Mission data
        mission_id = LazyFunction(lambda: str(fake.uuid4()))
        mission_status = LazyFunction(
            lambda: fake.random_element(
                elements=["planned", "active", "paused", "completed", "failed"]
            )
        )
        waypoints_completed = LazyFunction(lambda: fake.random_int(min=0, max=10))
        waypoints_total = LazyAttribute(
            lambda o: max(o.waypoints_completed + fake.random_int(min=0, max=5), 1)
        )

        # Timestamp
        timestamp = LazyFunction(lambda: fake.unix_time())

        # Computed fields
        progress_percentage = LazyAttribute(
            lambda o: (
                (o.waypoints_completed / o.waypoints_total * 100)
                if o.waypoints_total > 0
                else 0
            )
        )

    class WaypointFactory(URCBaseFactory):
        """Factory for waypoint data."""

        class Meta:
            model = dict

        name = LazyFunction(lambda: fake.word())
        x = LazyFunction(
            lambda: fake.pyfloat(min_value=-1000, max_value=1000, precision=2)
        )
        y = LazyFunction(
            lambda: fake.pyfloat(min_value=-1000, max_value=1000, precision=2)
        )
        z = LazyFunction(lambda: fake.pyfloat(min_value=-5, max_value=5, precision=2))

        heading = LazyFunction(
            lambda: fake.pyfloat(min_value=-180, max_value=180, precision=1)
        )
        tolerance = LazyFunction(
            lambda: fake.pyfloat(min_value=0.1, max_value=2.0, precision=2)
        )

        description = LazyFunction(lambda: fake.sentence(nb_words=6))
        priority = LazyFunction(lambda: fake.random_int(min=1, max=10))

        # Optional fields
        max_speed = LazyFunction(
            lambda: fake.pyfloat(min_value=0.1, max_value=2.0, precision=2)
        )
        timeout = LazyFunction(lambda: fake.random_int(min=30, max=300))

        # Computed validation
        is_valid = LazyAttribute(
            lambda o: -1000 <= o.x <= 1000 and -1000 <= o.y <= 1000
        )

    class SensorDataFactory(URCBaseFactory):
        """Factory for sensor data."""

        class Meta:
            model = dict

        sensor_id = Sequence(lambda n: f"sensor_{n:03d}")
        sensor_type = LazyFunction(
            lambda: fake.random_element(
                elements=["imu", "gps", "camera", "lidar", "encoder"]
            )
        )
        timestamp = LazyFunction(lambda: fake.unix_time())

        # IMU data
        accel_x = LazyFunction(
            lambda: fake.pyfloat(min_value=-19.6, max_value=19.6, precision=3)
        )
        accel_y = LazyFunction(
            lambda: fake.pyfloat(min_value=-19.6, max_value=19.6, precision=3)
        )
        accel_z = LazyFunction(
            lambda: fake.pyfloat(min_value=-19.6, max_value=19.6, precision=3)
        )

        gyro_x = LazyFunction(
            lambda: fake.pyfloat(min_value=-500, max_value=500, precision=2)
        )
        gyro_y = LazyFunction(
            lambda: fake.pyfloat(min_value=-500, max_value=500, precision=2)
        )
        gyro_z = LazyFunction(
            lambda: fake.pyfloat(min_value=-500, max_value=500, precision=2)
        )

        # GPS data
        latitude = LazyFunction(lambda: fake.latitude())
        longitude = LazyFunction(lambda: fake.longitude())
        altitude = LazyFunction(
            lambda: fake.pyfloat(min_value=-100, max_value=5000, precision=1)
        )
        satellites = LazyFunction(lambda: fake.random_int(min=0, max=20))

        # Camera data
        image_width = LazyFunction(lambda: fake.random_int(min=640, max=1920))
        image_height = LazyFunction(lambda: fake.random_int(min=480, max=1080))
        features_detected = LazyFunction(lambda: fake.random_int(min=0, max=1000))

        # Quality metrics
        signal_strength = LazyFunction(
            lambda: fake.pyfloat(min_value=0, max_value=1, precision=3)
        )
        noise_level = LazyFunction(
            lambda: fake.pyfloat(min_value=0, max_value=0.1, precision=4)
        )
        data_quality = LazyFunction(
            lambda: fake.random_element(elements=["excellent", "good", "fair", "poor"])
        )

    class MissionFactory(URCBaseFactory):
        """Factory for mission data."""

        class Meta:
            model = dict

        id = LazyFunction(lambda: str(fake.uuid4()))
        name = LazyFunction(lambda: fake.sentence(nb_words=3))
        type = LazyFunction(
            lambda: fake.random_element(
                elements=[
                    "autonomous_navigation",
                    "sample_collection",
                    "delivery",
                    "survey",
                ]
            )
        )

        description = LazyFunction(lambda: fake.paragraph(nb_sentences=2))
        priority = LazyFunction(lambda: fake.random_int(min=1, max=10))

        # Timing
        created_at = LazyFunction(lambda: fake.date_time_this_year())
        started_at = LazyAttribute(
            lambda o: (
                fake.date_time_between(start_date=o.created_at)
                if fake.boolean()
                else None
            )
        )
        completed_at = LazyAttribute(
            lambda o: (
                fake.date_time_between(start_date=o.started_at)
                if o.started_at and fake.boolean()
                else None
            )
        )

        # Status
        status = LazyAttribute(
            lambda o: (
                "completed"
                if o.completed_at
                else ("active" if o.started_at else "planned")
            )
        )

        # Waypoints (variable-length list of waypoint dicts)
        waypoints = LazyFunction(
            lambda: [
                WaypointFactory.build() for _ in range(fake.random_int(min=2, max=10))
            ]
        )

        # Mission parameters
        max_duration = LazyFunction(
            lambda: fake.random_int(min=300, max=3600)
        )  # 5 min to 1 hour
        max_speed = LazyFunction(
            lambda: fake.pyfloat(min_value=0.5, max_value=2.0, precision=2)
        )
        safety_enabled = LazyFunction(lambda: fake.boolean(chance_of_getting_true=95))

        # Results
        distance_traveled = LazyAttribute(
            lambda o: (
                sum(
                    np.sqrt(
                        (o.waypoints[i + 1]["x"] - o.waypoints[i]["x"]) ** 2
                        + (o.waypoints[i + 1]["y"] - o.waypoints[i]["y"]) ** 2
                    )
                    for i in range(len(o.waypoints) - 1)
                )
                if len(o.waypoints) > 1
                else 0
            )
        )

        samples_collected = LazyAttribute(
            lambda o: (
                fake.random_int(min=0, max=len(o.waypoints))
                if o.type == "sample_collection"
                else 0
            )
        )

    class TelemetryFactory(URCBaseFactory):
        """Factory for telemetry data."""

        class Meta:
            model = dict

        type = "telemetry"
        timestamp = LazyFunction(lambda: fake.unix_time())
        source = "rover"

        # Rover state
        state = SubFactory(RoverStateFactory)

        # Sensor data
        sensors = LazyFunction(
            lambda: {
                sensor_id: SensorDataFactory()
                for sensor_id in [
                    f"sensor_{i:02d}" for i in range(fake.random_int(min=3, max=8))
                ]
            }
        )

        # System metrics
        cpu_usage = LazyFunction(
            lambda: fake.pyfloat(min_value=0, max_value=100, precision=1)
        )
        memory_usage = LazyFunction(
            lambda: fake.pyfloat(min_value=0, max_value=100, precision=1)
        )
        temperature = LazyFunction(
            lambda: fake.pyfloat(min_value=20, max_value=80, precision=1)
        )

        # Network status
        network_latency = LazyFunction(
            lambda: fake.pyfloat(min_value=0, max_value=500, precision=1)
        )
        packets_sent = LazyFunction(lambda: fake.random_int(min=0, max=1000))
        packets_received = LazyFunction(lambda: fake.random_int(min=0, max=1000))

    class CommandFactory(URCBaseFactory):
        """Factory for command data."""

        class Meta:
            model = dict

        type = "command"
        timestamp = LazyFunction(lambda: fake.unix_time())
        source = LazyFunction(
            lambda: fake.random_element(
                elements=["ground_station", "mission_control", "autonomous_planner"]
            )
        )
        correlation_id = LazyFunction(lambda: str(fake.uuid4()))

        command = LazyFunction(
            lambda: fake.random_element(
                elements=[
                    "start_mission",
                    "pause_mission",
                    "resume_mission",
                    "stop_mission",
                    "emergency_stop",
                    "return_home",
                    "switch_to_teleop",
                    "calibrate_sensors",
                ]
            )
        )

        # Command parameters
        parameters = LazyFunction(
            lambda: {
                "priority": fake.random_int(min=1, max=10),
                "timeout": fake.random_int(min=30, max=300),
                "description": fake.sentence(),
            }
        )

        # Add specific parameters based on command
        @factory.lazy_attribute
        def parameters(self):
            base_params = {
                "priority": fake.random_int(min=1, max=10),
                "timeout": fake.random_int(min=30, max=300),
                "description": fake.sentence(),
            }

            # Add command-specific parameters
            if self.command == "start_mission":
                base_params["mission_type"] = fake.random_element(
                    ["autonomous_navigation", "sample_collection"]
                )
                base_params["waypoints"] = WaypointFactory.create_batch(
                    fake.random_int(min=2, max=5)
                )
            elif self.command in ["pause_mission", "resume_mission"]:
                base_params["reason"] = fake.sentence()

            return base_params

    # Specialized factories for different scenarios
    class EmergencyScenarioFactory(URCBaseFactory):
        """Factory for emergency scenarios."""

        class Meta:
            model = dict

        # Critical rover state
        battery_level = LazyFunction(
            lambda: fake.pyfloat(min_value=0, max_value=15, precision=1)
        )  # Critical battery
        communication_status = LazyFunction(
            lambda: fake.random_element(elements=["lost", "degraded"])
        )
        system_health = "critical"

        # Emergency telemetry
        telemetry = SubFactory(TelemetryFactory)
        emergency_type = LazyFunction(
            lambda: fake.random_element(
                elements=[
                    "battery_critical",
                    "communication_lost",
                    "motor_failure",
                    "sensor_failure",
                ]
            )
        )

        # Emergency command
        emergency_command = SubFactory(CommandFactory)
        emergency_command.command = "emergency_stop"

    class PerformanceTestFactory(URCBaseFactory):
        """Factory for performance testing data."""

        class Meta:
            model = dict

        # High-frequency data generation
        sample_count = 1000
        frequency_hz = 100

        # Generate time series data
        timestamps = LazyFunction(
            lambda: [i / 100.0 for i in range(1000)]
        )  # 10 seconds at 100Hz

        # Sinusoidal sensor data with noise
        sensor_values = LazyFunction(
            lambda: [
                10.0 + 2.0 * np.sin(2 * np.pi * 0.1 * t) + 0.1 * np.random.normal()
                for t in [i / 100.0 for i in range(1000)]
            ]
        )

        # Performance metrics
        avg_response_time = LazyFunction(
            lambda: fake.pyfloat(min_value=0.001, max_value=0.1, precision=4)
        )
        max_memory_usage = LazyFunction(
            lambda: fake.pyfloat(min_value=50, max_value=200, precision=1)
        )
        cpu_utilization = LazyFunction(
            lambda: fake.pyfloat(min_value=10, max_value=95, precision=1)
        )

    # Utility functions
    def create_test_rover_state(**kwargs) -> Dict[str, Any]:
        """Create a single test rover state."""
        return RoverStateFactory(**kwargs)

    def create_test_mission(**kwargs) -> Dict[str, Any]:
        """Create a test mission."""
        return MissionFactory(**kwargs)

    def create_emergency_scenario(**kwargs) -> Dict[str, Any]:
        """Create an emergency test scenario."""
        return EmergencyScenarioFactory(**kwargs)

    def generate_sensor_noise(base_value: float, noise_level: float = 0.1) -> float:
        """Generate realistic sensor noise."""
        return base_value + noise_level * np.random.normal()

    def create_time_series_data(
        points: int = 100, frequency: float = 1.0, pattern: str = "sinusoidal"
    ) -> Dict[str, List[float]]:
        """Create time series test data."""
        times = [i / frequency for i in range(points)]

        if pattern == "sinusoidal":
            values = [
                10.0 + 2.0 * np.sin(2 * np.pi * 0.1 * t) + 0.1 * np.random.normal()
                for t in times
            ]
        elif pattern == "linear":
            values = [0.1 * t + np.random.normal() for t in times]
        elif pattern == "step":
            values = [10.0 if t > points / (2 * frequency) else 0.0 for t in times]
        else:
            values = [np.random.normal(10, 1) for _ in times]

        return {"timestamps": times, "values": values}

else:
    # Fallback functions when libraries not available
    def create_test_rover_state(**kwargs):
        """Fallback rover state creation."""
        return {
            "battery_level": 85.0,
            "communication_status": "connected",
            "system_health": "nominal",
            "position_x": 0.0,
            "position_y": 0.0,
            **kwargs,
        }

    def create_test_mission(**kwargs):
        """Fallback mission creation."""
        return {
            "id": "test_mission_001",
            "name": "Test Mission",
            "type": "autonomous_navigation",
            "waypoints": [
                {"name": "wp1", "x": 0, "y": 0},
                {"name": "wp2", "x": 10, "y": 5},
            ],
            **kwargs,
        }

    def create_emergency_scenario(**kwargs):
        """Fallback emergency scenario."""
        return {
            "battery_level": 5.0,
            "communication_status": "lost",
            "emergency_type": "battery_critical",
            **kwargs,
        }


# Export main factory functions
__all__ = [
    "RoverStateFactory",
    "WaypointFactory",
    "SensorDataFactory",
    "MissionFactory",
    "TelemetryFactory",
    "CommandFactory",
    "EmergencyScenarioFactory",
    "PerformanceTestFactory",
    "create_test_rover_state",
    "create_test_mission",
    "create_emergency_scenario",
    "generate_sensor_noise",
    "create_time_series_data",
]
