#!/usr/bin/env python3
"""
URC 2026 Basic Test Data Factories - Working Implementation

Simple test data generation without complex dependencies.

Author: URC 2026 Testing Team
"""

from typing import Dict, List, Any
import random
import time
import math


class TestDataFactory:
    """Basic test data factory."""

    @staticmethod
    def create_rover_state(**overrides) -> Dict[str, Any]:
        """Create a realistic rover state for testing."""
        base_data = {
            'battery_level': round(random.uniform(0.0, 100.0), 2),
            'communication_status': random.choice(['connected', 'degraded', 'lost']),
            'system_health': random.choice(['nominal', 'degraded', 'critical']),
            'current_mode': random.choice(['BOOT', 'CALIBRATION', 'IDLE', 'TELEOPERATION', 'AUTONOMOUS']),
            'position_x': round(random.uniform(-100, 100), 2),
            'position_y': round(random.uniform(-100, 100), 2),
            'position_z': 0.0,
            'linear_velocity_x': round(random.uniform(-1, 1), 2),
            'angular_velocity_z': round(random.uniform(-0.5, 0.5), 2),
            'mission_status': random.choice(['planned', 'active', 'completed']),
            'timestamp': time.time()
        }

        base_data.update(overrides)
        return base_data

    @staticmethod
    def create_waypoint(**overrides) -> Dict[str, Any]:
        """Create a realistic waypoint for testing."""
        base_data = {
            'name': f"Waypoint{random.randint(1,100)}",
            'x': round(random.uniform(-50, 50), 2),
            'y': round(random.uniform(-50, 50), 2),
            'z': 0.0,
            'heading': round(random.uniform(-180, 180), 1),
            'tolerance': round(random.uniform(0.1, 2.0), 2)
        }

        base_data.update(overrides)
        return base_data

    @staticmethod
    def create_mission(**overrides) -> Dict[str, Any]:
        """Create a realistic mission for testing."""
        waypoints = [TestDataFactory.create_waypoint() for _ in range(random.randint(2, 5))]

        base_data = {
            'id': f"mission_{random.randint(1000,9999)}",
            'name': f"Mission {random.randint(1,100)}",
            'type': random.choice(['autonomous_navigation', 'sample_collection', 'delivery']),
            'waypoints': waypoints,
            'max_speed': round(random.uniform(0.5, 2.0), 2),
            'status': 'planned',
            'created_at': time.time()
        }

        base_data.update(overrides)
        return base_data

    @staticmethod
    def create_telemetry(**overrides) -> Dict[str, Any]:
        """Create realistic telemetry data."""
        rover_state = TestDataFactory.create_rover_state()

        base_data = {
            'type': 'telemetry',
            'timestamp': time.time(),
            'source': 'rover',
            'rover_state': rover_state,
            'cpu_usage': round(random.uniform(10, 90), 1),
            'memory_usage': round(random.uniform(20, 80), 1),
            'temperature': round(random.uniform(25, 75), 1)
        }

        base_data.update(overrides)
        return base_data

    @staticmethod
    def create_batch_rover_states(count: int = 10) -> List[Dict[str, Any]]:
        """Create a batch of rover states."""
        return [TestDataFactory.create_rover_state() for _ in range(count)]

    @staticmethod
    def create_emergency_scenario(**overrides) -> Dict[str, Any]:
        """Create an emergency test scenario."""
        emergency_overrides = {
            'battery_level': round(random.uniform(1, 15), 1),
            'communication_status': 'lost',
            'system_health': 'critical'
        }
        emergency_overrides.update(overrides)
        return TestDataFactory.create_rover_state(**emergency_overrides)


# Convenience functions (maintain compatibility)
def create_test_rover_state(**kwargs) -> Dict[str, Any]:
    """Create a single test rover state."""
    return TestDataFactory.create_rover_state(**kwargs)

def create_test_mission(**kwargs) -> Dict[str, Any]:
    """Create a test mission."""
    return TestDataFactory.create_mission(**kwargs)

def create_emergency_scenario(**kwargs) -> Dict[str, Any]:
    """Create an emergency test scenario."""
    return TestDataFactory.create_emergency_scenario(**kwargs)

def generate_sensor_noise(base_value: float, noise_level: float = 0.1) -> float:
    """Generate realistic sensor noise."""
    return base_value + noise_level * random.gauss(0, 1)

def create_time_series_data(points: int = 100, frequency: float = 1.0,
                          pattern: str = 'sinusoidal') -> Dict[str, List[float]]:
    """Create time series test data."""
    times = [i / frequency for i in range(points)]

    if pattern == 'sinusoidal':
        values = [10.0 + 2.0 * math.sin(2 * math.pi * 0.1 * t) + 0.1 * random.gauss(0, 1) for t in times]
    elif pattern == 'linear':
        values = [0.1 * t + random.gauss(0, 1) for t in times]
    elif pattern == 'step':
        values = [10.0 if t > points/(2*frequency) else 0.0 for t in times]
    else:
        values = [random.gauss(10, 1) for _ in times]

    return {'timestamps': times, 'values': values}

# Legacy compatibility
RoverStateFactory = TestDataFactory
WaypointFactory = TestDataFactory
MissionFactory = TestDataFactory

# Export functions
__all__ = [
    'TestDataFactory',
    'create_test_rover_state',
    'create_test_mission',
    'create_emergency_scenario',
    'generate_sensor_noise',
    'create_time_series_data'
]

