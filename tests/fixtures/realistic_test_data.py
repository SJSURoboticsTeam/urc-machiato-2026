#!/usr/bin/env python3
"""
Realistic test data bootstrap from real config and profiles.

Loads waypoints, mission profiles, and safety limits from config/rover.yaml.
Generates sensor data matching real IMU/GPS profiles for tests that need
production-like data without hardware.

Author: URC 2026 Testing Team
"""

import os
from pathlib import Path
from typing import Any, Dict, List, Optional

# Project root relative to this file: tests/fixtures/ -> project root is ../..
_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
_CONFIG_PATH = _PROJECT_ROOT / "config" / "rover.yaml"


def _load_rover_yaml() -> Dict[str, Any]:
    """Load rover config YAML. Returns empty dict if file missing or invalid."""
    if not _CONFIG_PATH.is_file():
        return {}
    try:
        import yaml
        with open(_CONFIG_PATH) as f:
            return yaml.safe_load(f) or {}
    except Exception:
        return {}


def load_waypoints_from_config() -> List[Dict[str, Any]]:
    """Load waypoints from config/rover.yaml waypoints section."""
    config = _load_rover_yaml()
    waypoints = config.get("waypoints") or []
    if not isinstance(waypoints, list):
        return []
    return [
        {
            "name": w.get("name", ""),
            "x": float(w.get("x", 0)),
            "y": float(w.get("y", 0)),
            "tolerance": float(w.get("tolerance", 1.0)),
        }
        for w in waypoints
        if isinstance(w, dict)
    ]


def get_mission_profile(profile_name: str) -> Dict[str, Any]:
    """Get mission profile from config/rover.yaml mission_profiles."""
    config = _load_rover_yaml()
    profiles = config.get("mission_profiles") or {}
    if not isinstance(profiles, dict):
        return {}
    return dict(profiles.get(profile_name, {}))


def get_safety_limits() -> Dict[str, float]:
    """Get safety limits from config/rover.yaml for sensor data bounds."""
    config = _load_rover_yaml()
    limits = {}
    keys = [
        "safety_imu_accel_max",
        "safety_imu_gyro_max",
        "safety_max_speed_mps",
        "safety_boundary_radius_meters",
        "safety_battery_critical_percent",
        "safety_distance_meters",
    ]
    for key in keys:
        val = config.get(key)
        if val is not None:
            try:
                limits[key] = float(val)
            except (TypeError, ValueError):
                pass
    # Defaults from rover.yaml when key missing
    defaults = {
        "safety_imu_accel_max": 50.0,
        "safety_imu_gyro_max": 20.0,
        "safety_max_speed_mps": 2.0,
        "safety_boundary_radius_meters": 100.0,
        "safety_battery_critical_percent": 10.0,
        "safety_distance_meters": 0.3,
    }
    for k, v in defaults.items():
        if k not in limits:
            limits[k] = v
    return limits


def generate_imu_data(
    accel_bounds: Optional[tuple] = None,
    gyro_bounds: Optional[tuple] = None,
    use_safety_limits: bool = True,
) -> Dict[str, float]:
    """Generate IMU-like data within safe bounds (from config when use_safety_limits)."""
    limits = get_safety_limits() if use_safety_limits else {}
    accel_max = limits.get("safety_imu_accel_max", 50.0)
    gyro_max = limits.get("safety_imu_gyro_max", 20.0)
    if accel_bounds is not None:
        accel_max = min(accel_max, max(abs(b) for b in accel_bounds))
    if gyro_bounds is not None:
        gyro_max = min(gyro_max, max(abs(b) for b in gyro_bounds))
    import random
    random.seed(42)
    return {
        "linear_acceleration_x": random.uniform(-accel_max * 0.5, accel_max * 0.5),
        "linear_acceleration_y": random.uniform(-accel_max * 0.5, accel_max * 0.5),
        "linear_acceleration_z": 9.81 + random.uniform(-0.5, 0.5),
        "angular_velocity_x": random.uniform(-gyro_max * 0.5, gyro_max * 0.5),
        "angular_velocity_y": random.uniform(-gyro_max * 0.5, gyro_max * 0.5),
        "angular_velocity_z": random.uniform(-gyro_max * 0.5, gyro_max * 0.5),
    }


def generate_gps_data(
    lat: Optional[float] = None,
    lon: Optional[float] = None,
    altitude: float = 0.0,
) -> Dict[str, float]:
    """Generate GPS-like data. Uses config waypoints area if lat/lon not given."""
    waypoints = load_waypoints_from_config()
    if lat is None or lon is None:
        if waypoints:
            xs = [w["x"] for w in waypoints]
            ys = [w["y"] for w in waypoints]
            # Simple offset: treat x,y as meters, use a base lat/lon
            base_lat, base_lon = 37.0, -122.0
            lat = base_lat + (sum(ys) / len(ys)) / 111320.0 if ys else base_lat
            lon = base_lon + (sum(xs) / len(xs)) / (111320.0 * 0.7) if xs else base_lon
        else:
            lat = 37.0
            lon = -122.0
    return {
        "latitude": lat,
        "longitude": lon,
        "altitude": altitude,
        "status": 2,
    }


def get_blackboard_initial_state() -> Dict[str, Any]:
    """Return realistic initial blackboard state for tests (keys match blackboard_keys)."""
    waypoints = load_waypoints_from_config()
    first_waypoint = waypoints[0] if waypoints else {"x": 0.0, "y": 0.0}
    return {
        "robot_x": float(first_waypoint.get("x", 0)),
        "robot_y": float(first_waypoint.get("y", 0)),
        "mission_active": False,
        "sensors_ok": True,
        "perception_confidence": 0.0,
        "map_quality": 0.0,
        "closest_obstacle_distance": 999.0,
        "samples_collected": 0,
        "waypoints_completed": 0,
        "waypoints_total": len(waypoints) or 1,
        "system_state": "idle",
    }


def mission_scenario_from_config(
    profile_name: str = "waypoint_navigation",
) -> Dict[str, Any]:
    """Build a mission scenario dict from config mission profile and waypoints."""
    profile = get_mission_profile(profile_name)
    waypoints = load_waypoints_from_config()
    return {
        "profile": profile_name,
        "profile_config": profile,
        "waypoints": waypoints,
        "safety_limits": get_safety_limits(),
    }
