#!/usr/bin/env python3
"""
Configuration Validator - Validates Hardware and Software Configurations

Ensures that configuration files are valid and compatible before system startup.
Prevents runtime errors from invalid configurations during hardware integration.

Author: URC 2026 Autonomy Team
"""

import logging
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


class ConfigurationValidator:
    """
    Validates configuration files and ensures compatibility.

    Supports validation of:
    - Hardware interface configurations
    - ROS2 parameter configurations
    - Mission configurations
    - Calibration configurations
    """

    def __init__(self, config_dir: Optional[str] = None):
        """
        Initialize configuration validator.

        Args:
            config_dir: Directory containing configuration files
        """
        if config_dir is None:
            self.config_dir = Path(__file__).parent
        else:
            self.config_dir = Path(config_dir)
        self.schemas = self._load_validation_schemas()

    def _load_validation_schemas(self) -> Dict[str, Dict[str, Any]]:
        """Load validation schemas for different configuration types."""
        return {
            "hardware": {
                "required_components": [
                    "can_bus",
                    "motor_controller",
                    "drive_system",
                    "robotic_arm",
                    "science_payload",
                    "power_system",
                ],
                "valid_modes": ["mock", "hardware"],
                "component_dependencies": {
                    "motor_controller": ["can_bus"],
                    "drive_system": ["can_bus", "motor_controller"],
                    "robotic_arm": ["can_bus", "motor_controller"],
                    "science_payload": ["can_bus"],
                    "power_system": ["can_bus"],
                },
            },
            "ros2": {
                "required_nodes": ["state_machine_director", "rosbridge_server"],
                "parameter_ranges": {
                    "state_update_rate": (1.0, 100.0),
                    "control_loop_rate": (10.0, 1000.0),
                    "emergency_stop_enabled": [True, False],
                },
            },
            "mission": {
                "required_fields": ["name", "waypoints"],
                "waypoint_fields": ["x", "y", "heading"],
                "valid_mission_types": [
                    "waypoint_navigation",
                    "sample_collection",
                    "science_survey",
                ],
            },
        }

    def validate_hardware_config(self, config: Dict[str, Any]) -> List[str]:
        """
        Validate hardware interface configuration.

        Args:
            config: Hardware configuration dictionary

        Returns:
            List of validation error messages (empty if valid)
        """
        errors = []
        schema = self.schemas["hardware"]

        # Check required components
        for component in schema["required_components"]:
            if component not in config:
                errors.append(f"Missing required component: {component}")
                continue

            mode = config[component]
            if mode not in schema["valid_modes"]:
                errors.append(
                    f"Invalid mode '{mode}' for {component}. "
                    f"Must be one of: {schema['valid_modes']}"
                )

        # Check component dependencies
        for component, dependencies in schema["component_dependencies"].items():
            if component in config and config[component] == "hardware":
                for dependency in dependencies:
                    if dependency not in config or config[dependency] != "hardware":
                        errors.append(
                            f"Component {component} requires {dependency} to be "
                            f"hardware, but {dependency} is set to "
                            f"{config.get(dependency, 'missing')}"
                        )

        # Check for incompatible combinations
        incompatible_combinations = [
            # Can't have hardware sensors without CAN bus
            (["sensor_imu", "sensor_gps"], "hardware", ["can_bus"], "hardware")
        ]

        for sensors, sensor_mode, requirements, req_mode in incompatible_combinations:
            sensor_hardware = any(
                config.get(sensor, "mock") == sensor_mode for sensor in sensors
            )
            req_missing = any(
                config.get(req, "mock") != req_mode for req in requirements
            )

            if sensor_hardware and req_missing:
                errors.append(
                    f"Sensors {sensors} require {requirements} to be {req_mode}"
                )

        return errors

    def validate_ros2_config(self, config: Dict[str, Any]) -> List[str]:
        """
        Validate ROS2 configuration.

        Args:
            config: ROS2 configuration dictionary

        Returns:
            List of validation error messages (empty if valid)
        """
        errors = []
        schema = self.schemas["ros2"]

        # Check required nodes
        if "nodes" in config:
            configured_nodes = set(config["nodes"].keys())
            required_nodes = set(schema["required_nodes"])

            missing_nodes = required_nodes - configured_nodes
            if missing_nodes:
                errors.append(f"Missing required ROS2 nodes: {missing_nodes}")

        # Validate parameter ranges
        if "parameters" in config:
            for param_name, param_value in config["parameters"].items():
                if param_name in schema["parameter_ranges"]:
                    valid_range = schema["parameter_ranges"][param_name]

                    if isinstance(valid_range, tuple):  # Numeric range
                        min_val, max_val = valid_range
                        if not (min_val <= param_value <= max_val):
                            errors.append(
                                f"Parameter {param_name} value {param_value} "
                                f"outside valid range [{min_val}, {max_val}]"
                            )
                    elif isinstance(valid_range, list):  # Discrete values
                        if param_value not in valid_range:
                            errors.append(
                                f"Parameter {param_name} value {param_value} "
                                f"not in valid values: {valid_range}"
                            )

        return errors

    def validate_mission_config(self, config: Dict[str, Any]) -> List[str]:
        """
        Validate mission configuration.

        Args:
            config: Mission configuration dictionary

        Returns:
            List of validation error messages (empty if valid)
        """
        errors = []
        schema = self.schemas["mission"]

        # Check required fields
        for field in schema["required_fields"]:
            if field not in config:
                errors.append(f"Missing required mission field: {field}")

        # Validate mission type
        if "type" in config and config["type"] not in schema["valid_mission_types"]:
            errors.append(
                f"Invalid mission type '{config['type']}'. "
                f"Must be one of: {schema['valid_mission_types']}"
            )

        # Validate waypoints
        if "waypoints" in config:
            waypoints = config["waypoints"]
            if not isinstance(waypoints, list):
                errors.append("Waypoints must be a list")
            else:
                for i, waypoint in enumerate(waypoints):
                    if not isinstance(waypoint, dict):
                        errors.append(f"Waypoint {i} must be a dictionary")
                        continue

                    for field in schema["waypoint_fields"]:
                        if field not in waypoint:
                            errors.append(
                                f"Waypoint {i} missing required field: {field}"
                            )

                    # Validate coordinate ranges
                    if "x" in waypoint and not (-1000 <= waypoint["x"] <= 1000):
                        errors.append(
                            f"Waypoint {i} x coordinate {waypoint['x']} out of range"
                        )

                    if "y" in waypoint and not (-1000 <= waypoint["y"] <= 1000):
                        errors.append(
                            f"Waypoint {i} y coordinate {waypoint['y']} out of range"
                        )

        return errors

    def validate_file(self, config_file: str) -> Tuple[bool, List[str]]:
        """
        Validate a configuration file.

        Args:
            config_file: Path to configuration file

        Returns:
            Tuple of (is_valid, error_messages)
        """
        try:
            # Determine config type from filename
            config_path = Path(config_file)
            config_name = config_path.stem.lower()

            # Load configuration
            if config_path.suffix.lower() in [".yaml", ".yml"]:
                import yaml

                with open(config_path, "r") as f:
                    config = yaml.safe_load(f)
            elif config_path.suffix.lower() == ".json":
                import json

                with open(config_path, "r") as f:
                    config = json.load(f)
            else:
                return False, [
                    f"Unsupported configuration file format: {config_path.suffix}"
                ]

            # Validate based on config type
            if "hardware" in config_name or "interface" in config_name:
                errors = self.validate_hardware_config(config)
            elif "ros2" in config_name or "ros" in config_name:
                errors = self.validate_ros2_config(config)
            elif "mission" in config_name:
                errors = self.validate_mission_config(config)
            else:
                # Generic validation - just check it's a valid dict
                if not isinstance(config, dict):
                    errors = ["Configuration must be a dictionary"]
                else:
                    errors = []

            is_valid = len(errors) == 0
            return is_valid, errors

        except Exception as e:
            return False, [f"Configuration loading/parsing error: {e}"]

    def validate_directory(
        self, config_dir: Optional[str] = None
    ) -> Dict[str, List[str]]:
        """
        Validate all configuration files in a directory.

        Args:
            config_dir: Directory to validate (defaults to config dir)

        Returns:
            Dict mapping filenames to their validation errors
        """
        if config_dir is None:
            config_path = self.config_dir
        else:
            config_path = Path(config_dir)

        results = {}

        if not config_path.exists():
            return {str(config_path): ["Configuration directory does not exist"]}

        # Find all config files
        config_files = (
            list(config_path.glob("*.yaml"))
            + list(config_path.glob("*.yml"))
            + list(config_path.glob("*.json"))
        )

        for config_file in config_files:
            is_valid, errors = self.validate_file(str(config_file))
            if not is_valid:
                results[config_file.name] = errors

        return results

    def generate_default_configs(
        self, output_dir: Optional[str] = None
    ) -> Dict[str, str]:
        """
        Generate default configuration files.

        Args:
            output_dir: Directory to write configs to

        Returns:
            Dict mapping config names to file paths
        """
        if output_dir is None:
            output_path = self.config_dir
        else:
            output_path = Path(output_dir)
        output_path.mkdir(exist_ok=True)

        configs = {}

        # Hardware configuration
        hardware_config = {
            "hardware": {
                "can_bus": "mock",
                "motor_controller": "mock",
                "drive_system": "mock",
                "robotic_arm": "mock",
                "science_payload": "mock",
                "power_system": "mock",
                "sensor_imu": "mock",
                "sensor_gps": "mock",
            }
        }

        # ROS2 configuration
        ros2_config = {
            "ros2": {
                "domain_id": 42,
                "nodes": {
                    "state_machine_director": {
                        "enabled": True,
                        "parameters": {
                            "state_update_rate": 10.0,
                            "emergency_stop_enabled": True,
                        },
                    }
                },
                "topics": {"cmd_vel": {"qos": "reliable"}, "odom": {"qos": "reliable"}},
            }
        }

        # Mission configuration
        mission_config = {
            "name": "default_mission",
            "type": "waypoint_navigation",
            "waypoints": [
                {"x": 0.0, "y": 0.0, "heading": 0.0},
                {"x": 5.0, "y": 0.0, "heading": 0.0},
                {"x": 5.0, "y": 5.0, "heading": 1.57},
                {"x": 0.0, "y": 5.0, "heading": 3.14},
                {"x": 0.0, "y": 0.0, "heading": 0.0},
            ],
            "timeout": 300.0,
        }

        # Write configs
        import yaml

        configs["hardware_config.yaml"] = str(output_path / "hardware_config.yaml")
        with open(configs["hardware_config.yaml"], "w") as f:
            yaml.dump(hardware_config, f, default_flow_style=False)

        configs["ros2_config.yaml"] = str(output_path / "ros2_config.yaml")
        with open(configs["ros2_config.yaml"], "w") as f:
            yaml.dump(ros2_config, f, default_flow_style=False)

        configs["default_mission.yaml"] = str(output_path / "default_mission.yaml")
        with open(configs["default_mission.yaml"], "w") as f:
            yaml.dump(mission_config, f, default_flow_style=False)

        logger.info(f"Generated default configuration files in {output_dir}")
        return configs


def validate_all_configs(
    config_dir: Optional[str] = None, fail_on_errors: bool = True
) -> bool:
    """
    Validate all configurations in the config directory.

    Args:
        config_dir: Directory to validate
        fail_on_errors: If True, exit with error code on validation failures

    Returns:
        bool: True if all configs are valid
    """
    validator = ConfigurationValidator(config_dir)

    print("[MAGNIFY] Validating configuration files...")
    validation_results = validator.validate_directory()

    if validation_results:
        print("[FAIL] Configuration validation failed:")
        for filename, errors in validation_results.items():
            print(f"   {filename}:")
            for error in errors:
                print(f"    [FAIL] {error}")
        print()

        if fail_on_errors:
            print(" To generate default configurations, run:")
            print(
                '   python3 -c "from config.config_validator import '
                "ConfigurationValidator; "
                'ConfigurationValidator().generate_default_configs()"'
            )
            return False
    else:
        print("[PASS] All configuration files are valid")
        return True


if __name__ == "__main__":
    import sys

    # Validate all configs
    success = validate_all_configs(fail_on_errors=False)

    # Generate defaults if validation failed
    if not success:
        print("\n[TOOL] Generating default configuration files...")
        validator = ConfigurationValidator()
        configs = validator.generate_default_configs()
        print("[PASS] Default configurations generated:")
        for name, path in configs.items():
            print(f"   {name} -> {path}")

        # Re-validate
        print("\n[MAGNIFY] Re-validating generated configurations...")
        success = validate_all_configs()

    sys.exit(0 if success else 1)
