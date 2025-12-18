#!/usr/bin/env python3
"""
Configuration Manager Tests - Competition Critical
Tests the configuration management system for competition reliability.
"""

import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml


class TestConfigurationManager(unittest.TestCase):
    """Test configuration manager functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.config_dir = Path(self.temp_dir) / "config"
        self.config_dir.mkdir()

        # Import after setting up paths
        import sys

        sys.path.insert(0, str(Path(__file__).parent.parent.parent))

        from config.config_manager import ConfigurationManager

        self.manager = ConfigurationManager(str(self.config_dir))

    def tearDown(self):
        """Clean up test fixtures."""
        import shutil

        shutil.rmtree(self.temp_dir)

    def test_config_loading_priority(self):
        """Test configuration loading with proper priority."""
        # Create test config files
        base_config = {
            "simulation": {"update_rate_hz": 10},
            "mission": {"timeout_seconds": 300},
        }

        local_config = {
            "simulation": {"update_rate_hz": 20},  # Override
            "safety": {"emergency_stop_enabled": True},  # New section
        }

        competition_config = {
            "simulation": {"update_rate_hz": 50},  # Highest priority
            "performance": {"monitoring_enabled": True},  # Competition specific
        }

        # Write config files
        with open(self.config_dir / "rover.yaml", "w") as f:
            yaml.dump(base_config, f)

        with open(self.config_dir / "local.yaml", "w") as f:
            yaml.dump(local_config, f)

        with open(self.config_dir / "competition.yaml", "w") as f:
            yaml.dump(competition_config, f)

        # Test loading
        config = self.manager.load_config("competition")

        # Should have competition overrides
        self.assertEqual(config["simulation"]["update_rate_hz"], 50)
        self.assertTrue(config["performance"]["monitoring_enabled"])

        # Should have local additions
        self.assertTrue(config["safety"]["emergency_stop_enabled"])

        # Should have base config
        self.assertEqual(config["mission"]["timeout_seconds"], 300)

    def test_config_validation(self):
        """Test configuration validation against schemas."""
        # Valid config
        valid_config = {
            "simulation": {
                "simulation_update_rate_hz": 20,
                "simulation_odom_noise": 0.01,
                "simulation_imu_noise": 0.001,
                "simulation_gps_noise": 0.5,
            },
            "mission": {
                "mission_execution_rate_hz": 15,
                "mission_timeout_seconds": 600,
                "mission_max_speed_mps": 1.2,
            },
        }

        self.manager.save_config(valid_config, "test_valid")
        self.assertTrue(self.manager.validate_config("test_valid"))

        # Invalid config - out of range
        invalid_config = {
            "simulation": {
                "simulation_update_rate_hz": 200,  # Too high
                "simulation_odom_noise": 0.01,
                "simulation_imu_noise": 0.001,
                "simulation_gps_noise": 0.5,
            }
        }

        self.manager.save_config(invalid_config, "test_invalid")
        self.assertFalse(self.manager.validate_config("test_invalid"))

        # Check error messages
        errors = self.manager.get_validation_errors()
        self.assertTrue(len(errors) > 0)
        self.assertIn("Too high", str(errors))

    def test_config_get_set(self):
        """Test getting and setting configuration values."""
        config = {
            "simulation": {"update_rate_hz": 30, "noise": {"odom": 0.02, "imu": 0.002}}
        }

        self.manager.save_config(config, "test_get_set")

        # Test simple get
        value = self.manager.get_value(
            "simulation.update_rate_hz", environment="test_get_set"
        )
        self.assertEqual(value, 30)

        # Test nested get
        value = self.manager.get_value(
            "simulation.noise.odom", environment="test_get_set"
        )
        self.assertEqual(value, 0.02)

        # Test set
        self.manager.set_value(
            "simulation.update_rate_hz", 45, environment="test_get_set"
        )

        # Verify change
        value = self.manager.get_value(
            "simulation.update_rate_hz", environment="test_get_set"
        )
        self.assertEqual(value, 45)

        # Test default values
        value = self.manager.get_value("nonexistent.key", default="default_value")
        self.assertEqual(value, "default_value")

    def test_environment_detection(self):
        """Test environment detection and switching."""
        from config.config_manager import get_config, get_environment

        # Test default environment
        with patch.dict(os.environ, {}, clear=True):
            env = get_environment()
            self.assertEqual(env, "development")

        # Test competition environment
        with patch.dict(os.environ, {"URC_ENV": "competition"}):
            env = get_environment()
            self.assertEqual(env, "competition")

        # Test config retrieval with environment
        config = {"simulation": {"update_rate_hz": 25}}
        self.manager.save_config(config, "test_env")

        with patch.dict(os.environ, {"URC_ENV": "test_env"}):
            value = get_config("simulation.update_rate_hz")
            self.assertEqual(value, 25)

    def test_competition_config_creation(self):
        """Test creation of competition-ready configuration."""
        from config.config_manager import create_competition_config

        comp_config = create_competition_config()

        # Check required sections exist
        required_sections = [
            "simulation",
            "mission",
            "safety",
            "network",
            "performance",
        ]
        for section in required_sections:
            self.assertIn(section, comp_config)

        # Check competition-specific values
        self.assertEqual(comp_config["simulation"]["simulation_update_rate_hz"], 20)
        self.assertEqual(comp_config["mission"]["mission_max_speed_mps"], 1.5)
        self.assertTrue(comp_config["safety"]["safety_emergency_stop_enabled"])
        self.assertTrue(comp_config["performance"]["performance_monitoring_enabled"])

    def test_config_persistence(self):
        """Test configuration persistence across loads."""
        original_config = {
            "test": {
                "value1": "test_string",
                "value2": 42,
                "value3": [1, 2, 3],
                "nested": {"key": "nested_value"},
            }
        }

        # Save config
        self.manager.save_config(original_config, "persistence_test")

        # Load config
        loaded_config = self.manager.load_config("persistence_test")

        # Verify all data types preserved
        self.assertEqual(loaded_config["test"]["value1"], "test_string")
        self.assertEqual(loaded_config["test"]["value2"], 42)
        self.assertEqual(loaded_config["test"]["value3"], [1, 2, 3])
        self.assertEqual(loaded_config["test"]["nested"]["key"], "nested_value")

    def test_invalid_config_handling(self):
        """Test handling of invalid configuration files."""
        # Create invalid YAML
        invalid_yaml_path = self.config_dir / "invalid.yaml"
        with open(invalid_yaml_path, "w") as f:
            f.write("invalid: yaml: content: [\n")  # Invalid YAML

        # Should not crash, should return empty or base config
        config = self.manager.load_config("invalid")
        self.assertIsInstance(config, dict)

    def test_schema_validation_comprehensive(self):
        """Test comprehensive schema validation."""
        # Test all schema types
        test_configs = {
            "number_validation": {
                "simulation": {
                    "simulation_update_rate_hz": 150,  # Too high
                    "simulation_odom_noise": -0.1,  # Too low
                }
            },
            "boolean_validation": {
                "safety": {"safety_emergency_stop_enabled": "not_boolean"}  # Wrong type
            },
            "string_validation": {"mission": {"mission_type": 123}},  # Should be string
        }

        for test_name, config in test_configs.items():
            self.manager.save_config(config, f"test_{test_name}")
            is_valid = self.manager.validate_config(f"test_{test_name}")
            self.assertFalse(is_valid, f"Config {test_name} should be invalid")

            errors = self.manager.get_validation_errors()
            self.assertTrue(
                len(errors) > 0, f"Should have validation errors for {test_name}"
            )


if __name__ == "__main__":
    unittest.main()
