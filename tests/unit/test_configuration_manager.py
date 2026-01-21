#!/usr/bin/env python3
"""
Configuration Manager Tests - URC 2026

Tests the ConfigurationManager system for:
- YAML/JSON loading and validation
- Environment switching (dev/test/prod)
- Parameter override handling
- Configuration schema validation
- Hot reloading capabilities

Author: URC 2026 Testing Team
"""

import os
import tempfile
import pytest
import yaml
import json
from pathlib import Path
from unittest.mock import Mock, patch, mock_open

from src.core.configuration_manager import (
    ConfigurationManager,
    get_config_manager,
    load_system_config,
    get_system_config,
    SystemConfig
)


class TestConfigurationManager:
    """Test ConfigurationManager functionality."""

    @pytest.fixture
    def temp_config_dir(self):
        """Create temporary config directory."""
        with tempfile.TemporaryDirectory() as temp_dir:
            yield Path(temp_dir)

    @pytest.fixture
    def sample_config_data(self):
        """Sample configuration data."""
        return {
            "system": {
                "name": "URC 2026 Rover",
                "version": "1.0.0",
                "environment": "testing"
            },
            "navigation": {
                "gps_timeout": 5.0,
                "imu_timeout": 1.0,
                "waypoint_tolerance": 0.5,
                "max_velocity": 2.0
            },
            "communication": {
                "websocket_port": 8080,
                "ros_domain_id": 42,
                "heartbeat_interval": 1.0
            },
            "sensors": {
                "imu_rate": 100,
                "gps_rate": 10,
                "lidar_rate": 20,
                "camera_fps": 30
            }
        }

    def test_configuration_manager_initialization(self, temp_config_dir):
        """Test ConfigurationManager initialization."""
        config_file = temp_config_dir / "config.yaml"
        config_file.write_text("system:\n  name: test\n")

        manager = ConfigurationManager(config_file.parent)

        assert manager.config_dir == config_file.parent
        assert manager.current_config is None
        assert isinstance(manager.config_cache, dict)

    def test_yaml_config_loading(self, temp_config_dir, sample_config_data):
        """Test YAML configuration file loading."""
        config_file = temp_config_dir / "config.yaml"
        with open(config_file, 'w') as f:
            yaml.dump(sample_config_data, f)

        manager = ConfigurationManager(temp_config_dir)
        config = manager.load_config("test")

        assert config is not None
        assert config["system"]["name"] == "URC 2026 Rover"
        assert config["navigation"]["max_velocity"] == 2.0
        assert config["communication"]["websocket_port"] == 8080

    def test_json_config_loading(self, temp_config_dir, sample_config_data):
        """Test JSON configuration file loading."""
        config_file = temp_config_dir / "config.json"
        with open(config_file, 'w') as f:
            json.dump(sample_config_data, f)

        manager = ConfigurationManager(temp_config_dir)
        config = manager.load_config("test")

        assert config is not None
        assert config["system"]["name"] == "URC 2026 Rover"

    def test_environment_config_switching(self, temp_config_dir, sample_config_data):
        """Test switching between different environments."""
        # Create different environment configs
        environments = {
            "development": {**sample_config_data, "system": {**sample_config_data["system"], "environment": "dev"}},
            "testing": {**sample_config_data, "system": {**sample_config_data["system"], "environment": "test"}},
            "production": {**sample_config_data, "system": {**sample_config_data["system"], "environment": "prod"}}
        }

        for env, config_data in environments.items():
            config_file = temp_config_dir / f"{env}.yaml"
            with open(config_file, 'w') as f:
                yaml.dump(config_data, f)

        manager = ConfigurationManager(temp_config_dir)

        # Test loading different environments
        dev_config = manager.load_config("development")
        test_config = manager.load_config("testing")
        prod_config = manager.load_config("production")

        assert dev_config["system"]["environment"] == "dev"
        assert test_config["system"]["environment"] == "test"
        assert prod_config["system"]["environment"] == "prod"

    def test_config_validation(self, temp_config_dir):
        """Test configuration validation."""
        # Valid config
        valid_config = {
            "system": {"name": "Test", "version": "1.0.0"},
            "navigation": {"gps_timeout": 5.0},
            "communication": {"websocket_port": 8080}
        }

        # Invalid config (missing required fields)
        invalid_config = {
            "system": {"name": "Test"}  # Missing version
        }

        config_file = temp_config_dir / "config.yaml"

        # Test valid config
        with open(config_file, 'w') as f:
            yaml.dump(valid_config, f)

        manager = ConfigurationManager(temp_config_dir)
        config = manager.load_config("test")
        errors = manager.validate_config(config)
        assert len(errors) == 0  # Should be valid

        # Test invalid config
        with open(config_file, 'w') as f:
            yaml.dump(invalid_config, f)

        config = manager.load_config("test")
        errors = manager.validate_config(config)
        assert len(errors) > 0  # Should have validation errors

    def test_parameter_override(self, temp_config_dir, sample_config_data):
        """Test parameter override functionality."""
        config_file = temp_config_dir / "config.yaml"
        with open(config_file, 'w') as f:
            yaml.dump(sample_config_data, f)

        manager = ConfigurationManager(temp_config_dir)
        config = manager.load_config("test")

        # Test parameter override
        manager.set_override("navigation.max_velocity", 3.0)
        assert manager.get_parameter("navigation.max_velocity") == 3.0

        # Test nested parameter access
        assert manager.get_parameter("communication.websocket_port") == 8080

        # Test parameter with default
        assert manager.get_parameter("nonexistent.param", "default") == "default"

    def test_config_caching(self, temp_config_dir, sample_config_data):
        """Test configuration caching."""
        config_file = temp_config_dir / "config.yaml"
        with open(config_file, 'w') as f:
            yaml.dump(sample_config_data, f)

        manager = ConfigurationManager(temp_config_dir)

        # Load config twice - should use cache
        config1 = manager.load_config("test")
        config2 = manager.load_config("test")

        assert config1 is config2  # Should be the same cached object

    def test_missing_config_file_handling(self, temp_config_dir):
        """Test handling of missing configuration files."""
        manager = ConfigurationManager(temp_config_dir)

        # Try to load non-existent config
        config = manager.load_config("nonexistent")
        assert config == {}  # Should return empty dict

    def test_config_hot_reload(self, temp_config_dir, sample_config_data):
        """Test configuration hot reloading."""
        config_file = temp_config_dir / "config.yaml"
        with open(config_file, 'w') as f:
            yaml.dump(sample_config_data, f)

        manager = ConfigurationManager(temp_config_dir)
        config1 = manager.load_config("test")

        # Modify config file
        sample_config_data["system"]["version"] = "2.0.0"
        with open(config_file, 'w') as f:
            yaml.dump(sample_config_data, f)

        # Force reload
        manager.clear_cache()
        config2 = manager.load_config("test")

        assert config2["system"]["version"] == "2.0.0"

    def test_environment_variable_override(self, temp_config_dir, sample_config_data):
        """Test environment variable overrides."""
        config_file = temp_config_dir / "config.yaml"
        with open(config_file, 'w') as f:
            yaml.dump(sample_config_data, f)

        manager = ConfigurationManager(temp_config_dir)

        # Set environment variable
        os.environ["ROVER_NAVIGATION_MAX_VELOCITY"] = "2.5"

        config = manager.load_config("test")

        # Environment variable should override config file
        assert config["navigation"]["max_velocity"] == 2.5

        # Clean up
        del os.environ["ROVER_NAVIGATION_MAX_VELOCITY"]

    def test_configuration_schema_validation(self, temp_config_dir):
        """Test configuration schema validation."""
        # Define schema
        schema = {
            "system": {
                "name": {"type": "string", "required": True},
                "version": {"type": "string", "required": True},
                "environment": {"type": "string", "enum": ["dev", "test", "prod"]}
            },
            "navigation": {
                "gps_timeout": {"type": "number", "min": 0, "max": 60},
                "max_velocity": {"type": "number", "min": 0, "max": 5}
            }
        }

        # Valid config
        valid_config = {
            "system": {"name": "Test", "version": "1.0.0", "environment": "test"},
            "navigation": {"gps_timeout": 5.0, "max_velocity": 2.0}
        }

        # Invalid config
        invalid_config = {
            "system": {"name": "Test", "version": "1.0.0", "environment": "invalid"},
            "navigation": {"gps_timeout": -1, "max_velocity": 10}
        }

        config_file = temp_config_dir / "config.yaml"

        # Test schema validation
        with open(config_file, 'w') as f:
            yaml.dump(valid_config, f)

        manager = ConfigurationManager(temp_config_dir)
        config = manager.load_config("test")
        errors = manager.validate_schema(config, schema)
        assert len(errors) == 0

        with open(config_file, 'w') as f:
            yaml.dump(invalid_config, f)

        config = manager.load_config("test")
        errors = manager.validate_schema(config, schema)
        assert len(errors) > 0


class TestConfigurationManagerIntegration:
    """Integration tests for ConfigurationManager."""

    def test_global_config_manager_singleton(self):
        """Test that get_config_manager returns singleton."""
        manager1 = get_config_manager()
        manager2 = get_config_manager()

        assert manager1 is manager2

    def test_load_system_config_integration(self):
        """Test load_system_config function."""
        config = load_system_config("development")

        # Should return a SystemConfig object or dict
        assert config is not None
        assert isinstance(config, (dict, SystemConfig))

    def test_get_system_config_integration(self):
        """Test get_system_config function."""
        config = get_system_config()

        assert config is not None
        assert isinstance(config, (dict, SystemConfig))

    def test_configuration_persistence(self, tmp_path):
        """Test configuration persistence across sessions."""
        config_data = {"test": {"value": 123}}
        config_file = tmp_path / "persistent_config.yaml"

        with open(config_file, 'w') as f:
            yaml.dump(config_data, f)

        # First session
        manager1 = ConfigurationManager(tmp_path)
        config1 = manager1.load_config("persistent")

        # Second session (simulated)
        manager2 = ConfigurationManager(tmp_path)
        config2 = manager2.load_config("persistent")

        assert config1["test"]["value"] == config2["test"]["value"] == 123


class TestConfigurationErrorHandling:
    """Test error handling in configuration system."""

    def test_malformed_yaml_handling(self, temp_config_dir):
        """Test handling of malformed YAML."""
        config_file = temp_config_dir / "bad_config.yaml"
        config_file.write_text("invalid: yaml: content: [\n")

        manager = ConfigurationManager(temp_config_dir)

        with pytest.raises(Exception):  # Should raise YAML parsing error
            manager.load_config("bad")

    def test_malformed_json_handling(self, temp_config_dir):
        """Test handling of malformed JSON."""
        config_file = temp_config_dir / "bad_config.json"
        config_file.write_text('{"invalid": json content}')

        manager = ConfigurationManager(temp_config_dir)

        with pytest.raises(Exception):  # Should raise JSON parsing error
            manager.load_config("bad")

    def test_file_permission_error_handling(self, temp_config_dir):
        """Test handling of file permission errors."""
        config_file = temp_config_dir / "no_perm_config.yaml"
        config_file.write_text("test: data\n")

        # Remove read permission
        os.chmod(config_file, 0o000)

        try:
            manager = ConfigurationManager(temp_config_dir)

            with pytest.raises(Exception):  # Should raise permission error
                manager.load_config("no_perm")
        finally:
            # Restore permissions for cleanup
            os.chmod(config_file, 0o644)

    def test_network_config_timeout(self):
        """Test handling of network configuration timeouts."""
        # This would test remote config loading timeouts
        # For now, test the framework exists
        manager = get_config_manager()

        # Should handle timeout gracefully
        assert manager is not None



