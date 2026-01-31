#!/usr/bin/env python3
"""
Configuration Manager Tests - URC 2026

Tests the consolidated config_manager API:
- get_config(), get_config_manager(), load_system_config(), get_system_config()
- ConfigurationManager wrapper (current_config, get_config(), update_config())
- RoverConfig / SystemConfig attributes

Author: URC 2026 Testing Team
"""

import tempfile
import pytest
from pathlib import Path

from src.infrastructure.config import (
    ConfigurationManager,
    get_config,
    get_config_manager,
    get_system_config,
    load_system_config,
    reload_config,
    RoverConfig,
)


class TestConfigurationManager:
    """Test ConfigurationManager (wrapper) and config_manager API."""

    def test_configuration_manager_initialization(self):
        """ConfigurationManager accepts config_dir and exposes get_config."""
        manager = ConfigurationManager("config")
        assert manager._config_dir == Path("config")
        config = manager.get_config()
        assert config is not None

    def test_current_config_returns_rover_config(self):
        """current_config property returns same as get_config()."""
        manager = ConfigurationManager("config")
        assert manager.current_config is not None
        assert manager.current_config is manager.get_config()
        assert isinstance(manager.current_config, RoverConfig)

    def test_load_config_returns_config(self):
        """load_config(environment) returns current config."""
        manager = ConfigurationManager("config")
        config = manager.load_config("development")
        assert config is not None
        assert isinstance(config, RoverConfig)

    def test_update_config_returns_bool(self):
        """update_config(updates) returns bool (reloads global config)."""
        manager = ConfigurationManager("config")
        result = manager.update_config({})
        assert result is True


class TestConfigManagerIntegration:
    """Integration tests for get_config_manager and global config."""

    def test_get_config_manager_singleton(self):
        """get_config_manager returns same instance."""
        m1 = get_config_manager()
        m2 = get_config_manager()
        assert m1 is m2

    def test_get_config_returns_rover_config(self):
        """get_config() returns RoverConfig."""
        config = get_config()
        assert config is not None
        assert isinstance(config, RoverConfig)
        assert hasattr(config, "environment")
        assert hasattr(config, "network")
        assert hasattr(config, "mission")
        assert hasattr(config, "title")
        assert hasattr(config, "navigation")

    def test_get_system_config_returns_config(self):
        """get_system_config() returns same as get_config()."""
        c1 = get_system_config()
        c2 = get_config()
        assert c1 is c2

    def test_load_system_config_returns_config(self):
        """load_system_config(env) returns config."""
        config = load_system_config("development")
        assert config is not None
        assert isinstance(config, RoverConfig)

    def test_system_config_alias(self):
        """get_system_config() returns RoverConfig (SystemConfig alias)."""
        config = get_system_config()
        assert isinstance(config, RoverConfig)
        assert get_system_config() is get_config()


class TestRoverConfigAttributes:
    """Test RoverConfig / SystemConfig attributes used by dashboard and registry."""

    def test_rover_config_has_title_and_navigation(self):
        """RoverConfig has title, enable_real_time, refresh_interval, navigation."""
        config = get_config()
        assert config.title == "URC 2026 Rover"
        assert config.enable_real_time is True
        assert config.refresh_interval == 1.0
        assert hasattr(config.navigation, "update_rate_hz")
        assert hasattr(config.navigation, "waypoint_tolerance_m")

    def test_rover_config_network_has_ports(self):
        """RoverConfig.network has websocket_port, http_port, enable_cors, max_connections."""
        config = get_config()
        assert hasattr(config.network, "websocket_port")
        assert hasattr(config.network, "http_port")
        assert hasattr(config.network, "enable_cors")
        assert hasattr(config.network, "max_connections")

    def test_component_registry_config_sections(self):
        """Config has sections used by component_registry (sync, network, safety, mission)."""
        config = get_config()
        assert getattr(config, "sync", None) is not None
        assert getattr(config, "network", None) is not None
        assert getattr(config, "safety", None) is not None
        assert getattr(config, "mission", None) is not None


class TestConfigManagerExports:
    """Test that config_manager exports work (canonical module)."""

    def test_config_manager_imports(self):
        """from src.infrastructure.config import ... works."""
        from src.infrastructure.config import (
            ConfigurationManager,
            RoverConfig,
            get_config_manager,
            get_system_config,
            load_system_config,
            PerformanceConfig,
        )
        assert get_config_manager() is not None
        assert get_system_config() is not None
        assert load_system_config("development") is not None
        assert get_system_config() is get_config()

    def test_config_manager_wrapper(self):
        """get_config_manager().get_config() returns same as get_config()."""
        from src.infrastructure.config import get_config_manager, get_config, load_system_config
        m = get_config_manager()
        assert m.get_config() is get_config()
        assert load_system_config("test") is not None
