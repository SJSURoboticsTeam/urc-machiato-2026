#!/usr/bin/env python3
"""
Environment Configuration Manager

Centralizes all environment variable management for URC 2026.
Replaces hardcoded values with secure, configurable environment variables.

Environment Variables:
- URC_ROVER_ID: Unique rover identifier
- URC_API_HOST: API server host (default: localhost)
- URC_API_PORT: API server port (default: 5000)
- URC_WEBSOCKET_HOST: WebSocket server host (default: localhost)
- URC_WEBSOCKET_PORT: WebSocket server port (default: 8080)
- URC_REDIS_HOST: Redis server host (default: localhost)
- URC_REDIS_PORT: Redis server port (default: 6379)
- URC_DASHBOARD_PORT: Dashboard web server port (default: 3000)
- URC_LOG_LEVEL: Logging level (default: INFO)
- URC_ENVIRONMENT: Environment (development/production/competition)

Author: URC 2026 Configuration Team
"""

import os
from typing import Dict, Any, Optional
from dataclasses import dataclass
from pathlib import Path


@dataclass
class NetworkConfig:
    """Network configuration from environment variables."""

    api_host: str
    api_port: int
    websocket_host: str
    websocket_port: int
    redis_host: str
    redis_port: int
    dashboard_port: int

    @classmethod
    def from_env(cls) -> "NetworkConfig":
        """Create configuration from environment variables."""
        return cls(
            api_host=os.getenv("URC_API_HOST", "localhost"),
            api_port=int(os.getenv("URC_API_PORT", "5000")),
            websocket_host=os.getenv("URC_WEBSOCKET_HOST", "localhost"),
            websocket_port=int(os.getenv("URC_WEBSOCKET_PORT", "8080")),
            redis_host=os.getenv("URC_REDIS_HOST", "localhost"),
            redis_port=int(os.getenv("URC_REDIS_PORT", "6379")),
            dashboard_port=int(os.getenv("URC_DASHBOARD_PORT", "3000")),
        )


@dataclass
class SystemConfig:
    """System-wide configuration from environment variables."""

    rover_id: str
    environment: str
    log_level: str
    data_dir: Path

    @classmethod
    def from_env(cls) -> "SystemConfig":
        """Create configuration from environment variables."""
        return cls(
            rover_id=os.getenv("URC_ROVER_ID", "urc-rover-001"),
            environment=os.getenv("URC_ENVIRONMENT", "development"),
            log_level=os.getenv("URC_LOG_LEVEL", "INFO"),
            data_dir=Path(os.getenv("URC_DATA_DIR", "/var/lib/urc")),
        )


class EnvironmentManager:
    """Centralized environment variable management."""

    def __init__(self):
        self.network = NetworkConfig.from_env()
        self.system = SystemConfig.from_env()
        self._validate_environment()

    def _validate_environment(self):
        """Validate critical environment variables."""
        if self.system.environment not in ["development", "production", "competition"]:
            raise ValueError(f"Invalid environment: {self.system.environment}")

        valid_log_levels = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
        if self.system.log_level not in valid_log_levels:
            raise ValueError(f"Invalid log level: {self.system.log_level}")

        # Validate port ranges
        for port_name, port_value in [
            ("API_PORT", self.network.api_port),
            ("WEBSOCKET_PORT", self.network.websocket_port),
            ("REDIS_PORT", self.network.redis_port),
            ("DASHBOARD_PORT", self.network.dashboard_port),
        ]:
            if not (1 <= port_value <= 65535):
                raise ValueError(f"Invalid {port_name}: {port_value} (must be 1-65535)")

    def get_api_url(self) -> str:
        """Get full API URL."""
        return f"http://{self.network.api_host}:{self.network.api_port}"

    def get_websocket_url(self) -> str:
        """Get full WebSocket URL."""
        return f"ws://{self.network.websocket_host}:{self.network.websocket_port}"

    def get_redis_url(self) -> str:
        """Get full Redis URL."""
        return f"redis://{self.network.redis_host}:{self.network.redis_port}"

    def get_dashboard_url(self) -> str:
        """Get full Dashboard URL."""
        return f"http://localhost:{self.network.dashboard_port}"

    def is_production(self) -> bool:
        """Check if running in production."""
        return self.system.environment == "production"

    def is_competition(self) -> bool:
        """Check if running in competition mode."""
        return self.system.environment == "competition"

    def export_environment(self) -> Dict[str, Any]:
        """Export all environment variables as dictionary."""
        return {
            "network": {
                "api_host": self.network.api_host,
                "api_port": self.network.api_port,
                "websocket_host": self.network.websocket_host,
                "websocket_port": self.network.websocket_port,
                "redis_host": self.network.redis_host,
                "redis_port": self.network.redis_port,
                "dashboard_port": self.network.dashboard_port,
            },
            "system": {
                "rover_id": self.system.rover_id,
                "environment": self.system.environment,
                "log_level": self.system.log_level,
                "data_dir": str(self.system.data_dir),
            },
            "urls": {
                "api": self.get_api_url(),
                "websocket": self.get_websocket_url(),
                "redis": self.get_redis_url(),
                "dashboard": self.get_dashboard_url(),
            },
        }


# Global environment manager instance
_env_manager: Optional[EnvironmentManager] = None


def get_env_manager() -> EnvironmentManager:
    """Get the global environment manager instance."""
    global _env_manager
    if _env_manager is None:
        _env_manager = EnvironmentManager()
    return _env_manager


def get_network_config() -> NetworkConfig:
    """Get network configuration."""
    return get_env_manager().network


def get_system_config() -> SystemConfig:
    """Get system configuration."""
    return get_env_manager().system


def setup_environment_example():
    """Generate example .env file for development."""
    example_env = """# URC 2026 Environment Configuration
# Copy this file to .env and adjust values as needed

# Rover Identification
URC_ROVER_ID=urc-rover-001

# Environment
URC_ENVIRONMENT=development
URC_LOG_LEVEL=INFO

# Network Configuration
URC_API_HOST=localhost
URC_API_PORT=5000
URC_WEBSOCKET_HOST=localhost
URC_WEBSOCKET_PORT=8080
URC_REDIS_HOST=localhost
URC_REDIS_PORT=6379
URC_DASHBOARD_PORT=3000

# Data Storage
URC_DATA_DIR=/var/lib/urc
"""

    env_file = Path(".env.example")
    env_file.write_text(example_env)
    print(f"📝 Example environment file created: {env_file}")
    print("   Copy to '.env' and modify for your environment.")


if __name__ == "__main__":
    # Test the environment manager
    env_mgr = get_env_manager()

    print("🔧 Environment Configuration:")
    print(f"   Rover ID: {env_mgr.system.rover_id}")
    print(f"   Environment: {env_mgr.system.environment}")
    print(f"   Log Level: {env_mgr.system.log_level}")
    print()
    print(f"🌐 Network Configuration:")
    print(f"   API URL: {env_mgr.get_api_url()}")
    print(f"   WebSocket URL: {env_mgr.get_websocket_url()}")
    print(f"   Redis URL: {env_mgr.get_redis_url()}")
    print(f"   Dashboard URL: {env_mgr.get_dashboard_url()}")
