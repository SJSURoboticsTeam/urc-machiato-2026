#!/usr/bin/env python3
"""
Configuration Loader for Simulation Framework

Loads and validates YAML configuration files for simulation components.

Author: URC 2026 Configuration Team
"""

import yaml
import os
from pathlib import Path
from typing import Dict, Any, Optional
import logging

logger = logging.getLogger(__name__)


class ConfigurationError(Exception):
    """Configuration loading or validation error."""
    pass


class ConfigLoader:
    """Load and validate simulation configuration files."""
    
    def __init__(self, config_dir: Optional[str] = None):
        """Initialize configuration loader.
        
        Args:
            config_dir: Directory containing config files (defaults to simulation/config)
        """
        if config_dir is None:
            # Default to simulation/config directory
            config_dir = Path(__file__).parent
        
        self.config_dir = Path(config_dir)
        
        if not self.config_dir.exists():
            raise ConfigurationError(f"Config directory not found: {self.config_dir}")
    
    def load_config(self, config_name: str) -> Dict[str, Any]:
        """Load configuration file.
        
        Args:
            config_name: Name of config file (with or without .yaml extension)
            
        Returns:
            Configuration dictionary
        """
        # Add .yaml extension if not present
        if not config_name.endswith('.yaml'):
            config_name = f"{config_name}.yaml"
        
        config_path = self.config_dir / config_name
        
        if not config_path.exists():
            raise ConfigurationError(f"Config file not found: {config_path}")
        
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            logger.info(f"Loaded configuration from {config_name}")
            return config or {}
            
        except yaml.YAMLError as e:
            raise ConfigurationError(f"Invalid YAML in {config_name}: {e}")
        except Exception as e:
            raise ConfigurationError(f"Error loading {config_name}: {e}")
    
    def load_full_stack_config(self, profile: str = 'full_stack') -> Dict[str, Any]:
        """Load full stack simulator configuration.
        
        Args:
            profile: Configuration profile name
            
        Returns:
            Full stack configuration dictionary
        """
        config_name = f"{profile}_config"
        return self.load_config(config_name)
    
    def load_scenarios(self) -> Dict[str, Any]:
        """Load test scenario configurations.
        
        Returns:
            Scenarios dictionary
        """
        return self.load_config('scenarios')
    
    def validate_config(self, config: Dict[str, Any], schema: Dict[str, Any]) -> bool:
        """Validate configuration against schema.
        
        Args:
            config: Configuration dictionary
            schema: Schema dictionary with required fields
            
        Returns:
            True if valid
            
        Raises:
            ConfigurationError: If validation fails
        """
        for key, required_type in schema.items():
            if key not in config:
                raise ConfigurationError(f"Missing required field: {key}")
            
            if not isinstance(config[key], required_type):
                raise ConfigurationError(
                    f"Invalid type for {key}: expected {required_type}, got {type(config[key])}"
                )
        
        return True
    
    def merge_configs(self, base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
        """Merge two configurations (override takes precedence).
        
        Args:
            base: Base configuration
            override: Override configuration
            
        Returns:
            Merged configuration
        """
        result = base.copy()
        
        for key, value in override.items():
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                # Recursively merge nested dicts
                result[key] = self.merge_configs(result[key], value)
            else:
                result[key] = value
        
        return result
    
    def list_available_configs(self) -> list:
        """List all available configuration files.
        
        Returns:
            List of config file names
        """
        configs = list(self.config_dir.glob("*.yaml"))
        return [c.stem for c in configs]
    
    def get_config_path(self, config_name: str) -> Path:
        """Get full path to configuration file.
        
        Args:
            config_name: Config file name
            
        Returns:
            Path to config file
        """
        if not config_name.endswith('.yaml'):
            config_name = f"{config_name}.yaml"
        
        return self.config_dir / config_name


def load_simulation_config(profile: str = 'full_stack') -> Dict[str, Any]:
    """Convenience function to load simulation configuration.
    
    Args:
        profile: Configuration profile (full_stack, perfect, stressed, hil)
        
    Returns:
        Configuration dictionary
    """
    loader = ConfigLoader()
    return loader.load_full_stack_config(profile)


def load_scenario_config(scenario_name: str) -> Dict[str, Any]:
    """Load specific scenario configuration.
    
    Args:
        scenario_name: Name of scenario
        
    Returns:
        Scenario configuration
    """
    loader = ConfigLoader()
    scenarios = loader.load_scenarios()
    
    if 'scenarios' in scenarios and scenario_name in scenarios['scenarios']:
        return scenarios['scenarios'][scenario_name]
    
    raise ConfigurationError(f"Scenario not found: {scenario_name}")


if __name__ == '__main__':
    # Test configuration loader
    print("Testing Configuration Loader...")
    
    loader = ConfigLoader()
    
    # List available configs
    print(f"\n📁 Available configurations:")
    for config in loader.list_available_configs():
        print(f"  • {config}")
    
    # Load full stack config
    print(f"\n⚙️  Loading full_stack_config...")
    config = loader.load_full_stack_config('full_stack')
    print(f"  ✅ Loaded with {len(config)} sections")
    print(f"  Sections: {list(config.keys())}")
    
    # Load scenarios
    print(f"\n🎬 Loading scenarios...")
    scenarios = loader.load_scenarios()
    if 'scenarios' in scenarios:
        print(f"  ✅ Found {len(scenarios['scenarios'])} scenarios")
        for name in scenarios['scenarios'].keys():
            print(f"    • {name}")
    
    # Test merge
    print(f"\n🔀 Testing config merge...")
    base = {'a': 1, 'b': {'c': 2}}
    override = {'b': {'d': 3}, 'e': 4}
    merged = loader.merge_configs(base, override)
    print(f"  ✅ Merged: {merged}")
    
    print(f"\n✅ All configuration tests passed!")
