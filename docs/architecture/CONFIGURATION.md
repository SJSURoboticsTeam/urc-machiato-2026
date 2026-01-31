# Configuration Architecture

## Overview

Configuration is provided by two mechanisms: a typed Pydantic-based config (canonical API) and an environment-based Dynaconf loader for merged, environment-specific settings.

## Canonical API: config_manager

**Module:** `src/core/config_manager.py`

- **RoverConfig** (Pydantic) holds typed configuration with validation.
- **get_config_manager()** / **get_system_config()** / **load_system_config()** are the main entry points.
- Loads from `ROVER_CONFIG_FILE` or a configured path; supports JSON (and YAML where used).
- Use this when you need typed config (RoverConfig, SyncConfig, NetworkConfig, etc.) or the existing API.

## Unified Loading: Dynaconf

**Module:** `src/config/dynaconf_config.py`

- **get_urc_config()** returns a **URCDynaconf** instance (class-based, with validators).
- **get_settings()** returns a single Dynaconf instance for environment-based loading.
- **Loading order:** rover.yaml (base) -> {URC_ENV}.yaml (e.g. development, competition) -> local.yaml -> environment variables (URC_*).
- Use when you need environment-based overrides or a single merged settings object.

**Files:**
- `config/rover.yaml`: Base defaults.
- `config/development.yaml`: Development overrides.
- `config/competition.yaml`: Competition overrides.
- `config/local.yaml`: Local overrides (typically gitignored).

## When to Use Which

| Need                         | Use |
|-----------------------------|-----|
| Typed config (RoverConfig)   | config_manager.get_system_config() |
| Environment-based merge     | dynaconf_config.get_urc_config() or get_settings() |
| Single key (e.g. from env)  | get_urc_config().get(key) or get_settings()[key] |

## Component Configs

Component-specific YAML (e.g. SLAM, sensor bridge, LED) live in their packages and are loaded by launch files or nodes. They are not replaced by the central config; they remain the source for those components.
