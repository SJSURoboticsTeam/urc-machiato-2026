"""Phase A: Isaac-related code paths without GPU / omni install (stub omni.* and deps)."""

from __future__ import annotations

import importlib.util
import logging
import sys
from pathlib import Path
from types import ModuleType
from typing import Dict, Iterator
from unittest.mock import MagicMock

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
ISAAC_DIR = REPO_ROOT / "services" / "simulation" / "python" / "isaac"


@pytest.fixture
def clean_sys_modules() -> Iterator[None]:
    """Remove injected stub / dynamically loaded modules after each test."""
    preserved = set(sys.modules.keys())
    yield
    for name in list(sys.modules.keys()):
        if name in preserved:
            continue
        if (
            name == "omni"
            or name.startswith("omni.")
            or name == "pxr"
            or name.startswith("simulation")
            or name.startswith("isaac_")
        ):
            del sys.modules[name]


def _ensure_module(name: str) -> ModuleType:
    mod = ModuleType(name)
    sys.modules[name] = mod
    return mod


def _install_omni_and_pxr_stubs() -> None:
    """Minimal omni.isaac.* and pxr stubs for MarsSceneSetup.initialize_isaac."""
    omni = _ensure_module("omni")
    omni.isaac = _ensure_module("omni.isaac")
    omni.isaac.kit = _ensure_module("omni.isaac.kit")

    class _SimulationApp:
        def __init__(self, config: Dict) -> None:
            self.config = config

        def close(self) -> None:
            return None

    omni.isaac.kit.SimulationApp = _SimulationApp

    omni.isaac.core = _ensure_module("omni.isaac.core")

    class _World:
        def __init__(self, **kwargs) -> None:
            pass

    omni.isaac.core.World = _World

    omni_usd = _ensure_module("omni.usd")
    stage = MagicMock()
    ctx = MagicMock()
    ctx.get_stage.return_value = stage
    omni_usd.get_context = lambda: ctx
    omni.usd = omni_usd

    pxr = _ensure_module("pxr")
    for attr in ("Usd", "UsdGeom", "Gf", "Sdf"):
        setattr(pxr, attr, MagicMock())


def _install_simulation_logging_stub() -> None:
    """Stub simulation.core.logging_config for isaac modules loaded from file."""
    sim = _ensure_module("simulation")
    sim.core = _ensure_module("simulation.core")
    log_cfg = _ensure_module("simulation.core.logging_config")

    def get_simulation_logger(name: str) -> logging.Logger:
        return logging.getLogger(name)

    log_cfg.get_simulation_logger = get_simulation_logger  # type: ignore[attr-defined]


def _load_module_from_path(module_name: str, path: Path):
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.unit
def test_mars_scene_setup_initialize_isaac_with_omni_stubs(clean_sys_modules) -> None:
    """MarsSceneSetup.initialize_isaac succeeds when omni.* resolves to stubs."""
    _install_omni_and_pxr_stubs()
    mod = _load_module_from_path(
        "isaac_setup_mars_scene", ISAAC_DIR / "setup_mars_scene.py"
    )
    setup = mod.MarsSceneSetup(headless=True)
    assert setup.initialize_isaac() is True
    assert setup.World is not None
    assert setup.simulation_app is not None


@pytest.mark.unit
def test_isaac_sensor_simulation_fallback_without_world(clean_sys_modules) -> None:
    """IsaacSensorSimulation runs fallback path when world is None."""
    _install_simulation_logging_stub()
    mod = _load_module_from_path(
        "isaac_sensor_simulation", ISAAC_DIR / "sensor_simulation.py"
    )
    sim = mod.IsaacSensorSimulation(None, {"sensors_enabled": {"camera": True}})
    assert sim.initialize() is True
    assert sim.is_initialized is True
