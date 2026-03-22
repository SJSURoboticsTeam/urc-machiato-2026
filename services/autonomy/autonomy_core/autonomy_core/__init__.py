"""
Autonomy Core Package - URC 2026

Subpackages load lazily so unit tests can import ``control`` / ``perception``
stubs without pulling in ROS-backed navigation when ``geometry_msgs`` is absent.
"""

from __future__ import annotations

import importlib
from typing import Any, List

__version__ = "2.0.0"

__all__: List[str] = []


def __getattr__(name: str) -> Any:
    if name in ("navigation", "safety", "control", "perception"):
        try:
            mod = importlib.import_module(f"{__name__}.{name}")
        except ImportError:
            raise AttributeError(name) from None
        globals()[name] = mod
        if name not in __all__:
            __all__.append(name)
        return mod
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> List[str]:
    return sorted(list(globals().keys()) + ["navigation", "safety", "control", "perception"])
