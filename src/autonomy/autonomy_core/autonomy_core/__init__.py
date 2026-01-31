"""
Autonomy Core Package - URC 2026

Unified package containing:
- Navigation: Path planning, motion control, GNSS processing
- Safety: Emergency stop, watchdog, proximity monitoring
- Control: Hardware interfaces, motor control
- Perception: Computer vision, SLAM, sensor simulation

Author: URC 2026 Team
"""

__version__ = "2.0.0"

# Re-export submodules for easy access
from . import navigation
from . import safety
from . import control
from . import perception

__all__ = [
    "navigation",
    "safety",
    "control",
    "perception",
]
