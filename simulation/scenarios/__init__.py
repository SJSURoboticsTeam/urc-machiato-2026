"""Scenarios package for competition day simulation.

Provides realistic competition day scenarios with combined stressors
and environmental challenges for URC mission testing.

Author: URC 2026 Autonomy Team
"""

from .competition_day_scenarios import (
    CompetitionDay,
    CompetitionDayManager,
    CompetitionScenario,
)

__all__ = [
    "CompetitionDay",
    "CompetitionDayManager",
    "CompetitionScenario",
]
