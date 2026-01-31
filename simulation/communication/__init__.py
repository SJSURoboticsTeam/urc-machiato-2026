"""Communication simulation package for URC competition testing.

Provides mission-specific communication profiles, network emulation,
and integrated testing capabilities for all URC missions.

Author: URC 2026 Autonomy Team
"""

from .mission_profiles import (
    MissionCommunicationManager,
    URCMission,
    URC_COMMUNICATION_PROFILES,
)

__all__ = [
    "MissionCommunicationManager",
    "URCMission",
    "URC_COMMUNICATION_PROFILES",
]
