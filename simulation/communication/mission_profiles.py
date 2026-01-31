"""Mission-specific communication profiles for URC competition testing.

Defines communication patterns, bandwidth requirements, and testing
scenarios for each of the 4 core URC missions.

Author: URC 2026 Autonomy Team
"""

import logging
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Any

from simulation.network.network_emulator import NetworkProfile, NETWORK_PROFILES


class URCMission(Enum):
    """URC 2026 competition missions."""

    AUTONOMOUS_TRAVERSAL = "autonomous_traversal"
    SCIENCE_MISSION = "science_mission"
    EQUIPMENT_SERVICING = "equipment_servicing"
    DELIVERY_MISSION = "delivery_mission"


@dataclass
class CommunicationRequirement:
    """Communication requirement definition."""

    bandwidth_requirement: str  # "low", "medium", "high", "critical"
    latency_tolerance: str  # "tight", "moderate", "relaxed"
    message_frequency: str  # Messages per second
    data_types: List[str]
    critical_path: bool  # Critical for mission success
    failure_tolerance: str  # "none", "minimal", "moderate"


@dataclass
class MissionCommunicationProfile:
    """Complete communication profile for a URC mission."""

    mission: URCMission
    time_limit_minutes: int
    communication_requirements: Dict[str, CommunicationRequirement]
    network_profile: NetworkProfile
    special_constraints: List[str]
    testing_scenarios: List[str]


# URC Mission Communication Profiles
URC_COMMUNICATION_PROFILES: Dict[URCMission, MissionCommunicationProfile] = {
    URCMission.AUTONOMOUS_TRAVERSAL: MissionCommunicationProfile(
        mission=URCMission.AUTONOMOUS_TRAVERSAL,
        time_limit_minutes=30,
        communication_requirements={
            "telemetry": CommunicationRequirement(
                bandwidth_requirement="low",
                latency_tolerance="relaxed",
                message_frequency="1Hz",
                data_types=["position", "velocity", "battery_status"],
                critical_path=True,
                failure_tolerance="moderate",
            ),
            "waypoint_updates": CommunicationRequirement(
                bandwidth_requirement="low",
                latency_tolerance="tight",
                message_frequency="0.1Hz",
                data_types=["waypoint_coordinates", "navigation_status"],
                critical_path=True,
                failure_tolerance="minimal",
            ),
            "emergency_stop": CommunicationRequirement(
                bandwidth_requirement="critical",
                latency_tolerance="tight",
                message_frequency="on_demand",
                data_types=["stop_command"],
                critical_path=True,
                failure_tolerance="none",
            ),
            "status_updates": CommunicationRequirement(
                bandwidth_requirement="low",
                latency_tolerance="relaxed",
                message_frequency="0.5Hz",
                data_types=["system_health", "obstacle_detection"],
                critical_path=False,
                failure_tolerance="moderate",
            ),
        },
        network_profile=NetworkProfile.RURAL_WIFI,
        special_constraints=[
            "minimal_human_intervention_allowed",
            "autonomous_decision_making_required",
            "gps_denied_areas_expected",
            "emergency_stop_must_work_100_percent",
        ],
        testing_scenarios=[
            "gps_denied_navigation",
            "obstacle_avoidance_challenge",
            "communication_loss_recovery",
            "emergency_stop_propagation",
            "waypoint_timeout_handling",
        ],
    ),
    URCMission.SCIENCE_MISSION: MissionCommunicationProfile(
        mission=URCMission.SCIENCE_MISSION,
        time_limit_minutes=60,
        communication_requirements={
            "telemetry": CommunicationRequirement(
                bandwidth_requirement="low",
                latency_tolerance="moderate",
                message_frequency="1Hz",
                data_types=["position", "arm_status", "instrument_status"],
                critical_path=True,
                failure_tolerance="minimal",
            ),
            "video_stream": CommunicationRequirement(
                bandwidth_requirement="high",
                latency_tolerance="moderate",
                message_frequency="30fps",
                data_types=["camera_feed", "instrument_view"],
                critical_path=True,
                failure_tolerance="minimal",
            ),
            "sensor_data": CommunicationRequirement(
                bandwidth_requirement="medium",
                latency_tolerance="relaxed",
                message_frequency="5Hz",
                data_types=[
                    "spectrometer_data",
                    "temperature_readings",
                    "sample_analysis",
                ],
                critical_path=True,
                failure_tolerance="moderate",
            ),
            "arm_control": CommunicationRequirement(
                bandwidth_requirement="medium",
                latency_tolerance="tight",
                message_frequency="10Hz",
                data_types=["arm_commands", "gripper_commands", "tool_selection"],
                critical_path=True,
                failure_tolerance="minimal",
            ),
            "sample_analysis": CommunicationRequirement(
                bandwidth_requirement="medium",
                latency_tolerance="relaxed",
                message_frequency="1Hz",
                data_types=["chemical_analysis", "microscopy_images"],
                critical_path=True,
                failure_tolerance="moderate",
            ),
        },
        network_profile=NetworkProfile.RURAL_WIFI,
        special_constraints=[
            "high_bandwidth_video_required",
            "precise_arm_control_needed",
            "real_time_instrument_data",
            "sample_handling_integrity",
        ],
        testing_scenarios=[
            "high_bandwidth_video_streaming",
            "precise_manipulator_control",
            "sensor_data_transmission",
            "sample_collection_protocol",
            "instrument_calibration",
        ],
    ),
    URCMission.EQUIPMENT_SERVICING: MissionCommunicationProfile(
        mission=URCMission.EQUIPMENT_SERVICING,
        time_limit_minutes=30,
        communication_requirements={
            "telemetry": CommunicationRequirement(
                bandwidth_requirement="low",
                latency_tolerance="moderate",
                message_frequency="1Hz",
                data_types=["position", "arm_status", "tool_status"],
                critical_path=True,
                failure_tolerance="minimal",
            ),
            "precise_control": CommunicationRequirement(
                bandwidth_requirement="medium",
                latency_tolerance="tight",
                message_frequency="20Hz",
                data_types=["arm_position", "gripper_force", "tool_operation"],
                critical_path=True,
                failure_tolerance="none",
            ),
            "video_feedback": CommunicationRequirement(
                bandwidth_requirement="high",
                latency_tolerance="tight",
                message_frequency="30fps",
                data_types=["workspace_camera", "tool_camera"],
                critical_path=True,
                failure_tolerance="minimal",
            ),
            "task_status": CommunicationRequirement(
                bandwidth_requirement="low",
                latency_tolerance="moderate",
                message_frequency="2Hz",
                data_types=["task_progress", "error_codes", "completion_status"],
                critical_path=True,
                failure_tolerance="minimal",
            ),
        },
        network_profile=NetworkProfile.RURAL_WIFI,
        special_constraints=[
            "millimeter_precision_required",
            "real_time_video_mandatory",
            "no_intervention_penalties",
            "tool_switching_robustness",
        ],
        testing_scenarios=[
            "precise_tool_operation",
            "real_time_video_latency",
            "force_feedback_control",
            "task_sequence_completion",
            "error_recovery_handling",
        ],
    ),
    URCMission.DELIVERY_MISSION: MissionCommunicationProfile(
        mission=URCMission.DELIVERY_MISSION,
        time_limit_minutes=45,
        communication_requirements={
            "telemetry": CommunicationRequirement(
                bandwidth_requirement="low",
                latency_tolerance="moderate",
                message_frequency="1Hz",
                data_types=["position", "drone_status", "cargo_status"],
                critical_path=True,
                failure_tolerance="minimal",
            ),
            "drone_control": CommunicationRequirement(
                bandwidth_requirement="medium",
                latency_tolerance="tight",
                message_frequency="10Hz",
                data_types=["flight_commands", "waypoint_commands", "landing_commands"],
                critical_path=True,
                failure_tolerance="none",
            ),
            "cargo_management": CommunicationRequirement(
                bandwidth_requirement="low",
                latency_tolerance="moderate",
                message_frequency="2Hz",
                data_types=["cargo_status", "delivery_confirmation", "payload_data"],
                critical_path=True,
                failure_tolerance="minimal",
            ),
            "video_stream": CommunicationRequirement(
                bandwidth_requirement="high",
                latency_tolerance="moderate",
                message_frequency="30fps",
                data_types=["drone_camera", "delivery_site_camera"],
                critical_path=True,
                failure_tolerance="moderate",
            ),
            "coordination": CommunicationRequirement(
                bandwidth_requirement="medium",
                latency_tolerance="tight",
                message_frequency="5Hz",
                data_types=[
                    "rover_drone_coordination",
                    "handoff_commands",
                    "sync_status",
                ],
                critical_path=True,
                failure_tolerance="minimal",
            ),
        },
        network_profile=NetworkProfile.CELLULAR_4G,
        special_constraints=[
            "rover_drone_coordination",
            "autonomous_flight_required",
            "cargo_delivery_precision",
            "handoff_protocol_critical",
        ],
        testing_scenarios=[
            "rover_drone_coordination",
            "autonomous_flight_control",
            "cargo_delivery_precision",
            "handoff_protocol_testing",
            "emergency_landing_procedures",
        ],
    ),
}


class MissionCommunicationManager:
    """Manager for mission-specific communication testing."""

    def __init__(self):
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
        self.current_mission = None
        self.active_requirements = {}

    def set_mission(self, mission: URCMission):
        """Set active mission for communication testing."""
        self.current_mission = mission
        profile = URC_COMMUNICATION_PROFILES[mission]
        self.active_requirements = profile.communication_requirements

        self.logger.info(f"Set mission to {mission.value}")
        self.logger.info(f"Network profile: {profile.network_profile.value}")
        self.logger.info(f"Time limit: {profile.time_limit_minutes} minutes")

    def get_communication_requirements(self) -> Dict[str, CommunicationRequirement]:
        """Get current mission communication requirements."""
        return self.active_requirements.copy()

    def get_bandwidth_requirement(self, data_type: str) -> str:
        """Get bandwidth requirement for specific data type."""
        for req in self.active_requirements.values():
            if data_type in req.data_types:
                return req.bandwidth_requirement
        return "low"

    def get_latency_tolerance(self, data_type: str) -> str:
        """Get latency tolerance for specific data type."""
        for req in self.active_requirements.values():
            if data_type in req.data_types:
                return req.latency_tolerance
        return "moderate"

    def get_critical_paths(self) -> List[str]:
        """Get list of critical communication paths."""
        return [
            name for name, req in self.active_requirements.items() if req.critical_path
        ]

    def validate_communication_plan(self, plan: Dict[str, Any]) -> Dict[str, Any]:
        """Validate communication plan against mission requirements."""
        if not self.current_mission:
            return {"valid": False, "error": "No mission set"}

        profile = URC_COMMUNICATION_PROFILES[self.current_mission]
        validation_result = {
            "valid": True,
            "warnings": [],
            "errors": [],
            "recommendations": [],
        }

        # Check network profile compatibility
        if plan.get("network_profile") != profile.network_profile.value:
            validation_result["warnings"].append(
                f"Network profile mismatch. Recommended: {profile.network_profile.value}"
            )

        # Check bandwidth allocation
        total_bandwidth = plan.get("total_bandwidth_mbps", 0)
        if total_bandwidth > self._get_max_bandwidth():
            validation_result["errors"].append("Insufficient bandwidth allocation")

        # Check critical path coverage
        planned_paths = plan.get("communication_paths", [])
        missing_critical = set(self.get_critical_paths()) - set(planned_paths)
        if missing_critical:
            validation_result["errors"].append(
                f"Missing critical paths: {missing_critical}"
            )

        # Check special constraints
        for constraint in profile.special_constraints:
            if not self._check_constraint_satisfaction(plan, constraint):
                validation_result["warnings"].append(
                    f"Constraint not addressed: {constraint}"
                )

        validation_result["valid"] = len(validation_result["errors"]) == 0
        return validation_result

    def _get_max_bandwidth(self) -> float:
        """Get maximum available bandwidth for current mission."""
        if not self.current_mission:
            return 1000.0  # Default high bandwidth

        profile = URC_COMMUNICATION_PROFILES[self.current_mission]
        network_condition = None

        network_condition = NETWORK_PROFILES[profile.network_profile]

        return network_condition.bandwidth_mbps

    def _check_constraint_satisfaction(
        self, plan: Dict[str, Any], constraint: str
    ) -> bool:
        """Check if communication plan satisfies specific constraint."""
        constraint_handlers = {
            "minimal_human_intervention_allowed": lambda p: p.get("autonomy_level", 0)
            >= 8,
            "high_bandwidth_video_required": lambda p: p.get("video_bandwidth_mbps", 0)
            >= 5,
            "millimeter_precision_required": lambda p: p.get("control_latency_ms", 100)
            <= 50,
            "real_time_video_mandatory": lambda p: p.get("video_latency_ms", 200)
            <= 100,
            "autonomous_flight_required": lambda p: p.get("drone_autonomy", False),
            "rover_drone_coordination": lambda p: p.get("coordination_protocol", "")
            != "",
        }

        handler = constraint_handlers.get(constraint)
        if handler:
            return handler(plan)
        return True  # Assume satisfied if no handler

    def get_testing_scenarios(self) -> List[str]:
        """Get testing scenarios for current mission."""
        if not self.current_mission:
            return []

        profile = URC_COMMUNICATION_PROFILES[self.current_mission]
        return profile.testing_scenarios.copy()

    def create_mission_test_plan(self) -> Dict[str, Any]:
        """Create comprehensive test plan for current mission."""
        if not self.current_mission:
            return {"error": "No mission set"}

        profile = URC_COMMUNICATION_PROFILES[self.current_mission]

        test_plan = {
            "mission": self.current_mission.value,
            "time_limit_minutes": profile.time_limit_minutes,
            "network_profile": profile.network_profile.value,
            "communication_tests": [],
            "stress_tests": [],
            "failure_scenarios": [],
        }

        # Create communication tests for each requirement
        for name, req in profile.communication_requirements.items():
            test_plan["communication_tests"].append(
                {
                    "name": f"{name}_test",
                    "bandwidth_test": True,
                    "latency_test": True,
                    "reliability_test": req.critical_path,
                    "duration_seconds": 60,
                    "data_types": req.data_types,
                }
            )

            # Add stress tests for critical paths
            if req.critical_path:
                test_plan["stress_tests"].append(
                    {
                        "name": f"{name}_stress_test",
                        "load_factor": 2.0,  # 2x normal load
                        "duration_seconds": 300,  # 5 minutes
                        "success_criteria": "99_percent_success_rate",
                    }
                )

        # Add failure scenarios based on special constraints
        for constraint in profile.special_constraints:
            if "communication" in constraint.lower() or "video" in constraint.lower():
                test_plan["failure_scenarios"].append(
                    {
                        "name": f"failure_{constraint}",
                        "type": "communication_loss",
                        "duration_seconds": 30,
                        "recovery_required": True,
                    }
                )

        return test_plan


if __name__ == "__main__":
    # Test mission communication profiles
    print("[MISSION] Testing Mission Communication Profiles")
    print("=" * 60)

    manager = MissionCommunicationManager()

    for mission in URCMission:
        print(f"\n[MISSION] {mission.value.upper()}:")
        manager.set_mission(mission)

        profile = URC_COMMUNICATION_PROFILES[mission]
        print(f"  Time Limit: {profile.time_limit_minutes} minutes")
        print(f"  Network Profile: {profile.network_profile.value}")
        print(
            f"  Communication Requirements: {len(profile.communication_requirements)}"
        )
        print(f"  Testing Scenarios: {len(profile.testing_scenarios)}")
        print(f"  Special Constraints: {len(profile.special_constraints)}")

        # Show critical paths
        critical_paths = manager.get_critical_paths()
        print(f"  Critical Paths: {critical_paths}")

    print("\n[PASS] Mission communication profiles test complete")
