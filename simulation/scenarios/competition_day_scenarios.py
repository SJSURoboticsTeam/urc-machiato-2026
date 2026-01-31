"""Competition day scenarios with combined stressors for URC simulation.

Implements realistic competition day scenarios that combine environmental
challenges, communication stressors, and time pressure to simulate actual
URC competition conditions.

Author: URC 2026 Autonomy Team
"""

import logging
import math
import random
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

from simulation.communication.mission_profiles import (
    MissionCommunicationManager,
    URCMission,
    URC_COMMUNICATION_PROFILES,
)
from simulation.environments.mdrs_environment import (
    EnvironmentalStressor,
    MDRSEnvironment,
    TerrainType,
)
from simulation.network.network_emulator import NetworkEmulator, NetworkProfile


class CompetitionDay(Enum):
    """Competition day difficulty levels."""

    PRACTICE_DAY = "practice_day"
    QUALIFICATION_DAY = "qualification_day"
    FINALS_DAY = "finals_day"
    WORST_CASE = "worst_case"


@dataclass
class TimeConstraint:
    """Time constraint for mission execution."""

    mission_time_limit_minutes: int
    setup_time_minutes: int
    intervention_penalty_percent: int = 20
    scoring_deduction_per_minute: float = 2.0


@dataclass
class TeamCoordinationConstraint:
    """Constraints on team coordination and operator limits."""

    max_operators: int = 6
    max_operators_stations: int = 6
    advisor_access: bool = False
    handoff_required: bool = False
    role_based_access: bool = True


@dataclass
class CompetitionScenario:
    """Complete competition day scenario."""

    name: str
    description: str
    day_type: CompetitionDay
    environmental_conditions: Dict[str, Any]
    network_profile: NetworkProfile
    time_constraints: Dict[URCMission, TimeConstraint]
    team_constraints: TeamCoordinationConstraint
    stressors: List[EnvironmentalStressor]
    scoring_modifiers: Dict[str, float]
    special_events: List[Dict[str, Any]]


class CompetitionDayManager:
    """Manages competition day scenarios and execution."""

    def __init__(self):
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
        self.current_scenario: Optional[CompetitionScenario] = None
        self.mission_manager = MissionCommunicationManager()

        # Competition state
        self.active_mission: Optional[URCMission] = None
        self.mission_start_time: Optional[float] = None
        self.interventions_count = 0
        self.score_deductions = 0.0

        # Predefined scenarios
        self.scenarios = self._initialize_scenarios()

    def _initialize_scenarios(self) -> Dict[CompetitionDay, CompetitionScenario]:
        """Initialize predefined competition day scenarios."""
        scenarios = {}

        # Practice Day - Ideal conditions for testing
        scenarios[CompetitionDay.PRACTICE_DAY] = CompetitionScenario(
            name="Practice Day",
            description="Ideal conditions for system testing and debugging",
            day_type=CompetitionDay.PRACTICE_DAY,
            environmental_conditions={
                "base_temperature": 22.0,
                "daily_temp_range": 20.0,
                "dust_storm_probability": 0.05,
                "wind_gust_probability": 0.1,
                "time_of_day": 10.0,
                "operating_from_hab": False,
            },
            network_profile=NetworkProfile.PERFECT,
            time_constraints={
                URCMission.AUTONOMOUS_TRAVERSAL: TimeConstraint(
                    mission_time_limit_minutes=45
                ),
                URCMission.SCIENCE_MISSION: TimeConstraint(
                    mission_time_limit_minutes=90
                ),
                URCMission.EQUIPMENT_SERVICING: TimeConstraint(
                    mission_time_limit_minutes=45
                ),
                URCMission.DELIVERY_MISSION: TimeConstraint(
                    mission_time_limit_minutes=60
                ),
            },
            team_constraints=TeamCoordinationConstraint(
                max_operators=8,  # Relaxed for practice
                max_operators_stations=8,
                advisor_access=True,
                handoff_required=False,
                role_based_access=False,
            ),
            stressors=[],
            scoring_modifiers={"difficulty_multiplier": 0.8, "time_bonus": 1.2},
            special_events=[],
        )

        # Qualification Day - Moderate challenges
        scenarios[CompetitionDay.QUALIFICATION_DAY] = CompetitionScenario(
            name="Qualification Day",
            description="Standard competition conditions with moderate challenges",
            day_type=CompetitionDay.QUALIFICATION_DAY,
            environmental_conditions={
                "base_temperature": 25.0,
                "daily_temp_range": 25.0,
                "dust_storm_probability": 0.15,
                "wind_gust_probability": 0.25,
                "time_of_day": 11.0,
                "operating_from_hab": True,  # Must operate from Hab for terrain task
            },
            network_profile=NetworkProfile.RURAL_WIFI,
            time_constraints={
                URCMission.AUTONOMOUS_TRAVERSAL: TimeConstraint(
                    mission_time_limit_minutes=30
                ),
                URCMission.SCIENCE_MISSION: TimeConstraint(
                    mission_time_limit_minutes=60
                ),
                URCMission.EQUIPMENT_SERVICING: TimeConstraint(
                    mission_time_limit_minutes=30
                ),
                URCMission.DELIVERY_MISSION: TimeConstraint(
                    mission_time_limit_minutes=45
                ),
            },
            team_constraints=TeamCoordinationConstraint(
                max_operators=6,
                max_operators_stations=6,
                advisor_access=False,
                handoff_required=True,
                role_based_access=True,
            ),
            stressors=[
                EnvironmentalStressor(
                    stressor_type="moderate_wind_gusts",
                    severity=0.4,
                    duration_minutes=20,
                    effects={"wind_speed": 8.0, "communication_attenuation": 3.0},
                    recovery_time_minutes=10,
                ),
            ],
            scoring_modifiers={"difficulty_multiplier": 1.0, "time_bonus": 1.0},
            special_events=[
                {
                    "type": "terrain_change",
                    "time_minutes": 15,
                    "description": "Transition to rocky outcrop terrain",
                },
            ],
        )

        # Finals Day - Challenging conditions
        scenarios[CompetitionDay.FINALS_DAY] = CompetitionScenario(
            name="Finals Day",
            description="Challenging conditions testing system limits",
            day_type=CompetitionDay.FINALS_DAY,
            environmental_conditions={
                "base_temperature": 30.0,
                "daily_temp_range": 30.0,
                "dust_storm_probability": 0.25,
                "wind_gust_probability": 0.35,
                "time_of_day": 13.0,
                "operating_from_hab": True,
            },
            network_profile=NetworkProfile.CELLULAR_4G,
            time_constraints={
                URCMission.AUTONOMOUS_TRAVERSAL: TimeConstraint(
                    mission_time_limit_minutes=30
                ),
                URCMission.SCIENCE_MISSION: TimeConstraint(
                    mission_time_limit_minutes=60
                ),
                URCMission.EQUIPMENT_SERVICING: TimeConstraint(
                    mission_time_limit_minutes=30
                ),
                URCMission.DELIVERY_MISSION: TimeConstraint(
                    mission_time_limit_minutes=45
                ),
            },
            team_constraints=TeamCoordinationConstraint(
                max_operators=6,
                max_operators_stations=6,
                advisor_access=False,
                handoff_required=True,
                role_based_access=True,
            ),
            stressors=[
                EnvironmentalStressor(
                    stressor_type="dust_storm",
                    severity=0.6,
                    duration_minutes=30,
                    effects={
                        "dust_density": 0.4,
                        "visibility_reduction": 0.5,
                        "communication_attenuation": 8.0,
                    },
                    recovery_time_minutes=20,
                ),
                EnvironmentalStressor(
                    stressor_type="thermal_stress",
                    severity=0.7,
                    duration_minutes=45,
                    effects={"thermal_noise": 1.5, "battery_drain": 1.3},
                    recovery_time_minutes=15,
                ),
            ],
            scoring_modifiers={"difficulty_multiplier": 1.3, "time_bonus": 0.8},
            special_events=[
                {
                    "type": "dust_storm_start",
                    "time_minutes": 10,
                    "description": "Sudden dust storm reduces visibility",
                },
                {
                    "type": "equipment_malfunction",
                    "time_minutes": 25,
                    "description": "Minor sensor requiring recovery procedure",
                },
            ],
        )

        # Worst Case - Extreme stress testing
        scenarios[CompetitionDay.WORST_CASE] = CompetitionScenario(
            name="Worst Case Scenario",
            description="Extreme conditions for robustness validation",
            day_type=CompetitionDay.WORST_CASE,
            environmental_conditions={
                "base_temperature": 38.0,
                "daily_temp_range": 35.0,
                "dust_storm_probability": 0.4,
                "wind_gust_probability": 0.5,
                "time_of_day": 15.0,
                "operating_from_hab": True,
            },
            network_profile=NetworkProfile.EXTREME,
            time_constraints={
                URCMission.AUTONOMOUS_TRAVERSAL: TimeConstraint(
                    mission_time_limit_minutes=25
                ),
                URCMission.SCIENCE_MISSION: TimeConstraint(
                    mission_time_limit_minutes=50
                ),
                URCMission.EQUIPMENT_SERVICING: TimeConstraint(
                    mission_time_limit_minutes=25
                ),
                URCMission.DELIVERY_MISSION: TimeConstraint(
                    mission_time_limit_minutes=40
                ),
            },
            team_constraints=TeamCoordinationConstraint(
                max_operators=6,
                max_operators_stations=6,
                advisor_access=False,
                handoff_required=True,
                role_based_access=True,
            ),
            stressors=[
                EnvironmentalStressor(
                    stressor_type="severe_dust_storm",
                    severity=0.9,
                    duration_minutes=60,
                    effects={
                        "dust_density": 0.8,
                        "visibility_reduction": 0.8,
                        "communication_attenuation": 15.0,
                    },
                    recovery_time_minutes=30,
                ),
                EnvironmentalStressor(
                    stressor_type="extreme_heat",
                    severity=0.8,
                    duration_minutes=90,
                    effects={"thermal_noise": 2.0, "battery_drain": 1.5},
                    recovery_time_minutes=45,
                ),
                EnvironmentalStressor(
                    stressor_type="communication_failure",
                    severity=0.7,
                    duration_minutes=20,
                    effects={"packet_loss": 0.3, "latency_increase": 2.0},
                    recovery_time_minutes=15,
                ),
            ],
            scoring_modifiers={"difficulty_multiplier": 1.5, "time_bonus": 0.6},
            special_events=[
                {
                    "type": "complete_communication_loss",
                    "time_minutes": 15,
                    "description": "Total communication loss for 2 minutes",
                    "duration_minutes": 2,
                },
                {
                    "type": "power_brownout",
                    "time_minutes": 30,
                    "description": "Partial power system failure",
                    "duration_minutes": 5,
                },
            ],
        )

        return scenarios

    def set_competition_day(self, day_type: CompetitionDay):
        """Set current competition day scenario."""
        if day_type not in self.scenarios:
            raise ValueError(f"Unknown competition day: {day_type}")

        self.current_scenario = self.scenarios[day_type]
        self.logger.info(f"Set competition day: {self.current_scenario.name}")
        self.logger.info(f"Description: {self.current_scenario.description}")

    def start_mission(self, mission: URCMission) -> Dict[str, Any]:
        """Start a mission under current competition scenario."""
        if not self.current_scenario:
            raise ValueError("No competition scenario set")

        self.active_mission = mission
        self.mission_start_time = None  # Will be set when actually starting
        self.interventions_count = 0
        self.score_deductions = 0.0

        # Set mission for communication manager
        self.mission_manager.set_mission(mission)

        # Get mission constraints
        time_constraint = self.current_scenario.time_constraints[mission]

        mission_info = {
            "mission": mission.value,
            "time_limit_minutes": time_constraint.mission_time_limit_minutes,
            "network_profile": self.current_scenario.network_profile.value,
            "intervention_penalty": time_constraint.intervention_penalty_percent,
            "max_operators": self.current_scenario.team_constraints.max_operators,
            "special_events": len(self.current_scenario.special_events),
            "active_stressors": len(self.current_scenario.stressors),
        }

        self.logger.info(f"Started mission: {mission.value}")
        self.logger.info(
            f"Time limit: {time_constraint.mission_time_limit_minutes} minutes"
        )

        return mission_info

    def execute_mission_with_scenario(
        self,
        mission: URCMission,
        environment: MDRSEnvironment,
        network_emulator: NetworkEmulator,
        execution_time_minutes: float = 30.0,
    ) -> Dict[str, Any]:
        """Execute mission with full competition scenario simulation."""
        if not self.current_scenario:
            raise ValueError("No competition scenario set")

        self.logger.info(
            f"Executing {mission.value} under {self.current_scenario.name} scenario"
        )

        # Apply scenario conditions
        self._apply_scenario_to_environment(environment)
        self._apply_scenario_to_network(network_emulator)

        # Start mission
        mission_info = self.start_mission(mission)

        # Simulate mission execution
        execution_results = self._simulate_mission_execution(
            mission, environment, network_emulator, execution_time_minutes
        )

        # Calculate final score
        final_score = self._calculate_mission_score(execution_results)

        results = {
            "mission": mission.value,
            "scenario": self.current_scenario.name,
            "mission_info": mission_info,
            "execution_results": execution_results,
            "final_score": final_score,
            "performance_metrics": self._calculate_performance_metrics(
                execution_results
            ),
        }

        return results

    def _apply_scenario_to_environment(self, environment: MDRSEnvironment):
        """Apply scenario environmental conditions."""
        conditions = self.current_scenario.environmental_conditions

        # Set base conditions
        for param, value in conditions.items():
            if hasattr(environment, param):
                setattr(environment, param, value)

        # Apply stressors
        environment.active_stressors = self.current_scenario.stressors.copy()

        # Set Hab operation
        environment.set_operating_from_hab(conditions.get("operating_from_hab", False))

        self.logger.info("Applied environmental scenario conditions")

    def _apply_scenario_to_network(self, network_emulator: NetworkEmulator):
        """Apply scenario network conditions."""
        # Note: NetworkEmulator doesn't support profile change after creation
        # This would require enhancement to the NetworkEmulator class
        target_profile = self.current_scenario.network_profile
        current_profile = network_emulator.profile

        if target_profile != current_profile:
            self.logger.warning(
                f"Network profile mismatch: target={target_profile.value}, current={current_profile.value}"
            )
            self.logger.warning("Network emulator profile change not implemented")

    def _simulate_mission_execution(
        self,
        mission: URCMission,
        environment: MDRSEnvironment,
        network_emulator: NetworkEmulator,
        execution_time_minutes: float,
    ) -> Dict[str, Any]:
        """Simulate mission execution with time-based events."""
        import time

        start_time = time.time()
        end_time = start_time + execution_time_minutes * 60.0

        execution_log = []
        special_events_triggered = []
        interventions = []

        mission_progress = 0.0

        while time.time() < end_time:
            current_time = time.time()
            elapsed_minutes = (current_time - start_time) / 60.0

            # Update environment
            environment.step(1.0)  # 1-second steps

            # Check for special events
            for event in self.current_scenario.special_events:
                if (
                    event["time_minutes"] <= elapsed_minutes
                    and event not in special_events_triggered
                ):
                    special_events_triggered.append(event)
                    execution_log.append(
                        {
                            "timestamp": current_time,
                            "type": "special_event",
                            "event": event,
                            "description": event["description"],
                        }
                    )

                    # Apply event effects
                    self._apply_special_event(event, environment, network_emulator)

            # Simulate mission progress
            progress_rate = self._calculate_progress_rate(
                mission, environment, network_emulator
            )
            mission_progress += progress_rate * (1.0 / 60.0)  # Convert to minutes
            mission_progress = min(1.0, mission_progress)

            # Random interventions based on conditions
            if random.random() < self._calculate_intervention_probability(
                environment, network_emulator
            ):
                interventions.append(
                    {
                        "timestamp": current_time,
                        "elapsed_minutes": elapsed_minutes,
                        "reason": "communication_loss_or_equipment_issue",
                    }
                )
                self.interventions_count += 1

            # Check for mission completion
            if mission_progress >= 1.0:
                break

            time.sleep(0.1)  # Small delay to avoid excessive CPU usage

        total_time_minutes = (time.time() - start_time) / 60.0

        return {
            "total_time_minutes": total_time_minutes,
            "mission_completed": mission_progress >= 1.0,
            "mission_progress_percent": mission_progress * 100,
            "interventions": interventions,
            "special_events_triggered": special_events_triggered,
            "execution_log": execution_log,
            "environmental_state": environment.get_state(),
            "network_statistics": network_emulator.get_statistics(),
        }

    def _apply_special_event(
        self,
        event: Dict[str, Any],
        environment: MDRSEnvironment,
        network_emulator: NetworkEmulator,
    ):
        """Apply special event effects."""
        event_type = event["type"]

        if event_type == "dust_storm_start":
            environment._initiate_dust_storm()

        elif event_type == "terrain_change":
            environment._transition_to_new_terrain()

        elif event_type == "equipment_malfunction":
            # Add temporary equipment stressor
            equipment_stress = EnvironmentalStressor(
                stressor_type="equipment_malfunction",
                severity=0.5,
                duration_minutes=10,
                effects={"sensor_reliability": 0.7},
                recovery_time_minutes=5,
            )
            environment.active_stressors.append(equipment_stress)

        elif event_type == "complete_communication_loss":
            # This would require network emulator enhancement
            self.logger.warning(
                "Complete communication loss simulation not implemented"
            )

        elif event_type == "power_brownout":
            power_stress = EnvironmentalStressor(
                stressor_type="power_brownout",
                severity=0.6,
                duration_minutes=event.get("duration_minutes", 5),
                effects={"battery_drain": 2.0, "system_performance": 0.5},
                recovery_time_minutes=10,
            )
            environment.active_stressors.append(power_stress)

    def _calculate_progress_rate(
        self,
        mission: URCMission,
        environment: MDRSEnvironment,
        network_emulator: NetworkEmulator,
    ) -> float:
        """Calculate mission progress rate based on conditions."""
        base_rate = 0.02  # 2% per minute base rate

        # Environmental factors
        env_effects = environment.get_environmental_effects()

        # Visibility effect
        visibility_factor = env_effects.get("visibility", 1.0)

        # Terrain difficulty
        terrain_factor = 1.0 - env_effects.get("terrain_difficulty", 0.0) * 0.5

        # Temperature stress
        temp_stress = env_effects.get("thermal_stress_factor", 0.0)
        thermal_factor = 1.0 - temp_stress * 0.3

        # Network factors
        net_stats = network_emulator.get_statistics()
        packet_loss_factor = 1.0 - (net_stats.get("packet_loss_percent", 0.0) / 100.0)
        latency_factor = 1.0 - min(
            0.5, net_stats.get("average_latency_ms", 0.0) / 1000.0
        )

        # Combined factors
        total_factor = (
            visibility_factor
            * terrain_factor
            * thermal_factor
            * packet_loss_factor
            * latency_factor
        )

        # Mission-specific adjustments
        mission_factors = {
            URCMission.AUTONOMOUS_TRAVERSAL: 1.0,
            URCMission.SCIENCE_MISSION: 0.8,  # More sensitive to conditions
            URCMission.EQUIPMENT_SERVICING: 0.6,  # Very sensitive to conditions
            URCMission.DELIVERY_MISSION: 0.9,
        }

        mission_factor = mission_factors.get(mission, 1.0)

        return base_rate * total_factor * mission_factor

    def _calculate_intervention_probability(
        self, environment: MDRSEnvironment, network_emulator: NetworkEmulator
    ) -> float:
        """Calculate probability of requiring intervention."""
        base_probability = 0.01  # 1% per minute base

        # Environmental stress increases intervention probability
        env_effects = environment.get_environmental_effects()
        stress_factor = env_effects.get("active_stressors", 0) * 0.05

        # Network issues increase intervention probability
        net_stats = network_emulator.get_statistics()
        packet_loss_factor = net_stats.get("packet_loss_percent", 0.0) / 100.0

        total_probability = base_probability + stress_factor + packet_loss_factor
        return min(0.1, total_probability)  # Cap at 10% per minute

    def _calculate_mission_score(
        self, execution_results: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Calculate final mission score."""
        if not self.current_scenario or not self.active_mission:
            return {"error": "No active mission/scenario"}

        time_constraint = self.current_scenario.time_constraints[self.active_mission]
        scoring_mods = self.current_scenario.scoring_modifiers

        # Base score
        base_score = 100.0

        # Time deductions
        time_used = execution_results["total_time_minutes"]
        time_limit = time_constraint.mission_time_limit_minutes
        time_deduction = 0.0

        if time_used > time_limit:
            overtime_minutes = time_used - time_limit
            time_deduction = (
                overtime_minutes * time_constraint.scoring_deduction_per_minute
            )

        # Intervention deductions
        intervention_deduction = (
            self.interventions_count * time_constraint.intervention_penalty_percent
        )

        # Completion bonus
        completion_bonus = 50.0 if execution_results["mission_completed"] else 0.0

        # Progress score
        progress_score = execution_results["mission_progress_percent"]

        # Apply scoring modifiers
        difficulty_multiplier = scoring_mods.get("difficulty_multiplier", 1.0)
        time_bonus = scoring_mods.get("time_bonus", 1.0)

        # Calculate final score
        final_score = (
            (
                base_score
                - time_deduction
                - intervention_deduction
                + completion_bonus
                + progress_score
            )
            * difficulty_multiplier
            * time_bonus
        )

        final_score = max(0.0, final_score)  # Ensure non-negative

        return {
            "base_score": base_score,
            "time_deduction": time_deduction,
            "intervention_deduction": intervention_deduction,
            "completion_bonus": completion_bonus,
            "progress_score": progress_score,
            "difficulty_multiplier": difficulty_multiplier,
            "time_bonus": time_bonus,
            "final_score": final_score,
            "score_breakdown": {
                "time_used_minutes": time_used,
                "time_limit_minutes": time_limit,
                "interventions": self.interventions_count,
                "mission_completed": execution_results["mission_completed"],
            },
        }

    def _calculate_performance_metrics(
        self, execution_results: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Calculate performance metrics from execution results."""
        net_stats = execution_results["network_statistics"]
        env_state = execution_results["environmental_state"]

        return {
            "communication_performance": {
                "success_rate_percent": (
                    (net_stats["messages_received"] / net_stats["messages_sent"] * 100)
                    if net_stats["messages_sent"] > 0
                    else 0.0
                ),
                "average_latency_ms": net_stats["average_latency_ms"],
                "packet_loss_percent": net_stats["packet_loss_percent"],
            },
            "environmental_performance": {
                "visibility": env_state.get("visibility", 1.0),
                "dust_density": env_state.get("dust_density", 0.0),
                "temperature": env_state.get("temperature", 20.0),
                "terrain_difficulty": env_state.get("terrain_difficulty", 0.0),
            },
            "mission_efficiency": {
                "time_efficiency": (
                    execution_results["mission_progress_percent"]
                    / (
                        execution_results["total_time_minutes"] / 30.0
                    )  # Normalized to 30 minutes
                ),
                "intervention_rate": (
                    len(execution_results["interventions"])
                    / execution_results["total_time_minutes"]
                ),
                "special_events_handled": len(
                    execution_results["special_events_triggered"]
                ),
            },
        }

    def get_scenario_summary(self, day_type: CompetitionDay) -> Dict[str, Any]:
        """Get summary of a competition day scenario."""
        if day_type not in self.scenarios:
            return {"error": "Unknown scenario"}

        scenario = self.scenarios[day_type]

        return {
            "name": scenario.name,
            "description": scenario.description,
            "network_profile": scenario.network_profile.value,
            "team_constraints": {
                "max_operators": scenario.team_constraints.max_operators,
                "advisor_access": scenario.team_constraints.advisor_access,
                "handoff_required": scenario.team_constraints.handoff_required,
            },
            "environmental_challenges": {
                "operating_from_hab": scenario.environmental_conditions.get(
                    "operating_from_hab", False
                ),
                "dust_storm_probability": scenario.environmental_conditions.get(
                    "dust_storm_probability", 0.0
                ),
                "base_temperature": scenario.environmental_conditions.get(
                    "base_temperature", 20.0
                ),
            },
            "stressors": len(scenario.stressors),
            "special_events": len(scenario.special_events),
            "scoring_modifiers": scenario.scoring_modifiers,
        }


if __name__ == "__main__":
    # Test competition day scenarios
    print("[COMPETITION] Testing Competition Day Scenarios")
    print("=" * 60)

    manager = CompetitionDayManager()

    for day_type in CompetitionDay:
        print(f"\n[SCENARIO] {day_type.value.upper()}:")
        summary = manager.get_scenario_summary(day_type)

        print(f"  Name: {summary['name']}")
        print(f"  Network: {summary['network_profile']}")
        print(f"  Max operators: {summary['team_constraints']['max_operators']}")
        print(
            f"  Operating from Hab: {summary['environmental_challenges']['operating_from_hab']}"
        )
        print(f"  Active stressors: {summary['stressors']}")
        print(f"  Special events: {summary['special_events']}")
        print(
            f"  Difficulty multiplier: {summary['scoring_modifiers']['difficulty_multiplier']:.1f}"
        )

    print("\n[PASS] Competition day scenarios test complete")
