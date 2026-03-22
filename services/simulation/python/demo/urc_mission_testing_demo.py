#!/usr/bin/env python3
"""
URC Mission Testing and Environmental Stressors Demo

Demonstrates the new mission-specific communication testing and MDRS
environmental stressors for URC competition preparation.

Usage:
    python simulation/demo/urc_mission_testing_demo.py

Author: URC 2026 Autonomy Team
"""

import asyncio
import logging
import sys
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from simulation.communication.mission_profiles import (
    MissionCommunicationManager,
    URCMission,
    URC_COMMUNICATION_PROFILES,
)
from simulation.environments.mdrs_environment import MDRSEnvironment
from simulation.environments.environment_factory import EnvironmentFactory
from simulation.network.network_emulator import NetworkEmulator, NetworkProfile
from simulation.scenarios.competition_day_scenarios import (
    CompetitionDay,
    CompetitionDayManager,
)
from simulation.testing.mission_test_suites import MissionTestSuiteFactory


def setup_logging():
    """Setup logging for demo."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%H:%M:%S",
    )


def demo_mission_communication_profiles():
    """Demonstrate mission-specific communication profiles."""
    print("\n" + "=" * 80)
    print("MISSION-SPECIFIC COMMUNICATION PROFILES DEMO")
    print("=" * 80)

    manager = MissionCommunicationManager()

    for mission in URCMission:
        print(f"\n{'=' * 60}")
        print(f"MISSION: {mission.value.upper()}")
        print(f"{'=' * 60}")

        manager.set_mission(mission)
        profile = URC_COMMUNICATION_PROFILES[mission]

        print(f"Time Limit: {profile.time_limit_minutes} minutes")
        print(f"Network Profile: {profile.network_profile.value}")
        print(f"Communication Requirements: {len(profile.communication_requirements)}")

        # Show communication requirements
        print("\nCommunication Requirements:")
        for name, req in profile.communication_requirements.items():
            print(f"  {name}:")
            print(f"    Bandwidth: {req.bandwidth_requirement}")
            print(f"    Latency Tolerance: {req.latency_tolerance}")
            print(f"    Frequency: {req.message_frequency}")
            print(f"    Data Types: {', '.join(req.data_types)}")
            print(f"    Critical Path: {req.critical_path}")

        # Show critical paths
        critical_paths = manager.get_critical_paths()
        print(f"\nCritical Paths: {critical_paths}")

        # Show testing scenarios
        scenarios = manager.get_testing_scenarios()
        print(f"\nTesting Scenarios: {len(scenarios)}")
        for scenario in scenarios[:3]:  # Show first 3
            print(f"  - {scenario}")
        if len(scenarios) > 3:
            print(f"  ... and {len(scenarios) - 3} more")


def demo_mdrs_environment():
    """Demonstrate MDRS environmental stressors."""
    print("\n" + "=" * 80)
    print("MDRS ENVIRONMENTAL STRESSORS DEMO")
    print("=" * 80)

    # Create MDRS environment
    config = {
        "tier": "mdrs_competition",
        "base_temperature": 20.0,
        "daily_temp_range": 25.0,
        "dust_storm_probability": 0.15,
        "wind_gust_probability": 0.25,
        "time_of_day": 10.0,
    }

    mdrs_env = MDRSEnvironment(config)

    print(f"Location: {mdrs_env.location_name}")
    print(f"Coordinates: {mdrs_env.latitude:.4f}, {mdrs_env.longitude:.4f}")
    print(f"Elevation: {mdrs_env.elevation_m}m")

    # Test different scenarios
    scenarios = ["perfect_day", "moderate_challenges", "challenging_day", "worst_case"]

    for scenario in scenarios:
        print(f"\n{'-' * 60}")
        print(f"SCENARIO: {scenario.upper()}")
        print(f"{'-' * 60}")

        mdrs_env.apply_scenario(scenario)

        # Run simulation for a few steps
        for i in range(5):
            state = mdrs_env.step(1.0)
            if i == 0:
                print(f"Temperature: {state['temperature']:.1f}°C")
                print(f"Visibility: {state['visibility']:.2f}")
                print(f"Dust Density: {state['dust_density']:.2f}")
                print(f"Wind Speed: {state['wind_speed']:.1f} m/s")
                print(f"Terrain Type: {mdrs_env.current_terrain.value}")
                break

        # Show stressors
        stressors = mdrs_env.get_stressor_summary()
        print(f"Active Stressors: {stressors['active_stressors']}")

        if stressors["active_stressors"] > 0:
            for stressor in stressors["active_stressors"]:
                print(
                    f"  - {stressor['type']}: severity={stressor['severity']:.2f}, "
                    f"remaining={stressor['remaining_minutes']:.1f}min"
                )

    # Test metal Hab operation
    print(f"\n{'-' * 60}")
    print("METAL HAB COMMUNICATION TEST")
    print(f"{'-' * 60}")

    mdrs_env.set_operating_from_hab(True)
    effects = mdrs_env.get_environmental_effects()
    attenuation = effects.get("communication_attenuation_db", 0)
    print(f"Communication Attenuation: {attenuation:.1f} dB")
    print(f"Antenna Cable Length: {mdrs_env.antenna_cable_length_m}m")


def demo_competition_day_scenarios():
    """Demonstrate competition day scenarios."""
    print("\n" + "=" * 80)
    print("COMPETITION DAY SCENARIOS DEMO")
    print("=" * 80)

    manager = CompetitionDayManager()

    for day_type in CompetitionDay:
        print(f"\n{'=' * 60}")
        print(f"COMPETITION DAY: {day_type.value.upper()}")
        print(f"{'=' * 60}")

        summary = manager.get_scenario_summary(day_type)

        print(f"Name: {summary['name']}")
        print(f"Description: {summary['description']}")
        print(f"Network Profile: {summary['network_profile']}")

        # Team constraints
        team = summary["team_constraints"]
        print(f"Max Operators: {team['max_operators']}")
        print(f"Advisor Access: {team['advisor_access']}")
        print(f"Handoff Required: {team['handoff_required']}")

        # Environmental challenges
        env = summary["environmental_challenges"]
        print(f"Operating from Hab: {env['operating_from_hab']}")
        print(f"Dust Storm Probability: {env['dust_storm_probability']:.2f}")
        print(f"Base Temperature: {env['base_temperature']:.1f}°C")

        # Stressors and events
        print(f"Active Stressors: {summary['stressors']}")
        print(f"Special Events: {summary['special_events']}")

        # Scoring modifiers
        scoring = summary["scoring_modifiers"]
        print(f"Difficulty Multiplier: {scoring['difficulty_multiplier']:.1f}x")
        print(f"Time Bonus: {scoring['time_bonus']:.1f}x")


async def demo_mission_testing():
    """Demonstrate mission-specific testing."""
    print("\n" + "=" * 80)
    print("MISSION-SPECIFIC TESTING DEMO")
    print("=" * 80)

    # Test one mission as example
    mission = URCMission.SCIENCE_MISSION
    print(f"Testing {mission.value} mission...")

    test_suite = MissionTestSuiteFactory.create_test_suite(mission)

    # Run a subset of tests for demo
    print("Running communication tests...")
    try:
        # Setup test infrastructure manually for demo
        profile = URC_COMMUNICATION_PROFILES[mission]
        network_emulator = NetworkEmulator(profile.network_profile)
        network_emulator.start()

        env_config = {
            "tier": "mdrs_competition",
            "base_temperature": 25.0,
            "daily_temp_range": 25.0,
            "dust_storm_probability": 0.15,
            "time_of_day": 12.0,
        }
        mdrs_env = MDRSEnvironment(env_config)

        # Apply moderate challenges scenario
        mdrs_env.apply_scenario("moderate_challenges")

        print(f"Network Profile: {profile.network_profile.value}")
        print(f"Time Limit: {profile.time_limit_minutes} minutes")
        print(f"Communication Requirements: {len(profile.communication_requirements)}")

        # Show critical paths
        manager = MissionCommunicationManager()
        manager.set_mission(mission)
        critical_paths = manager.get_critical_paths()
        print(f"Critical Paths: {critical_paths}")

        # Show test plan
        test_plan = manager.create_mission_test_plan()
        print(
            f"Test Plan: {len(test_plan['communication_tests'])} communication tests, "
            f"{len(test_plan['stress_tests'])} stress tests"
        )

        network_emulator.stop()

    except Exception as e:
        print(f"Test execution error: {e}")
        import traceback

        traceback.print_exc()


def demo_environment_factory():
    """Demonstrate environment factory integration."""
    print("\n" + "=" * 80)
    print("ENVIRONMENT FACTORY INTEGRATION DEMO")
    print("=" * 80)

    # Show available environments
    available_tiers = EnvironmentFactory.get_available_tiers()
    print(f"Available Environment Tiers: {available_tiers}")

    # Create MDRS environment through factory
    mdrs_config = {
        "tier": "mdrs_competition",
        "base_temperature": 22.0,
        "daily_temp_range": 20.0,
        "dust_storm_probability": 0.1,
        "time_of_day": 9.0,
    }

    try:
        mdrs_env = EnvironmentFactory.create(mdrs_config)
        print(f"Created MDRS environment: {type(mdrs_env).__name__}")

        # Get initial state
        state = mdrs_env.get_state()
        print(f"Initial Temperature: {state['temperature']:.1f}°C")
        print(f"Initial Terrain: {state['surface_type']}")
        print(f"Initial Visibility: {state['visibility']:.2f}")

    except Exception as e:
        print(f"Environment creation error: {e}")


def main():
    """Run all demos."""
    setup_logging()

    print("URC MACHIATO 2026 - MISSION TESTING & ENVIRONMENTAL STRESSORS DEMO")
    print("=" * 80)

    # Demo 1: Mission Communication Profiles
    demo_mission_communication_profiles()

    # Demo 2: MDRS Environmental Stressors
    demo_mdrs_environment()

    # Demo 3: Competition Day Scenarios
    demo_competition_day_scenarios()

    # Demo 4: Mission-Specific Testing (async)
    asyncio.run(demo_mission_testing())

    # Demo 5: Environment Factory Integration
    demo_environment_factory()

    print("\n" + "=" * 80)
    print("DEMO COMPLETE")
    print("=" * 80)

    print("\nSUMMARY OF NEW CAPABILITIES:")
    print("✓ Mission-specific communication profiles for all 4 URC missions")
    print("✓ MDRS-specific environmental stressors (dust, temperature, terrain)")
    print("✓ Metal Hab communication challenges (25m antenna cables)")
    print("✓ Competition day scenarios with time pressure and scoring")
    print("✓ Mission-specific test suites with validation")
    print("✓ Environmental stressors (dust storms, thermal stress, etc.)")
    print("✓ Team coordination constraints (6 operators, handoffs)")
    print("✓ Combined stressor scenarios for robustness testing")

    print("\nNEXT STEPS:")
    print("1. Run: python simulation/testing/mission_test_suites.py")
    print("2. Test: python simulation/scenarios/competition_day_scenarios.py")
    print("3. Validate: python -m pytest tests/simulation/ -v")
    print("4. Integrate: Add mission profiles to your autonomy stack")


if __name__ == "__main__":
    main()
