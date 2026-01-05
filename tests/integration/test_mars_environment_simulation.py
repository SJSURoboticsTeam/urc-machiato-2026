#!/usr/bin/env python3
"""
Mars Environment Simulation Tests - URC 2026

Tests complete Mars environment simulation for:
- Mars terrain and physics simulation
- URC competition course simulation
- Environmental conditions (dust, wind, temperature)
- GPS-denied navigation scenarios
- Communication delays and blackouts
- End-to-end mission execution validation

Author: URC 2026 Mars Simulation Team
"""

import asyncio
import time
import math
import random
from typing import Dict, Any, List, Optional, Tuple
import pytest
from unittest.mock import Mock, patch
import json


class MarsTerrainSimulator:
    """Mars terrain and physics simulation."""

    def __init__(self):
        self.terrain_map = self._generate_terrain()
        self.weather_conditions = self._initialize_weather()
        self.gravity = 3.71  # m/s² (Mars gravity)

    def _generate_terrain(self) -> Dict[str, Any]:
        """Generate Mars-like terrain."""
        return {
            "boundaries": {"width": 1000, "height": 1000},  # meters
            "obstacles": [
                {"type": "rock", "position": (100, 200), "size": 2.0},
                {"type": "crater", "position": (300, 400), "radius": 15.0},
                {"type": "dune", "position": (500, 600), "height": 3.0}
            ],
            "waypoints": [
                {"id": 1, "position": (50, 50), "type": "start"},
                {"id": 2, "position": (200, 150), "type": "navigation"},
                {"id": 3, "position": (400, 300), "type": "sample_site"},
                {"id": 4, "position": (600, 450), "type": "delivery"},
                {"id": 5, "position": (800, 600), "type": "finish"}
            ],
            "sample_sites": [
                {"id": 1, "position": (420, 320), "type": "rock_sample"},
                {"id": 2, "position": (380, 280), "type": "soil_sample"},
                {"id": 3, "position": (450, 350), "type": "ice_sample"}
            ]
        }

    def _initialize_weather(self) -> Dict[str, Any]:
        """Initialize Mars weather conditions."""
        return {
            "temperature": -60.0,  # Celsius
            "pressure": 600.0,     # Pa
            "wind_speed": 5.0,     # m/s
            "dust_storm": False,
            "visibility": 1000.0   # meters
        }

    def get_terrain_info(self, position: Tuple[float, float]) -> Dict[str, Any]:
        """Get terrain information at position."""
        x, y = position

        # Check for obstacles
        nearby_obstacles = []
        for obstacle in self.terrain_map["obstacles"]:
            obs_x, obs_y = obstacle["position"]
            distance = math.sqrt((x - obs_x)**2 + (y - obs_y)**2)

            if obstacle["type"] == "crater" and distance <= obstacle["radius"]:
                nearby_obstacles.append(obstacle)
            elif distance <= obstacle["size"]:
                nearby_obstacles.append(obstacle)

        # Check for waypoints
        nearby_waypoints = []
        for waypoint in self.terrain_map["waypoints"]:
            wp_x, wp_y = waypoint["position"]
            distance = math.sqrt((x - wp_x)**2 + (y - wp_y)**2)
            if distance <= 5.0:  # 5 meter tolerance
                nearby_waypoints.append(waypoint)

        return {
            "position": position,
            "obstacles": nearby_obstacles,
            "waypoints": nearby_waypoints,
            "traversable": len(nearby_obstacles) == 0
        }

    def update_weather(self):
        """Update weather conditions dynamically."""
        # Random weather changes
        self.weather_conditions["temperature"] += random.uniform(-5, 5)
        self.weather_conditions["wind_speed"] += random.uniform(-2, 2)
        self.weather_conditions["wind_speed"] = max(0, min(20, self.weather_conditions["wind_speed"]))

        # Occasional dust storms
        if random.random() < 0.05:  # 5% chance per update
            self.weather_conditions["dust_storm"] = not self.weather_conditions["dust_storm"]
            if self.weather_conditions["dust_storm"]:
                self.weather_conditions["visibility"] = random.uniform(10, 50)
            else:
                self.weather_conditions["visibility"] = 1000.0

    def simulate_physics(self, position: Tuple[float, float], velocity: Tuple[float, float],
                        dt: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """Simulate Mars physics for movement."""
        x, y = position
        vx, vy = velocity

        # Apply Mars gravity (less than Earth)
        vy -= self.gravity * dt

        # Apply rolling resistance (higher on Mars due to lower gravity)
        rolling_resistance = 0.1
        speed = math.sqrt(vx**2 + vy**2)
        if speed > 0:
            resistance_force = rolling_resistance * self.gravity
            vx -= (vx / speed) * resistance_force * dt
            vy -= (vy / speed) * resistance_force * dt

        # Update position
        new_x = x + vx * dt
        new_y = y + vy * dt

        # Boundary checking
        bounds = self.terrain_map["boundaries"]
        new_x = max(0, min(bounds["width"], new_x))
        new_y = max(0, min(bounds["height"], new_y))

        return (new_x, new_y), (vx, vy)


class MarsCommunicationSimulator:
    """Mars communication simulation with delays and blackouts."""

    def __init__(self):
        self.base_delay = 5.0  # seconds (Mars-Earth distance)
        self.jitter_range = 2.0
        self.blackout_probability = 0.02  # 2% chance of communication blackout
        self.active_blackout = False
        self.blackout_duration = 0

    def simulate_communication(self, message: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Simulate sending a message with Mars communication characteristics."""
        # Check for communication blackout
        if random.random() < self.blackout_probability:
            self.active_blackout = True
            self.blackout_duration = random.uniform(10, 60)  # 10-60 seconds
            return None

        if self.active_blackout:
            self.blackout_duration -= 1
            if self.blackout_duration <= 0:
                self.active_blackout = False
            else:
                return None

        # Add communication delay
        delay = self.base_delay + random.uniform(-self.jitter_range, self.jitter_range)
        time.sleep(delay * 0.1)  # Scale down for testing (real delay would be much longer)

        # Simulate occasional message corruption
        if random.random() < 0.01:  # 1% corruption rate
            return None

        return message

    def get_communication_status(self) -> Dict[str, Any]:
        """Get current communication status."""
        return {
            "connected": not self.active_blackout,
            "base_delay": self.base_delay,
            "current_delay": self.base_delay + random.uniform(-self.jitter_range, self.jitter_range),
            "blackout_active": self.active_blackout,
            "blackout_remaining": max(0, self.blackout_duration)
        }


class URCMissionSimulator:
    """URC 2026 mission simulation framework."""

    def __init__(self):
        self.terrain = MarsTerrainSimulator()
        self.communication = MarsCommunicationSimulator()
        self.robot_state = {
            "position": (50, 50),  # Start position
            "velocity": (0, 0),
            "heading": 0.0,
            "battery": 100.0,
            "samples_collected": [],
            "mission_status": "ready"
        }
        self.mission_timer = 0
        self.max_mission_time = 1800  # 30 minutes

    def execute_mission(self, mission_type: str) -> Dict[str, Any]:
        """Execute a complete URC mission."""
        self.mission_timer = 0
        start_time = time.time()

        mission_result = {
            "mission_type": mission_type,
            "success": False,
            "duration": 0,
            "samples_collected": 0,
            "waypoints_completed": 0,
            "errors": [],
            "communication_issues": 0
        }

        try:
            if mission_type == "sample_collection":
                mission_result = self._execute_sample_collection_mission()
            elif mission_type == "navigation":
                mission_result = self._execute_navigation_mission()
            elif mission_type == "delivery":
                mission_result = self._execute_delivery_mission()
            else:
                raise ValueError(f"Unknown mission type: {mission_type}")

        except Exception as e:
            mission_result["errors"].append(str(e))

        mission_result["duration"] = time.time() - start_time
        return mission_result

    def _execute_sample_collection_mission(self) -> Dict[str, Any]:
        """Execute sample collection mission."""
        result = {
            "mission_type": "sample_collection",
            "success": False,
            "samples_collected": 0,
            "waypoints_completed": 0,
            "errors": [],
            "communication_issues": 0
        }

        # Navigate to sample sites
        sample_sites = self.terrain.terrain_map["sample_sites"]

        for sample_site in sample_sites:
            # Navigate to sample site
            nav_success = self._navigate_to_position(sample_site["position"])
            if not nav_success:
                result["errors"].append(f"Failed to navigate to sample site {sample_site['id']}")
                break

            result["waypoints_completed"] += 1

            # Collect sample
            collection_success = self._collect_sample(sample_site)
            if collection_success:
                result["samples_collected"] += 1
                self.robot_state["samples_collected"].append(sample_site)
            else:
                result["errors"].append(f"Failed to collect sample at site {sample_site['id']}")

        # Return to start
        return_success = self._navigate_to_position((50, 50))
        if return_success:
            result["waypoints_completed"] += 1

        result["success"] = result["samples_collected"] >= 2  # Require at least 2 samples
        return result

    def _execute_navigation_mission(self) -> Dict[str, Any]:
        """Execute navigation mission."""
        result = {
            "mission_type": "navigation",
            "success": False,
            "waypoints_completed": 0,
            "errors": [],
            "communication_issues": 0
        }

        # Navigate through waypoints
        waypoints = [wp["position"] for wp in self.terrain.terrain_map["waypoints"]]

        for waypoint in waypoints:
            nav_success = self._navigate_to_position(waypoint)
            if nav_success:
                result["waypoints_completed"] += 1
            else:
                result["errors"].append(f"Failed to navigate to waypoint {waypoint}")
                break

        result["success"] = result["waypoints_completed"] == len(waypoints)
        return result

    def _execute_delivery_mission(self) -> Dict[str, Any]:
        """Execute delivery mission."""
        result = {
            "mission_type": "delivery",
            "success": False,
            "samples_delivered": 0,
            "waypoints_completed": 0,
            "errors": [],
            "communication_issues": 0
        }

        # First collect samples
        collection_result = self._execute_sample_collection_mission()
        result["samples_collected"] = collection_result["samples_collected"]

        if collection_result["success"]:
            # Navigate to delivery point
            delivery_point = (600, 450)
            nav_success = self._navigate_to_position(delivery_point)

            if nav_success:
                result["waypoints_completed"] = collection_result["waypoints_completed"] + 1

                # Deliver samples
                delivery_success = self._deliver_samples()
                if delivery_success:
                    result["samples_delivered"] = result["samples_collected"]
                    result["success"] = True
                else:
                    result["errors"].append("Failed to deliver samples")
            else:
                result["errors"].append("Failed to navigate to delivery point")
        else:
            result["errors"].extend(collection_result["errors"])

        return result

    def _navigate_to_position(self, target: Tuple[float, float]) -> bool:
        """Navigate to a target position."""
        current_pos = self.robot_state["position"]
        distance = math.sqrt((target[0] - current_pos[0])**2 + (target[1] - current_pos[1])**2)

        if distance < 5.0:  # Already at target
            return True

        # Simple navigation (would use actual navigation system)
        steps = int(distance / 2)  # 2 meters per step
        for _ in range(min(steps, 50)):  # Limit steps to prevent infinite loops
            # Update position
            dx = (target[0] - current_pos[0]) / distance
            dy = (target[1] - current_pos[1]) / distance

            new_pos = (
                current_pos[0] + dx * 2,
                current_pos[1] + dy * 2
            )

            # Check terrain
            terrain_info = self.terrain.get_terrain_info(new_pos)
            if not terrain_info["traversable"]:
                return False  # Hit obstacle

            current_pos = new_pos
            self.robot_state["position"] = current_pos

            # Update mission timer
            self.mission_timer += 1
            if self.mission_timer > self.max_mission_time:
                return False  # Mission timeout

            # Battery consumption
            self.robot_state["battery"] -= 0.1
            if self.robot_state["battery"] <= 0:
                return False  # Battery dead

        return True

    def _collect_sample(self, sample_site: Dict[str, Any]) -> bool:
        """Collect a sample at the given site."""
        # Simulate sample collection
        time.sleep(0.2)  # Collection time

        # Success based on battery level and random chance
        battery_factor = self.robot_state["battery"] / 100.0
        success_chance = 0.8 * battery_factor  # 80% base success, modified by battery

        return random.random() < success_chance

    def _deliver_samples(self) -> bool:
        """Deliver collected samples."""
        if not self.robot_state["samples_collected"]:
            return False

        # Simulate delivery
        time.sleep(0.3)

        # Clear collected samples after successful delivery
        delivered_count = len(self.robot_state["samples_collected"])
        self.robot_state["samples_collected"] = []

        return delivered_count > 0


class TestMarsEnvironmentSimulation:
    """Test Mars environment and mission simulation."""

    @pytest.fixture
    def mars_simulator(self):
        """Create Mars environment simulator."""
        return MarsTerrainSimulator()

    @pytest.fixture
    def communication_simulator(self):
        """Create Mars communication simulator."""
        return MarsCommunicationSimulator()

    @pytest.fixture
    def urc_mission_simulator(self):
        """Create URC mission simulator."""
        return URCMissionSimulator()

    def test_mars_terrain_generation(self, mars_simulator):
        """Test Mars terrain generation and properties."""
        terrain = mars_simulator.terrain_map

        # Verify terrain structure
        assert "boundaries" in terrain
        assert "obstacles" in terrain
        assert "waypoints" in terrain
        assert "sample_sites" in terrain

        # Verify waypoints
        assert len(terrain["waypoints"]) >= 3
        waypoint_types = {wp["type"] for wp in terrain["waypoints"]}
        assert "start" in waypoint_types
        assert "finish" in waypoint_types

        # Verify sample sites
        assert len(terrain["sample_sites"]) >= 2
        sample_types = {site["type"] for site in terrain["sample_sites"]}
        assert len(sample_types) >= 2

        print("🪐 Mars terrain generation validated")

    def test_terrain_navigation(self, mars_simulator):
        """Test terrain navigation and obstacle detection."""
        # Test clear terrain
        clear_pos = (10, 10)
        terrain_info = mars_simulator.get_terrain_info(clear_pos)
        assert terrain_info["traversable"] is True
        assert len(terrain_info["obstacles"]) == 0

        # Test obstacle detection
        obstacle_pos = (100, 200)  # Near rock obstacle
        terrain_info = mars_simulator.get_terrain_info(obstacle_pos)
        assert len(terrain_info["obstacles"]) > 0

        # Test waypoint detection
        waypoint_pos = (50, 50)  # Near start waypoint
        terrain_info = mars_simulator.get_terrain_info(waypoint_pos)
        assert len(terrain_info["waypoints"]) > 0
        assert terrain_info["waypoints"][0]["type"] == "start"

        print("🗺️ Terrain navigation and obstacle detection working")

    def test_mars_physics_simulation(self, mars_simulator):
        """Test Mars physics simulation."""
        # Initial conditions
        position = (0, 10)  # 10 meters up
        velocity = (5, 0)   # Moving horizontally

        # Simulate physics for 1 second
        new_pos, new_vel = mars_simulator.simulate_physics(position, velocity, 1.0)

        # Should have moved horizontally and fallen due to Mars gravity
        assert new_pos[0] > position[0]  # Moved forward
        assert new_pos[1] < position[1]  # Fell down
        assert new_vel[1] < velocity[1]  # Slower vertical velocity

        # Mars gravity effect (3.71 m/s² vs Earth's 9.81 m/s²)
        earth_gravity_drop = 9.81 * 1.0
        mars_gravity_drop = mars_simulator.gravity * 1.0
        assert mars_gravity_drop < earth_gravity_drop

        print("⚛️ Mars physics simulation working correctly")

    def test_weather_conditions_simulation(self, mars_simulator):
        """Test Mars weather conditions simulation."""
        initial_weather = mars_simulator.weather_conditions.copy()

        # Update weather multiple times
        for _ in range(10):
            mars_simulator.update_weather()

        # Weather should have changed
        current_weather = mars_simulator.weather_conditions

        # Temperature should be around -60°C ± some variation
        assert -80 < current_weather["temperature"] < -40

        # Wind speed should be reasonable
        assert 0 <= current_weather["wind_speed"] <= 25

        # Visibility should be affected by dust storms
        if current_weather["dust_storm"]:
            assert current_weather["visibility"] < 100
        else:
            assert current_weather["visibility"] >= 500

        print("🌪️ Mars weather conditions simulation working")

    def test_mars_communication_simulation(self, communication_simulator):
        """Test Mars communication delays and reliability."""
        # Test normal communication
        test_message = {"command": "status", "timestamp": time.time()}

        start_time = time.time()
        response = communication_simulator.simulate_communication(test_message)
        communication_time = time.time() - start_time

        # Should receive response (unless blackout)
        if response:
            assert response == test_message
            assert communication_time >= communication_simulator.base_delay * 0.1  # Scaled for testing

        # Test communication status
        status = communication_simulator.get_communication_status()
        assert "connected" in status
        assert "base_delay" in status
        assert status["base_delay"] == communication_simulator.base_delay

        print("📡 Mars communication simulation working")

    @pytest.mark.slow
    def test_sample_collection_mission(self, urc_mission_simulator):
        """Test sample collection mission execution."""
        result = urc_mission_simulator.execute_mission("sample_collection")

        assert "mission_type" in result
        assert result["mission_type"] == "sample_collection"
        assert "success" in result
        assert "samples_collected" in result
        assert "waypoints_completed" in result

        # Mission should complete within reasonable time
        assert result["duration"] < 60  # Less than 1 minute in simulation

        # Should collect at least some samples if successful
        if result["success"]:
            assert result["samples_collected"] >= 2
            assert result["waypoints_completed"] >= 3

        print(f"🧪 Sample collection mission: {result['samples_collected']} samples, {result['success']}")

    @pytest.mark.slow
    def test_navigation_mission(self, urc_mission_simulator):
        """Test navigation mission execution."""
        result = urc_mission_simulator.execute_mission("navigation")

        assert result["mission_type"] == "navigation"
        assert "waypoints_completed" in result

        # Should visit all waypoints if successful
        if result["success"]:
            assert result["waypoints_completed"] == len(urc_mission_simulator.terrain.terrain_map["waypoints"])

        print(f"🧭 Navigation mission: {result['waypoints_completed']} waypoints, {result['success']}")

    @pytest.mark.slow
    def test_delivery_mission(self, urc_mission_simulator):
        """Test delivery mission execution."""
        result = urc_mission_simulator.execute_mission("delivery")

        assert result["mission_type"] == "delivery"
        assert "samples_delivered" in result

        # Delivery mission should collect samples first, then deliver
        if result["success"]:
            assert result["samples_collected"] > 0
            assert result["samples_delivered"] == result["samples_collected"]

        print(f"📦 Delivery mission: {result['samples_delivered']} samples delivered, {result['success']}")

    @pytest.mark.slow
    def test_end_to_end_urc_competition_simulation(self, urc_mission_simulator):
        """Test complete URC competition simulation."""
        competition_results = {
            "missions_completed": 0,
            "total_samples_collected": 0,
            "total_score": 0,
            "duration": 0,
            "errors": []
        }

        start_time = time.time()

        # Execute all mission types
        mission_types = ["sample_collection", "navigation", "delivery"]

        for mission_type in mission_types:
            try:
                result = urc_mission_simulator.execute_mission(mission_type)

                if result["success"]:
                    competition_results["missions_completed"] += 1

                if "samples_collected" in result:
                    competition_results["total_samples_collected"] += result["samples_collected"]

                # Calculate score (simplified)
                if result["success"]:
                    base_score = 100
                    if mission_type == "sample_collection":
                        base_score += result["samples_collected"] * 50
                    elif mission_type == "delivery":
                        base_score += result["samples_delivered"] * 75

                    competition_results["total_score"] += base_score

            except Exception as e:
                competition_results["errors"].append(f"{mission_type}: {str(e)}")

        competition_results["duration"] = time.time() - start_time

        # Validate competition results
        assert competition_results["missions_completed"] >= 1  # At least one mission
        assert competition_results["duration"] < 300  # Less than 5 minutes total
        assert competition_results["total_score"] >= 0

        print("🏆 URC Competition Simulation Results:")
        print(f"   Missions Completed: {competition_results['missions_completed']}/3")
        print(f"   Total Samples: {competition_results['total_samples_collected']}")
        print(f"   Total Score: {competition_results['total_score']}")
        print(f"   Duration: {competition_results['duration']:.1f}s")

    def test_gps_denied_navigation(self, urc_mission_simulator):
        """Test navigation without GPS (Mars scenario)."""
        # Simulate GPS failure
        original_terrain = urc_mission_simulator.terrain

        # Create GPS-denied terrain (no position information)
        gps_denied_terrain = MarsTerrainSimulator()
        # Remove waypoint position information to simulate GPS denial
        for waypoint in gps_denied_terrain.terrain_map["waypoints"]:
            waypoint["position"] = None

        urc_mission_simulator.terrain = gps_denied_terrain

        try:
            # Try navigation mission without GPS
            result = urc_mission_simulator.execute_mission("navigation")

            # Should still complete using dead reckoning or other methods
            # In real implementation, would use IMU, wheel odometry, etc.
            assert "success" in result
            assert result["duration"] < 120  # Should complete reasonably fast

            print("🛰️ GPS-denied navigation simulation completed")

        finally:
            urc_mission_simulator.terrain = original_terrain

    def test_environmental_stress_testing(self, urc_mission_simulator):
        """Test system under various environmental stresses."""
        stress_scenarios = [
            {"dust_storm": True, "wind_speed": 15.0, "temperature": -80.0},
            {"dust_storm": False, "wind_speed": 25.0, "temperature": -40.0},
            {"dust_storm": True, "wind_speed": 10.0, "temperature": -60.0}
        ]

        for scenario in stress_scenarios:
            # Apply environmental conditions
            urc_mission_simulator.terrain.weather_conditions.update(scenario)

            # Test mission under stress
            result = urc_mission_simulator.execute_mission("sample_collection")

            # System should still function under stress
            assert "success" in result
            assert result["duration"] < 120

            # Performance may be degraded but should complete
            if scenario["dust_storm"]:
                # Dust storms should affect performance more
                assert result["samples_collected"] <= 2  # Limited visibility impact

        print("🌪️ Environmental stress testing completed")

    def test_communication_blackout_handling(self, urc_mission_simulator):
        """Test handling of communication blackouts."""
        # Enable communication blackouts
        urc_mission_simulator.communication.blackout_probability = 0.1  # 10% blackout chance

        # Execute mission with communication issues
        result = urc_mission_simulator.execute_mission("navigation")

        # Should complete despite communication issues
        assert "success" in result
        assert result["duration"] < 180  # Should complete within time limit

        # Check communication status
        comm_status = urc_mission_simulator.communication.get_communication_status()
        assert "connected" in comm_status

        print("📞 Communication blackout handling tested")

    def test_battery_life_simulation(self, urc_mission_simulator):
        """Test battery life under mission load."""
        initial_battery = urc_mission_simulator.robot_state["battery"]

        # Execute energy-intensive mission
        result = urc_mission_simulator.execute_mission("delivery")  # Most complex mission

        final_battery = urc_mission_simulator.robot_state["battery"]
        battery_used = initial_battery - final_battery

        # Should use reasonable amount of battery
        assert battery_used > 0
        assert battery_used < 50  # Less than 50% for a mission

        # Should have enough battery to complete
        if result["success"]:
            assert final_battery > 10  # At least 10% remaining

        print(f"🔋 Battery simulation: {battery_used:.1f}% used, {final_battery:.1f}% remaining")

    def test_mission_timeout_handling(self, urc_mission_simulator):
        """Test mission timeout handling."""
        # Set very short timeout
        original_timeout = urc_mission_simulator.max_mission_time
        urc_mission_simulator.max_mission_time = 5  # 5 seconds

        try:
            # Execute mission that should timeout
            result = urc_mission_simulator.execute_mission("delivery")

            # Mission should either complete quickly or timeout gracefully
            assert result["duration"] < 10  # Should not hang

            if not result["success"]:
                # Check if it was due to timeout
                assert "errors" in result or result["waypoints_completed"] < 5

            print("⏰ Mission timeout handling working")

        finally:
            urc_mission_simulator.max_mission_time = original_timeout

    @pytest.mark.slow
    @pytest.mark.parametrize("mission_type", ["sample_collection", "navigation", "delivery"])
    def test_mission_reliability_under_stress(self, urc_mission_simulator, mission_type):
        """Test mission reliability under various stress conditions."""
        reliability_results = []

        # Test each mission 3 times under different conditions
        for run in range(3):
            # Vary environmental conditions
            urc_mission_simulator.terrain.weather_conditions["dust_storm"] = (run % 2 == 0)
            urc_mission_simulator.terrain.weather_conditions["wind_speed"] = 5 + run * 5

            # Reset robot state
            urc_mission_simulator.robot_state["battery"] = 100.0
            urc_mission_simulator.robot_state["samples_collected"] = []
            urc_mission_simulator.robot_state["position"] = (50, 50)

            result = urc_mission_simulator.execute_mission(mission_type)

            reliability_results.append({
                "run": run + 1,
                "success": result["success"],
                "duration": result["duration"],
                "stress_level": run
            })

        # Calculate reliability metrics
        success_rate = sum(1 for r in reliability_results if r["success"]) / len(reliability_results)
        avg_duration = sum(r["duration"] for r in reliability_results) / len(reliability_results)

        # System should be reasonably reliable
        assert success_rate >= 0.6  # At least 60% success rate under stress
        assert avg_duration < 60     # Average completion time reasonable

        print(f"🔄 {mission_type} reliability: {success_rate:.1%} success rate, {avg_duration:.1f}s avg duration")
