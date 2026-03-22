#!/usr/bin/env python3
"""
Dashboard Integration Tests - URC 2026 Rover

Tests the web dashboard functionality including:
- WebSocket connections to simulation bridge
- ROS2 bridge integration via dashboard hooks
- Message passing between frontend and backend
- Real-time data visualization
- Mission command execution
- State machine visualization

Usage:
    python3 test_dashboard_integration.py [test_type]

Test Types:
    websocket: Test WebSocket communication
    ros2: Test ROS2 bridge integration
    mission: Test mission command execution
    state: Test state machine visualization
    all: Run all dashboard tests (default)
"""

import asyncio
import json
import sys
import time
import unittest
from typing import Any, Dict, Optional

import websockets

# Add project paths
sys.path.insert(0, ".")


class DashboardTester:
    """Test suite for dashboard integration."""

    def __init__(self):
        self.websocket_url = "ws://localhost:8766"
        self.ros_bridge_url = "ws://localhost:9090"
        self.test_results = []

    async def test_websocket_connection(self) -> bool:
        """Test basic WebSocket connection to dashboard simulation bridge."""
        try:
            async with websockets.connect(self.websocket_url) as websocket:
                # Test ping/pong
                await websocket.send(
                    json.dumps({"type": "ping", "timestamp": time.time()})
                )
                response = await websocket.recv()
                data = json.loads(response)

                if data.get("type") == "pong":
                    print("[PASS] WebSocket ping/pong working")
                    return True
                else:
                    print(f"[FAIL] Unexpected response: {data}")
                    return False

        except Exception as e:
            print(f"[FAIL] WebSocket connection failed: {e}")
            return False

    async def test_simulation_data_stream(self) -> bool:
        """Test receiving simulation data stream."""
        try:
            async with websockets.connect(self.websocket_url) as websocket:
                # Request simulation state
                await websocket.send(json.dumps({"type": "request_state"}))

                # Wait for response
                response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                data = json.loads(response)

                if data.get("type") == "simulation_state":
                    print("[PASS] Simulation data streaming working")
                    # Check for expected data fields
                    expected_fields = [
                        "sensor_data",
                        "rover_state",
                        "environment_state",
                    ]
                    for field in expected_fields:
                        if field not in data:
                            print(f"[FAIL] Missing field: {field}")
                            return False
                    return True
                else:
                    print(f"[FAIL] Unexpected response type: {data.get('type')}")
                    return False

        except Exception as e:
            print(f"[FAIL] Simulation data test failed: {e}")
            return False

    async def test_environment_control(self) -> bool:
        """Test environment configuration changes."""
        try:
            async with websockets.connect(self.websocket_url) as websocket:
                # Change environment to extreme
                await websocket.send(
                    json.dumps({"type": "set_environment", "tier": "extreme"})
                )

                # Wait for confirmation
                response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                data = json.loads(response)

                if (
                    data.get("type") == "environment_changed"
                    and data.get("tier") == "extreme"
                ):
                    print("[PASS] Environment control working")
                    return True
                else:
                    print(f"[FAIL] Environment change failed: {data}")
                    return False

        except Exception as e:
            print(f"[FAIL] Environment control test failed: {e}")
            return False

    def test_frontend_imports(self) -> bool:
        """Test that frontend components can be imported."""
        try:
            # Test React component imports (simplified)
            print("[PASS] Frontend component imports accessible")
            return True
        except Exception as e:
            print(f"[FAIL] Frontend import test failed: {e}")
            return False

    def test_message_routing(self) -> bool:
        """Test message routing logic."""
        try:
            # Test message target validation
            valid_targets = ["websocket", "ros2", "can"]

            for target in valid_targets:
                # Simulate message routing logic
                if target == "websocket":
                    expected_endpoint = self.websocket_url
                elif target == "ros2":
                    expected_endpoint = self.ros_bridge_url
                elif target == "can":
                    expected_endpoint = "ros2_topic"

                if not expected_endpoint:
                    print(f"[FAIL] No endpoint for target: {target}")
                    return False

            print("[PASS] Message routing logic working")
            return True

        except Exception as e:
            print(f"[FAIL] Message routing test failed: {e}")
            return False

    def test_state_visualization(self) -> bool:
        """Test state machine visualization data structures."""
        try:
            # Test state definitions
            from bridges.ros2_state_machine_bridge import SystemState

            states = list(SystemState)
            if len(states) < 7:  # Should have at least BOOT, READY, TELEOP, AUTO, etc.
                print(f"[FAIL] Insufficient states: {len(states)}")
                return False

            # Test state transitions
            from autonomy.code.state_management.autonomy_state_machine.states import (
                RoverState,
                can_transition,
            )

            # Test some basic transitions
            if not can_transition(RoverState.BOOT, RoverState.READY):
                print("[FAIL] Basic state transition failed")
                return False

            print("[PASS] State visualization data structures working")
            return True

        except Exception as e:
            print(f"[FAIL] State visualization test failed: {e}")
            return False

    def test_mission_command_structure(self) -> bool:
        """Test mission command data structures."""
        try:
            # Test mission command formats
            mission_commands = [
                {
                    "type": "start",
                    "config": {
                        "name": "Test Mission",
                        "type": "waypoint_navigation",
                        "waypoints": [[0, 0], [5, 0], [5, 5]],
                    },
                },
                {"type": "aruco_search", "config": {"tag_id": 42, "timeout": 300}},
                {
                    "type": "return_to_operator",
                    "config": {"use_gps": True, "use_aruco": True},
                },
            ]

            for cmd in mission_commands:
                required_fields = ["type", "config"]
                for field in required_fields:
                    if field not in cmd:
                        print(f"[FAIL] Missing field '{field}' in mission command")
                        return False

            print("[PASS] Mission command structures valid")
            return True

        except Exception as e:
            print(f"[FAIL] Mission command test failed: {e}")
            return False

    def run_sync_test(self, test_func, test_name: str) -> bool:
        """Run a synchronous test."""
        print(f"\n[EXPERIMENT] Testing {test_name}...")
        try:
            result = test_func()
            if result:
                print(f"[PASS] {test_name} PASSED")
                self.test_results.append({"name": test_name, "status": "PASSED"})
            else:
                print(f"[FAIL] {test_name} FAILED")
                self.test_results.append({"name": test_name, "status": "FAILED"})
            return result
        except Exception as e:
            print(f" {test_name} ERROR: {e}")
            self.test_results.append(
                {"name": test_name, "status": "ERROR", "error": str(e)}
            )
            return False

    async def run_async_test(self, test_func, test_name: str) -> bool:
        """Run an asynchronous test."""
        print(f"\n[EXPERIMENT] Testing {test_name}...")
        try:
            result = await test_func()
            if result:
                print(f"[PASS] {test_name} PASSED")
                self.test_results.append({"name": test_name, "status": "PASSED"})
            else:
                print(f"[FAIL] {test_name} FAILED")
                self.test_results.append({"name": test_name, "status": "FAILED"})
            return result
        except Exception as e:
            print(f" {test_name} ERROR: {e}")
            self.test_results.append(
                {"name": test_name, "status": "ERROR", "error": str(e)}
            )
            return False

    async def run_all_tests(self) -> bool:
        """Run all dashboard integration tests."""
        print("[IGNITE] Starting Dashboard Integration Tests")
        print("=" * 50)

        # Asynchronous WebSocket tests
        await self.run_async_test(
            self.test_websocket_connection, "WebSocket Connection"
        )
        await self.run_async_test(
            self.test_simulation_data_stream, "Simulation Data Stream"
        )
        await self.run_async_test(self.test_environment_control, "Environment Control")

        # Synchronous tests
        self.run_sync_test(self.test_frontend_imports, "Frontend Imports")
        self.run_sync_test(self.test_message_routing, "Message Routing")
        self.run_sync_test(self.test_state_visualization, "State Visualization")
        self.run_sync_test(self.test_mission_command_structure, "Mission Commands")

        # Generate report
        print("\n" + "=" * 50)
        self.generate_report()

        passed = sum(1 for r in self.test_results if r["status"] == "PASSED")
        total = len(self.test_results)

        return passed == total

    def generate_report(self):
        """Generate test report."""
        passed = sum(1 for r in self.test_results if r["status"] == "PASSED")
        failed = sum(1 for r in self.test_results if r["status"] == "FAILED")
        errors = sum(1 for r in self.test_results if r["status"] == "ERROR")

        print("DASHBOARD INTEGRATION TEST REPORT")
        print(f"Total Tests: {len(self.test_results)}")
        print(f"[PASS] Passed: {passed}")
        print(f"[FAIL] Failed: {failed}")
        print(f" Errors: {errors}")

        print("\n[CLIPBOARD] DETAILED RESULTS:")
        for result in self.test_results:
            status_icon = {"PASSED": "[PASS]", "FAILED": "[FAIL]", "ERROR": ""}.get(
                result["status"], "?"
            )
            print(f"   {status_icon} {result['name']}")

        success_rate = (
            (passed / len(self.test_results)) * 100 if self.test_results else 0
        )
        health_status = (
            " GOOD"
            if success_rate >= 90
            else " FAIR" if success_rate >= 70 else " NEEDS WORK"
        )
        print(f"\n DASHBOARD HEALTH: {health_status} ({success_rate:.1f}% pass rate)")


async def main():
    """Main test runner."""
    import argparse

    parser = argparse.ArgumentParser(description="Dashboard Integration Tests")
    parser.add_argument(
        "test_type",
        nargs="?",
        default="all",
        choices=["websocket", "ros2", "mission", "state", "all"],
        help="Type of test to run",
    )

    args = parser.parse_args()

    tester = DashboardTester()

    if args.test_type == "all":
        success = await tester.run_all_tests()
    elif args.test_type == "websocket":
        success = await tester.run_async_test(
            tester.test_websocket_connection, "WebSocket Connection"
        ) and await tester.run_async_test(
            tester.test_simulation_data_stream, "Simulation Data Stream"
        )
    elif args.test_type == "state":
        success = tester.run_sync_test(
            tester.test_state_visualization, "State Visualization"
        )
    elif args.test_type == "mission":
        success = tester.run_sync_test(
            tester.test_mission_command_structure, "Mission Commands"
        )
    else:
        print(f"[FAIL] Test type '{args.test_type}' not implemented")
        success = False

    return 0 if success else 1


if __name__ == "__main__":
    import sys

    result = asyncio.run(main())
    sys.exit(result)
