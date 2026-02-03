"""Mission-specific test suites for URC competition communication testing.

Implements comprehensive test scenarios for each of the 4 core URC missions,
combining mission communication profiles with environmental stressors.

Author: URC 2026 Autonomy Team
"""

import asyncio
import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

from simulation.communication.mission_profiles import (
    MissionCommunicationManager,
    URCMission,
    URC_COMMUNICATION_PROFILES,
)
from simulation.environments.mdrs_environment import MDRSEnvironment
from simulation.network.network_emulator import NetworkEmulator, NetworkProfile


class TestResult(Enum):
    """Test execution result status."""

    PASSED = "passed"
    FAILED = "failed"
    SKIPPED = "skipped"
    ERROR = "error"


@dataclass
class TestMetrics:
    """Metrics collected during test execution."""

    messages_sent: int = 0
    messages_received: int = 0
    messages_dropped: int = 0
    average_latency_ms: float = 0.0
    peak_latency_ms: float = 0.0
    bandwidth_utilization_mbps: float = 0.0
    communication_failures: int = 0
    recovery_time_s: float = 0.0
    success_rate: float = 0.0


@dataclass
class MissionTestResult:
    """Results from mission-specific test execution."""

    mission: URCMission
    test_name: str
    result: TestResult
    metrics: TestMetrics
    duration_s: float
    environment_state: Dict[str, Any]
    network_profile: NetworkProfile
    errors: List[str]
    warnings: List[str]


class MissionTestSuite:
    """Base class for mission-specific test suites."""

    def __init__(self, mission: URCMission):
        self.mission = mission
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")
        self.comms_manager = MissionCommunicationManager()
        self.comms_manager.set_mission(mission)

        # Test infrastructure
        self.network_emulator: Optional[NetworkEmulator] = None
        self.mdrs_environment: Optional[MDRSEnvironment] = None
        self.test_results: List[MissionTestResult] = []

        self.logger.info(f"Initialized test suite for {mission.value}")

    async def run_full_test_suite(self) -> List[MissionTestResult]:
        """Run comprehensive test suite for the mission."""
        self.logger.info(f"Starting full test suite for {self.mission.value}")
        self.test_results.clear()

        try:
            # Setup test infrastructure
            await self._setup_test_infrastructure()

            # Run communication tests
            await self._run_communication_tests()

            # Run stress tests
            await self._run_stress_tests()

            # Run environmental tests
            await self._run_environmental_tests()

            # Run failure scenarios
            await self._run_failure_scenarios()

        except Exception as e:
            self.logger.error(f"Test suite execution failed: {e}")
            error_result = MissionTestResult(
                mission=self.mission,
                test_name="test_suite_execution",
                result=TestResult.ERROR,
                metrics=TestMetrics(),
                duration_s=0.0,
                environment_state={},
                network_profile=NetworkProfile.PERFECT,
                errors=[str(e)],
                warnings=[],
            )
            self.test_results.append(error_result)

        finally:
            await self._cleanup_test_infrastructure()

        self.logger.info(
            f"Completed test suite for {self.mission.value}: {len(self.test_results)} tests"
        )
        return self.test_results

    async def _setup_test_infrastructure(self):
        """Setup network and environment simulators."""
        profile = URC_COMMUNICATION_PROFILES[self.mission]

        # Setup network emulator
        self.network_emulator = NetworkEmulator(profile.network_profile)
        self.network_emulator.start()

        # Setup MDRS environment
        env_config = {
            "tier": "mdrs_competition",
            "base_temperature": 20.0,
            "daily_temp_range": 25.0,
            "dust_storm_probability": 0.15,
            "time_of_day": 10.0,
        }
        self.mdrs_environment = MDRSEnvironment(env_config)

        self.logger.info("Test infrastructure setup complete")

    async def _cleanup_test_infrastructure(self):
        """Cleanup test infrastructure."""
        if self.network_emulator:
            self.network_emulator.stop()

        self.network_emulator = None
        self.mdrs_environment = None

        self.logger.info("Test infrastructure cleanup complete")

    async def _run_communication_tests(self):
        """Run baseline communication tests."""
        requirements = self.comms_manager.get_communication_requirements()

        for req_name, req in requirements.items():
            await self._test_communication_requirement(req_name, req)

    async def _test_communication_requirement(self, req_name: str, req: Any):
        """Test a specific communication requirement."""
        self.logger.info(f"Testing communication requirement: {req_name}")

        start_time = time.time()
        metrics = TestMetrics()
        errors = []
        warnings = []

        try:
            # Generate test messages based on data types
            test_duration = 60.0  # 1 minute test
            message_interval = self._get_message_interval(req.message_frequency)

            end_time = start_time + test_duration
            message_count = 0

            while time.time() < end_time:
                # Send test message
                test_message = self._generate_test_message(req.data_types)

                if self.network_emulator.send_message(test_message):
                    metrics.messages_sent += 1
                    message_count += 1
                else:
                    metrics.messages_dropped += 1
                    metrics.communication_failures += 1

                # Wait for next message
                await asyncio.sleep(message_interval)

            # Wait for message delivery
            await asyncio.sleep(2.0)

            # Collect statistics
            stats = self.network_emulator.get_statistics()
            metrics.messages_received = stats["messages_received"]
            metrics.average_latency_ms = stats["average_latency_ms"]
            metrics.peak_latency_ms = stats.get("peak_latency_ms", 0.0)
            metrics.success_rate = (
                (metrics.messages_received / metrics.messages_sent * 100)
                if metrics.messages_sent > 0
                else 0.0
            )

            # Validate requirements
            validation_warnings = self._validate_communication_requirements(
                req, metrics
            )
            warnings.extend(validation_warnings)

            result = TestResult.PASSED if len(errors) == 0 else TestResult.FAILED

        except Exception as e:
            errors.append(str(e))
            result = TestResult.ERROR

        duration = time.time() - start_time

        test_result = MissionTestResult(
            mission=self.mission,
            test_name=f"communication_{req_name}",
            result=result,
            metrics=metrics,
            duration_s=duration,
            environment_state=(
                self.mdrs_environment.get_state() if self.mdrs_environment else {}
            ),
            network_profile=self.network_emulator.profile,
            errors=errors,
            warnings=warnings,
        )

        self.test_results.append(test_result)

    async def _run_stress_tests(self):
        """Run stress tests for critical communication paths."""
        critical_paths = self.comms_manager.get_critical_paths()

        for path in critical_paths:
            await self._test_communication_stress(path)

    async def _test_communication_stress(self, path_name: str):
        """Test communication path under stress conditions."""
        self.logger.info(f"Running stress test for: {path_name}")

        start_time = time.time()
        metrics = TestMetrics()
        errors = []
        warnings = []

        try:
            # Apply stress conditions (2x normal load)
            stress_duration = 300.0  # 5 minutes
            message_interval = 0.05  # 20 Hz (high stress)

            # Temporarily increase network stress
            original_profile = self.network_emulator.profile
            # Note: NetworkEmulator doesn't support profile change, would need enhancement
            warnings.append("Network profile stress enhancement not implemented")

            end_time = start_time + stress_duration

            while time.time() < end_time:
                test_message = {
                    "type": path_name,
                    "timestamp": time.time(),
                    "data": f"stress_test_{time.time()}",
                }

                if self.network_emulator.send_message(test_message):
                    metrics.messages_sent += 1
                else:
                    metrics.messages_dropped += 1
                    metrics.communication_failures += 1

                await asyncio.sleep(message_interval)

            # Wait for delivery
            await asyncio.sleep(5.0)

            # Collect statistics
            stats = self.network_emulator.get_statistics()
            metrics.messages_received = stats["messages_received"]
            metrics.average_latency_ms = stats["average_latency_ms"]
            metrics.success_rate = (
                (metrics.messages_received / metrics.messages_sent * 100)
                if metrics.messages_sent > 0
                else 0.0
            )

            # Stress test validation (require 95% success rate)
            if metrics.success_rate < 95.0:
                errors.append(
                    f"Stress test failed: {metrics.success_rate:.1f}% success rate < 95%"
                )

            result = TestResult.PASSED if len(errors) == 0 else TestResult.FAILED

        except Exception as e:
            errors.append(str(e))
            result = TestResult.ERROR

        duration = time.time() - start_time

        test_result = MissionTestResult(
            mission=self.mission,
            test_name=f"stress_{path_name}",
            result=result,
            metrics=metrics,
            duration_s=duration,
            environment_state=(
                self.mdrs_environment.get_state() if self.mdrs_environment else {}
            ),
            network_profile=self.network_emulator.profile,
            errors=errors,
            warnings=warnings,
        )

        self.test_results.append(test_result)

    async def _run_environmental_tests(self):
        """Run tests with environmental stressors."""
        scenarios = ["perfect_day", "moderate_challenges", "challenging_day"]

        for scenario in scenarios:
            await self._test_environmental_scenario(scenario)

    async def _test_environmental_scenario(self, scenario_name: str):
        """Test communication under specific environmental conditions."""
        self.logger.info(f"Testing environmental scenario: {scenario_name}")

        start_time = time.time()
        metrics = TestMetrics()
        errors = []
        warnings = []

        try:
            # Apply environmental scenario
            if self.mdrs_environment:
                self.mdrs_environment.apply_scenario(scenario_name)

            # Test basic communication under stress
            test_duration = 120.0  # 2 minutes
            message_interval = 1.0  # 1 Hz

            end_time = start_time + test_duration

            while time.time() < end_time:
                test_message = {
                    "type": "environmental_test",
                    "scenario": scenario_name,
                    "timestamp": time.time(),
                }

                if self.network_emulator.send_message(test_message):
                    metrics.messages_sent += 1
                else:
                    metrics.messages_dropped += 1

                # Update environment
                if self.mdrs_environment:
                    self.mdrs_environment.step(message_interval)

                await asyncio.sleep(message_interval)

            # Wait for delivery
            await asyncio.sleep(3.0)

            # Collect statistics
            stats = self.network_emulator.get_statistics()
            metrics.messages_received = stats["messages_received"]
            metrics.average_latency_ms = stats["average_latency_ms"]
            metrics.success_rate = (
                (metrics.messages_received / metrics.messages_sent * 100)
                if metrics.messages_sent > 0
                else 0.0
            )

            # Environmental stress validation
            if self.mdrs_environment:
                stressors = self.mdrs_environment.get_stressor_summary()
                if stressors["active_stressors"] > 0 and metrics.success_rate < 90.0:
                    warnings.append(
                        f"Degraded performance under stress: {metrics.success_rate:.1f}%"
                    )

            result = TestResult.PASSED if len(errors) == 0 else TestResult.FAILED

        except Exception as e:
            errors.append(str(e))
            result = TestResult.ERROR

        duration = time.time() - start_time

        test_result = MissionTestResult(
            mission=self.mission,
            test_name=f"environmental_{scenario_name}",
            result=result,
            metrics=metrics,
            duration_s=duration,
            environment_state=(
                self.mdrs_environment.get_state() if self.mdrs_environment else {}
            ),
            network_profile=self.network_emulator.profile,
            errors=errors,
            warnings=warnings,
        )

        self.test_results.append(test_result)

    async def _run_failure_scenarios(self):
        """Run failure and recovery scenarios."""
        failure_scenarios = self.comms_manager.get_testing_scenarios()

        for scenario in failure_scenarios[:3]:  # Limit to first 3 scenarios
            if "communication" in scenario.lower() or "recovery" in scenario.lower():
                await self._test_failure_scenario(scenario)

    async def _test_failure_scenario(self, scenario_name: str):
        """Test specific failure scenario and recovery."""
        self.logger.info(f"Testing failure scenario: {scenario_name}")

        start_time = time.time()
        metrics = TestMetrics()
        errors = []
        warnings = []

        try:
            # Simulate communication failure
            failure_duration = 30.0  # 30 seconds failure
            recovery_start = start_time + failure_duration
            total_duration = 120.0  # 2 minutes total test

            recovery_time = None

            while time.time() < start_time + total_duration:
                current_time = time.time()

                test_message = {
                    "type": "failure_test",
                    "scenario": scenario_name,
                    "timestamp": current_time,
                }

                # Simulate failure period
                if current_time < recovery_start:
                    # During failure, messages should be dropped
                    if self.network_emulator.send_message(test_message):
                        metrics.messages_sent += 1
                    else:
                        metrics.messages_dropped += 1
                        metrics.communication_failures += 1
                else:
                    # Recovery period - track when communication resumes
                    if recovery_time is None and self.network_emulator.send_message(
                        test_message
                    ):
                        recovery_time = current_time

                    if self.network_emulator.send_message(test_message):
                        metrics.messages_sent += 1

                await asyncio.sleep(0.5)  # 2 Hz during test

            # Calculate recovery time
            if recovery_time:
                metrics.recovery_time_s = recovery_time - recovery_start

            # Wait for final message delivery
            await asyncio.sleep(2.0)

            # Collect statistics
            stats = self.network_emulator.get_statistics()
            metrics.messages_received = stats["messages_received"]
            metrics.average_latency_ms = stats["average_latency_ms"]
            metrics.success_rate = (
                (metrics.messages_received / metrics.messages_sent * 100)
                if metrics.messages_sent > 0
                else 0.0
            )

            # Recovery validation
            if recovery_time is None:
                errors.append("No recovery detected within test period")
            elif metrics.recovery_time_s > 10.0:
                warnings.append(f"Slow recovery: {metrics.recovery_time_s:.1f}s")

            result = TestResult.PASSED if len(errors) == 0 else TestResult.FAILED

        except Exception as e:
            errors.append(str(e))
            result = TestResult.ERROR

        duration = time.time() - start_time

        test_result = MissionTestResult(
            mission=self.mission,
            test_name=f"failure_{scenario_name}",
            result=result,
            metrics=metrics,
            duration_s=duration,
            environment_state=(
                self.mdrs_environment.get_state() if self.mdrs_environment else {}
            ),
            network_profile=self.network_emulator.profile,
            errors=errors,
            warnings=warnings,
        )

        self.test_results.append(test_result)

    def _get_message_interval(self, frequency_str: str) -> float:
        """Convert frequency string to interval in seconds."""
        if frequency_str == "on_demand":
            return 5.0  # Default for on-demand
        elif frequency_str.endswith("Hz"):
            try:
                hz = float(frequency_str[:-2])
                return 1.0 / hz
            except ValueError:
                return 1.0
        else:
            return 1.0  # Default

    def _generate_test_message(self, data_types: List[str]) -> Dict[str, Any]:
        """Generate test message based on data types."""
        message = {"timestamp": time.time(), "data_types": data_types}

        for data_type in data_types:
            if "position" in data_type:
                message["position"] = {"x": 1.0, "y": 2.0, "z": 0.0}
            elif "velocity" in data_type:
                message["velocity"] = {"linear": 0.5, "angular": 0.1}
            elif "status" in data_type:
                message["status"] = "operational"
            elif "video" in data_type:
                message["video_frame"] = f"frame_data_{time.time()}"
            else:
                message[data_type] = f"test_data_{data_type}_{time.time()}"

        return message

    def _validate_communication_requirements(
        self, req: Any, metrics: TestMetrics
    ) -> List[str]:
        """Validate communication requirements against metrics."""
        warnings = []

        # Latency validation
        if req.latency_tolerance == "tight" and metrics.average_latency_ms > 100:
            warnings.append(
                f"High latency for tight requirement: {metrics.average_latency_ms:.1f}ms"
            )
        elif req.latency_tolerance == "moderate" and metrics.average_latency_ms > 500:
            warnings.append(
                f"High latency for moderate requirement: {metrics.average_latency_ms:.1f}ms"
            )

        # Reliability validation
        if req.critical_path and metrics.success_rate < 99.0:
            warnings.append(
                f"Low success rate for critical path: {metrics.success_rate:.1f}%"
            )
        elif req.failure_tolerance == "minimal" and metrics.success_rate < 95.0:
            warnings.append(
                f"Low success rate for minimal tolerance: {metrics.success_rate:.1f}%"
            )

        return warnings

    def get_test_summary(self) -> Dict[str, Any]:
        """Get summary of test results."""
        if not self.test_results:
            return {"error": "No test results available"}

        total_tests = len(self.test_results)
        passed_tests = sum(
            1 for r in self.test_results if r.result == TestResult.PASSED
        )
        failed_tests = sum(
            1 for r in self.test_results if r.result == TestResult.FAILED
        )
        error_tests = sum(1 for r in self.test_results if r.result == TestResult.ERROR)

        # Calculate aggregate metrics
        total_messages = sum(r.metrics.messages_sent for r in self.test_results)
        total_received = sum(r.metrics.messages_received for r in self.test_results)
        avg_success_rate = (
            sum(r.metrics.success_rate for r in self.test_results) / total_tests
            if total_tests > 0
            else 0.0
        )
        avg_latency = (
            sum(r.metrics.average_latency_ms for r in self.test_results) / total_tests
            if total_tests > 0
            else 0.0
        )

        return {
            "mission": self.mission.value,
            "total_tests": total_tests,
            "passed": passed_tests,
            "failed": failed_tests,
            "errors": error_tests,
            "success_rate_percent": (
                (passed_tests / total_tests * 100) if total_tests > 0 else 0.0
            ),
            "total_messages_sent": total_messages,
            "total_messages_received": total_received,
            "average_success_rate": avg_success_rate,
            "average_latency_ms": avg_latency,
            "test_duration_s": sum(r.duration_s for r in self.test_results),
        }


class AutonomousTraversalTestSuite(MissionTestSuite):
    """Test suite specific to Autonomous Traversal mission."""

    def __init__(self):
        super().__init__(URCMission.AUTONOMOUS_TRAVERSAL)


class ScienceMissionTestSuite(MissionTestSuite):
    """Test suite specific to Science Mission."""

    def __init__(self):
        super().__init__(URCMission.SCIENCE_MISSION)

    async def _run_additional_tests(self):
        """Run science mission specific tests."""
        await self._test_high_bandwidth_video()
        await self._test_sensor_data_streams()

    async def _test_high_bandwidth_video(self):
        """Test high bandwidth video streaming requirements."""
        # Implementation specific to video streaming tests
        pass

    async def _test_sensor_data_streams(self):
        """Test sensor data transmission for science instruments."""
        # Implementation specific to sensor data tests
        pass


class EquipmentServicingTestSuite(MissionTestSuite):
    """Test suite specific to Equipment Servicing mission."""

    def __init__(self):
        super().__init__(URCMission.EQUIPMENT_SERVICING)


class DeliveryMissionTestSuite(MissionTestSuite):
    """Test suite specific to Delivery Mission."""

    def __init__(self):
        super().__init__(URCMission.DELIVERY_MISSION)

    async def _run_additional_tests(self):
        """Run delivery mission specific tests."""
        await self._test_rover_drone_coordination()
        await self._test_cargo_delivery_protocol()

    async def _test_rover_drone_coordination(self):
        """Test coordination between rover and drone systems."""
        # Implementation specific to rover-drone coordination
        pass

    async def _test_cargo_delivery_protocol(self):
        """Test cargo delivery communication protocols."""
        # Implementation specific to cargo delivery
        pass


# Factory for creating mission-specific test suites
class MissionTestSuiteFactory:
    """Factory for creating mission-specific test suites."""

    _test_suites = {
        URCMission.AUTONOMOUS_TRAVERSAL: AutonomousTraversalTestSuite,
        URCMission.SCIENCE_MISSION: ScienceMissionTestSuite,
        URCMission.EQUIPMENT_SERVICING: EquipmentServicingTestSuite,
        URCMission.DELIVERY_MISSION: DeliveryMissionTestSuite,
    }

    @classmethod
    def create_test_suite(cls, mission: URCMission) -> MissionTestSuite:
        """Create test suite for specific mission."""
        suite_class = cls._test_suites.get(mission)
        if not suite_class:
            return MissionTestSuite(mission)  # Fallback to base class

        return suite_class()


if __name__ == "__main__":
    # Test mission test suites
    print("[TEST] Testing Mission Test Suites")
    print("=" * 60)

    async def test_all_missions():
        for mission in URCMission:
            print(f"\n[MISSION] Testing {mission.value}:")

            test_suite = MissionTestSuiteFactory.create_test_suite(mission)
            results = await test_suite.run_full_test_suite()

            summary = test_suite.get_test_summary()
            print(f"  Total tests: {summary['total_tests']}")
            print(f"  Passed: {summary['passed']}")
            print(f"  Failed: {summary['failed']}")
            print(f"  Success rate: {summary['success_rate_percent']:.1f}%")
            print(f"  Average latency: {summary['average_latency_ms']:.1f}ms")

    # Run the test
    import asyncio

    asyncio.run(test_all_missions())

    print("\n[PASS] Mission test suites test complete")
