#!/usr/bin/env python3
"""
Emergency Systems Optimization Tests - URC 2026

Tests emergency system performance optimizations including:
- Response time validation (<10ms from command to motor stop)
- Override priority verification (emergency overrides all commands)
- Recovery procedure validation (safe transition back to operational)
- False trigger rate monitoring (<0.1% under normal operation)
- Hardware interlock verification (mechanical brake engagement)

Author: URC 2026 Emergency Systems Optimization Team
"""

import time
import threading
import statistics
from typing import Dict, List, Any, Optional, Callable
import sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.core.safety_system import SafetySystem


class EmergencySystemsOptimizationTester:
    """Comprehensive emergency systems optimization testing."""

    def __init__(self):
        self.test_results = {}
        self.baseline_measurements = {}
        self.response_times = []
        self.false_triggers = []

    def run_comprehensive_emergency_tests(self) -> Dict[str, Any]:
        """Run comprehensive emergency systems optimization tests."""

        print("🚨 Starting Emergency Systems Optimization Tests")
        print("=" * 60)

        # Test response time validation
        self._test_response_time_validation()

        # Test override priority verification
        self._test_override_priority_verification()

        # Test recovery procedure validation
        self._test_recovery_procedure_validation()

        # Test false trigger rate monitoring
        self._test_false_trigger_rate_monitoring()

        # Test hardware interlock verification
        self._test_hardware_interlock_verification()

        # Generate comprehensive report
        return self._generate_emergency_test_report()

    def _test_response_time_validation(self):
        """Test response time validation (<10ms from command to motor stop)."""
        print("⏱️ Testing Emergency Response Time Validation...")

        try:
            safety_system = SafetySystem()
            safety_system.initialize()

            # Test emergency stop response times
            response_times = []
            test_iterations = 100  # Statistical significance

            print(f"  Testing {test_iterations} emergency stops...")

            for i in range(test_iterations):
                # Prepare system (simulate normal operation)
                safety_system.clear_emergency()

                # Small delay to ensure stable state
                time.sleep(0.001)

                # Measure emergency stop response time
                response_start = time.perf_counter()

                # Trigger emergency stop
                success = safety_system.emergency_stop("test_response_time")

                # Measure time to response confirmation
                response_end = time.perf_counter()
                response_time_ms = (response_end - response_start) * 1000

                response_times.append(response_time_ms)

                # Verify emergency state
                if not safety_system.is_emergency_active():
                    print(f"  ⚠️ Emergency not activated on iteration {i+1}")

                # Brief cooldown
                time.sleep(0.01)

            # Statistical analysis
            response_time_analysis = {
                "test_iterations": test_iterations,
                "average_response_time_ms": statistics.mean(response_times),
                "p50_response_time_ms": statistics.median(response_times),
                "p95_response_time_ms": statistics.quantiles(response_times, n=20)[18],
                "p99_response_time_ms": statistics.quantiles(response_times, n=100)[98],
                "max_response_time_ms": max(response_times),
                "min_response_time_ms": min(response_times),
                "response_time_std_dev": statistics.stdev(response_times),
                "violations_over_10ms": len([t for t in response_times if t > 10.0]),
                "violation_rate_percent": (len([t for t in response_times if t > 10.0]) / test_iterations) * 100,
                "meets_10ms_requirement": statistics.quantiles(response_times, n=20)[18] < 10.0,
                "meets_5ms_target": statistics.quantiles(response_times, n=20)[18] < 5.0,  # Aggressive target
                "response_time_stability": statistics.stdev(response_times) / statistics.mean(response_times)
            }

            self.test_results["response_time_validation"] = response_time_analysis

            status = "✅ PASS" if response_time_analysis["meets_10ms_requirement"] else "❌ FAIL"
            print(".2f")
            if response_time_analysis["violations_over_10ms"] > 0:
                print(f"  ⚠️ {response_time_analysis['violations_over_10ms']} violations detected")

            safety_system.cleanup()

        except Exception as e:
            print(f"⚠️ Response time validation error: {e}")
            self.test_results["response_time_validation"] = {"error": str(e)}

    def _test_override_priority_verification(self):
        """Test override priority verification (emergency overrides all commands)."""
        print("🔴 Testing Emergency Override Priority Verification...")

        try:
            safety_system = SafetySystem()
            safety_system.initialize()

            # Simulate concurrent command scenario
            override_test_results = []
            test_scenarios = [
                {"name": "motion_command_during_emergency", "motion_cmd": True, "emergency_active": True},
                {"name": "navigation_command_during_emergency", "nav_cmd": True, "emergency_active": True},
                {"name": "arm_command_during_emergency", "arm_cmd": True, "emergency_active": True},
                {"name": "normal_operation_after_clear", "motion_cmd": True, "emergency_active": False},
            ]

            for scenario in test_scenarios:
                print(f"  Testing scenario: {scenario['name']}")

                # Set emergency state
                if scenario.get("emergency_active", False):
                    safety_system.emergency_stop("override_test")
                else:
                    safety_system.clear_emergency()

                # Test command execution
                commands_blocked = 0
                commands_allowed = 0

                # Try to execute various commands
                if scenario.get("motion_cmd", False):
                    motion_allowed = safety_system.allow_motion_command()
                    if scenario["emergency_active"]:
                        commands_blocked += 1 if not motion_allowed else 0
                    else:
                        commands_allowed += 1 if motion_allowed else 0

                if scenario.get("nav_cmd", False):
                    nav_allowed = safety_system.allow_navigation_command()
                    if scenario["emergency_active"]:
                        commands_blocked += 1 if not nav_allowed else 0
                    else:
                        commands_allowed += 1 if nav_allowed else 0

                if scenario.get("arm_cmd", False):
                    arm_allowed = safety_system.allow_arm_command()
                    if scenario["emergency_active"]:
                        commands_blocked += 1 if not arm_allowed else 0
                    else:
                        commands_allowed += 1 if arm_allowed else 0

                scenario_result = {
                    "scenario": scenario["name"],
                    "emergency_active": scenario.get("emergency_active", False),
                    "commands_blocked": commands_blocked,
                    "commands_allowed": commands_allowed,
                    "override_working": (scenario.get("emergency_active", False) and commands_blocked > 0) or
                                      (not scenario.get("emergency_active", False) and commands_allowed > 0)
                }

                override_test_results.append(scenario_result)

            # Analyze override effectiveness
            emergency_scenarios = [r for r in override_test_results if r["emergency_active"]]
            normal_scenarios = [r for r in override_test_results if not r["emergency_active"]]

            override_priority_analysis = {
                "total_scenarios_tested": len(override_test_results),
                "emergency_scenarios": len(emergency_scenarios),
                "normal_scenarios": len(normal_scenarios),
                "emergency_override_effective": all(r["override_working"] for r in emergency_scenarios),
                "normal_operation_allowed": all(r["override_working"] for r in normal_scenarios),
                "override_priority_score": self._calculate_override_priority_score(override_test_results),
                "meets_override_requirement": all(r["override_working"] for r in override_test_results),
                "detailed_results": override_test_results
            }

            self.test_results["override_priority_verification"] = override_priority_analysis

            status = "✅ PASS" if override_priority_analysis["meets_override_requirement"] else "❌ FAIL"
            print(".1f")
            safety_system.cleanup()

        except Exception as e:
            print(f"⚠️ Override priority verification error: {e}")
            self.test_results["override_priority_verification"] = {"error": str(e)}

    def _test_recovery_procedure_validation(self):
        """Test recovery procedure validation (safe transition back to operational)."""
        print("🔄 Testing Recovery Procedure Validation...")

        try:
            safety_system = SafetySystem()
            safety_system.initialize()

            # Test emergency → recovery → normal sequence
            recovery_test_results = []
            test_iterations = 50  # Multiple recovery cycles

            for iteration in range(test_iterations):
                print(f"  Recovery cycle {iteration + 1}/{test_iterations}")

                # Phase 1: Normal operation
                safety_system.clear_emergency()
                normal_state_verified = not safety_system.is_emergency_active()

                # Phase 2: Emergency activation
                emergency_start = time.perf_counter()
                emergency_triggered = safety_system.emergency_stop(f"recovery_test_{iteration}")
                emergency_end = time.perf_counter()

                emergency_active = safety_system.is_emergency_active()
                emergency_activation_time = (emergency_end - emergency_start) * 1000

                # Phase 3: Recovery procedure
                recovery_start = time.perf_counter()
                recovery_success = safety_system.initiate_recovery()
                recovery_end = time.perf_counter()

                recovery_time = (recovery_end - recovery_start) * 1000
                system_recovered = not safety_system.is_emergency_active()

                # Phase 4: Verify normal operation restored
                post_recovery_commands_allowed = safety_system.allow_motion_command()

                recovery_cycle_result = {
                    "cycle": iteration + 1,
                    "normal_state_initial": normal_state_verified,
                    "emergency_triggered": emergency_triggered,
                    "emergency_active": emergency_active,
                    "emergency_activation_time_ms": emergency_activation_time,
                    "recovery_initiated": recovery_success,
                    "recovery_time_ms": recovery_time,
                    "system_recovered": system_recovered,
                    "post_recovery_commands_allowed": post_recovery_commands_allowed,
                    "complete_recovery_cycle": (normal_state_verified and emergency_active and
                                              system_recovered and post_recovery_commands_allowed)
                }

                recovery_test_results.append(recovery_cycle_result)

                # Brief delay between cycles
                time.sleep(0.01)

            # Analyze recovery effectiveness
            successful_cycles = sum(1 for r in recovery_test_results if r["complete_recovery_cycle"])
            total_cycles = len(recovery_test_results)

            recovery_activation_times = [r["emergency_activation_time_ms"] for r in recovery_test_results]
            recovery_times = [r["recovery_time_ms"] for r in recovery_test_results]

            recovery_procedure_analysis = {
                "total_recovery_cycles": total_cycles,
                "successful_cycles": successful_cycles,
                "success_rate_percent": (successful_cycles / total_cycles) * 100,
                "emergency_activation_avg_ms": statistics.mean(recovery_activation_times),
                "emergency_activation_p95_ms": statistics.quantiles(recovery_activation_times, n=20)[18],
                "recovery_time_avg_ms": statistics.mean(recovery_times),
                "recovery_time_p95_ms": statistics.quantiles(recovery_times, n=20)[18],
                "meets_recovery_requirement": (successful_cycles / total_cycles) > 0.95,  # >95% success rate
                "recovery_time_acceptable": statistics.quantiles(recovery_times, n=20)[18] < 5000.0,  # <5 seconds
                "emergency_response_fast": statistics.quantiles(recovery_activation_times, n=20)[18] < 50.0,  # <50ms
                "detailed_results": recovery_test_results[:10]  # First 10 cycles for detail
            }

            self.test_results["recovery_procedure_validation"] = recovery_procedure_analysis

            status = "✅ PASS" if (recovery_procedure_analysis["meets_recovery_requirement"] and
                                 recovery_procedure_analysis["recovery_time_acceptable"]) else "❌ FAIL"
            print(".1f")
            safety_system.cleanup()

        except Exception as e:
            print(f"⚠️ Recovery procedure validation error: {e}")
            self.test_results["recovery_procedure_validation"] = {"error": str(e)}

    def _test_false_trigger_rate_monitoring(self):
        """Test false trigger rate monitoring (<0.1% under normal operation)."""
        print("🎯 Testing False Trigger Rate Monitoring...")

        try:
            safety_system = SafetySystem()
            safety_system.initialize()

            # Clear any emergency state
            safety_system.clear_emergency()

            # Monitor for false triggers during normal operation
            monitoring_duration_seconds = 60  # 1 minute monitoring
            check_interval = 0.01  # Check every 10ms
            total_checks = int(monitoring_duration_seconds / check_interval)

            false_triggers = []
            monitoring_start = time.time()

            print(f"  Monitoring for false triggers over {monitoring_duration_seconds} seconds...")

            for check in range(total_checks):
                # Check if emergency is active (should be false)
                if safety_system.is_emergency_active():
                    trigger_time = time.time()
                    false_triggers.append({
                        "check_number": check,
                        "time_seconds": trigger_time - monitoring_start,
                        "emergency_reason": safety_system.get_emergency_reason()
                    })

                    # Clear false trigger
                    safety_system.clear_emergency()

                time.sleep(check_interval)

            monitoring_end = time.time()
            actual_duration = monitoring_end - monitoring_start

            # Analyze false trigger rate
            false_trigger_analysis = {
                "monitoring_duration_seconds": actual_duration,
                "total_checks": total_checks,
                "false_triggers_detected": len(false_triggers),
                "false_trigger_rate_percent": (len(false_triggers) / total_checks) * 100,
                "meets_requirement": (len(false_triggers) / total_checks) * 100 < 0.1,  # <0.1% requirement
                "triggers_per_hour": (len(false_triggers) / (actual_duration / 3600)) if actual_duration > 0 else 0,
                "mean_time_between_false_triggers": actual_duration / len(false_triggers) if false_triggers else float('inf'),
                "false_trigger_details": false_triggers[:5] if false_triggers else []  # First 5 for analysis
            }

            self.test_results["false_trigger_rate_monitoring"] = false_trigger_analysis

            status = "✅ PASS" if false_trigger_analysis["meets_requirement"] else "❌ FAIL"
            print(".4f")
            if false_triggers:
                print(f"  ⚠️ {len(false_triggers)} false triggers detected")

            safety_system.cleanup()

        except Exception as e:
            print(f"⚠️ False trigger rate monitoring error: {e}")
            self.test_results["false_trigger_rate_monitoring"] = {"error": str(e)}

    def _test_hardware_interlock_verification(self):
        """Test hardware interlock verification (mechanical brake engagement)."""
        print("🔧 Testing Hardware Interlock Verification...")

        try:
            safety_system = SafetySystem()
            safety_system.initialize()

            # Test hardware interlock scenarios
            interlock_test_results = []
            test_scenarios = [
                {"name": "emergency_stop_brake_engagement", "emergency": True, "brake_expected": True},
                {"name": "normal_operation_brake_release", "emergency": False, "brake_expected": False},
                {"name": "recovery_brake_release", "emergency": True, "recovery": True, "brake_expected": False},
            ]

            for scenario in test_scenarios:
                print(f"  Testing scenario: {scenario['name']}")

                # Set initial state
                if scenario.get("emergency", False):
                    safety_system.emergency_stop("interlock_test")
                else:
                    safety_system.clear_emergency()

                # Allow state to settle
                time.sleep(0.01)

                # Check brake status (simulated - would check actual hardware)
                brake_engaged = safety_system.is_brake_engaged()
                expected_brake = scenario["brake_expected"]

                # If recovery scenario, initiate recovery
                if scenario.get("recovery", False):
                    safety_system.initiate_recovery()
                    time.sleep(0.01)  # Allow recovery
                    brake_engaged = safety_system.is_brake_engaged()
                    expected_brake = False

                interlock_result = {
                    "scenario": scenario["name"],
                    "emergency_active": safety_system.is_emergency_active(),
                    "brake_engaged": brake_engaged,
                    "brake_expected": expected_brake,
                    "interlock_correct": brake_engaged == expected_brake,
                    "hardware_consistent": True  # Would verify against actual hardware sensors
                }

                interlock_test_results.append(interlock_result)

            # Analyze hardware interlock effectiveness
            correct_interlocks = sum(1 for r in interlock_test_results if r["interlock_correct"])
            total_tests = len(interlock_test_results)

            hardware_interlock_analysis = {
                "total_interlock_tests": total_tests,
                "correct_interlocks": correct_interlocks,
                "interlock_accuracy_percent": (correct_interlocks / total_tests) * 100,
                "meets_interlock_requirement": correct_interlocks == total_tests,
                "emergency_brake_engagement": all(r["brake_engaged"] for r in interlock_test_results
                                                if r["scenario"].startswith("emergency")),
                "recovery_brake_release": all(not r["brake_engaged"] for r in interlock_test_results
                                            if "recovery" in r["scenario"]),
                "normal_operation_brake_release": all(not r["brake_engaged"] for r in interlock_test_results
                                                    if "normal" in r["scenario"]),
                "detailed_results": interlock_test_results
            }

            self.test_results["hardware_interlock_verification"] = hardware_interlock_analysis

            status = "✅ PASS" if hardware_interlock_analysis["meets_interlock_requirement"] else "❌ FAIL"
                print(".1f")
            safety_system.cleanup()

        except Exception as e:
            print(f"⚠️ Hardware interlock verification error: {e}")
            self.test_results["hardware_interlock_verification"] = {"error": str(e)}

    def _calculate_override_priority_score(self, override_results: List[Dict]) -> float:
        """Calculate override priority effectiveness score (0-100)."""
        if not override_results:
            return 0.0

        total_scenarios = len(override_results)
        working_scenarios = sum(1 for r in override_results if r["override_working"])

        # Base score from working scenarios
        base_score = (working_scenarios / total_scenarios) * 100

        # Bonus for emergency scenarios working perfectly
        emergency_scenarios = [r for r in override_results if r["emergency_active"]]
        if emergency_scenarios:
            emergency_working = sum(1 for r in emergency_scenarios if r["override_working"])
            emergency_bonus = 10.0 if emergency_working == len(emergency_scenarios) else 0.0
        else:
            emergency_bonus = 0.0

        # Penalty for normal operation not working
        normal_scenarios = [r for r in override_results if not r["emergency_active"]]
        if normal_scenarios:
            normal_working = sum(1 for r in normal_scenarios if r["override_working"])
            normal_penalty = 20.0 if normal_working < len(normal_scenarios) else 0.0
        else:
            normal_penalty = 0.0

        final_score = max(0.0, min(100.0, base_score + emergency_bonus - normal_penalty))
        return final_score

    def _generate_emergency_test_report(self) -> Dict[str, Any]:
        """Generate comprehensive emergency systems test report."""
        print("📋 Generating Emergency Systems Optimization Report...")

        report = {
            "test_metadata": {
                "test_type": "emergency_systems_optimization",
                "timestamp": time.time(),
                "optimization_requirements": [
                    "Response time <10ms from command to motor stop",
                    "Emergency overrides all other commands",
                    "Safe transition back to operational state",
                    "False trigger rate <0.1% under normal operation",
                    "Hardware interlock verification (mechanical brake engagement)"
                ]
            },
            "optimization_results": self.test_results,
            "performance_analysis": self._analyze_emergency_performance(),
            "recommendations": self._generate_emergency_recommendations()
        }

        return report

    def _analyze_emergency_performance(self) -> Dict[str, Any]:
        """Analyze emergency system performance improvements."""
        analysis = {}

        # Response time analysis
        if "response_time_validation" in self.test_results:
            timing = self.test_results["response_time_validation"]
            if isinstance(timing, dict):
                p95_time = timing.get("p95_response_time_ms", 999)
                analysis["response_time_performance"] = {
                    "p95_time_ms": p95_time,
                    "meets_requirement": timing.get("meets_10ms_requirement", False),
                    "violation_rate": timing.get("violation_rate_percent", 100),
                    "stability": timing.get("response_time_stability", 1.0),
                    "optimization_needed": p95_time > 8.0
                }

        # Override priority analysis
        if "override_priority_verification" in self.test_results:
            override = self.test_results["override_priority_verification"]
            if isinstance(override, dict):
                priority_score = override.get("override_priority_score", 0)
                analysis["override_priority"] = {
                    "priority_score": priority_score,
                    "emergency_override_effective": override.get("emergency_override_effective", False),
                    "normal_operation_allowed": override.get("normal_operation_allowed", False),
                    "optimization_effective": priority_score > 90.0
                }

        # Recovery analysis
        if "recovery_procedure_validation" in self.test_results:
            recovery = self.test_results["recovery_procedure_validation"]
            if isinstance(recovery, dict):
                success_rate = recovery.get("success_rate_percent", 0)
                recovery_time = recovery.get("recovery_time_p95_ms", 99999)
                analysis["recovery_performance"] = {
                    "success_rate_percent": success_rate,
                    "p95_recovery_time_ms": recovery_time,
                    "meets_success_requirement": recovery.get("meets_recovery_requirement", False),
                    "recovery_time_acceptable": recovery.get("recovery_time_acceptable", False),
                    "optimization_needed": success_rate < 98.0 or recovery_time > 3000.0
                }

        # False trigger analysis
        if "false_trigger_rate_monitoring" in self.test_results:
            false_triggers = self.test_results["false_trigger_rate_monitoring"]
            if isinstance(false_triggers, dict):
                trigger_rate = false_triggers.get("false_trigger_rate_percent", 100)
                analysis["false_trigger_prevention"] = {
                    "false_trigger_rate_percent": trigger_rate,
                    "meets_requirement": false_triggers.get("meets_requirement", False),
                    "triggers_per_hour": false_triggers.get("triggers_per_hour", 999),
                    "optimization_needed": trigger_rate > 0.05
                }

        return analysis

    def _generate_emergency_recommendations(self) -> List[str]:
        """Generate emergency system optimization recommendations."""
        recommendations = []

        # Analyze each test result
        if "response_time_validation" in self.test_results:
            timing = self.test_results["response_time_validation"]
            if isinstance(timing, dict):
                if not timing.get("meets_10ms_requirement", False):
                    recommendations.append("Optimize emergency response time to meet <10ms requirement")
                if timing.get("violation_rate_percent", 100) > 1:
                    recommendations.append("Reduce response time violations through hardware/software optimization")

        if "override_priority_verification" in self.test_results:
            override = self.test_results["override_priority_verification"]
            if isinstance(override, dict):
                if not override.get("emergency_override_effective", False):
                    recommendations.append("Fix emergency override to block all commands during emergency")
                if not override.get("normal_operation_allowed", False):
                    recommendations.append("Ensure normal commands work when emergency is cleared")

        if "recovery_procedure_validation" in self.test_results:
            recovery = self.test_results["recovery_procedure_validation"]
            if isinstance(recovery, dict):
                if not recovery.get("meets_recovery_requirement", False):
                    recommendations.append("Improve recovery success rate to >95%")
                if not recovery.get("recovery_time_acceptable", False):
                    recommendations.append("Optimize recovery time to <5 seconds")

        if "false_trigger_rate_monitoring" in self.test_results:
            false_triggers = self.test_results["false_trigger_rate_monitoring"]
            if isinstance(false_triggers, dict):
                if not false_triggers.get("meets_requirement", False):
                    recommendations.append("Reduce false trigger rate to <0.1% through better filtering")
                if false_triggers.get("triggers_per_hour", 999) > 1:
                    recommendations.append("Implement additional trigger validation logic")

        if "hardware_interlock_verification" in self.test_results:
            interlock = self.test_results["hardware_interlock_verification"]
            if isinstance(interlock, dict):
                if not interlock.get("meets_interlock_requirement", False):
                    recommendations.append("Fix hardware interlock verification for brake engagement")

        if not recommendations:
            recommendations.append("✅ Emergency systems optimizations are performing well")

        return recommendations


def run_emergency_systems_optimization_tests():
    """Run comprehensive emergency systems optimization tests."""
    tester = EmergencySystemsOptimizationTester()
    report = tester.run_comprehensive_emergency_tests()

    # Print summary
    print("\n" + "="*60)
    print("🚨 EMERGENCY SYSTEMS OPTIMIZATION TEST SUMMARY")
    print("="*60)

    analysis = report.get("performance_analysis", {})

    if analysis:
        for test_name, results in analysis.items():
            print(f"\n{test_name.replace('_', ' ').title()}:")
            for key, value in results.items():
                if isinstance(value, bool):
                    status = "✅" if value else "❌"
                    print(f"  {key}: {status}")
                elif isinstance(value, (int, float)):
                    if "percent" in key or "rate" in key:
                        print(f"  {key}: {value:.3f}")
                    elif "time" in key:
                        print(f"  {key}: {value:.2f}ms")
                    elif "score" in key:
                        print(f"  {key}: {value:.1f}")
                    else:
                        print(f"  {key}: {value}")

    recommendations = report.get("recommendations", [])
    if recommendations:
        print("\n💡 RECOMMENDATIONS:")
        for rec in recommendations:
            print(f"  • {rec}")

    return report


if __name__ == "__main__":
    report = run_emergency_systems_optimization_tests()

    # Save detailed report
    import json
    report_file = f"emergency_systems_optimization_test_report_{int(time.time())}.json"
    with open(report_file, 'w') as f:
        json.dump(report, f, indent=2, default=str)

    print(f"\n📁 Detailed report saved to: {report_file}")
