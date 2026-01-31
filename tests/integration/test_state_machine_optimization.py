#!/usr/bin/env python3
"""
State Machine Optimization Tests - URC 2026

Tests state machine performance optimizations including:
- Transition time validation (<50ms between states)
- State consistency checking (no invalid combinations)
- Concurrent access thread safety
- Persistence and recovery capabilities
- Transition success rate monitoring

Author: URC 2026 State Machine Optimization Team
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

import pytest
try:
    from src.core.simplified_state_manager import UnifiedStateManager, get_state_manager, SystemState
    _STATE_MANAGER_AVAILABLE = True
except ImportError:
    _STATE_MANAGER_AVAILABLE = False
    UnifiedStateManager = None
    get_state_manager = None
    SystemState = None

# Backward compatibility: tests expected AdaptiveStateMachine
AdaptiveStateMachine = UnifiedStateManager
if not _STATE_MANAGER_AVAILABLE:
    pytest.skip("Simplified state manager not available", allow_module_level=True)


class StateMachineOptimizationTester:
    """Comprehensive state machine optimization testing."""

    def __init__(self):
        self.test_results = {}
        self.baseline_measurements = {}
        self.transition_times = []
        self.validity_checks = []
        self.thread_safety_results = []

    def run_comprehensive_sm_tests(self) -> Dict[str, Any]:
        """Run comprehensive state machine optimization tests."""

        print("🔄 Starting State Machine Optimization Tests")
        print("=" * 60)

        # Test transition time validation
        self._test_transition_time_validation()

        # Test state consistency checking
        self._test_state_consistency_checking()

        # Test concurrent access thread safety
        self._test_concurrent_access_thread_safety()

        # Test persistence and recovery
        self._test_persistence_and_recovery()

        # Test transition success rate monitoring
        self._test_transition_success_rate_monitoring()

        # Generate comprehensive report
        return self._generate_sm_test_report()

    def _test_transition_time_validation(self):
        """Test transition time validation (<50ms between states)."""
        print("⏱️ Testing Transition Time Validation...")

        try:
            # Create state machine
            sm = AdaptiveStateMachine()
            sm.initialize()

            # Define state transition sequences
            transition_sequences = [
                ["IDLE", "INITIALIZING", "NAVIGATING", "SAMPLING", "RETURNING", "IDLE"],
                ["IDLE", "EMERGENCY", "RECOVERY", "IDLE"],
                ["NAVIGATING", "OBSTACLE_DETECTED", "AVOIDING", "NAVIGATING"],
            ]

            all_transition_times = []

            for seq_num, sequence in enumerate(transition_sequences):
                print(f"  Testing sequence {seq_num + 1}: {' → '.join(sequence)}")

                seq_times = []

                for i in range(len(sequence) - 1):
                    from_state = sequence[i]
                    to_state = sequence[i + 1]

                    # Set current state
                    sm.current_state = from_state

                    # Measure transition time
                    transition_start = time.perf_counter()

                    # Execute transition
                    success = sm.transition_to(to_state)

                    transition_end = time.perf_counter()
                    transition_time_ms = (transition_end - transition_start) * 1000

                    seq_times.append(transition_time_ms)
                    all_transition_times.append(transition_time_ms)

                    if transition_time_ms > 50.0:
                        print(".2f")
                transition_sequences[f"sequence_{seq_num + 1}"] = {
                    "states": sequence,
                    "transition_times_ms": seq_times,
                    "average_time_ms": statistics.mean(seq_times),
                    "max_time_ms": max(seq_times),
                    "violations": len([t for t in seq_times if t > 50.0])
                }

            # Overall statistics
            transition_validation = {
                "total_transitions": len(all_transition_times),
                "average_transition_time_ms": statistics.mean(all_transition_times),
                "p95_transition_time_ms": statistics.quantiles(all_transition_times, n=20)[18],
                "max_transition_time_ms": max(all_transition_times),
                "violations_count": len([t for t in all_transition_times if t > 50.0]),
                "violation_rate_percent": (len([t for t in all_transition_times if t > 50.0]) / len(all_transition_times)) * 100,
                "meets_50ms_requirement": statistics.quantiles(all_transition_times, n=20)[18] < 50.0,
                "sequences": transition_sequences
            }

            self.test_results["transition_time_validation"] = transition_validation

            status = "✅ PASS" if transition_validation["meets_50ms_requirement"] else "❌ FAIL"
            print(".2f")
            sm.cleanup()

        except Exception as e:
            print(f"⚠️ Transition time validation error: {e}")
            self.test_results["transition_time_validation"] = {"error": str(e)}

    def _test_state_consistency_checking(self):
        """Test state consistency checking (no invalid combinations)."""
        print("🔍 Testing State Consistency Checking...")

        try:
            sm = AdaptiveStateMachine()
            sm.initialize()

            # Define valid and invalid state combinations
            test_cases = [
                # Valid transitions
                {"from": "IDLE", "to": "INITIALIZING", "expected_valid": True},
                {"from": "INITIALIZING", "to": "NAVIGATING", "expected_valid": True},
                {"from": "NAVIGATING", "to": "SAMPLING", "expected_valid": True},
                {"from": "SAMPLING", "to": "RETURNING", "expected_valid": True},
                {"from": "RETURNING", "to": "IDLE", "expected_valid": True},

                # Emergency transitions (should be valid)
                {"from": "NAVIGATING", "to": "EMERGENCY", "expected_valid": True},
                {"from": "EMERGENCY", "to": "RECOVERY", "expected_valid": True},
                {"from": "RECOVERY", "to": "IDLE", "expected_valid": True},

                # Invalid transitions (should be rejected)
                {"from": "IDLE", "to": "SAMPLING", "expected_valid": False},  # Skip initialization
                {"from": "NAVIGATING", "to": "INITIALIZING", "expected_valid": False},  # Backward transition
                {"from": "EMERGENCY", "to": "SAMPLING", "expected_valid": False},  # Invalid from emergency
            ]

            consistency_results = []

            for test_case in test_cases:
                sm.current_state = test_case["from"]

                # Test transition validity
                transition_start = time.perf_counter()
                actual_valid = sm.transition_to(test_case["to"])
                transition_end = time.perf_counter()

                expected_valid = test_case["expected_valid"]

                result = {
                    "transition": f"{test_case['from']} → {test_case['to']}",
                    "expected_valid": expected_valid,
                    "actual_valid": actual_valid,
                    "correct": actual_valid == expected_valid,
                    "transition_time_ms": (transition_end - transition_start) * 1000
                }

                consistency_results.append(result)

            # Analyze results
            correct_decisions = sum(1 for r in consistency_results if r["correct"])
            total_decisions = len(consistency_results)

            consistency_analysis = {
                "total_tests": total_decisions,
                "correct_decisions": correct_decisions,
                "accuracy_percent": (correct_decisions / total_decisions) * 100,
                "invalid_transitions_prevented": len([r for r in consistency_results if not r["expected_valid"] and not r["actual_valid"]]),
                "valid_transitions_allowed": len([r for r in consistency_results if r["expected_valid"] and r["actual_valid"]]),
                "false_positives": len([r for r in consistency_results if not r["expected_valid"] and r["actual_valid"]]),
                "false_negatives": len([r for r in consistency_results if r["expected_valid"] and not r["actual_valid"]]),
                "average_validation_time_ms": statistics.mean([r["transition_time_ms"] for r in consistency_results]),
                "meets_consistency_requirement": correct_decisions == total_decisions,
                "detailed_results": consistency_results
            }

            self.test_results["state_consistency_checking"] = consistency_analysis

            status = "✅ PASS" if consistency_analysis["meets_consistency_requirement"] else "❌ FAIL"
            print(".1f")
            sm.cleanup()

        except Exception as e:
            print(f"⚠️ State consistency checking error: {e}")
            self.test_results["state_consistency_checking"] = {"error": str(e)}

    def _test_concurrent_access_thread_safety(self):
        """Test concurrent access thread safety."""
        print("🔒 Testing Concurrent Access Thread Safety...")

        try:
            sm = AdaptiveStateMachine()
            sm.initialize()

            # Test concurrent state transitions
            concurrent_results = []
            error_count = 0
            successful_transitions = 0

            def worker_thread(thread_id: int, results: List[Dict]):
                """Worker thread for concurrent testing."""
                local_results = []

                for i in range(50):  # 50 transitions per thread
                    try:
                        # Random state transitions
                        states = ["IDLE", "INITIALIZING", "NAVIGATING", "SAMPLING", "RETURNING"]
                        from_state = states[i % len(states)]
                        to_state = states[(i + 1) % len(states)]

                        transition_start = time.perf_counter()
                        success = sm.transition_to(to_state)
                        transition_end = time.perf_counter()

                        local_results.append({
                            "thread_id": thread_id,
                            "transition": f"{from_state} → {to_state}",
                            "success": success,
                            "time_ms": (transition_end - transition_start) * 1000,
                            "error": None
                        })

                    except Exception as e:
                        local_results.append({
                            "thread_id": thread_id,
                            "transition": f"error_{i}",
                            "success": False,
                            "time_ms": 0,
                            "error": str(e)
                        })

                results.extend(local_results)

            # Run concurrent threads
            thread_results = []
            threads = []

            for thread_id in range(5):  # 5 concurrent threads
                thread = threading.Thread(target=worker_thread, args=(thread_id, thread_results))
                threads.append(thread)
                thread.start()

            # Wait for all threads
            for thread in threads:
                thread.join(timeout=10.0)

            # Analyze results
            total_transitions = len(thread_results)
            successful_transitions = len([r for r in thread_results if r["success"]])
            errors = [r for r in thread_results if r.get("error")]

            thread_safety_analysis = {
                "total_threads": 5,
                "transitions_per_thread": 50,
                "total_transitions_attempted": total_transitions,
                "successful_transitions": successful_transitions,
                "success_rate_percent": (successful_transitions / total_transitions) * 100 if total_transitions > 0 else 0,
                "errors_count": len(errors),
                "error_rate_percent": (len(errors) / total_transitions) * 100 if total_transitions > 0 else 0,
                "average_transition_time_ms": statistics.mean([r["time_ms"] for r in thread_results if r["success"]]),
                "max_transition_time_ms": max([r["time_ms"] for r in thread_results if r["success"]] or [0]),
                "thread_safety_score": self._calculate_thread_safety_score(thread_results),
                "meets_thread_safety_requirement": len(errors) == 0 and successful_transitions > total_transitions * 0.95,
                "error_details": errors[:10] if errors else []  # First 10 errors
            }

            self.test_results["concurrent_access_thread_safety"] = thread_safety_analysis

            status = "✅ PASS" if thread_safety_analysis["meets_thread_safety_requirement"] else "❌ FAIL"
            print(".1f")
            sm.cleanup()

        except Exception as e:
            print(f"⚠️ Concurrent access thread safety error: {e}")
            self.test_results["concurrent_access_thread_safety"] = {"error": str(e)}

    def _test_persistence_and_recovery(self):
        """Test persistence and recovery capabilities."""
        print("💾 Testing Persistence and Recovery...")

        try:
            import tempfile
            import os

            # Create temporary persistence file
            with tempfile.NamedTemporaryFile(mode='w+', suffix='.json', delete=False) as temp_file:
                persistence_file = temp_file.name

            try:
                # Test persistence
                sm1 = AdaptiveStateMachine(persistence_file=persistence_file)
                sm1.initialize()

                # Set specific state and data
                test_state = "NAVIGATING"
                test_data = {"waypoint": 5, "battery": 85.2, "timestamp": time.time()}

                sm1.current_state = test_state
                sm1.state_data.update(test_data)

                # Save state
                persistence_start = time.perf_counter()
                sm1.save_state()
                persistence_end = time.perf_counter()

                # Create new instance and load state
                sm2 = AdaptiveStateMachine(persistence_file=persistence_file)
                recovery_start = time.perf_counter()
                sm2.load_state()
                recovery_end = time.perf_counter()

                # Verify state restoration
                state_restored = sm2.current_state == test_state
                data_restored = all(sm2.state_data.get(k) == v for k, v in test_data.items())

                persistence_analysis = {
                    "persistence_time_ms": (persistence_end - persistence_start) * 1000,
                    "recovery_time_ms": (recovery_end - recovery_start) * 1000,
                    "state_preservation": state_restored,
                    "data_preservation": data_restored,
                    "complete_persistence": state_restored and data_restored,
                    "meets_persistence_requirement": state_restored and data_restored,
                    "performance_acceptable": ((persistence_end - persistence_start) + (recovery_end - recovery_start)) * 1000 < 100.0
                }

                self.test_results["persistence_and_recovery"] = persistence_analysis

                status = "✅ PASS" if persistence_analysis["meets_persistence_requirement"] else "❌ FAIL"
                print(".1f")
                sm1.cleanup()
                sm2.cleanup()

            finally:
                # Clean up temp file
                if os.path.exists(persistence_file):
                    os.unlink(persistence_file)

        except Exception as e:
            print(f"⚠️ Persistence and recovery error: {e}")
            self.test_results["persistence_and_recovery"] = {"error": str(e)}

    def _test_transition_success_rate_monitoring(self):
        """Test transition success rate monitoring."""
        print("📊 Testing Transition Success Rate Monitoring...")

        try:
            sm = AdaptiveStateMachine()
            sm.initialize()

            # Simulate various transition scenarios
            transition_scenarios = []

            # Normal operation transitions
            normal_transitions = [
                ("IDLE", "INITIALIZING"),
                ("INITIALIZING", "NAVIGATING"),
                ("NAVIGATING", "SAMPLING"),
                ("SAMPLING", "RETURNING"),
                ("RETURNING", "IDLE"),
            ]

            # Add some failure scenarios
            failure_transitions = [
                ("IDLE", "SAMPLING"),  # Invalid transition
                ("NAVIGATING", "INITIALIZING"),  # Backward transition
                ("EMERGENCY", "SAMPLING"),  # Invalid from emergency
            ]

            all_transitions = []
            success_count = 0
            total_count = 0

            # Test normal transitions (should succeed)
            for from_state, to_state in normal_transitions:
                sm.current_state = from_state
                success = sm.transition_to(to_state)
                all_transitions.append({
                    "transition": f"{from_state} → {to_state}",
                    "expected_success": True,
                    "actual_success": success,
                    "correct": success == True
                })
                success_count += 1 if success else 0
                total_count += 1

            # Test failure transitions (should fail)
            for from_state, to_state in failure_transitions:
                sm.current_state = from_state
                success = sm.transition_to(to_state)
                all_transitions.append({
                    "transition": f"{from_state} → {to_state}",
                    "expected_success": False,
                    "actual_success": success,
                    "correct": success == False
                })
                success_count += 1 if success == False else 0  # Correct failure
                total_count += 1

            # Add some error conditions
            for i in range(10):  # Simulate 10% error rate
                try:
                    sm.current_state = "INVALID_STATE"
                    success = sm.transition_to("NAVIGATING")
                    all_transitions.append({
                        "transition": f"INVALID_STATE → NAVIGATING",
                        "expected_success": False,
                        "actual_success": success,
                        "correct": success == False
                    })
                    success_count += 1 if success == False else 0
                    total_count += 1
                except:
                    all_transitions.append({
                        "transition": f"INVALID_STATE → NAVIGATING",
                        "expected_success": False,
                        "actual_success": False,
                        "correct": True
                    })
                    success_count += 1
                    total_count += 1

            # Calculate success rates
            success_rate_monitoring = {
                "total_transitions_tested": total_count,
                "successful_transitions": success_count,
                "success_rate_percent": (success_count / total_count) * 100,
                "meets_success_rate_requirement": (success_count / total_count) > 0.95,  # >95% success rate
                "transition_categories": {
                    "normal_transitions": len(normal_transitions),
                    "failure_transitions": len(failure_transitions),
                    "error_conditions": 10
                },
                "detailed_results": all_transitions
            }

            self.test_results["transition_success_rate_monitoring"] = success_rate_monitoring

            status = "✅ PASS" if success_rate_monitoring["meets_success_rate_requirement"] else "❌ FAIL"
            print(".1f")
            sm.cleanup()

        except Exception as e:
            print(f"⚠️ Transition success rate monitoring error: {e}")
            self.test_results["transition_success_rate_monitoring"] = {"error": str(e)}

    def _calculate_thread_safety_score(self, thread_results: List[Dict]) -> float:
        """Calculate thread safety score (0-100)."""
        if not thread_results:
            return 0.0

        successful_transitions = len([r for r in thread_results if r["success"]])
        total_transitions = len(thread_results)

        if total_transitions == 0:
            return 0.0

        success_rate = successful_transitions / total_transitions

        # Penalize errors heavily
        errors = len([r for r in thread_results if r.get("error")])
        error_penalty = min(50.0, errors * 5.0)  # Max 50 point penalty

        # Base score from success rate
        base_score = success_rate * 100

        # Final score with error penalty
        final_score = max(0.0, base_score - error_penalty)

        return final_score

    def _generate_sm_test_report(self) -> Dict[str, Any]:
        """Generate comprehensive SM test report."""
        print("📋 Generating State Machine Optimization Report...")

        report = {
            "test_metadata": {
                "test_type": "state_machine_optimization",
                "timestamp": time.time(),
                "optimization_requirements": [
                    "Transition time <50ms between states",
                    "State consistency (no invalid combinations)",
                    "Thread-safe concurrent access",
                    "State persistence and recovery",
                    "Transition success rate >95%"
                ]
            },
            "optimization_results": self.test_results,
            "performance_analysis": self._analyze_sm_performance(),
            "recommendations": self._generate_sm_recommendations()
        }

        return report

    def _analyze_sm_performance(self) -> Dict[str, Any]:
        """Analyze state machine performance improvements."""
        analysis = {}

        # Transition time analysis
        if "transition_time_validation" in self.test_results:
            timing = self.test_results["transition_time_validation"]
            if isinstance(timing, dict):
                p95_time = timing.get("p95_transition_time_ms", 999)
                analysis["transition_performance"] = {
                    "p95_time_ms": p95_time,
                    "meets_requirement": timing.get("meets_50ms_requirement", False),
                    "violation_rate": timing.get("violation_rate_percent", 100),
                    "optimization_needed": p95_time > 40.0
                }

        # Consistency analysis
        if "state_consistency_checking" in self.test_results:
            consistency = self.test_results["state_consistency_checking"]
            if isinstance(consistency, dict):
                accuracy = consistency.get("accuracy_percent", 0)
                analysis["consistency_validation"] = {
                    "accuracy_percent": accuracy,
                    "invalid_transitions_prevented": consistency.get("invalid_transitions_prevented", 0),
                    "false_positives": consistency.get("false_positives", 0),
                    "optimization_effective": accuracy > 95.0
                }

        # Thread safety analysis
        if "concurrent_access_thread_safety" in self.test_results:
            thread_safety = self.test_results["concurrent_access_thread_safety"]
            if isinstance(thread_safety, dict):
                safety_score = thread_safety.get("thread_safety_score", 0)
                analysis["thread_safety"] = {
                    "safety_score": safety_score,
                    "success_rate": thread_safety.get("success_rate_percent", 0),
                    "error_rate": thread_safety.get("error_rate_percent", 0),
                    "optimization_needed": safety_score < 90.0
                }

        return analysis

    def _generate_sm_recommendations(self) -> List[str]:
        """Generate state machine optimization recommendations."""
        recommendations = []

        # Analyze each test result
        if "transition_time_validation" in self.test_results:
            timing = self.test_results["transition_time_validation"]
            if isinstance(timing, dict):
                if not timing.get("meets_50ms_requirement", False):
                    recommendations.append("Optimize transition times to meet <50ms requirement")
                if timing.get("violation_rate_percent", 100) > 5:
                    recommendations.append("Reduce transition time violations through optimization")

        if "state_consistency_checking" in self.test_results:
            consistency = self.test_results["state_consistency_checking"]
            if isinstance(consistency, dict):
                if consistency.get("accuracy_percent", 0) < 95:
                    recommendations.append("Improve state consistency validation accuracy")
                if consistency.get("false_positives", 0) > 0:
                    recommendations.append("Fix false positive state transition validations")

        if "concurrent_access_thread_safety" in self.test_results:
            thread_safety = self.test_results["concurrent_access_thread_safety"]
            if isinstance(thread_safety, dict):
                if not thread_safety.get("meets_thread_safety_requirement", False):
                    recommendations.append("Implement proper thread synchronization for state transitions")
                if thread_safety.get("error_rate_percent", 100) > 1:
                    recommendations.append("Reduce concurrent access errors through better locking")

        if "persistence_and_recovery" in self.test_results:
            persistence = self.test_results["persistence_and_recovery"]
            if isinstance(persistence, dict):
                if not persistence.get("meets_persistence_requirement", False):
                    recommendations.append("Fix state persistence and recovery mechanisms")
                if not persistence.get("performance_acceptable", False):
                    recommendations.append("Optimize persistence/recovery performance")

        if "transition_success_rate_monitoring" in self.test_results:
            success_rate = self.test_results["transition_success_rate_monitoring"]
            if isinstance(success_rate, dict):
                if not success_rate.get("meets_success_rate_requirement", False):
                    recommendations.append("Improve transition success rate to >95%")

        if not recommendations:
            recommendations.append("✅ State machine optimizations are performing well")

        return recommendations


def run_state_machine_optimization_tests():
    """Run comprehensive state machine optimization tests."""
    tester = StateMachineOptimizationTester()
    report = tester.run_comprehensive_sm_tests()

    # Print summary
    print("\n" + "="*60)
    print("🔄 STATE MACHINE OPTIMIZATION TEST SUMMARY")
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
                    if "percent" in key:
                        print(f"  {key}: {value:.1f}%")
                    elif "time" in key or "score" in key:
                        print(f"  {key}: {value:.2f}")
                    else:
                        print(f"  {key}: {value}")

    recommendations = report.get("recommendations", [])
    if recommendations:
        print("\n💡 RECOMMENDATIONS:")
        for rec in recommendations:
            print(f"  • {rec}")

    return report


if __name__ == "__main__":
    report = run_state_machine_optimization_tests()

    # Save detailed report
    import json
    report_file = f"sm_optimization_test_report_{int(time.time())}.json"
    with open(report_file, 'w') as f:
        json.dump(report, f, indent=2, default=str)

    print(f"\n📁 Detailed report saved to: {report_file}")
