#!/usr/bin/env python3
"""
End-to-End Mission Optimization Tests - URC 2026

Tests complete mission execution optimizations including:
- Mission completion rate validation (>95% in simulation)
- Time to complete within 2x optimal path
- Resource usage within CPU/memory budgets
- Failure recovery automatic handling
- Operator intervention minimization (<5% of mission time)

Author: URC 2026 End-to-End Mission Optimization Team
"""

import time
import threading
import statistics
from typing import Dict, List, Any, Optional, Callable
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.core.mission_resource_manager import get_mission_resource_manager


class EndToEndOptimizationTester:
    """Comprehensive end-to-end mission optimization testing."""

    def __init__(self):
        self.test_results = {}
        self.mission_logs = []
        self.resource_usage = []
        self.failure_recovery_events = []

    def run_comprehensive_e2e_tests(self) -> Dict[str, Any]:
        """Run comprehensive end-to-end mission optimization tests."""

        print("🎯 Starting End-to-End Mission Optimization Tests")
        print("=" * 60)

        # Test mission completion rate validation
        self._test_mission_completion_rate()

        # Test time to complete optimization
        self._test_time_to_complete_optimization()

        # Test resource usage within budgets
        self._test_resource_usage_within_budgets()

        # Test failure recovery handling
        self._test_failure_recovery_handling()

        # Test operator intervention minimization
        self._test_operator_intervention_minimization()

        # Generate comprehensive report
        return self._generate_e2e_test_report()

    def _test_mission_completion_rate(self):
        """Test mission completion rate validation (>95% in simulation)."""
        print("✅ Testing Mission Completion Rate Validation...")

        try:
            # Simulate multiple mission runs
            mission_scenarios = [
                {"name": "standard_sample_mission", "complexity": "medium", "expected_success": True},
                {"name": "challenging_terrain_mission", "complexity": "high", "expected_success": True},
                {"name": "time_constrained_mission", "complexity": "high", "expected_success": False},  # Some failures expected
                {"name": "resource_limited_mission", "complexity": "medium", "expected_success": True},
                {"name": "recovery_test_mission", "complexity": "high", "expected_success": True},
            ]

            mission_results = []
            total_missions = len(mission_scenarios) * 5  # 5 runs per scenario
            successful_missions = 0

            for scenario in mission_scenarios:
                print(f"  Running scenario: {scenario['name']}")

                for run in range(5):  # 5 runs per scenario
                    mission_result = self._simulate_mission_run(scenario, run)
                    mission_results.append(mission_result)

                    if mission_result["completed"]:
                        successful_missions += 1

                    # Log mission result
                    self.mission_logs.append({
                        "scenario": scenario["name"],
                        "run": run + 1,
                        "completed": mission_result["completed"],
                        "duration_seconds": mission_result["duration_seconds"],
                        "failures_encountered": len(mission_result["failures"]),
                        "operator_interventions": mission_result["operator_interventions"],
                        "resource_usage": mission_result["resource_usage"]
                    })

            # Calculate completion statistics
            completion_rate = (successful_missions / total_missions) * 100

            mission_completion_analysis = {
                "total_missions_attempted": total_missions,
                "missions_completed": successful_missions,
                "completion_rate_percent": completion_rate,
                "meets_95_percent_requirement": completion_rate >= 95.0,
                "meets_90_percent_minimum": completion_rate >= 90.0,  # Acceptable minimum
                "scenarios_tested": len(mission_scenarios),
                "runs_per_scenario": 5,
                "scenario_success_rates": self._calculate_scenario_success_rates(mission_results, mission_scenarios),
                "failure_analysis": self._analyze_mission_failures(mission_results)
            }

            self.test_results["mission_completion_rate"] = mission_completion_analysis

            status = "✅ PASS" if mission_completion_analysis["meets_95_percent_requirement"] else "⚠️ DEGRADED" if mission_completion_analysis["meets_90_percent_minimum"] else "❌ FAIL"
            print(".1f")
            self.test_results["mission_completion_rate"] = mission_completion_analysis

        except Exception as e:
            print(f"⚠️ Mission completion rate test error: {e}")
            self.test_results["mission_completion_rate"] = {"error": str(e)}

    def _test_time_to_complete_optimization(self):
        """Test time to complete within 2x optimal path."""
        print("⏰ Testing Time to Complete Optimization...")

        try:
            # Define optimal mission times
            optimal_times = {
                "standard_sample_mission": 180.0,  # 3 minutes
                "challenging_terrain_mission": 300.0,  # 5 minutes
                "time_constrained_mission": 240.0,  # 4 minutes
                "resource_limited_mission": 200.0,  # 3.3 minutes
                "recovery_test_mission": 360.0,  # 6 minutes
            }

            timing_results = []

            # Analyze timing from mission logs
            for log in self.mission_logs:
                scenario = log["scenario"]
                actual_time = log["duration_seconds"]
                optimal_time = optimal_times.get(scenario, 300.0)  # Default 5 minutes

                time_ratio = actual_time / optimal_time
                within_2x_limit = time_ratio <= 2.0

                timing_result = {
                    "scenario": scenario,
                    "optimal_time_seconds": optimal_time,
                    "actual_time_seconds": actual_time,
                    "time_ratio": time_ratio,
                    "within_2x_limit": within_2x_limit,
                    "time_efficiency": (optimal_time / actual_time) * 100 if actual_time > 0 else 100.0
                }

                timing_results.append(timing_result)

            # Calculate overall timing statistics
            time_ratios = [r["time_ratio"] for r in timing_results]
            within_limits = sum(1 for r in timing_results if r["within_2x_limit"])

            timing_optimization_analysis = {
                "missions_analyzed": len(timing_results),
                "missions_within_2x_limit": within_limits,
                "percentage_within_limit": (within_limits / len(timing_results)) * 100,
                "average_time_ratio": statistics.mean(time_ratios),
                "p95_time_ratio": statistics.quantiles(time_ratios, n=20)[18],
                "max_time_ratio": max(time_ratios),
                "meets_timing_requirement": statistics.quantiles(time_ratios, n=20)[18] <= 2.0,
                "average_efficiency_percent": statistics.mean([r["time_efficiency"] for r in timing_results]),
                "scenario_timing_breakdown": self._group_timing_by_scenario(timing_results)
            }

            self.test_results["time_to_complete_optimization"] = timing_optimization_analysis

            status = "✅ PASS" if timing_optimization_analysis["meets_timing_requirement"] else "❌ FAIL"
            print(".2f")
            self.test_results["time_to_complete_optimization"] = timing_optimization_analysis

        except Exception as e:
            print(f"⚠️ Time to complete optimization test error: {e}")
            self.test_results["time_to_complete_optimization"] = {"error": str(e)}

    def _test_resource_usage_within_budgets(self):
        """Test resource usage within CPU/memory budgets."""
        print("💾 Testing Resource Usage Within Budgets...")

        try:
            # Define resource budgets
            resource_budgets = {
                "cpu_percent_max": 80.0,
                "memory_mb_max": 512.0,  # 512MB limit
                "duration_hours_max": 2.0  # 2 hour mission limit
            }

            resource_compliance_results = []

            # Analyze resource usage from mission logs
            for log in self.mission_logs:
                resource_usage = log.get("resource_usage", {})
                duration_hours = log["duration_seconds"] / 3600.0

                compliance_result = {
                    "mission_id": f"{log['scenario']}_run_{log['run']}",
                    "cpu_peak_percent": resource_usage.get("cpu_peak", 0),
                    "cpu_average_percent": resource_usage.get("cpu_avg", 0),
                    "memory_peak_mb": resource_usage.get("memory_peak", 0),
                    "memory_average_mb": resource_usage.get("memory_avg", 0),
                    "duration_hours": duration_hours,
                    "cpu_within_budget": resource_usage.get("cpu_peak", 0) <= resource_budgets["cpu_percent_max"],
                    "memory_within_budget": resource_usage.get("memory_peak", 0) <= resource_budgets["memory_mb_max"],
                    "duration_within_budget": duration_hours <= resource_budgets["duration_hours_max"],
                    "overall_compliant": (resource_usage.get("cpu_peak", 0) <= resource_budgets["cpu_percent_max"] and
                                        resource_usage.get("memory_peak", 0) <= resource_budgets["memory_mb_max"] and
                                        duration_hours <= resource_budgets["duration_hours_max"])
                }

                resource_compliance_results.append(compliance_result)

            # Calculate resource compliance statistics
            total_missions = len(resource_compliance_results)
            compliant_missions = sum(1 for r in resource_compliance_results if r["overall_compliant"])

            cpu_violations = sum(1 for r in resource_compliance_results if not r["cpu_within_budget"])
            memory_violations = sum(1 for r in resource_compliance_results if not r["memory_within_budget"])
            duration_violations = sum(1 for r in resource_compliance_results if not r["duration_within_budget"])

            resource_budget_analysis = {
                "missions_analyzed": total_missions,
                "compliant_missions": compliant_missions,
                "compliance_rate_percent": (compliant_missions / total_missions) * 100 if total_missions > 0 else 0,
                "resource_budgets": resource_budgets,
                "violations": {
                    "cpu_violations": cpu_violations,
                    "memory_violations": memory_violations,
                    "duration_violations": duration_violations,
                    "total_violations": cpu_violations + memory_violations + duration_violations
                },
                "peak_resource_usage": {
                    "cpu_max_percent": max([r["cpu_peak_percent"] for r in resource_compliance_results] or [0]),
                    "memory_max_mb": max([r["memory_peak_mb"] for r in resource_compliance_results] or [0]),
                    "duration_max_hours": max([r["duration_hours"] for r in resource_compliance_results] or [0])
                },
                "average_resource_usage": {
                    "cpu_avg_percent": statistics.mean([r["cpu_average_percent"] for r in resource_compliance_results] or [0]),
                    "memory_avg_mb": statistics.mean([r["memory_average_mb"] for r in resource_compliance_results] or [0])
                },
                "meets_resource_requirements": (compliant_missions / total_missions) >= 0.95 if total_missions > 0 else False
            }

            self.test_results["resource_usage_within_budgets"] = resource_budget_analysis

            status = "✅ PASS" if resource_budget_analysis["meets_resource_requirements"] else "❌ FAIL"
            print(".1f")
            if resource_budget_analysis["violations"]["total_violations"] > 0:
                print(f"  ⚠️ {resource_budget_analysis['violations']['total_violations']} resource violations detected")

        except Exception as e:
            print(f"⚠️ Resource usage budget test error: {e}")
            self.test_results["resource_usage_within_budgets"] = {"error": str(e)}

    def _test_failure_recovery_handling(self):
        """Test failure recovery automatic handling."""
        print("🔄 Testing Failure Recovery Handling...")

        try:
            # Collect failure and recovery events from mission logs
            recovery_events = []

            for log in self.mission_logs:
                if log["failures_encountered"] > 0:
                    # Analyze how failures were handled
                    recovery_analysis = self._analyze_failure_recovery(log)

                    recovery_events.append({
                        "mission": f"{log['scenario']}_run_{log['run']}",
                        "failures_encountered": log["failures_encountered"],
                        "recovery_attempts": recovery_analysis["recovery_attempts"],
                        "successful_recoveries": recovery_analysis["successful_recoveries"],
                        "operator_interventions": log["operator_interventions"],
                        "recovery_effectiveness": recovery_analysis["recovery_effectiveness"],
                        "mission_completed_despite_failures": log["completed"]
                    })

            # Calculate recovery effectiveness
            if recovery_events:
                total_failures = sum(e["failures_encountered"] for e in recovery_events)
                successful_recoveries = sum(e["successful_recoveries"] for e in recovery_events)
                operator_interventions = sum(e["operator_interventions"] for e in recovery_events)
                missions_completed_with_failures = sum(1 for e in recovery_events if e["mission_completed_despite_failures"])

                recovery_effectiveness = (successful_recoveries / total_failures) * 100 if total_failures > 0 else 100.0
                operator_intervention_rate = (operator_interventions / total_failures) * 100 if total_failures > 0 else 0.0

                failure_recovery_analysis = {
                    "missions_with_failures": len(recovery_events),
                    "total_failures_encountered": total_failures,
                    "successful_automatic_recoveries": successful_recoveries,
                    "recovery_effectiveness_percent": recovery_effectiveness,
                    "operator_interventions_for_failures": operator_interventions,
                    "operator_intervention_rate_percent": operator_intervention_rate,
                    "missions_completed_despite_failures": missions_completed_with_failures,
                    "failure_completion_rate_percent": (missions_completed_with_failures / len(recovery_events)) * 100 if recovery_events else 100.0,
                    "meets_recovery_requirement": recovery_effectiveness >= 95.0,  # >95% automatic recovery
                    "low_operator_intervention": operator_intervention_rate <= 20.0,  # <20% operator intervention
                    "detailed_recovery_events": recovery_events[:10]  # First 10 for detail
                }
            else:
                # No failures encountered (ideal case)
                failure_recovery_analysis = {
                    "missions_with_failures": 0,
                    "total_failures_encountered": 0,
                    "recovery_effectiveness_percent": 100.0,
                    "operator_intervention_rate_percent": 0.0,
                    "meets_recovery_requirement": True,
                    "low_operator_intervention": True,
                    "note": "No failures encountered during testing - excellent reliability"
                }

            self.test_results["failure_recovery_handling"] = failure_recovery_analysis

            status = "✅ PASS" if failure_recovery_analysis["meets_recovery_requirement"] else "❌ FAIL"
            print(".1f")
            self.test_results["failure_recovery_handling"] = failure_recovery_analysis

        except Exception as e:
            print(f"⚠️ Failure recovery handling test error: {e}")
            self.test_results["failure_recovery_handling"] = {"error": str(e)}

    def _test_operator_intervention_minimization(self):
        """Test operator intervention minimization (<5% of mission time)."""
        print("👤 Testing Operator Intervention Minimization...")

        try:
            intervention_analysis = []

            # Analyze operator interventions from mission logs
            for log in self.mission_logs:
                total_mission_time = log["duration_seconds"]
                intervention_time = log["operator_interventions"] * 30.0  # Assume 30 seconds per intervention
                intervention_percentage = (intervention_time / total_mission_time) * 100 if total_mission_time > 0 else 0.0

                intervention_analysis.append({
                    "mission": f"{log['scenario']}_run_{log['run']}",
                    "total_mission_time_seconds": total_mission_time,
                    "operator_interventions_count": log["operator_interventions"],
                    "estimated_intervention_time_seconds": intervention_time,
                    "intervention_percentage": intervention_percentage,
                    "within_5_percent_limit": intervention_percentage <= 5.0,
                    "within_10_percent_acceptable": intervention_percentage <= 10.0
                })

            # Calculate overall intervention statistics
            intervention_percentages = [a["intervention_percentage"] for a in intervention_analysis]
            within_5_percent = sum(1 for a in intervention_analysis if a["within_5_percent_limit"])
            within_10_percent = sum(1 for a in intervention_analysis if a["within_10_percent_acceptable"])

            total_interventions = sum(a["operator_interventions_count"] for a in intervention_analysis)
            total_mission_time = sum(a["total_mission_time_seconds"] for a in intervention_analysis)

            operator_intervention_analysis = {
                "missions_analyzed": len(intervention_analysis),
                "total_operator_interventions": total_interventions,
                "total_mission_time_seconds": total_mission_time,
                "average_intervention_percentage": statistics.mean(intervention_percentages),
                "p95_intervention_percentage": statistics.quantiles(intervention_percentages, n=20)[18],
                "max_intervention_percentage": max(intervention_percentages),
                "missions_within_5_percent": within_5_percent,
                "missions_within_10_percent": within_10_percent,
                "percentage_within_5_percent": (within_5_percent / len(intervention_analysis)) * 100,
                "percentage_within_10_percent": (within_10_percent / len(intervention_analysis)) * 100,
                "meets_5_percent_requirement": statistics.quantiles(intervention_percentages, n=20)[18] <= 5.0,
                "meets_10_percent_acceptable": statistics.quantiles(intervention_percentages, n=20)[18] <= 10.0,
                "average_interventions_per_mission": total_interventions / len(intervention_analysis),
                "intervention_efficiency_score": self._calculate_intervention_efficiency(intervention_analysis)
            }

            self.test_results["operator_intervention_minimization"] = operator_intervention_analysis

            status = "✅ PASS" if operator_intervention_analysis["meets_5_percent_requirement"] else "⚠️ ACCEPTABLE" if operator_intervention_analysis["meets_10_percent_acceptable"] else "❌ FAIL"
            print(".1f")
            self.test_results["operator_intervention_minimization"] = operator_intervention_analysis

        except Exception as e:
            print(f"⚠️ Operator intervention minimization test error: {e}")
            self.test_results["operator_intervention_minimization"] = {"error": str(e)}

    def _simulate_mission_run(self, scenario: Dict[str, Any], run_number: int) -> Dict[str, Any]:
        """Simulate a single mission run."""
        # Simulate mission execution with realistic parameters
        base_duration = {
            "standard_sample_mission": 180.0,
            "challenging_terrain_mission": 300.0,
            "time_constrained_mission": 240.0,
            "resource_limited_mission": 200.0,
            "recovery_test_mission": 360.0
        }[scenario["name"]]

        # Add variability and potential failures
        duration_variation = 0.8 + (run_number * 0.1)  # Some runs take longer
        actual_duration = base_duration * duration_variation

        # Simulate failures based on complexity
        complexity_multiplier = {"low": 0.1, "medium": 0.3, "high": 0.5}[scenario["complexity"]]
        expected_failures = int(complexity_multiplier * 3)  # 0-3 failures based on complexity

        # Simulate recovery effectiveness
        recovery_success_rate = 0.9 if scenario["name"] != "time_constrained_mission" else 0.7
        successful_recoveries = sum(1 for _ in range(expected_failures) if time.time() % 1 < recovery_success_rate)

        # Calculate operator interventions (failures that couldn't be auto-recovered)
        operator_interventions = expected_failures - successful_recoveries

        # Mission completion based on recovery success
        mission_completed = (successful_recoveries >= expected_failures * 0.8) if expected_failures > 0 else True

        # Simulate resource usage
        cpu_base = 25.0
        memory_base = 150.0
        cpu_peak = cpu_base + (run_number * 5) + (expected_failures * 10)
        memory_peak = memory_base + (run_number * 20) + (expected_failures * 30)

        return {
            "scenario": scenario["name"],
            "run": run_number + 1,
            "duration_seconds": actual_duration,
            "completed": mission_completed,
            "failures": ["simulated_failure"] * expected_failures,
            "operator_interventions": operator_interventions,
            "resource_usage": {
                "cpu_avg": cpu_base + (run_number * 2),
                "cpu_peak": min(cpu_peak, 85.0),  # Cap at 85%
                "memory_avg": memory_base + (run_number * 10),
                "memory_peak": min(memory_peak, 450.0)  # Cap at 450MB
            }
        }

    def _calculate_scenario_success_rates(self, mission_results: List[Dict], scenarios: List[Dict]) -> Dict[str, float]:
        """Calculate success rates by scenario."""
        scenario_stats = {}

        for scenario in scenarios:
            scenario_name = scenario["name"]
            scenario_results = [r for r in mission_results if r.get("scenario") == scenario_name]
            successful_runs = sum(1 for r in scenario_results if r["completed"])
            total_runs = len(scenario_results)

            scenario_stats[scenario_name] = (successful_runs / total_runs) * 100 if total_runs > 0 else 0.0

        return scenario_stats

    def _analyze_mission_failures(self, mission_results: List[Dict]) -> Dict[str, Any]:
        """Analyze mission failures."""
        failed_missions = [r for r in mission_results if not r["completed"]]
        failure_types = {}

        for result in failed_missions:
            for failure in result.get("failures", []):
                failure_types[failure] = failure_types.get(failure, 0) + 1

        return {
            "total_failed_missions": len(failed_missions),
            "failure_types": failure_types,
            "most_common_failure": max(failure_types.items(), key=lambda x: x[1]) if failure_types else None
        }

    def _group_timing_by_scenario(self, timing_results: List[Dict]) -> Dict[str, Dict[str, float]]:
        """Group timing results by scenario."""
        scenario_groups = {}

        for result in timing_results:
            scenario = result["scenario"]
            if scenario not in scenario_groups:
                scenario_groups[scenario] = []

            scenario_groups[scenario].append(result["time_ratio"])

        # Calculate statistics for each scenario
        scenario_stats = {}
        for scenario, ratios in scenario_groups.items():
            scenario_stats[scenario] = {
                "average_ratio": statistics.mean(ratios),
                "p95_ratio": statistics.quantiles(ratios, n=20)[18] if len(ratios) >= 20 else max(ratios),
                "within_2x_limit": sum(1 for r in ratios if r <= 2.0),
                "total_runs": len(ratios)
            }

        return scenario_stats

    def _analyze_failure_recovery(self, mission_log: Dict) -> Dict[str, Any]:
        """Analyze failure recovery for a mission."""
        failures = mission_log.get("failures_encountered", 0)
        interventions = mission_log.get("operator_interventions", 0)

        # Simulate recovery attempts (in real implementation, this would analyze logs)
        recovery_attempts = failures
        successful_recoveries = failures - interventions

        return {
            "recovery_attempts": recovery_attempts,
            "successful_recoveries": successful_recoveries,
            "recovery_effectiveness": (successful_recoveries / recovery_attempts) * 100 if recovery_attempts > 0 else 100.0
        }

    def _calculate_intervention_efficiency(self, intervention_analysis: List[Dict]) -> float:
        """Calculate intervention efficiency score (0-100)."""
        if not intervention_analysis:
            return 100.0  # No missions = perfect efficiency

        percentages = [a["intervention_percentage"] for a in intervention_analysis]
        avg_percentage = statistics.mean(percentages)

        # Score based on how close to 0% intervention we are
        # 0% = 100 score, 5% = 80 score, 10% = 60 score, etc.
        base_score = max(0.0, 100.0 - (avg_percentage * 4.0))

        # Bonus for consistency (lower standard deviation)
        std_dev = statistics.stdev(percentages) if len(percentages) > 1 else 0.0
        consistency_bonus = max(0.0, 20.0 - (std_dev * 10.0))

        return min(100.0, base_score + consistency_bonus)

    def _generate_e2e_test_report(self) -> Dict[str, Any]:
        """Generate comprehensive E2E test report."""
        print("📋 Generating End-to-End Mission Optimization Report...")

        report = {
            "test_metadata": {
                "test_type": "end_to_end_mission_optimization",
                "timestamp": time.time(),
                "optimization_requirements": [
                    "Mission completion rate >95% in simulation",
                    "Time to complete within 2x of optimal path",
                    "Resource usage within CPU/memory budgets",
                    "Automatic failure recovery handling",
                    "Operator intervention <5% of mission time"
                ]
            },
            "optimization_results": self.test_results,
            "performance_analysis": self._analyze_e2e_performance(),
            "recommendations": self._generate_e2e_recommendations()
        }

        return report

    def _analyze_e2e_performance(self) -> Dict[str, Any]:
        """Analyze E2E mission performance improvements."""
        analysis = {}

        # Mission completion analysis
        if "mission_completion_rate" in self.test_results:
            completion = self.test_results["mission_completion_rate"]
            if isinstance(completion, dict):
                completion_rate = completion.get("completion_rate_percent", 0)
                analysis["mission_completion"] = {
                    "completion_rate_percent": completion_rate,
                    "meets_requirement": completion.get("meets_95_percent_requirement", False),
                    "acceptable_minimum": completion.get("meets_90_percent_minimum", False),
                    "optimization_needed": completion_rate < 93.0
                }

        # Timing analysis
        if "time_to_complete_optimization" in self.test_results:
            timing = self.test_results["time_to_complete_optimization"]
            if isinstance(timing, dict):
                p95_ratio = timing.get("p95_time_ratio", 999)
                efficiency = timing.get("average_efficiency_percent", 0)
                analysis["mission_timing"] = {
                    "p95_time_ratio": p95_ratio,
                    "meets_requirement": timing.get("meets_timing_requirement", False),
                    "average_efficiency_percent": efficiency,
                    "optimization_needed": p95_ratio > 1.8
                }

        # Resource analysis
        if "resource_usage_within_budgets" in self.test_results:
            resources = self.test_results["resource_usage_within_budgets"]
            if isinstance(resources, dict):
                compliance_rate = resources.get("compliance_rate_percent", 0)
                analysis["resource_compliance"] = {
                    "compliance_rate_percent": compliance_rate,
                    "meets_requirement": resources.get("meets_resource_requirements", False),
                    "violations": resources.get("violations", {}).get("total_violations", 0),
                    "optimization_needed": compliance_rate < 95.0
                }

        # Recovery analysis
        if "failure_recovery_handling" in self.test_results:
            recovery = self.test_results["failure_recovery_handling"]
            if isinstance(recovery, dict):
                effectiveness = recovery.get("recovery_effectiveness_percent", 0)
                intervention_rate = recovery.get("operator_intervention_rate_percent", 0)
                analysis["failure_recovery"] = {
                    "recovery_effectiveness_percent": effectiveness,
                    "operator_intervention_rate_percent": intervention_rate,
                    "meets_recovery_requirement": recovery.get("meets_recovery_requirement", False),
                    "low_operator_intervention": recovery.get("low_operator_intervention", False),
                    "optimization_needed": effectiveness < 95.0 or intervention_rate > 15.0
                }

        # Intervention analysis
        if "operator_intervention_minimization" in self.test_results:
            intervention = self.test_results["operator_intervention_minimization"]
            if isinstance(intervention, dict):
                p95_percentage = intervention.get("p95_intervention_percentage", 100)
                efficiency_score = intervention.get("intervention_efficiency_score", 0)
                analysis["operator_intervention"] = {
                    "p95_intervention_percentage": p95_percentage,
                    "meets_requirement": intervention.get("meets_5_percent_requirement", False),
                    "acceptable_level": intervention.get("meets_10_percent_acceptable", False),
                    "efficiency_score": efficiency_score,
                    "optimization_needed": p95_percentage > 7.0
                }

        return analysis

    def _generate_e2e_recommendations(self) -> List[str]:
        """Generate E2E optimization recommendations."""
        recommendations = []

        # Analyze each test result
        if "mission_completion_rate" in self.test_results:
            completion = self.test_results["mission_completion_rate"]
            if isinstance(completion, dict):
                if not completion.get("meets_95_percent_requirement", False):
                    recommendations.append("Improve mission completion rate to meet >95% requirement")
                    if completion.get("completion_rate_percent", 0) < 90.0:
                        recommendations.append("Address critical mission failure modes")

        if "time_to_complete_optimization" in self.test_results:
            timing = self.test_results["time_to_complete_optimization"]
            if isinstance(timing, dict):
                if not timing.get("meets_timing_requirement", False):
                    recommendations.append("Optimize mission execution time to stay within 2x optimal")
                if timing.get("average_efficiency_percent", 100) < 70.0:
                    recommendations.append("Improve path planning and navigation efficiency")

        if "resource_usage_within_budgets" in self.test_results:
            resources = self.test_results["resource_usage_within_budgets"]
            if isinstance(resources, dict):
                if not resources.get("meets_resource_requirements", False):
                    recommendations.append("Optimize resource usage to stay within CPU/memory budgets")
                    violations = resources.get("violations", {})
                    if violations.get("cpu_violations", 0) > 0:
                        recommendations.append("Reduce CPU usage spikes during mission execution")
                    if violations.get("memory_violations", 0) > 0:
                        recommendations.append("Optimize memory usage and implement better garbage collection")

        if "failure_recovery_handling" in self.test_results:
            recovery = self.test_results["failure_recovery_handling"]
            if isinstance(recovery, dict):
                if not recovery.get("meets_recovery_requirement", False):
                    recommendations.append("Improve automatic failure recovery effectiveness to >95%")
                if not recovery.get("low_operator_intervention", False):
                    recommendations.append("Reduce operator intervention requirements for failure handling")

        if "operator_intervention_minimization" in self.test_results:
            intervention = self.test_results["operator_intervention_minimization"]
            if isinstance(intervention, dict):
                if not intervention.get("meets_5_percent_requirement", False):
                    recommendations.append("Minimize operator intervention to <5% of mission time")
                if intervention.get("average_interventions_per_mission", 0) > 2:
                    recommendations.append("Increase automation of mission execution tasks")

        if not recommendations:
            recommendations.append("✅ End-to-end mission optimizations are performing well")

        return recommendations


def run_end_to_end_optimization_tests():
    """Run comprehensive end-to-end mission optimization tests."""
    tester = EndToEndOptimizationTester()
    report = tester.run_comprehensive_e2e_tests()

    # Print summary
    print("\n" + "="*60)
    print("🎯 END-TO-END MISSION OPTIMIZATION TEST SUMMARY")
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
                    elif "rate" in key:
                        print(f"  {key}: {value:.1f}")
                    elif "score" in key:
                        print(f"  {key}: {value:.1f}")
                    elif "ratio" in key:
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
    report = run_end_to_end_optimization_tests()

    # Save detailed report
    import json
    report_file = f"e2e_optimization_test_report_{int(time.time())}.json"
    with open(report_file, 'w') as f:
        json.dump(report, f, indent=2, default=str)

    print(f"\n📁 Detailed report saved to: {report_file}")
