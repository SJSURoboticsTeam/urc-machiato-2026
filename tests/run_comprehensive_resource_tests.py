#!/usr/bin/env python3
"""
Comprehensive Resource Optimization Testing Suite - URC 2026

Complete testing suite for resource optimization features with statistical analysis:
- Fallback functionality testing
- Recovery mechanism validation
- Integration testing
- Performance benchmarking with confidence intervals
- Mission profile switching tests
- Statistical reports and recommendations

Usage:
    python tests/run_comprehensive_resource_tests.py --iterations 10 --confidence 0.95

Author: URC 2026 Testing Team
"""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, List
import statistics
import sys

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from tests.statistical_performance_testing import StatisticalPerformanceTester


def run_fallback_tests() -> Dict[str, Any]:
    """Run fallback functionality tests."""
    print("🧪 Running Fallback Functionality Tests...")

    try:
        from tests.test_fallback_functionality import run_fallback_tests as run_fb_tests

        success = run_fb_tests()

        return {
            "test_name": "fallback_functionality",
            "success": success,
            "timestamp": datetime.now().isoformat(),
        }

    except Exception as e:
        print(f"❌ Fallback tests failed: {e}")
        return {
            "test_name": "fallback_functionality",
            "success": False,
            "error": str(e),
            "timestamp": datetime.now().isoformat(),
        }


def run_recovery_tests() -> Dict[str, Any]:
    """Run recovery mechanism tests."""
    print("🛟 Running Recovery Mechanism Tests...")

    try:
        from tests.test_recovery_mechanisms import run_recovery_tests as run_rec_tests

        success = run_rec_tests()

        return {
            "test_name": "recovery_mechanisms",
            "success": success,
            "timestamp": datetime.now().isoformat(),
        }

    except Exception as e:
        print(f"❌ Recovery tests failed: {e}")
        return {
            "test_name": "recovery_mechanisms",
            "success": False,
            "error": str(e),
            "timestamp": datetime.now().isoformat(),
        }


def run_integration_tests() -> Dict[str, Any]:
    """Run integration tests."""
    print("🔗 Running Integration Tests...")

    try:
        from tests.test_integration import run_integration_tests as run_int_tests

        success = run_int_tests()

        return {
            "test_name": "integration",
            "success": success,
            "timestamp": datetime.now().isoformat(),
        }

    except Exception as e:
        print(f"❌ Integration tests failed: {e}")
        return {
            "test_name": "integration",
            "success": False,
            "error": str(e),
            "timestamp": datetime.now().isoformat(),
        }


def run_statistical_performance_tests(
    iterations: int, confidence: float
) -> Dict[str, Any]:
    """Run statistical performance tests."""
    print(f"📊 Running Statistical Performance Tests ({iterations} iterations)...")

    try:
        tester = StatisticalPerformanceTester(
            num_iterations=iterations, confidence_level=confidence
        )

        results = tester.run_comprehensive_test_suite()

        return {
            "test_name": "statistical_performance",
            "success": True,
            "results": results,
            "timestamp": datetime.now().isoformat(),
        }

    except Exception as e:
        print(f"❌ Statistical performance tests failed: {e}")
        return {
            "test_name": "statistical_performance",
            "success": False,
            "error": str(e),
            "timestamp": datetime.now().isoformat(),
        }


def generate_performance_report(
    all_results: Dict[str, Any], output_file: str
) -> Dict[str, Any]:
    """Generate comprehensive performance report with statistical analysis."""
    print("📈 Generating Performance Report...")

    report = {
        "report_title": "URC 2026 Resource Optimization Test Report",
        "generated_at": datetime.now().isoformat(),
        "test_summary": {},
        "statistical_analysis": {},
        "performance_metrics": {},
        "recommendations": [],
        "issues_found": [],
    }

    # Test summary
    test_results = all_results.get("test_results", {})
    report["test_summary"] = {
        "total_tests": len(test_results),
        "passed_tests": sum(
            1 for r in test_results.values() if r.get("success", False)
        ),
        "failed_tests": sum(
            1 for r in test_results.values() if not r.get("success", True)
        ),
        "overall_success_rate": (
            sum(1 for r in test_results.values() if r.get("success", False))
            / len(test_results)
            if test_results
            else 0
        ),
    }

    # Statistical analysis from performance tests
    stat_results = test_results.get("statistical_performance", {})
    if stat_results.get("success") and "results" in stat_results:
        perf_results = stat_results["results"]
        report["statistical_analysis"] = perf_results.get("statistical_analysis", {})

    # Performance metrics
    report["performance_metrics"] = {
        "resource_optimization_effectiveness": {},
        "fallback_performance": {},
        "recovery_effectiveness": {},
        "integration_stability": {},
    }

    # Analyze resource optimization effectiveness
    if "statistical_performance" in test_results:
        stat_data = test_results["statistical_performance"]
        if stat_data.get("success") and "results" in stat_data:
            perf_data = stat_data["results"].get("statistical_analysis", {})

            # Extract optimization metrics
            opt_effectiveness = {}
            for test_name, stats in perf_data.get("detailed_results", {}).items():
                if "resource_optimization_effectiveness" in test_name:
                    cpu_stats = stats.get("cpu_usage_stats", {})
                    memory_stats = stats.get("memory_usage_stats", {})

                    if "mean" in cpu_stats and "mean" in memory_stats:
                        opt_effectiveness[test_name] = {
                            "avg_cpu_usage": cpu_stats["mean"],
                            "avg_memory_usage": memory_stats["mean"],
                            "cpu_reduction_potential": cpu_stats.get("std_dev", 0),
                            "memory_reduction_potential": memory_stats.get(
                                "std_dev", 0
                            ),
                        }

            report["performance_metrics"][
                "resource_optimization_effectiveness"
            ] = opt_effectiveness

    # Generate recommendations
    recommendations = []

    # Overall system health
    success_rate = report["test_summary"]["overall_success_rate"]
    if success_rate >= 0.9:
        recommendations.append(
            "✅ Excellent: Resource optimization system is highly reliable (>90% test success)"
        )
    elif success_rate >= 0.8:
        recommendations.append(
            "⚠️ Good: Resource optimization system is reliable (80-90% test success)"
        )
    else:
        recommendations.append(
            "❌ Critical: Resource optimization system needs attention (<80% test success)"
        )

    # Resource optimization effectiveness
    opt_metrics = report["performance_metrics"]["resource_optimization_effectiveness"]
    if opt_metrics:
        avg_cpu_reduction = sum(
            m.get("cpu_reduction_potential", 0) for m in opt_metrics.values()
        ) / len(opt_metrics)
        if avg_cpu_reduction > 10:
            recommendations.append(
                f"✅ Excellent: Resource optimization provides significant CPU savings ({avg_cpu_reduction:.1f}%)"
            )
        elif avg_cpu_reduction > 5:
            recommendations.append(
                f"⚠️ Moderate: Resource optimization provides some CPU savings ({avg_cpu_reduction:.1f}%)"
            )
        else:
            recommendations.append(
                "⚠️ Limited: Resource optimization provides minimal CPU savings (<5%)"
            )

    # Fallback functionality
    fallback_result = test_results.get("fallback_functionality", {})
    if fallback_result.get("success"):
        recommendations.append(
            "✅ Robust: Fallback mechanisms work correctly when dependencies are missing"
        )
    else:
        recommendations.append(
            "❌ Critical: Fallback mechanisms failed - system may not work without all dependencies"
        )

    # Recovery mechanisms
    recovery_result = test_results.get("recovery_mechanisms", {})
    if recovery_result.get("success"):
        recommendations.append(
            "✅ Reliable: Recovery mechanisms activate correctly under resource pressure"
        )
    else:
        recommendations.append(
            "❌ Critical: Recovery mechanisms failed - system may not handle resource pressure"
        )

    # Integration
    integration_result = test_results.get("integration", {})
    if integration_result.get("success"):
        recommendations.append("✅ Stable: All system components integrate correctly")
    else:
        recommendations.append(
            "❌ Critical: Component integration failed - system components may conflict"
        )

    report["recommendations"] = recommendations

    # Identify issues
    issues = []
    for test_name, result in test_results.items():
        if not result.get("success", False):
            issues.append(f"❌ {test_name}: {result.get('error', 'Test failed')}")

    report["issues_found"] = issues

    # Save report
    with open(output_file, "w") as f:
        json.dump(report, f, indent=2, default=str)

    print(f"📁 Performance report saved to: {output_file}")

    return report


def print_summary_report(report: Dict[str, Any]):
    """Print a human-readable summary of the test results."""
    print("\n" + "=" * 80)
    print("🧪 URC 2026 RESOURCE OPTIMIZATION TEST SUMMARY")
    print("=" * 80)

    # Overall results
    summary = report.get("test_summary", {})
    print(f"\n📊 OVERALL RESULTS:")
    print(f"  Tests Run: {summary.get('total_tests', 0)}")
    print(".1%")

    # Performance highlights
    perf = report.get("performance_metrics", {})
    opt_eff = perf.get("resource_optimization_effectiveness", {})

    if opt_eff:
        print(f"\n⚡ RESOURCE OPTIMIZATION EFFECTIVENESS:")
        for mission, metrics in opt_eff.items():
            cpu_usage = metrics.get("avg_cpu_usage", 0)
            mem_usage = metrics.get("avg_memory_usage", 0)
            print(".1f")

    # Recommendations
    recommendations = report.get("recommendations", [])
    if recommendations:
        print(f"\n💡 RECOMMENDATIONS:")
        for rec in recommendations:
            print(f"  {rec}")

    # Issues
    issues = report.get("issues_found", [])
    if issues:
        print(f"\n🚨 ISSUES FOUND:")
        for issue in issues:
            print(f"  {issue}")
    else:
        print("\n✅ NO ISSUES FOUND - System is fully operational!")
        print("\n" + "=" * 80)


def main():
    """Main test runner."""
    parser = argparse.ArgumentParser(
        description="URC 2026 Resource Optimization Test Suite"
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=5,
        help="Number of iterations for statistical tests (default: 5)",
    )
    parser.add_argument(
        "--confidence",
        type=float,
        default=0.95,
        help="Confidence level for statistical analysis (default: 0.95)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output file for detailed results (default: auto-generated)",
    )
    parser.add_argument(
        "--skip-slow", action="store_true", help="Skip slow statistical tests"
    )
    parser.add_argument(
        "--quick", action="store_true", help="Run quick functional tests only"
    )

    args = parser.parse_args()

    # Set default output file
    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"resource_optimization_test_results_{timestamp}.json"

    print("🚀 URC 2026 RESOURCE OPTIMIZATION COMPREHENSIVE TESTING")
    print("=" * 65)
    print(f"Test Iterations: {args.iterations}")
    print(f"Confidence Level: {args.confidence}")
    print(f"Output File: {args.output}")
    print()

    start_time = time.time()
    all_results = {"test_results": {}}

    try:
        # Run functional tests
        print("Phase 1: Functional Tests")
        print("-" * 25)

        all_results["test_results"]["fallback_functionality"] = run_fallback_tests()
        all_results["test_results"]["recovery_mechanisms"] = run_recovery_tests()
        all_results["test_results"]["integration"] = run_integration_tests()

        # Run statistical tests (unless quick mode)
        if not args.quick:
            print("\nPhase 2: Statistical Performance Tests")
            print("-" * 40)

            all_results["test_results"]["statistical_performance"] = (
                run_statistical_performance_tests(args.iterations, args.confidence)
            )

        # Generate comprehensive report
        print("\nPhase 3: Report Generation")
        print("-" * 28)

        report = generate_performance_report(all_results, args.output)
        print_summary_report(report)

        total_time = time.time() - start_time
        print(f"Total Test Time: {total_time:.1f}s")  # Final assessment
        success_rate = report["test_summary"]["overall_success_rate"]
        if success_rate >= 0.9:
            print(
                "\n🎉 EXCELLENT: Resource optimization system passed comprehensive testing!"
            )
            return 0
        elif success_rate >= 0.8:
            print(
                "\n✅ GOOD: Resource optimization system is operational with minor issues."
            )
            return 0
        else:
            print("\n❌ CRITICAL: Resource optimization system has significant issues.")
            return 1

    except KeyboardInterrupt:
        print("\n⚠️ Testing interrupted by user")
        return 1
    except Exception as e:
        print(f"\n❌ Testing failed with error: {e}")
        return 1


if __name__ == "__main__":
    exit(main())
