#!/usr/bin/env python3
"""
URC 2026 Comprehensive System Test Suite
Focuses on Communication & Cognition for Hardware-in-the-Loop Testing

Tests organized by category:
1. Communication Stack (WebSocket, Bridges, Network)
2. Cognition System (Behavior Trees, State Machines, Missions)
3. Integration (Full system workflows)
4. Performance (Benchmarks)

Author: URC 2026 Testing Team
"""

import subprocess
import sys
import json
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Tuple

# Configuration
TEST_RESULTS_FILE = Path("test_results.json")
REPORT_FILE = Path("SYSTEM_TEST_REPORT.md")

# Test categories
TEST_SUITES = {
    "communication": {
        "name": "Communication Stack",
        "description": "WebSocket, bridges, network resilience",
        "tests": [
            "tests/unit/test_websocket_server_simulator.py",
            "tests/unit/test_bridge_communications.py",
            "tests/unit/test_simple_websocket_bridge.py",
            "tests/integration/test_websocket_bridge_integration.py",
            "tests/integration/test_complete_communication_stack.py",
            "tests/integration/test_network_integration.py",
            "tests/integration/test_network_resilience.py",
        ],
    },
    "cognition": {
        "name": "Cognition System",
        "description": "Behavior trees, state machines, mission execution",
        "tests": [
            "tests/unit/test_bt_orchestrator_implementation.py",
            "tests/unit/test_behavior_tree_failures.py",
            "tests/unit/test_mission_executor.py",
            "tests/integration/test_complete_bt_state_machine_flow.py",
            "tests/integration/test_bt_state_machine_runtime.py",
            "tests/integration/test_mission_system.py",
            "tests/integration/test_bt_runtime_integration.py",
        ],
    },
    "integration": {
        "name": "Full System Integration",
        "description": "Communication + Cognition + Perception workflows",
        "tests": [
            "tests/integration/test_bridges.py",
            "tests/integration/test_mission_validation.py",
            "tests/integration/test_comprehensive_bt_integration.py",
            "tests/integration/test_ros2_state_machine_bridge.py",
        ],
    },
    "performance": {
        "name": "Performance & Stress Testing",
        "description": "Communication performance, mission execution benchmarks",
        "tests": [
            "tests/performance/test_ros2_communication_performance.py",
            "tests/performance/stress_test_network_communication.py",
            "tests/performance/test_mission_execution_performance.py",
        ],
    },
}


def run_test(test_file: str, verbose: bool = True) -> Tuple[bool, str]:
    """Run a single test file and return status."""
    try:
        cmd = ["python3", "-m", "pytest", test_file, "-v", "--tb=short"]
        if not verbose:
            cmd.remove("-v")

        result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)

        return result.returncode == 0, result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        return False, f"Test timeout: {test_file}"
    except Exception as e:
        return False, f"Test error: {str(e)}"


def run_test_suite(suite_name: str, suite_config: Dict) -> Dict:
    """Run all tests in a suite and collect results."""
    print(f"\n{'='*70}")
    print(f"Running: {suite_config['name']}")
    print(f"{'='*70}")
    print(f"Description: {suite_config['description']}")

    results = {
        "name": suite_config["name"],
        "description": suite_config["description"],
        "tests": [],
        "passed": 0,
        "failed": 0,
        "skipped": 0,
        "total": 0,
    }

    for test_file in suite_config["tests"]:
        test_path = Path(test_file)
        if not test_path.exists():
            print(f"⚠️  SKIP: {test_file} (not found)")
            results["skipped"] += 1
            results["tests"].append(
                {"file": test_file, "status": "skipped", "reason": "file not found"}
            )
            continue

        print(f"\n📋 Testing: {test_file}")
        success, output = run_test(test_file, verbose=False)

        if success:
            print(f"✅ PASS: {test_file}")
            results["passed"] += 1
            results["tests"].append({"file": test_file, "status": "passed"})
        else:
            print(f"❌ FAIL: {test_file}")
            results["failed"] += 1
            results["tests"].append(
                {
                    "file": test_file,
                    "status": "failed",
                    "reason": output[:200],  # First 200 chars of error
                }
            )

        results["total"] += 1

    return results


def generate_report(all_results: List[Dict]) -> str:
    """Generate markdown report of all test results."""
    report = "# URC 2026 System Test Report\n\n"
    report += f"**Generated**: {datetime.now().isoformat()}\n"
    report += f"**Environment**: ROS2 Jazzy, Python 3.12.3\n\n"

    # Summary
    total_suites = len(all_results)
    total_tests = sum(r["total"] for r in all_results)
    total_passed = sum(r["passed"] for r in all_results)
    total_failed = sum(r["failed"] for r in all_results)
    total_skipped = sum(r["skipped"] for r in all_results)

    report += "## Executive Summary\n\n"
    report += f"- **Test Suites**: {total_suites}\n"
    report += f"- **Total Tests**: {total_tests}\n"
    report += f"- **Passed**: {total_passed} ({100*total_passed//total_tests if total_tests else 0}%)\n"
    report += f"- **Failed**: {total_failed}\n"
    report += f"- **Skipped**: {total_skipped}\n\n"

    if total_failed == 0:
        report += "✅ **ALL TESTS PASSED!**\n\n"
    else:
        report += f"⚠️  **{total_failed} tests failed**\n\n"

    # Detailed results
    report += "## Test Results by Category\n\n"

    for suite in all_results:
        report += f"### {suite['name']}\n\n"
        report += f"**Status**: "
        if suite["failed"] == 0:
            report += "✅ PASS\n"
        else:
            report += f"❌ FAIL ({suite['failed']} failures)\n"

        report += f"- Passed: {suite['passed']}\n"
        report += f"- Failed: {suite['failed']}\n"
        report += f"- Skipped: {suite['skipped']}\n"
        report += f"- Total: {suite['total']}\n\n"

        if suite["failed"] > 0:
            report += "**Failed Tests**:\n\n"
            for test in suite["tests"]:
                if test["status"] == "failed":
                    report += f"- `{test['file']}`\n"
                    if "reason" in test:
                        report += f"  Error: {test['reason']}\n"
        report += "\n"

    # Recommendations
    report += "## Recommendations for Hardware-in-the-Loop Testing\n\n"

    if total_failed == 0:
        report += "✅ **All systems ready for hardware integration!**\n\n"
        report += "### Next Steps:\n"
        report += "1. Deploy to hardware platform\n"
        report += "2. Test motor control with actual hardware\n"
        report += "3. Validate sensor integration\n"
        report += "4. Run safety-critical mission tests\n"
        report += "5. Perform field trials\n"
    else:
        report += (
            f"⚠️  **{total_failed} systems need attention before hardware testing**\n\n"
        )
        report += "### Issues to Address:\n"
        for suite in all_results:
            if suite["failed"] > 0:
                report += f"- {suite['name']}: Fix {suite['failed']} failing tests\n"

    report += "\n---\n"
    report += f"*Report Generated: {datetime.now().isoformat()}*\n"

    return report


def main():
    """Run all test suites and generate report."""
    print("\n" + "=" * 70)
    print("URC 2026 COMPREHENSIVE SYSTEM TEST SUITE")
    print("Communication & Cognition Testing for Hardware-in-the-Loop")
    print("=" * 70)

    all_results = []

    # Run each test suite
    for suite_name, suite_config in TEST_SUITES.items():
        results = run_test_suite(suite_name, suite_config)
        all_results.append(results)

    # Generate report
    report = generate_report(all_results)

    # Write report
    with open(REPORT_FILE, "w") as f:
        f.write(report)

    print("\n" + "=" * 70)
    print("TEST EXECUTION COMPLETE")
    print("=" * 70)

    # Final summary
    total_suites = len(all_results)
    total_tests = sum(r["total"] for r in all_results)
    total_passed = sum(r["passed"] for r in all_results)
    total_failed = sum(r["failed"] for r in all_results)

    print(f"\n📊 Final Results:")
    print(f"   Test Suites: {total_suites}")
    print(f"   Total Tests: {total_tests}")
    print(f"   ✅ Passed: {total_passed}")
    print(f"   ❌ Failed: {total_failed}")

    if total_failed == 0:
        print(f"\n🚀 SUCCESS! All {total_tests} tests passed!")
        print(f"System ready for hardware-in-the-loop testing.\n")
        return 0
    else:
        print(f"\n⚠️  {total_failed} tests failed. Review {REPORT_FILE} for details.\n")
        return 1


if __name__ == "__main__":
    sys.exit(main())
