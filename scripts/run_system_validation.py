#!/usr/bin/env python3
"""
URC 2026 System Validation - Single entry point for post-migration system tests.

Runs critical systems (blackboard, state machine, behavior tree), communication,
infrastructure, and integration tests. Supports smoke, category, and full runs
with optional simulation.

Usage:
  python scripts/run_system_validation.py --all
  python scripts/run_system_validation.py --category critical
  python scripts/run_system_validation.py --with-simulation
  python scripts/run_system_validation.py --smoke

Author: URC 2026 Testing Team
"""

import argparse
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Test categories and their pytest targets (paths or markers)
TEST_CATEGORIES: Dict[str, Dict[str, str]] = {
    "critical": {
        "name": "Critical Systems",
        "description": "Blackboard, state machine, behavior tree",
        "target": "tests/critical/",
        "marker": "critical",
    },
    "communication": {
        "name": "Communication",
        "description": "ROS2 topics, WebSocket, CAN bus",
        "target": "tests/unit/test_websocket_server_simulator.py tests/unit/test_bridge_communications.py tests/integration/test_complete_communication_stack.py tests/integration/test_network_integration.py",
        "marker": "",
    },
    "infrastructure": {
        "name": "Infrastructure",
        "description": "Config loading, simulation, monitoring",
        "target": "tests/unit/ tests/integration/test_bridges.py",
        "marker": "",
    },
    "integration": {
        "name": "Integration",
        "description": "Full stack, mission execution",
        "target": "tests/integration/test_comprehensive_bt_integration.py tests/integration/test_bt_state_machine_runtime.py tests/integration/test_mission_system.py tests/integration/test_mission_validation.py",
        "marker": "integration",
    },
}

# Smoke: minimal set for fast feedback (critical systems only; unit BT may need launch_testing)
SMOKE_TARGETS = [
    "tests/critical/test_critical_systems_integration.py",
]

# With-simulation: add simulation-based tests
SIMULATION_TARGETS = [
    "tests/integration/test_simulation_validated_systems.py",
    "tests/integration/test_full_system_with_simulator.py",
]


def run_pytest(
    targets: List[str],
    verbose: bool = True,
    timeout: int = 300,
    extra_args: Optional[List[str]] = None,
) -> Tuple[int, str]:
    """Run pytest on given paths/markers. Returns (exit_code, output)."""
    cmd = [
        sys.executable,
        "-m",
        "pytest",
        *targets,
        "-v",
        "--tb=short",
    ]
    if not verbose:
        cmd.remove("-v")
    if extra_args:
        cmd.extend(extra_args)
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout + 30,
            cwd=Path(__file__).resolve().parent.parent,
        )
        out = result.stdout + result.stderr
        return result.returncode, out
    except subprocess.TimeoutExpired:
        return -1, "Test run timed out."
    except Exception as e:
        return -1, str(e)


def run_by_marker(marker: str, verbose: bool = True, timeout: int = 120) -> Tuple[int, str]:
    """Run tests by pytest marker (e.g. critical, integration)."""
    cmd = [
        sys.executable,
        "-m",
        "pytest",
        "tests/",
        "-v",
        "--tb=short",
        f"-m{marker}",
    ]
    if not verbose:
        cmd.remove("-v")
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout + 30,
            cwd=Path(__file__).resolve().parent.parent,
        )
        return result.returncode, result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        return -1, "Test run timed out."
    except Exception as e:
        return -1, str(e)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="URC 2026 System Validation - run post-migration system tests."
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Run all validation categories.",
    )
    parser.add_argument(
        "--category",
        choices=list(TEST_CATEGORIES),
        help="Run only this category (critical, communication, infrastructure, integration).",
    )
    parser.add_argument(
        "--with-simulation",
        action="store_true",
        help="Include simulation-based integration tests.",
    )
    parser.add_argument(
        "--smoke",
        action="store_true",
        help="Quick smoke test (critical + minimal unit).",
    )
    parser.add_argument(
        "-q",
        "--quiet",
        action="store_true",
        help="Less verbose output.",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=120,
        metavar="SEC",
        help="Per-test timeout in seconds (default 120).",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parent.parent
    if not (root / "tests").is_dir():
        print("Error: tests/ not found. Run from project root.", file=sys.stderr)
        return 1

    targets: List[str] = []
    if args.smoke:
        targets = SMOKE_TARGETS.copy()
        print("Running smoke tests...")
    elif args.category:
        cat = TEST_CATEGORIES[args.category]
        if cat.get("marker"):
            exit_code, out = run_by_marker(
                cat["marker"], verbose=not args.quiet, timeout=args.timeout
            )
            print(out)
            return exit_code
        targets = cat["target"].strip().split()
        print(f"Running category: {cat['name']} - {cat['description']}")
    elif args.all:
        for key, cat in TEST_CATEGORIES.items():
            if cat.get("marker"):
                exit_code, out = run_by_marker(
                    cat["marker"], verbose=not args.quiet, timeout=args.timeout
                )
                if exit_code != 0:
                    print(out)
                    return exit_code
            else:
                targets.extend(cat["target"].strip().split())
        if targets:
            targets = list(dict.fromkeys(targets))
        print("Running all validation categories...")
    else:
        parser.print_help()
        return 0

    if args.with_simulation and not args.smoke:
        for t in SIMULATION_TARGETS:
            if (root / t).exists() and t not in targets:
                targets.append(t)
        print("Including simulation-based tests.")

    if not targets:
        return 0

    exit_code, out = run_pytest(
        targets, verbose=not args.quiet, timeout=args.timeout
    )
    print(out)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
