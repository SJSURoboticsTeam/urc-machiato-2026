#!/usr/bin/env python3
"""
ROS2 Integration Tests for URC 2026 Mars Rover
"""

import subprocess
import time
import sys
import os
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent
SRC_ROOT = PROJECT_ROOT / "src"
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(SRC_ROOT))


def test_ros2_nodes():
    """Test ROS2 node discovery and basic functionality."""
    print("🔍 Testing ROS2 Node Discovery...")

    try:
        # Test ros2 node list
        result = subprocess.run(
            ["ros2", "node", "list"], capture_output=True, text=True, timeout=10
        )

        if result.returncode == 0:
            nodes = result.stdout.strip().split("\n") if result.stdout.strip() else []
            print(f"   Found {len(nodes)} ROS2 nodes running")
            return {"status": "passed", "nodes_found": len(nodes)}
        else:
            return {"status": "failed", "error": result.stderr}

    except Exception as e:
        return {"status": "failed", "error": str(e)}


def test_ros2_topics():
    """Test ROS2 topic discovery."""
    print("📡 Testing ROS2 Topic Discovery...")

    try:
        # Test ros2 topic list
        result = subprocess.run(
            ["ros2", "topic", "list"], capture_output=True, text=True, timeout=10
        )

        if result.returncode == 0:
            topics = result.stdout.strip().split("\n") if result.stdout.strip() else []
            print(f"   Found {len(topics)} ROS2 topics")
            return {"status": "passed", "topics_found": len(topics)}
        else:
            return {"status": "failed", "error": result.stderr}

    except Exception as e:
        return {"status": "failed", "error": str(e)}


def test_gazebo_simulation():
    """Test Gazebo simulation launch."""
    print("🎮 Testing Gazebo Simulation...")

    # Check if gazebo worlds exist
    worlds_dir = PROJECT_ROOT / "simulation" / "gazebo_simulation" / "worlds"
    if worlds_dir.exists():
        worlds = list(worlds_dir.glob("*.world"))
        if worlds:
            print(f"   Found {len(worlds)} Gazebo world files")
            return {"status": "passed", "worlds": len(worlds)}

    return {"status": "failed", "error": "No Gazebo worlds found"}


def test_mission_files():
    """Test mission file structure."""
    print("🎯 Testing Mission Files...")

    missions_dir = PROJECT_ROOT / "missions"
    if missions_dir.exists():
        mission_files = list(missions_dir.glob("*.py"))
        mission_count = len([f for f in mission_files if not f.name.startswith("__")])

        print(f"   Found {mission_count} mission files")
        return {"status": "passed", "missions": mission_count}

    return {"status": "failed", "error": "Missions directory not found"}


def main():
    print("🚀 URC 2026 Mars Rover - ROS2 Integration Test Report")
    print("=" * 60)

    start_time = time.time()

    # Run ROS2 tests
    test_results = {
        "ros2_nodes": test_ros2_nodes(),
        "ros2_topics": test_ros2_topics(),
        "gazebo_simulation": test_gazebo_simulation(),
        "mission_files": test_mission_files(),
    }

    execution_time = time.time() - start_time

    # Calculate statistics
    total_tests = len(test_results)
    passed_tests = sum(
        1 for result in test_results.values() if result.get("status") == "passed"
    )
    failed_tests = total_tests - passed_tests

    # Generate report
    print(
        f"""
📊 **ROS2 INTEGRATION TEST SUMMARY**
   • Total Tests: {total_tests}
   • Passed: {passed_tests}
   • Failed: {failed_tests}
   • Pass Rate: {(passed_tests/total_tests*100):.1f}% 
   • Execution Time: {execution_time:.2f}s

🔍 **ROS2 ENVIRONMENT STATUS**
   • Nodes: {test_results['ros2_nodes'].get('nodes_found', 0)} running
   • Topics: {test_results['ros2_topics'].get('topics_found', 0)} available

🎮 **SIMULATION ENVIRONMENT**
   • Gazebo Worlds: {test_results['gazebo_simulation'].get('worlds', 0)}
   • Status: {'✅ Available' if test_results['gazebo_simulation'].get('status') == 'passed' else '❌ Missing'}

🎯 **MISSION SYSTEM**
   • Mission Files: {test_results['mission_files'].get('missions', 0)}
   • Status: {'✅ Complete' if test_results['mission_files'].get('status') == 'passed' else '❌ Incomplete'}

🚀 **ROS2 INTEGRATION READINESS**
   {'✅ ROS2 ENVIRONMENT READY' if passed_tests >= total_tests * 0.75 else '⚠️ ROS2 ENVIRONMENT NEEDS SETUP'}
"""
    )


if __name__ == "__main__":
    main()
