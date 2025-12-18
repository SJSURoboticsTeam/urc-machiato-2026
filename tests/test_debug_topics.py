#!/usr/bin/env python3
"""
Debug test to check what ROS2 topics and services are available during testing.
"""

import os
import subprocess
import time


def test_available_topics():
    """Test what topics are available."""
    print("[MAGNIFY] Checking available ROS2 topics...")

    try:
        # Source ROS2 and check topics
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = "42"

        result = subprocess.run(
            ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 topic list"],
            env=env,
            capture_output=True,
            text=True,
            timeout=10,
        )

        topics = result.stdout.strip().split("\n")
        print(f"[ANTENNA] Found {len(topics)} topics:")
        for topic in topics:
            if topic.strip():
                print(f"   {topic}")

        # Check for expected topics
        expected_topics = [
            "/state_machine/current_state",
            "/state_machine/change_state",
            "/state_machine/execute_mission",
            "/slam/depth/processed",
            "/slam/odom/fused",
            "/plan",
            "/compute_path_to_pose",
        ]

        found = [topic for topic in expected_topics if topic in topics]
        missing = [topic for topic in expected_topics if topic not in topics]

        print("\n[PASS] Found expected topics:")
        for topic in found:
            print(f"   {topic}")

        if missing:
            print("\n[FAIL] Missing expected topics:")
            for topic in missing:
                print(f"   {topic}")

        # Check services
        print("\n[TOOL] Checking available ROS2 services...")
        service_result = subprocess.run(
            ["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 service list"],
            env=env,
            capture_output=True,
            text=True,
            timeout=10,
        )

        services = service_result.stdout.strip().split("\n")
        print(f"[TOOL] Found {len(services)} services:")
        for service in services:
            if service.strip():
                print(f"   {service}")

    except Exception as e:
        print(f"[FAIL] Error checking topics/services: {e}")


if __name__ == "__main__":
    test_available_topics()
