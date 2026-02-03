#!/usr/bin/env python3
"""
ROS2 Simulator Test: Direct CAN-to-Blackboard path.

Starts mock blackboard service and hardware_interface_node (mock CAN),
waits for telemetry to write to blackboard, then reads keys via
/blackboard/get_value and asserts expected values. Exits 0 on success.

Usage (from repo root, with ROS2 sourced):
  source /opt/ros/jazzy/setup.bash   # or humble
  source install/setup.bash          # if you built autonomy_interfaces
  export PYTHONPATH=$PWD/src:$PYTHONPATH
  python3 scripts/hardware/test_ros2_simulator_blackboard.py

If autonomy_interfaces is not built, the script will skip the service call
and report that the build is required.
"""

import os
import subprocess
import sys
import time
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[2]
SRC = WORKSPACE / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


def main():
    mock_proc = None
    hw_proc = None
    try:
        import rclpy
        from autonomy_interfaces.srv import GetBlackboardValue
    except ImportError as e:
        print("SKIP: ROS2 or autonomy_interfaces not available:", e)
        print(
            "  Build: colcon build --packages-select autonomy_interfaces autonomy_core"
        )
        print("  Source: source install/setup.bash")
        return 0 if "autonomy_interfaces" in str(e) else 2

    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(
        [str(SRC)] + env.get("PYTHONPATH", "").split(os.pathsep)
    )

    try:
        print("Starting mock blackboard service...")
        mock_proc = subprocess.Popen(
            [
                sys.executable,
                str(WORKSPACE / "scripts/hardware/mock_blackboard_service.py"),
            ],
            cwd=str(WORKSPACE),
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        time.sleep(1.5)

        print("Starting hardware_interface_node (mock CAN)...")
        hw_proc = subprocess.Popen(
            [
                sys.executable,
                str(WORKSPACE / "scripts/hardware/run_hardware_interface.py"),
            ],
            cwd=str(WORKSPACE),
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
        )
        time.sleep(3.0)

        rclpy.init()
        node = rclpy.create_node("test_ros2_sim_client")
        client = node.create_client(GetBlackboardValue, "/blackboard/get_value")
        if not client.wait_for_service(timeout_sec=5.0):
            print("FAIL: /blackboard/get_value not available")
            return 1

        req = GetBlackboardValue.Request()
        req.key = "battery_level"
        req.value_type = "double"
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done() or not future.result().success:
            print(
                "FAIL: battery_level not in blackboard:",
                getattr(future.result(), "error_message", ""),
            )
            node.destroy_node()
            rclpy.shutdown()
            return 1
        battery = float(future.result().value)
        node.destroy_node()
        rclpy.shutdown()

        if not (0.0 <= battery <= 100.0):
            print("FAIL: battery_level expected 0-100, got", battery)
            return 1
        print("OK: battery_level =", battery, "(direct path write)")

        req.key = "robot_velocity_x"
        rclpy.init()
        node2 = rclpy.create_node("test_ros2_sim_client2")
        client2 = node2.create_client(GetBlackboardValue, "/blackboard/get_value")
        client2.wait_for_service(timeout_sec=2.0)
        future2 = client2.call_async(req)
        rclpy.spin_until_future_complete(node2, future2, timeout_sec=3.0)
        if future2.result().success:
            print("OK: robot_velocity_x =", future2.result().value)
        else:
            print("WARN: robot_velocity_x not set (optional)")
        node2.destroy_node()
        rclpy.shutdown()

        print("PASS: ROS2 simulator test - direct path writes verified")
        return 0
    except Exception as e:
        print("FAIL:", e)
        return 1
    finally:
        for p, name in [
            (mock_proc, "mock_blackboard"),
            (hw_proc, "hardware_interface"),
        ]:
            if p is not None and p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    p.kill()


if __name__ == "__main__":
    sys.exit(main())
