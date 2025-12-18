#!/usr/bin/env python3
"""
Test Environment Setup Validation

Validates that the test environment setup works correctly
and provides helpful diagnostics for troubleshooting.
"""

import os
import subprocess
import sys
from pathlib import Path


def check_python_environment():
    """Check Python environment setup."""
    print("🐍 Python Environment Check")
    print("-" * 30)

    # Python version
    version = sys.version_info
    print(f"Python Version: {version.major}.{version.minor}.{version.micro}")
    if version.major < 3 or (version.major == 3 and version.minor < 8):
        print("[ERROR] Python 3.8+ required")
        return False
    else:
        print("[SUCCESS] Python version OK")

    # Required packages
    required_packages = ["pytest", "psutil"]
    missing_packages = []

    for package in required_packages:
        try:
            __import__(package)
            print(f"[SUCCESS] {package} available")
        except ImportError:
            print(f"[ERROR] {package} missing")
            missing_packages.append(package)

    if missing_packages:
        print(
            f"\nTo install missing packages: pip install {' '.join(missing_packages)}"
        )
        return False

    return True


def check_ros2_environment():
    """Check ROS2 environment setup."""
    print("\n[ROBOT] ROS2 Environment Check")
    print("-" * 30)

    # Check environment variables
    ros_distro = os.environ.get("ROS_DISTRO")
    if ros_distro:
        print(f"ROS_DISTRO: {ros_distro}")
    else:
        print("[ERROR] ROS_DISTRO not set")
        return False

    # Check setup script
    setup_script = f"/opt/ros/{ros_distro}/setup.bash"
    if os.path.exists(setup_script):
        print(f"[SUCCESS] ROS2 setup script found: {setup_script}")
    else:
        print(f"[ERROR] ROS2 setup script not found: {setup_script}")
        return False

    # Try to source and test
    try:
        result = subprocess.run(
            [
                "bash",
                "-c",
                f"source {setup_script} && python3 -c \"import rclpy; print('OK')\"",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )

        if result.returncode == 0 and "OK" in result.stdout:
            print("[SUCCESS] ROS2 rclpy import successful")
            return True
        else:
            print("[ERROR] ROS2 rclpy import failed")
            if result.stderr:
                print(f"Error: {result.stderr}")
            return False

    except subprocess.TimeoutExpired:
        print("[ERROR] ROS2 test timed out")
        return False
    except Exception as e:
        print(f"[ERROR] ROS2 test failed: {e}")
        return False


def check_project_structure():
    """Check project structure and paths."""
    print("\n📁 Project Structure Check")
    print("-" * 30)

    test_dir = Path(__file__).parent
    project_root = test_dir.parent.parent.parent.parent

    # Check key directories
    required_paths = [
        test_dir / "run_adaptive_tests.py",
        test_dir / "test_adaptive_state_machine.py",
        test_dir / "mock_ros2.py",
        Path(
            "/home/ubuntu/urc-machiato-2026/src/autonomy/core/state_management/autonomy_state_machine"
        ),
    ]

    all_exist = True
    for path in required_paths:
        if path.exists():
            print(
                f"[SUCCESS] {path.relative_to(project_root) if path.is_relative_to(project_root) else path.name}"
            )
        else:
            print(f"[ERROR] Missing: {path}")
            all_exist = False

    return all_exist


def check_test_runner():
    """Check that test runner can be executed."""
    print("\n🏃 Test Runner Check")
    print("-" * 30)

    test_dir = Path(__file__).parent
    runner_script = test_dir / "run_adaptive_tests.py"

    try:
        # Test syntax check
        result = subprocess.run(
            [sys.executable, "-m", "py_compile", str(runner_script)],
            capture_output=True,
            text=True,
        )

        if result.returncode == 0:
            print("[SUCCESS] Test runner syntax OK")
        else:
            print("[ERROR] Test runner syntax error")
            if result.stderr:
                print(f"Error: {result.stderr}")
            return False

        # Test help output
        result = subprocess.run(
            [sys.executable, str(runner_script), "--help"],
            capture_output=True,
            text=True,
            cwd=test_dir,
        )

        if (
            result.returncode == 0
            and "Adaptive State Machine Test Runner" in result.stdout
        ):
            print("[SUCCESS] Test runner execution OK")
            return True
        else:
            print("[ERROR] Test runner execution failed")
            if result.stderr:
                print(f"Error: {result.stderr}")
            return False

    except Exception as e:
        print(f"[ERROR] Test runner check failed: {e}")
        return False


def check_mock_environment():
    """Check that mock environment works."""
    print("\n[DRAMA] Mock Environment Check")
    print("-" * 30)

    try:
        # Import and test mock
        sys.path.insert(0, str(Path(__file__).parent))
        from mock_ros2 import MockNode, setup_mock_environment

        if setup_mock_environment():
            print("[SUCCESS] Mock environment setup successful")

            # Test mock node creation
            node = MockNode("test_node")
            publisher = node.create_publisher(str, "/test_topic")

            # Test publishing
            publisher.publish("test message")

            if publisher.get_message_count() == 1:
                print("[SUCCESS] Mock ROS2 interfaces working")
                return True
            else:
                print("[ERROR] Mock ROS2 interface test failed")
                return False
        else:
            print("ℹ️  ROS2 available, mock not needed")
            return True

    except Exception as e:
        print(f"[ERROR] Mock environment test failed: {e}")
        return False


def provide_recommendations(python_ok, ros2_ok, structure_ok, runner_ok, mock_ok):
    """Provide setup recommendations based on check results."""
    print("\n[LIGHT] Setup Recommendations")
    print("-" * 30)

    issues = []

    if not python_ok:
        issues.append(
            "Install required Python packages: pip install pytest pytest-cov psutil numpy"
        )

    if not ros2_ok:
        issues.append(
            "Install ROS2 (Humble recommended) or use mock environment for testing"
        )

    if not structure_ok:
        issues.append("Ensure project structure is correct - check file paths")

    if not runner_ok:
        issues.append("Fix test runner script syntax errors")

    if not mock_ok:
        issues.append("Mock environment has issues - check mock_ros2.py")

    if issues:
        print("[ERROR] Issues found:")
        for i, issue in enumerate(issues, 1):
            print(f"  {i}. {issue}")
    else:
        print("[SUCCESS] All checks passed! Ready to run tests.")

    print("\n[START] Quick Start Commands:")
    print("  cd src/autonomy/core/state_management/tests")
    print("  python3 run_adaptive_tests.py unit    # Run basic unit tests")
    print("  python3 run_adaptive_tests.py all     # Run full test suite")


def main():
    """Run all environment checks."""
    print("🔬 Adaptive State Machine Environment Validation")
    print("=" * 55)

    # Run all checks
    python_ok = check_python_environment()
    ros2_ok = check_ros2_environment()
    structure_ok = check_project_structure()
    runner_ok = check_test_runner()
    mock_ok = check_mock_environment()

    # Summary
    print("\n[STATUS] Validation Summary")
    print("-" * 30)
    checks = [
        ("Python Environment", python_ok),
        ("ROS2 Environment", ros2_ok),
        ("Project Structure", structure_ok),
        ("Test Runner", runner_ok),
        ("Mock Environment", mock_ok),
    ]

    all_passed = True
    for check_name, passed in checks:
        status = "[SUCCESS] PASS" if passed else "[ERROR] FAIL"
        print("15")
        all_passed = all_passed and passed

    # Overall result
    if all_passed:
        print("\n[CELEBRATE] All validation checks passed!")
        print("   Ready to run the adaptive state machine tests.")
    else:
        print("\n[WARN]  Some validation checks failed.")
        print("   See recommendations below to fix issues.")

    # Recommendations
    provide_recommendations(python_ok, ros2_ok, structure_ok, runner_ok, mock_ok)

    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
