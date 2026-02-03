#!/usr/bin/env python3
"""
Post-Migration Validation Script

Validates that the URC 2026 migration completed successfully and
the system is ready for deployment.

Author: URC 2026 Migration Team
"""

import os
import sys
from pathlib import Path


def validate_package_structure():
    """Validate package consolidation."""
    print("🔍 Validating Package Structure...")

    autonomy_packages = list(Path("src/autonomy").glob("*/package.xml"))
    print(f"   ROS2 packages found: {len(autonomy_packages)}")

    for pkg in autonomy_packages:
        print(f"   - {pkg.parent.name}")

    # Should have exactly 3 packages now
    expected_packages = {"autonomy_interfaces", "autonomy_bt", "autonomy_core"}
    actual_packages = {pkg.parent.name for pkg in autonomy_packages}

    success = len(autonomy_packages) == 3 and expected_packages.issubset(
        actual_packages
    )
    print(f"   ✅ Package consolidation: {'PASS' if success else 'FAIL'}")
    return success


def validate_infrastructure():
    """Validate infrastructure consolidation."""
    print("\n🏗️  Validating Infrastructure...")

    # Check unified infrastructure exists
    infrastructure_dir = Path("src/infrastructure")
    bridges_dir = infrastructure_dir / "bridges"
    monitoring_dir = infrastructure_dir / "monitoring"
    config_dir = infrastructure_dir / "config"

    success = (
        infrastructure_dir.exists()
        and bridges_dir.exists()
        and monitoring_dir.exists()
        and config_dir.exists()
    )

    if success:
        print("   ✅ Infrastructure structure: PASS")
        print(f"   - Bridges: {len(list(bridges_dir.glob('*.py')))} files")
        print(f"   - Monitoring: {len(list(monitoring_dir.glob('*.py')))} files")
        print(f"   - Config: {len(list(config_dir.glob('*')))} files")
    else:
        print("   ❌ Infrastructure structure: FAIL")

    return success


def validate_configuration():
    """Validate unified configuration system."""
    print("\n⚙️  Validating Configuration System...")

    try:
        sys.path.insert(0, "src")
        from infrastructure.config import get_urc_config

        config = get_urc_config()
        print(f"   ✅ Configuration system: PASS")
        print(f"   - Config type: {type(config).__name__}")
        return True
    except Exception as e:
        print(f"   ❌ Configuration system: FAIL - {e}")
        return False


def validate_bridge_system():
    """Validate bridge system consolidation."""
    print("\n🌉 Validating Bridge System...")

    try:
        sys.path.insert(0, "src")
        from infrastructure.bridges.simple_bridge import get_simple_bridge

        bridge = get_simple_bridge()
        print(f"   ✅ Bridge system: PASS")
        print(f"   - Bridge type: {type(bridge).__name__}")
        return True
    except Exception as e:
        print(f"   ❌ Bridge system: FAIL - {e}")
        return False


def validate_launch_system():
    """Validate launch system consolidation."""
    print("\n🚀 Validating Launch System...")

    unified_launch = Path("src/autonomy/autonomy_core/launch/unified.launch.py")
    simulation_launch = Path("src/autonomy/autonomy_core/launch/simulation.launch.py")

    success = unified_launch.exists() and simulation_launch.exists()

    if success:
        print("   ✅ Launch system: PASS")
        print(f"   - Unified launch: {unified_launch}")
        print(f"   - Simulation launch: {simulation_launch}")
    else:
        print("   ❌ Launch system: FAIL")

    return success


def validate_build_system():
    """Validate unified build system."""
    print("\n🔨 Validating Build System...")

    build_script = Path("scripts/build.sh")
    success = build_script.exists() and build_script.is_file()

    if success:
        print("   ✅ Build system: PASS")
        print(f"   - Build script: {build_script}")
    else:
        print("   ❌ Build system: FAIL")

    return success


def validate_test_runner():
    """Validate test runner fixes."""
    print("\n🧪 Validating Test Runner...")

    try:
        import subprocess

        result = subprocess.run(
            [sys.executable, "scripts/run_tests.py", "--help"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        success = result.returncode == 0 and "Smart Test Runner" in result.stdout
        print(f"   ✅ Test runner: {'PASS' if success else 'FAIL'}")
        return success
    except Exception as e:
        print(f"   ❌ Test runner: FAIL - {e}")
        return False


def validate_old_directories_removed():
    """Validate that old directories were removed."""
    print("\n🧹 Validating Cleanup...")

    # Should NOT exist anymore
    old_dirs = [
        Path("src/bridges"),
        Path("src/comms"),
        Path("src/autonomy/control"),
        Path("src/autonomy/core"),
        Path("src/autonomy/perception"),
        Path("src/autonomy/utilities"),
    ]

    removed = [d for d in old_dirs if not d.exists()]
    remaining = [d for d in old_dirs if d.exists()]

    print(f"   - Old directories removed: {len(removed)}/{len(old_dirs)}")

    if remaining:
        print(f"   ⚠️  Still exists: {[d.name for d in remaining]}")

    success = len(remaining) == 0
    print(f"   ✅ Cleanup: {'PASS' if success else 'PARTIAL'}")
    return success


def main():
    """Run all validation checks."""
    print("=" * 60)
    print("🎯 URC 2026 Post-Migration Validation")
    print("=" * 60)

    validations = [
        ("Package Structure", validate_package_structure),
        ("Infrastructure", validate_infrastructure),
        ("Configuration", validate_configuration),
        ("Bridge System", validate_bridge_system),
        ("Launch System", validate_launch_system),
        ("Build System", validate_build_system),
        ("Test Runner", validate_test_runner),
        ("Cleanup", validate_old_directories_removed),
    ]

    results = []
    for name, validator in validations:
        try:
            result = validator()
            results.append((name, result))
        except Exception as e:
            print(f"❌ {name} validation error: {e}")
            results.append((name, False))

    print("\n" + "=" * 60)
    print("📊 VALIDATION SUMMARY")
    print("=" * 60)

    passed = sum(1 for _, result in results if result)
    total = len(results)

    for name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{name:20} {status}")

    print(f"\nOverall: {passed}/{total} validations passed")

    if passed == total:
        print("🎉 MIGRATION COMPLETE - System ready for deployment!")
        return 0
    else:
        print("⚠️  MIGRATION ISSUES DETECTED - Review failures above")
        return 1


if __name__ == "__main__":
    sys.exit(main())
