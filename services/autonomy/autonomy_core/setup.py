"""Setup for autonomy_core package."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "autonomy_core"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="URC 2026 Team",
    maintainer_email="urc2026@team.com",
    description="Unified autonomy core package for URC 2026",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Hardware interface (CAN bridge)
            "hardware_interface = autonomy_core.control.hardware_interface_node:main",
            # Navigation nodes
            "navigation_node = autonomy_core.navigation.navigation_node:main",
            "gnss_processor = autonomy_core.navigation.gnss_processor:main",
            "path_planner = autonomy_core.navigation.path_planner:main",
            "motion_controller = autonomy_core.navigation.motion_controller:main",
            # Safety nodes
            "safety_watchdog = autonomy_core.safety.safety_watchdog:main",
            "emergency_coordinator = autonomy_core.safety.emergency_response_coordinator:main",
            "safety_monitor = autonomy_core.safety.safety_monitor:main",
            "proximity_monitor = autonomy_core.safety.proximity_monitor:main",
            # Perception nodes (consolidated)
            "computer_vision_node = autonomy_core.perception.computer_vision_node:main",
            "slam_node = autonomy_core.perception.slam_node:main",
            "sensor_simulator = autonomy_core.perception.sensor_simulator:main",
            "depth_processor = autonomy_core.perception.depth_processor:main",
            "realsense_driver = autonomy_core.perception.realsense_driver:main",
            "odom_to_slam_pose_bridge = autonomy_core.perception.odom_to_slam_pose_bridge:main",
        ],
    },
)
