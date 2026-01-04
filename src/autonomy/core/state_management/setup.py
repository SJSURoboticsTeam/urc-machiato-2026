#!/usr/bin/env python3
"""
Setup script for Adaptive State Machine ROS2 package.

This setup script configures the adaptive state machine components
for use as a ROS2 package with proper entry points and dependencies.
"""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "autonomy_state_machine"

# Get the directory where this setup.py is located
setup_dir = os.path.dirname(os.path.abspath(__file__))

# Find all Python packages
packages = find_packages(where=setup_dir, exclude=["tests"])

# Entry points for ROS2 executables
console_scripts = [
    "adaptive_state_machine = autonomy_state_machine.adaptive_state_machine:main",
    "monitoring_service = autonomy_state_machine.monitoring_service:main",
    "qos_profiler = qos_profiler:main",
    "safety_monitor = safety_monitor:main",
    "urc_band_manager = urc_band_manager:main",
]

setup(
    name=package_name,
    version="1.0.0",
    packages=packages,
    data_files=[
        # Launch files
        ("share/autonomy_state_machine/launch", glob("launch/*.launch.py") if glob("launch/*.launch.py") else []),
        # Configuration files
        ("share/autonomy_state_machine/config", glob("config/*.yaml") if glob("config/*.yaml") else []),
    ],
    install_requires=[
        "setuptools",
        "psutil",  # For system monitoring
        "numpy",  # For numerical computations
    ],
    extras_require={
        "test": [
            "pytest",
            "pytest-ros",
            "pytest-mock",
        ],
        "dev": [
            "black",
            "flake8",
            "mypy",
        ],
        "dashboard": [
            "websockets",
            "flask",
        ],
    },
    entry_points={
        "console_scripts": console_scripts,
    },
    author="URC 2026 Team",
    author_email="urc2026@team.com",
    description="Adaptive State Machine for URC 2026 Mars Rover",
    long_description=open(os.path.join(setup_dir, "README.md")).read() if os.path.exists(os.path.join(setup_dir, "README.md")) else "Adaptive State Machine for URC 2026 Mars Rover",
    long_description_content_type="text/markdown",
    license="MIT",
    classifiers=[
        "Environment :: Robotics",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Topic :: Scientific/Engineering :: Robotics",
    ],
    python_requires=">=3.8",
    keywords="ros2 robotics state-machine adaptive autonomy urc mars-rover",
)
