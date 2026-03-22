# Package configuration for autonomy package
from setuptools import setup, find_packages
import os
from glob import glob

setup(
    name="autonomy",
    version="1.0.0",
    packages=find_packages(include=["isaac_sim_bridge"]),
    package_dir={"": "."},
    data_files=[
        (os.path.join('share', 'autonomy'), ['package.xml']),
        (os.path.join('share', 'autonomy', 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        "rclpy",
        "geometry_msgs",
        "sensor_msgs",
        "nav_msgs",
        "tf2_ros",
        "tf2_geometry_msgs",
    ],
    python_requires=">=3.8",
    entry_points={
        "console_scripts": [
            "bridge_node = isaac_sim_bridge.bridge_node:main",
            "sim_safety_adapter = isaac_sim_bridge.sim_safety_adapter:main",
        ],
    },
)
