# Package configuration for autonomy package
from setuptools import setup, find_packages

setup(
    name="autonomy",
    version="1.0.0",
    packages=find_packages(),
    install_requires=[
        "rclpy",
        "geometry_msgs",
        "sensor_msgs",
        "nav_msgs",
        "tf2_ros",
        "tf2_geometry_msgs",
    ],
    python_requires=">=3.8",
)