#!/usr/bin/env python3
"""Bridge Unitree L2 PointCloud2 to LaserScan on /scan for proximity_monitor.

Requires: ``sudo apt install ros-${ROS_DISTRO}-pointcloud-to-laserscan``
(see package.xml exec_depend on ``pointcloud_to_laserscan``).

Run after the L2 driver is publishing a cloud, e.g.::

    ros2 launch autonomy_core unitree_l2_pc2_to_scan.launch.py

Remap cloud topic if your driver differs::

    ros2 launch autonomy_core unitree_l2_pc2_to_scan.launch.py cloud_topic:=/unilidar/cloud
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("autonomy_core")
    params_file = os.path.join(pkg_share, "config", "unitree_l2_pc2_to_scan.yaml")

    cloud_topic = LaunchConfiguration("cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/unilidar/cloud",
                description="Input sensor_msgs/PointCloud2 topic from L2 driver",
            ),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="Output sensor_msgs/LaserScan for proximity_monitor",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Forward use_sim_time to the node",
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="unitree_l2_pc2_to_scan",
                output="screen",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("cloud_in", cloud_topic),
                    ("scan", scan_topic),
                ],
            ),
        ]
    )
