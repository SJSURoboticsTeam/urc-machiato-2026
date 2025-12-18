from setuptools import setup

package_name = "hardware_interface"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="URC 2026 Autonomy Team",
    maintainer_email="autonomy@urc2026.edu",
    description="ROS2 Hardware Interface for STM32 Control Systems",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hardware_interface_node = hardware_interface.hardware_interface_node:main",
        ],
    },
)
