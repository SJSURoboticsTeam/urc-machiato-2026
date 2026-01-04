import os
from glob import glob

from setuptools import setup

package_name = "autonomy_simulation"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Only include directories that exist
        (os.path.join("share", package_name, "config"), glob("config/*.yaml") if glob("config/*.yaml") else []),
        (
            os.path.join("share", package_name, "digitaltwins/config"),
            glob("digitaltwins/config/*.yaml") if glob("digitaltwins/config/*.yaml") else [],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="URC 2026 Team",
    maintainer_email="team@urc2026.edu",
    description="Simulation and Digital Twin subsystem for URC 2026 autonomy",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sensor_simulator = autonomy_simulation.sensor_simulator:main",
            "digital_twin_manager = autonomy_simulation.digital_twin_manager:main",
        ],
    },
)
