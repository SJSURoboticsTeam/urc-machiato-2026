from setuptools import setup

package_name = "autonomy_utilities"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="URC 2026 Team",
    maintainer_email="urc2026@team.com",
    description="Shared utilities for URC 2026 autonomy system",
    license="MIT",
    python_requires=">=3.8",
)





