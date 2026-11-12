# Copyright 2026 xxbzy727749

"""Package the UAV power-line inspection preview tool."""

from setuptools import find_packages, setup


package_name = "uav_powerline_inspection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="xxbzy727749",
    maintainer_email="xxbzy727749@users.noreply.github.com",
    description="Preview alternating UAV viewpoints for power-line inspection.",
    license="MIT",
    entry_points={"console_scripts": ["preview = uav_powerline_inspection.cli:main"]},
)
