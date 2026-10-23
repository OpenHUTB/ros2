from glob import glob

from setuptools import find_packages, setup


package_name = "survey_mission_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.json")),
    ],
    install_requires=["setuptools", "numpy", "Pillow"],
    zip_safe=True,
    maintainer="OpenHUTB contributor",
    maintainer_email="xy18574557330@users.noreply.github.com",
    description="GSD and overlap aware photogrammetry survey planner.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "survey_mission_planner_node = survey_mission_planner.ros_node:main",
        ],
    },
)
