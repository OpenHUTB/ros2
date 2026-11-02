from glob import glob
from setuptools import find_packages, setup

package_name = "mujoco_uav_rooftop_pv_coverage_survey"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    py_modules=["ros2_uav_controller"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["rooftop_pv_controller = ros2_uav_controller:main"]},
)
