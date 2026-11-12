from glob import glob
from setuptools import setup

package_name = "motor_thermal_scenario_evaluator"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/hook", ["hook/ament_prefix_path.dsv"]),
    ],
    install_requires=["setuptools", "numpy>=1.24", "matplotlib>=3.6"],
    zip_safe=True,
    maintainer="sirius185",
    description="Standalone analytical motor thermal scenario evaluator",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "motor_thermal_evaluate = motor_thermal_scenario_evaluator.main:main",
        "motor_thermal_plot = motor_thermal_scenario_evaluator.plot:main",
    ]},
)
