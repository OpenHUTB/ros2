from glob import glob
from setuptools import setup

package_name = "mujoco_motor_temperature_predictor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    py_modules=["main", "sample_data"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/hook", ["hook/ament_prefix_path.dsv"]),
    ],
    install_requires=["setuptools", "numpy>=1.21"],
    zip_safe=True,
    maintainer="sirius185",
    description="Standalone time-series motor temperature predictor",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "motor_temperature_forecast = main:main",
        "motor_temperature_sample = sample_data:main",
    ]},
)
