from glob import glob
from setuptools import setup

package_name = "mujoco_motor_thermal_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    py_modules=["main"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/hook", ["hook/ament_prefix_path.dsv"]),
    ],
    install_requires=["setuptools", "numpy>=1.24", "mujoco>=3.2,<4"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="sirius185",
    description="MuJoCo 差速小车驱动电机热模型",
    license="Apache-2.0",
    entry_points={"console_scripts": ["motor_thermal_main = main:main"]},
)
