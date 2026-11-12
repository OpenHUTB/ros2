from glob import glob
from setuptools import setup

package_name = "mujoco_motor_thermal_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sirius185",
    maintainer_email="fzy985211_2025@qq.com",
    description="MuJoCo 差速小车驱动电机热模型",
    license="Apache-2.0",
    entry_points={"console_scripts": ["motor_thermal_main = mujoco_motor_thermal_control.main:main"]},
)
