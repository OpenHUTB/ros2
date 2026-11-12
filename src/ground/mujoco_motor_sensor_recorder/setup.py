from glob import glob
from setuptools import setup

package_name = "mujoco_motor_sensor_recorder"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.csv")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sirius185",
    maintainer_email="fzy985211_2025@qq.com",
    description="Standalone reproducible motor sensor CSV recorder",
    license="Apache-2.0",
    entry_points={"console_scripts": ["motor_sensor_record = mujoco_motor_sensor_recorder.main:main"]},
)
