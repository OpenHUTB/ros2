from glob import glob
from setuptools import setup

package_name = "uav_airsim_attitude_estimator"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/models_trained", glob("models_trained/*.npz")),
    ],
    install_requires=["setuptools", "numpy", "pillow", "airsim"],
    zip_safe=True,
    maintainer="WZJ6666wudu",
    maintainer_email="WZJ6666wudu@users.noreply.github.com",
    description="基于 IMU 时序的无人机姿态角神经网络估计（AirSim 仿真）",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "attitude_estimator = uav_airsim_attitude_estimator.node:main",
        ],
    },
)