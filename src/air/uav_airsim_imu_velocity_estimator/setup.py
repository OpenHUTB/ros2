from glob import glob

from setuptools import setup

package_name = "uav_airsim_imu_velocity_estimator"

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
    install_requires=["setuptools", "numpy", "Pillow"],
    zip_safe=True,
    maintainer="WZJ6666wudu",
    maintainer_email="WZJ6666wudu@users.noreply.github.com",
    description="AirSim 无人机 IMU 神经网络速度估计",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_velocity_estimator = "
            "uav_airsim_imu_velocity_estimator.node:main",
        ],
    },
)
