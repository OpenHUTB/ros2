from glob import glob

from setuptools import find_packages, setup


package_name = "uav_ekf_state_estimation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy", "Pillow"],
    zip_safe=True,
    maintainer="OpenHUTB contributor",
    maintainer_email="xy18574557330@users.noreply.github.com",
    description="Loosely coupled IMU/GPS/barometer EKF for UAV state estimation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ekf_state_estimation_node = uav_ekf_state_estimation.ros_node:main",
        ],
    },
)
