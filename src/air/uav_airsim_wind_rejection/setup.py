from glob import glob

from setuptools import setup

package_name = "uav_airsim_wind_rejection"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/models_trained", glob("models_trained/*.npz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="WZJ6666wudu",
    maintainer_email="WZJ6666wudu@users.noreply.github.com",
    description="AirSim 无人机阵风扰动抑制（神经网络补偿）",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "wind_rejection = uav_airsim_wind_rejection.node:main",
        ],
    },
)
