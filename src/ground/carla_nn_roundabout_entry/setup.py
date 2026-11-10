from setuptools import find_packages, setup

package_name = "carla_nn_roundabout_entry"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="asdfghjkl384",
    maintainer_email="asdfghjkl384@example.com",
    description="相机端到端驾驶：图像到车辆控制（环岛汇入，轻量离线版）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "roundabout_e2e = carla_nn_roundabout_entry.main:main",
        ],
    },
)
