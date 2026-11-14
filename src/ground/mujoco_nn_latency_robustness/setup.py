from glob import glob

from setuptools import setup

package_name = "mujoco_nn_latency_robustness"

setup(
    name=package_name,
    version="0.1.0",
    packages=["lat_nav"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/models", glob("models/*.npz")),
        ("share/" + package_name, ["README.md"]),
    ],
    install_requires=["setuptools", "numpy", "matplotlib", "mujoco"],
    zip_safe=True,
    maintainer="nikedordo-debug",
    maintainer_email="nikedordo@gmail.com",
    description="神经网络跟踪策略的执行延迟与控制频率鲁棒性实验",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "latency_nav_node = lat_nav.node:main",
        ],
    },
)
