from setuptools import setup

package_name = "mujoco_mppi_neural_tracker"

setup(
    name=package_name,
    version="1.0.0",
    packages=["mppi_nn"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/main.launch.py"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
        ("share/" + package_name + "/models", ["models/track.xml", "models/policy.npz"]),
    ],
    install_requires=["setuptools", "numpy", "mujoco", "matplotlib", "pillow"],
    zip_safe=True,
    maintainer="nikedordo-debug",
    maintainer_email="nikedordo@gmail.com",
    description="8 字赛道轨迹跟踪：MPPI 采样 MPC 与蒸馏神经网络策略的定量对比",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["mppi_tracker_node = mppi_nn.node:main"]},
)
