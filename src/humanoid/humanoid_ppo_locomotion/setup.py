from glob import glob

from setuptools import find_packages, setup

package_name = "humanoid_ppo_locomotion"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test", "scripts"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "main.py", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/weights", glob("weights/*.pt")),
        # G1 场景与本体 XML（由 scripts/fetch_assets.py 下载）
        ("share/" + package_name + "/models/g1", glob("models/g1/*.xml")),
        # 网格资源约 32 MB，不入库，构建时若已下载则一并安装
        (
            "share/" + package_name + "/models/g1/meshes",
            glob("models/g1/meshes/*"),
        ),
    ],
    install_requires=[
        "setuptools>=30.3.0,<80",
        "numpy<2",
    ],
    zip_safe=True,
    maintainer="OpenHUTB contributor",
    maintainer_email="maintainer@example.com",
    description=(
        "PPO residual policy on top of an analytic CoM balance primitive, "
        "for stable standing of the Unitree G1 humanoid in MuJoCo, "
        "with a ROS 2 interface and keyboard upper-body control."
    ),
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "sim_node = humanoid_ppo_locomotion.sim_node:main",
            "policy_node = humanoid_ppo_locomotion.policy_node:main",
            "keyboard_teleop = humanoid_ppo_locomotion.keyboard_teleop:main",
        ],
    },
)
