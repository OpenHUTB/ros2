from glob import glob

from setuptools import find_packages, setup

package_name = "humanoid_h1_walk"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "scripts"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "main.py", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        # PPO 训练好的 TorchScript 策略（H1 137 KB、H1_2 143 KB）
        ("share/" + package_name + "/weights", glob("weights/*.pt")),
        # H1 场景与本体 XML
        ("share/" + package_name + "/assets/h1", glob("assets/h1/*.xml")),
        ("share/" + package_name + "/assets/h1/meshes", glob("assets/h1/meshes/*.STL")),
        # H1_2 场景与本体 XML（scene.xml 只引用 12 自由度版本）
        ("share/" + package_name + "/assets/h1_2", glob("assets/h1_2/*.xml")),
        ("share/" + package_name + "/assets/h1_2/meshes", glob("assets/h1_2/meshes/*.STL")),
    ],
    install_requires=[
        "setuptools>=30.3.0,<80",
        "numpy<3",
        "mujoco>=3.1",
        "pyyaml",
        # 策略推理依赖 PyTorch。这里故意不写死，避免 pip 自动拉取 200 MB+
        # 的 CUDA 版；请按 README 安装 CPU 版：
        #   pip install torch --index-url https://download.pytorch.org/whl/cpu
        # "torch>=2.0",
    ],
    zip_safe=True,
    maintainer="OpenHUTB contributor",
    maintainer_email="maintainer@example.com",
    description=(
        "Neural-network walking control for the Unitree H1 and H1_2 humanoids in MuJoCo, "
        "with a ROS 2 interface for velocity commands and keyboard teleoperation."
    ),
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "sim_node = humanoid_h1_walk.sim_node:main",
            "policy_node = humanoid_h1_walk.policy_node:main",
            "keyboard_teleop = humanoid_h1_walk.keyboard_teleop:main",
        ],
    },
)
