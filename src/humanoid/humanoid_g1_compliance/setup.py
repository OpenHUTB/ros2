from glob import glob

from setuptools import find_packages, setup

package_name = "humanoid_g1_compliance"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "scripts"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "main.py", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        # LAC 预训练策略（49 MB，TorchScript 之外的整份 checkpoint）
        ("share/" + package_name + "/weights", glob("weights/*.pt")),
        # 100 个上肢姿态的动作库（26 KB）
        ("share/" + package_name + "/motions", glob("motions/*.npz")),
        # G1 23 自由度场景与本体 XML
        ("share/" + package_name + "/assets/g1", glob("assets/g1/*.xml")),
        ("share/" + package_name + "/assets/g1/meshes", glob("assets/g1/meshes/*.STL")),
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
        "Whole-body compliance control for the Unitree G1 humanoid in MuJoCo, "
        "with a ROS 2 interface for stiffness commands and external-force experiments."
    ),
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "sim_node = humanoid_g1_compliance.sim_node:main",
            "stiffness_node = humanoid_g1_compliance.stiffness_node:main",
            "demo = humanoid_g1_compliance.demo:main",
        ],
    },
)
