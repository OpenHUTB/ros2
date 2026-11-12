"""ROS 2 ament_python 包与命令行入口。"""
from glob import glob
from os.path import join

from setuptools import setup

package_name = "pi0_mujoco_grasp"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (join("share", package_name), ["package.xml"]),
        (join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (join("share", package_name, "scripts"),
         glob("scripts/*.py") + glob("scripts/*.sh")),
        # 模型 XML / 说明 / 校验清单
        (join("share", package_name, "models", "franka_emika_panda"),
         glob("models/franka_emika_panda/*.xml")
         + glob("models/franka_emika_panda/*.md")
         + glob("models/franka_emika_panda/LICENSE")
         + glob("models/franka_emika_panda/assets.sha256")),
        # 网格资源（运行 scripts/setup_model.py 下载后才会存在；先下载再 colcon build
        # 即可随包安装，避免安装态缺 assets）
        (join("share", package_name, "models", "franka_emika_panda", "assets"),
         glob("models/franka_emika_panda/assets/*")),
    ],
    install_requires=["setuptools", "numpy>=1.21,<2", "Pillow>=9", "mujoco==2.3.7"],
    tests_require=["pytest"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "pi0_mujoco_grasp = pi0_mujoco_grasp.ros_node:main",
        ],
    },
)
