"""ROS 2 ament_python 包与命令行入口。"""
from glob import glob
from os.path import join
from pathlib import Path

from setuptools import setup

package_name = "hand_eye_calibration"
# 用本文件所在目录定位 requirements.txt：colcon 的 --symlink-install 分支在 build 目录
# 执行 setup.py（当前工作目录不是源码目录），相对 CWD 读取会 FileNotFoundError。
MODULE_ROOT = Path(__file__).resolve().parent
requirements = [line.strip() for line in (MODULE_ROOT / "requirements.txt").read_text(encoding="utf-8").splitlines()
                if line.strip() and not line.startswith("#")]

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (join("share", package_name), ["package.xml"]),
        (join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (join("share", package_name, "models", "franka_emika_panda"),
         glob("models/franka_emika_panda/*.xml") + glob("models/franka_emika_panda/LICENSE")
         + glob("models/franka_emika_panda/*.md")),
        (join("share", package_name, "models", "franka_emika_panda", "assets"),
         glob("models/franka_emika_panda/assets/*")),
    ],
    install_requires=["setuptools"] + requirements,
    tests_require=["pytest"],
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "hand_eye_calibration = hand_eye_calibration.app:main",
            "hand_eye_calibration_setup = hand_eye_calibration.setup_model:main",
        ],
    },
)