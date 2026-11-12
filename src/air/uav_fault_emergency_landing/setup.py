"""uav_fault_emergency_landing 的安装脚本（ROS2 ament_python 包）。

torch / mujoco 不列进 package.xml：它们由 pip 安装，不属于 ROS 发行版的 apt 包。
"""

import glob
import re
from pathlib import Path

from setuptools import find_packages, setup

package_name = "uav_fault_emergency_landing"

_version_text = Path(package_name, "__init__.py").read_text(encoding="utf-8")
_version = re.search(r'__version__\s*=\s*"([^"]+)"', _version_text).group(1)

setup(
    name=package_name,
    version=_version,
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md", "main.py"]),
        (f"share/{package_name}/launch", glob.glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob.glob("config/*.yaml")),
        # 两个权重合计约 41 KB，必须入库；数据集 300 KB 不入库，用 --gen-data 现场生成
        (f"share/{package_name}/results", glob.glob("results/*.pt")),
    ],
    install_requires=["setuptools", "numpy>=1.21", "torch>=2.0", "mujoco>=3.0.0"],
    zip_safe=True,
    maintainer="Fortranquility",
    maintainer_email="2920111320@qq.com",
    description="四旋翼桨故障后的自主迫降：下视深度相机在线建图 + 神经评分选点 + 闭环制导",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main = uav_fault_emergency_landing.cli:main",
            "landing_node = uav_fault_emergency_landing.ros_node:main",
        ],
    },
)
