# -*- coding: utf-8 -*-
import os, re
from glob import glob
from pathlib import Path
from setuptools import setup

package_name = "humanoid_stair_zmp_preview"
_cfg = Path(__file__).resolve().parent.joinpath(package_name, "config.py").read_text(encoding="utf-8")
_m = re.search(r'^VERSION\s*=\s*"([^"]+)"', _cfg, re.MULTILINE)
if _m is None:
    raise RuntimeError("无法从 config.py 解析出 VERSION")

setup(
    name=package_name, version=_m.group(1), packages=[package_name],
    data_files=[
        (os.path.join("share", "ament_index", "resource_index", "packages"),
         [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml", "main.py", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="jerrytsts", maintainer_email="jerrytsts@users.noreply.github.com",
    description="人形上下台阶：ZMP 预瞄控制 + 解析 IK", license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["stair_zmp_node = humanoid_stair_zmp_preview.ros_node:main"]},
)