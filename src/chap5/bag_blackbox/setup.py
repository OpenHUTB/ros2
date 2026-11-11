import os
from glob import glob

from setuptools import setup

package_name = "bag_blackbox"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="刘琪",
    maintainer_email="2754586324@qq.com",
    description="机器人的“黑匣子”——bag 录包数据的离线挖掘与可视化",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bag_blackbox_main = bag_blackbox.main:main",
            "image_record_main = bag_blackbox.image_record_node:main",
        ],
    },
)
