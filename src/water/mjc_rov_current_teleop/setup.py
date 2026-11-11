from setuptools import setup
import os
from glob import glob

package_name = "mjc_rov_current_teleop"

# 关键：console_scripts 只注册本包真实存在的入口函数（main），
# 否则 ros2 run 会报 "has no attribute 'main'"。
setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 注册 launch 目录
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
        # 注册 models / config，方便 install 后按包名找到
        (os.path.join("share", package_name, "models"),
         glob("models/*.xml")),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Dynamic-current ROV MuJoCo sim + keyboard teleop + MLP cruise.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_node = mjc_rov_current_teleop.sim_node:main",
            "keyboard_node = mjc_rov_current_teleop.keyboard_node:main",
            "cruise_node = mjc_rov_current_teleop.cruise_node:main",
        ],
    },
)
