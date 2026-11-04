import os
from glob import glob

from setuptools import setup

package_name = "mnist_digit_control"


def data_files():
    files = [
        ("share/ament_index/resource_index/packages",
         [os.path.join("resource", package_name)]),
        ("share/" + package_name, ["package.xml"]),
    ]
    # launch 文件
    files.append((os.path.join("share", package_name, "launch"),
                  glob("launch/*.launch.py")))
    # 模型权重与场景视频（小体积二进制，供开箱即用）
    files.append((os.path.join("share", package_name, "models"),
                  glob("models/*.pth")))
    for ext in ("mp4", "avi"):
        hits = glob(os.path.join("videos", f"scene_*.{ext}"))
        if hits:
            files.append((os.path.join("share", package_name, "videos"), hits))
    return files


setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Thirin-glitch",
    maintainer_email="247221616@qq.com",
    description="MNIST 手写数字 CNN 感知与 turtlesim 运动控制（任务2，无需仿真器）",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector = mnist_digit_control.detector_node:main",
            "commander = mnist_digit_control.command_node:main",
        ],
    },
)
