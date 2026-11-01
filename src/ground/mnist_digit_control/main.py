#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mnist_digit_control 模块一键入口。

用法（在模块目录 src/mnist_digit_control 下执行）：
    python3 main.py            # 自动构建（如需）并 ros2 launch 启动整个模块
    python3 main.py build      # 仅构建（colcon build）
    python3 main.py run ...    # 启动，额外参数透传给 launch，如：
                               #   python3 main.py run source:=webcam display:=true
    python3 main.py train      # 训练 MNIST CNN（首次会自动下载数据）
    python3 main.py scenes     # 生成多场景虚拟相机视频
    python3 main.py eval       # 离线评价多场景识别性能
"""
import os
import subprocess
import sys

MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(MODULE_DIR)))
# 模块位于 src/ground/ 下，仓库根为其上三级
ROS_SETUP = "/opt/ros/humble/setup.bash"
INSTALL_SETUP = os.path.join(REPO_ROOT, "install", "setup.bash")


def sh(cmd: str, cwd: str = REPO_ROOT) -> int:
    print(f"\n$ {cmd}\n(cwd={cwd})")
    return subprocess.call(["bash", "-c", cmd], cwd=cwd)


def ensure_built() -> int:
    """未构建过时自动执行 colcon build（幂等）。"""
    if os.path.isfile(INSTALL_SETUP):
        return 0
    print("检测到尚未构建，先执行 colcon build …")
    return sh(
        f"source {ROS_SETUP} && colcon build --packages-select mnist_digit_control"
    )


def main(argv: list) -> int:
    cmd = argv[0] if argv else "run"

    if cmd == "build":
        return sh(f"source {ROS_SETUP} && colcon build --packages-select mnist_digit_control")

    if cmd in ("run", ""):
        code = ensure_built()
        if code:
            return code
        extra = " ".join(argv[1:])
        return sh(
            f"source {ROS_SETUP} && source {INSTALL_SETUP} && "
            f"ros2 launch mnist_digit_control mnist_digit_control.launch.py {extra}"
        )

    if cmd == "train":
        return sh(f"python3 {os.path.join(MODULE_DIR, 'scripts', 'train_mnist.py')} "
                  + " ".join(argv[1:]), cwd=MODULE_DIR)

    if cmd == "scenes":
        return sh(f"python3 {os.path.join(MODULE_DIR, 'scripts', 'make_scene_videos.py')}",
                  cwd=MODULE_DIR)

    if cmd == "eval":
        return sh(f"python3 {os.path.join(MODULE_DIR, 'scripts', 'evaluate_scenes.py')}",
                  cwd=MODULE_DIR)

    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
