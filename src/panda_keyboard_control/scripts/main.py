#!/usr/bin/env python3
import os
import subprocess
import sys


def main():
    pkg = "panda_keyboard_control"
    # 自动定位到 launch 文件
    current_dir = os.path.dirname(os.path.abspath(__file__))
    launch_file = os.path.join(current_dir, "..", "launch", "main.launch")
    
    cmd = ["roslaunch", pkg, os.path.basename(launch_file)]
    try:
        print("正在启动: {}".format(' '.join(cmd)))
        subprocess.run(cmd, check=True)
    except FileNotFoundError:
        print("未找到 roslaunch，请先 source /opt/ros/noetic/setup.bash", file=sys.stderr)
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print("启动失败: {}".format(e), file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
