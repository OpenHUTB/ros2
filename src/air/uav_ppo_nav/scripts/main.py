#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_ppo_nav 统一入口（作业要求入口为 main.*）.

    roslaunch uav_ppo_nav main.launch     # 部署：运行策略控制无人机
    python3 main.py --train [参数]        # 训练 PPO
    python3 main.py --eval [参数]         # 评估成功率
"""
from __future__ import annotations

import os
import sys


def _prepare_import_path():
    """让 `import ppo_nav_node` 拿到【源码】而不是 catkin 生成的 wrapper.

    catkin_install_python 会给入口脚本包一层 wrapper（内容是 exec(compile(...))）。
    若被 import 的模块也走 catkin_install_python，拿到的就是 wrapper，其命名空间里
    没有 main 等函数，于是报 `module 'ppo_nav_node' has no attribute 'main'`。

    这里按优先级收集候选目录，逐个检查里面是否有【真正的源码】
    （判定标准：ppo_nav_node.py 里含 "class PpoNavNode"），找到就放到 sys.path 最前。
    """
    cands = [
        os.environ.get("UAV_PPO_SCRIPT_DIR", ""),                 # launch 通过 <env> 传入
        os.path.dirname(os.path.abspath(__file__)),               # 直接运行时的本文件目录
    ]
    if sys.argv and sys.argv[0]:
        cands.append(os.path.dirname(os.path.abspath(sys.argv[0])))
    for root in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if root:
            cands.append(os.path.join(root, "uav_ppo_nav", "scripts"))

    for d in cands:
        if not d or not os.path.isdir(d):
            continue
        f = os.path.join(d, "ppo_nav_node.py")
        if not os.path.isfile(f):
            continue
        try:
            with open(f, encoding="utf-8", errors="ignore") as fh:
                if "class PpoNavNode" in fh.read():
                    if d not in sys.path:
                        sys.path.insert(0, d)
                    return d
        except OSError:
            continue
    return None


_SOURCE_DIR = _prepare_import_path()


def main():
    argv = sys.argv[1:]
    if argv and argv[0] == "--train":
        from train import main as train_main  # noqa: WPS433
        sys.argv = [sys.argv[0]] + argv[1:]
        train_main()
    elif argv and argv[0] == "--eval":
        from eval import main as eval_main  # noqa: WPS433
        sys.argv = [sys.argv[0]] + argv[1:]
        eval_main()
    else:
        import ppo_nav_node  # noqa: WPS433
        ppo_nav_node.main()


if __name__ == "__main__":
    main()
