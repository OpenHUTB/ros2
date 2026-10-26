#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_route_cruise 统一入口（作业要求入口为 main.*）.

    roslaunch uav_route_cruise main.launch      # 部署：沿固定航线巡航
    python3 main.py                             # 同上（直接跑节点）
    python3 main.py --train [参数]              # 训练 PPO
    python3 main.py --eval [参数]               # 评估（含纯循迹对照）

注意：本模块与 uav_ppo_nav 是**两个独立包**，加载时都可能用到 sys.path，
所以这里除了自己的源码目录，还会把 uav_ppo_nav/scripts 之外的路径排除，
避免 `import eval` / `import policy` 之类的同名模块互相遮蔽。
"""
from __future__ import annotations

import os
import sys


def _prepare_import_path():
    """让 `import route_node` 拿到【源码】而不是 catkin 生成的 wrapper.

    catkin_install_python 会给入口脚本包一层 wrapper（内容是 exec(compile(...))）。
    若被 import 的模块也走 catkin_install_python，拿到的就是 wrapper，其命名空间里
    没有 main 等函数，于是报 `module 'route_node' has no attribute 'main'`。

    判定标准：目录里存在含 "class RouteCruiseNode" 的 route_node.py。
    """
    cands = [
        os.environ.get("UAV_ROUTE_SCRIPT_DIR", ""),               # launch 通过 <env> 传入
        os.path.dirname(os.path.abspath(__file__)),               # 直接运行时的本文件目录
    ]
    if sys.argv and sys.argv[0]:
        cands.append(os.path.dirname(os.path.abspath(sys.argv[0])))
    for root in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if root:
            cands.append(os.path.join(root, "uav_route_cruise", "scripts"))

    for d in cands:
        if not d or not os.path.isdir(d):
            continue
        f = os.path.join(d, "route_node.py")
        if not os.path.isfile(f):
            continue
        try:
            with open(f, encoding="utf-8", errors="ignore") as fh:
                if "class RouteCruiseNode" in fh.read():
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
    elif argv and argv[0] == "--compare":
        from compare import main as cmp_main  # noqa: WPS433
        sys.argv = [sys.argv[0]] + argv[1:]
        cmp_main()
    else:
        import route_node  # noqa: WPS433
        route_node.main()


if __name__ == "__main__":
    main()
