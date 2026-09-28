#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""uav_ppo_nav 统一入口（作业要求入口为 main.*）.

    roslaunch uav_ppo_nav main.launch     # 部署：运行策略控制无人机
    python3 main.py --train [参数]        # 训练 PPO
    python3 main.py --eval [参数]         # 评估成功率
"""
from __future__ import annotations

import sys


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
