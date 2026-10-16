#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定位 `uav_ppo_nav` 的训练环境（NavEnv），供对比实验复用。

对比实验必须用**完全相同的环境与接口**，否则比出来的差异没有意义；所以这里不复制
`env.py`，而是定位到 `uav_ppo_nav` 的脚本目录并导入。

适配两种布局（都已验证）：
  * 源码空间： ``src/air/uav_rl_benchmark/scripts`` 与 ``src/air/uav_ppo_nav/scripts`` 同级
  * devel 空间：``devel/lib/uav_rl_benchmark`` 与 ``devel/lib/uav_ppo_nav`` 同级
    （catkin_install_python 把脚本直接装在 lib/<包>/ 下，没有 scripts 子目录）
"""
from __future__ import annotations

import os
import sys


def _candidates():
    """按优先级给出 uav_ppo_nav 脚本目录的候选路径."""
    here = os.path.dirname(os.path.abspath(__file__))
    up1 = os.path.dirname(here)                      # devel: lib/uav_rl_benchmark -> lib
    up2 = os.path.dirname(up1)                       # 源码: air/uav_rl_benchmark/scripts -> air
    roots = [up1, up2]
    cands = [os.environ.get("UAV_PPO_SCRIPT_DIR", "")]
    for r in roots:
        cands.append(os.path.join(r, "uav_ppo_nav", "scripts"))   # 源码空间
        cands.append(os.path.join(r, "uav_ppo_nav"))              # devel 空间
    for root in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep):
        if root:
            cands.append(os.path.join(root, "uav_ppo_nav", "scripts"))
            cands.append(os.path.join(root, "uav_ppo_nav"))
    return [c for c in cands if c]


def find_nav_scripts() -> str:
    """返回含 env.py 的目录；找不到就抛异常并列出试过的路径."""
    tried = []
    for d in _candidates():
        tried.append(d)
        if os.path.isfile(os.path.join(d, "env.py")) and \
           os.path.isfile(os.path.join(d, "policy.py")):
            return d
    raise ImportError(
        "找不到 uav_ppo_nav 的脚本目录（需要含 env.py 与 policy.py）。\n"
        "已尝试：\n  " + "\n  ".join(tried) +
        "\n提示：可用环境变量 UAV_PPO_SCRIPT_DIR 显式指定。")


NAV_SCRIPTS = None


def load_nav_env():
    """导入并返回 (NavEnv, policy 模块)."""
    global NAV_SCRIPTS
    if NAV_SCRIPTS is None:
        NAV_SCRIPTS = find_nav_scripts()
        if NAV_SCRIPTS not in sys.path:
            sys.path.insert(0, NAV_SCRIPTS)
    from env import NavEnv            # noqa: WPS433
    import policy as policy_mod       # noqa: WPS433
    return NavEnv, policy_mod


def make_vec_env(NavEnv, obstacles=16, seed=0):
    """单环境 DummyVecEnv（与 uav_ppo_nav 的训练脚本一致）."""
    from stable_baselines3.common.vec_env import DummyVecEnv  # noqa: WPS433

    def _f():
        return NavEnv(num_obstacles=obstacles, seed=seed)
    return DummyVecEnv([_f])
