# -*- coding: utf-8 -*-
"""ROS 节点公共引导：路径与可选 venv 重执行。"""
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
try:
    import rospkg
    _p = rospkg.RosPack().get_path("uav_wind_aware_delivery")
    if _p and os.path.isdir(_p):
        PKG_ROOT = _p
except Exception:
    pass
SRC_DIR = os.path.join(PKG_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


def use_venv():
    """把仿真节点切到装有 mujoco 的 Python 3.11 venv。"""
    venv = os.environ.get("UAV_VENV_PYTHON", "/home/lxy/ground_nn_nav/.venv/bin/python")
    if os.path.exists(venv) and os.path.realpath(sys.executable) != os.path.realpath(venv):
        os.execv(venv, [venv, os.path.abspath(sys.argv[0])] + sys.argv[1:])
