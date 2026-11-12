# -*- coding: utf-8 -*-
"""ROS 节点公共引导：路径解析与项目 venv 重执行。"""
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

# 源码空间：<pkg>/src；安装空间：Python 包由 catkin 安装到 PYTHONPATH，
# 这里只在源码目录存在时才插入，避免把不存在的 share/src 放进去。
SRC_DIR = os.path.join(PKG_ROOT, "src")
if os.path.isdir(SRC_DIR) and SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)


def venv_python() -> str:
    """优先 UAV_VENV_PYTHON，否则使用包内 .venv（setup_env.sh 创建的位置）。"""
    env = os.environ.get("UAV_VENV_PYTHON", "").strip()
    if env:
        return env
    return os.path.join(PKG_ROOT, ".venv", "bin", "python")


def use_venv() -> None:
    """切到带 MuJoCo 的 Python 3.11 venv；当前解释器已可用则不切换。

    找不到 venv 时给出明确的下一步提示，不再静默回退到系统 Python。
    """
    try:
        import mujoco  # noqa: F401
        return
    except Exception:
        pass

    venv = venv_python()
    if not os.path.exists(venv):
        sys.stderr.write(
            "[uav] 找不到 MuJoCo venv: %s\n"
            "  请先在包目录运行: bash scripts/setup_env.sh\n"
            "  或设置: export UAV_VENV_PYTHON=/path/to/python3.11\n" % venv
        )
        raise SystemExit(1)
    if os.path.realpath(sys.executable) == os.path.realpath(venv):
        return
    os.execv(venv, [venv, os.path.abspath(sys.argv[0])] + sys.argv[1:])
