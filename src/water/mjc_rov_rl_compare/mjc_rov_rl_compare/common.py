"""common.py — 路径工具。"""
import os
from pathlib import Path

def get_package_root() -> Path:
    return Path(__file__).resolve().parent.parent

def set_mujoco_gl():
    os.environ.setdefault("MUJOCO_GL", "osmesa")
    os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")
