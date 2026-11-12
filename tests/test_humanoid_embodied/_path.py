"""共享工具：把 ``src/humanoid/humanoid_embodied_control`` 加进 sys.path。

仓库根目录下 pytest 运行时，模块的 import 路径取决于 CWD，这里统一处理，
保证 ``pytest tests/`` 在任意目录下都能跑。
"""

from __future__ import annotations

import os
import sys

def _find_repo_root(start: str) -> str:
    """从 ``start`` 逐级向上找含 ``mkdocs.yml`` 与 ``src/`` 的仓库根。"""
    cur = os.path.abspath(start)
    while True:
        if os.path.isfile(os.path.join(cur, "mkdocs.yml")) and \
                os.path.isdir(os.path.join(cur, "src")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            raise RuntimeError("未能从 %s 找到仓库根" % start)
        cur = parent


# 本文件位于 <repo>/tests/test_humanoid_embodied/_path.py
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = _find_repo_root(_HERE)
_PKG_PARENT = os.path.abspath(
    os.path.join(_REPO_ROOT, "src", "humanoid", "humanoid_embodied_control")
)

if not os.path.isdir(_PKG_PARENT):
    # 回退：从 tests/ 目录出发再解析一次（某些 clone 布局不同）
    _ALT = os.path.abspath(os.path.join(_REPO_ROOT, "src", "humanoid",
                                        "humanoid_embodied_control"))
    if os.path.isdir(_ALT):
        _PKG_PARENT = _ALT
    else:
        raise RuntimeError("找不到模块目录: %s" % _PKG_PARENT)

if _PKG_PARENT not in sys.path:
    sys.path.insert(0, _PKG_PARENT)

# .dsh 附件目录在别处时，允许通过环境变量覆盖仓库根
_ALT = os.environ.get("HUMANOID_EMBODIED_PKG")
if _ALT and os.path.isdir(_ALT) and _ALT not in sys.path:
    sys.path.insert(0, _ALT)


def have_mujoco() -> bool:
    """当前环境能否 import mujoco（没有时跳过物理相关用例）。"""
    try:
        import mujoco  # noqa: F401
        return True
    except Exception:
        return False


def model_path() -> str:
    """人形 MuJoCo 模型路径（相对于本文件，不依赖 CWD）。"""
    local = os.path.join(_HERE, "..", "..", "..", "src", "humanoid",
                         "humanoid_embodied_control", "models", "humanoid.xml")
    if os.path.isfile(local):
        return os.path.abspath(local)
    for root in sys.path:
        cand = os.path.join(root, "models", "humanoid.xml")
        if os.path.isfile(cand):
            return cand
    raise RuntimeError("找不到 models/humanoid.xml")
