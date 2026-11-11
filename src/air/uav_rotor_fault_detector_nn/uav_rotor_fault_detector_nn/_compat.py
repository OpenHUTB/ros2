"""定位并导入 MuJoCo 四旋翼仿真底座 ``uav_mujoco_sim``。

底座的磁盘布局是一个 ROS2 包目录::

    src/air/uav_mujoco_sim/          <- 项目目录（没有 __init__.py）
        uav_mujoco_sim/              <- 真正的 Python 包（有 __init__.py）
        setup.py
        ...

所以 ``sys.path`` 必须加 **项目目录的父目录**（也就是 ``src/air``），
而不是项目目录本身——否则 ``import uav_mujoco_sim`` 会命中那个同名项目目录，
被当成命名空间包，随后 ``from uav_mujoco_sim.core import ...`` 报
``No module named 'uav_mujoco_sim.core'``。

本模块与底座保持"只读"关系：不修改底座任何文件，只用它的公开接口。
"""

from __future__ import annotations

import importlib
import os
import sys
from pathlib import Path
from typing import NamedTuple

BASE_PACKAGE = "uav_mujoco_sim"
#: 底座把物理/场景/传感器符号都放在 ``uav_mujoco_sim.core`` 里（惰性导入），
#: 顶层 ``__init__`` 只有版本号，所以这里要 import 的是 core 子包。
CORE_MODULE = BASE_PACKAGE + ".core"
#: 允许用户用环境变量显式指定 ``src/air`` 目录（例如底座被装在别处时）
ENV_VAR = "UAV_MUJOCO_SIM_SRC"

_REQUIRED = (
    "Quadrotor",
    "QuadParams",
    "QuadrotorState",
    "GeometricController",
    "mixer",
    "make_scene",
    "SCENE_BUILDERS",
)


class BaseAPI(NamedTuple):
    """底座对外暴露的那部分符号，集中在这里方便测试里打桩。"""

    Quadrotor: type
    QuadParams: type
    QuadrotorState: type
    GeometricController: type
    mixer: object
    make_scene: object
    SCENE_BUILDERS: dict


def _candidate_roots() -> list[Path]:
    """按优先级给出 ``sys.path`` 候选根目录。

    判据是"这下面直接就有 ``uav_mujoco_sim/__init__.py``"，所以源码树里应当
    给出 **项目目录**（``src/air/uav_mujoco_sim``），而不是它的父目录
    ``src/air``——后者下面那个同名目录没有 ``__init__.py``，会被 Python 当成
    命名空间包，反而把真正的包遮住。
    """
    roots: list[Path] = []

    env = os.environ.get(ENV_VAR)
    if env:
        base = Path(env)
        roots.extend([base, base / BASE_PACKAGE])

    here = Path(__file__).resolve()
    # here.parents[0] = 包目录 ... [2] = src/air ... [4] = 仓库根
    for index in (1, 2, 3, 4):
        try:
            base = here.parents[index]
        except IndexError:  # pragma: no cover - 只在非法安装布局下触发
            continue
        roots.extend([base, base / BASE_PACKAGE])

    seen: set[Path] = set()
    unique: list[Path] = []
    for root in roots:
        if root in seen:
            continue
        seen.add(root)
        unique.append(root)
    return unique


def _is_project_root(root: Path) -> bool:
    """``root`` 里能不能直接 ``import uav_mujoco_sim``。"""
    return (root / BASE_PACKAGE / "__init__.py").is_file()


def _ensure_on_path() -> Path | None:
    """把第一个合法的 search root 插到 ``sys.path`` 最前面。"""
    for root in _candidate_roots():
        if _is_project_root(root):
            text = str(root)
            if text not in sys.path:
                sys.path.insert(0, text)
            return root
    return None


def _drop_namespace_shadow() -> None:
    """清掉之前被误判成命名空间包的 ``uav_mujoco_sim*``。"""
    for name in [key for key in sys.modules if key == BASE_PACKAGE
                 or key.startswith(BASE_PACKAGE + ".")]:
        module = sys.modules.get(name)
        if module is None or getattr(module, "__file__", None) is None:
            sys.modules.pop(name, None)


def load_base() -> BaseAPI:
    """导入底座并返回所需符号。失败时抛出带排查提示的 ImportError。"""
    _ensure_on_path()
    try:
        module = importlib.import_module(CORE_MODULE)
    except ImportError:
        _drop_namespace_shadow()
        _ensure_on_path()
        try:
            module = importlib.import_module(CORE_MODULE)
        except ImportError as exc:  # pragma: no cover - 环境问题
            raise ImportError(_hint()) from exc

    missing = [name for name in _REQUIRED if not hasattr(module, name)]
    if missing:
        raise ImportError(_hint(missing))

    values = {name: getattr(module, name) for name in _REQUIRED}
    return BaseAPI(**values)


def _hint(missing: tuple[str, ...] | list[str] = ()) -> str:
    lines = [
        f"无法从 {BASE_PACKAGE} 导入必要符号：{', '.join(missing) or '(导入失败)'}",
        "请确认：",
        "  1) 在仓库根目录下运行，或已经 colcon build 过 uav_mujoco_sim；",
        "  2) 仿真侧 Python >= 3.10 且已安装 mujoco>=3.0（pip install 'mujoco>=3.0'）；",
        f"  3) 若底座不在预期位置，设置环境变量 {ENV_VAR}=<仓库>/src/air。",
    ]
    return "\n".join(lines)
