"""无人机桨故障后的自主迫降：在线建图 + 神经网络评分 + 神经制导。

四个层次，全部只依赖已合并模块 :mod:`uav_mujoco_sim` 的公开接口：

- :mod:`mapping` —— 由激光点云在线累积 2.5D 高度图，并给出**规则评分**（当教师用）；
- :mod:`model` —— 两个小网络：全卷积可降落性评分（2625 参数）、制导 MLP（5251 参数）；
- :mod:`controller` —— 闭环迫降控制器（三种模式：垂直迫降 / 规则+专家律 / 神经+神经）；
- :mod:`evaluate` —— 闭环评测（安全迫降率、触地速度、落点误差、倾角）。

顶层只导入纯 numpy 的部分，`torch` 与 `mujoco` 都是按需延迟导入——所以没有仿真环境
也能 `import` 这个包来做数据检查。
"""

__version__ = "1.0.0"

from .data import (  # noqa: F401
    MAP_SCENES,
    ScanConfig,
    build_guidance_dataset,
    build_score_dataset,
    load_dataset,
    save_dataset,
    scan_scene,
    split_samples,
)
from .landing import (  # noqa: F401
    GUIDANCE_DIM,
    GUIDANCE_FEATURES,
    LandingConfig,
    expert_velocity,
    guidance_features,
    select_target,
    touchdown_ok,
)
from .mapping import (  # noqa: F401
    OBSTACLE_HEIGHT,
    GridConfig,
    HeightMap,
    best_cell,
    height_map_input,
    rule_landing_score,
    world_to_cells,
)
from .controller import MODES, ForcedLandingController, PlanState  # noqa: F401
from .evaluate import (  # noqa: F401
    FLIGHT_PLANS,
    LandingResult,
    aggregate,
    flight_table,
    run_landing,
)
from .faults import FAULT_CN, FAULT_KINDS, FaultPlan  # noqa: F401
from .model import ARCH_VERSION  # noqa: F401

__all__ = [
    "__version__",
    "ARCH_VERSION",
    "FAULT_CN",
    "FAULT_KINDS",
    "FLIGHT_PLANS",
    "ForcedLandingController",
    "GUIDANCE_DIM",
    "GUIDANCE_FEATURES",
    "GridConfig",
    "HeightMap",
    "LandingConfig",
    "LandingResult",
    "MAP_SCENES",
    "MODES",
    "OBSTACLE_HEIGHT",
    "PlanState",
    "ScanConfig",
    "aggregate",
    "best_cell",
    "build_guidance_dataset",
    "build_score_dataset",
    "expert_velocity",
    "flight_table",
    "FaultPlan",
    "guidance_features",
    "height_map_input",
    "load_dataset",
    "rule_landing_score",
    "run_landing",
    "save_dataset",
    "scan_scene",
    "select_target",
    "split_samples",
    "touchdown_ok",
    "world_to_cells",
]
