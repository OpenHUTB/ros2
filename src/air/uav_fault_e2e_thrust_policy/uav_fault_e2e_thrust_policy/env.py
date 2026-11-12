"""仿真环境封装：走廊任务 + 桨故障注入 + 深度相机观测。

一步的因果链是固定的，和仓库里其它 MuJoCo 模块一致：

    策略/专家给出 4 个归一化指令 → 乘满量程 → 乘故障增益加偏置 → clip →
    ``apply_rotor_thrust`` → ``quad.step()`` → 从真值状态生成观测

策略拿到的观测里**没有世界系位置**，只有深度图与机体系量，所以它不能靠"偷看答案"
飞；专家则不受这个限制（专家本来就有真值）。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import allocation as A
from . import obs as O
from .expert import TaskConfig, ThrustExpert
from .faults import FaultPlan


@dataclass
class StepInfo:
    t: float = 0.0
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gains: np.ndarray = field(default_factory=lambda: np.ones(4))
    commands: np.ndarray = field(default_factory=lambda: np.zeros(4))
    actual: np.ndarray = field(default_factory=lambda: np.zeros(4))
    crashed: bool = False
    success: bool = False
    done: bool = False


class ThrustEnv:
    """把底座四旋翼包成"观测 → 桨推力"的环境。"""

    def __init__(self, api, task: TaskConfig | None = None,
                 plan: FaultPlan | None = None, seed: int = 0,
                 camera_size: int = 64, depth_max_range: float = O.MAX_DEPTH):
        self.api = api
        self.task = task or TaskConfig()
        self.plan = plan or FaultPlan(kind="healthy", onset=1e9)
        self.seed = int(seed)
        self.camera_size = int(camera_size)
        self.depth_max_range = float(depth_max_range)

        self.quad = None
        self.sensors = None
        self.params = None
        self.effectiveness = None
        self.expert = None
        self.t = 0.0
        self.info = StepInfo()
        self._depth_cache = None
        self._last_commands = np.full(4, 0.35 * self.task.start_altitude / 1.5,
                                     dtype=float)

    # ------------------------------------------------------------------ 构建
    def _build(self) -> None:
        api = self.api
        scene = api.make_scene(self.task.scene, seed=self.seed)
        self.quad = api.Quadrotor(scene=scene, dt=self.task.dt,
                                  substeps=self.task.substeps)
        self.params = self.quad.params
        self.effectiveness = A.build_effectiveness(api.mixer, self.params)
        base = api.GeometricController(self.params)
        self.expert = ThrustExpert(self.params, api.mixer, base,
                                   self.effectiveness, self.task)
        if self.sensors is None:
            cfg = api.CameraConfig(
                width=self.camera_size, height=self.camera_size,
                name="front_cam", quality="fast",
                depth_max_range=self.depth_max_range, depth_min_range=0.2,
            )
            self.sensors = api.VisionSensors(self.quad.model, cfg)
        self.quad.reset(pos=(0.0, 0.0, self.task.start_altitude), yaw=0.0)
        import mujoco

        mujoco.mj_forward(self.quad.model, self.quad.data)
        self.t = 0.0
        self._last_commands = np.full(4, self.params.hover_thrust
                                     / self.params.max_thrust, dtype=float)
        self.info = StepInfo(t=0.0, position=self.quad.state().pos.copy(),
                             gains=np.ones(4))

    def reset(self, seed: int | None = None, plan: FaultPlan | None = None) -> dict:
        if seed is not None:
            self.seed = int(seed)
        if plan is not None:
            self.plan = plan
        self._build()
        return self.observation()

    # -------------------------------------------------------------- 观测/一步
    def observation(self, render: bool = True) -> dict:
        import mujoco

        mujoco.mj_forward(self.quad.model, self.quad.data)
        state = self.quad.state()
        # 深度图渲染是整条链路里最贵的一步（64×64 约 60 ms）。ROS 节点一次定时器要推
        # 好几个物理步，没必要每步都渲染，可以复用上一帧（策略输入的是 32×32，降采样后
        # 相邻两帧几乎没差别）。
        if render or self._depth_cache is None:
            self._depth_cache = self.sensors.depth(self.quad.data)
        depth = self._depth_cache
        gains = self.plan.gains(self.t)
        proprio = O.proprio_features(state, gains, self._last_commands,
                                     self.params.max_thrust)
        return O.build_observation(depth, proprio)

    def step(self, commands: np.ndarray, render: bool = True) -> tuple[dict, StepInfo]:
        cmd = np.clip(np.asarray(commands, dtype=float).reshape(4), 0.0, 1.0)
        gains = self.plan.gains(self.t)
        bias = self.plan.bias(self.t, self.params.hover_thrust)
        actual = np.clip(cmd * self.params.max_thrust * gains + bias,
                         0.0, self.params.max_thrust)
        self.quad.apply_rotor_thrust(actual)
        self.quad.step(1)
        self.t += self.task.dt
        self._last_commands = cmd.copy()

        state = self.quad.state()
        pos = np.asarray(state.pos, dtype=float)
        crashed = bool(pos[2] < self.task.crash_altitude)
        success = bool(pos[0] >= self.task.goal_x and not crashed)
        timeout = self.t >= self.task.duration
        lateral = bool(abs(pos[1]) > self.task.max_lateral)
        self.info = StepInfo(
            t=self.t, position=pos.copy(),
            velocity=np.asarray(state.vel, dtype=float).copy(),
            gains=np.asarray(gains, dtype=float).copy(),
            commands=cmd.copy(), actual=actual, crashed=crashed or lateral,
            success=success, done=bool(crashed or lateral or success or timeout),
        )
        return self.observation(render=render), self.info

    # ------------------------------------------------------------------ 便利
    def expert_command(self) -> np.ndarray:
        state = self.quad.state()
        gains = self.plan.gains(self.t)
        bias = self.plan.bias(self.t, self.params.hover_thrust)
        return self.expert.command(state, gains, bias, self.t)

    def close(self) -> None:
        if self.sensors is not None:
            try:
                self.sensors.close()
            except Exception:  # pragma: no cover - 关不掉也不该炸
                pass
            self.sensors = None


def make_env(api, plan: FaultPlan | None = None, seed: int = 0,
             task: TaskConfig | None = None) -> ThrustEnv:
    """便捷构造器。"""
    return ThrustEnv(api, task=task, plan=plan, seed=seed)
