"""故障场景采样 + 由 MuJoCo 底座批量生成训练/验证/测试数据。

这里的故障模型与 ``uav_actuator_fault_sim``（PR #576）保持同一套公式::

    T_i^act = g_i(t)·u_i^cmd + b_i(t)

本模块只关心 **g_i(t)**：它是"这个桨还剩多少推力权限"，也是检测器的学习目标。
所以本模块自己实现了一份最小的故障算子，既不依赖 #576 是否合并，也不改动底座。

一个 episode = 一次 8 s 的仿真飞行：随机场景、随机目标点、随机故障，
逐步记录特征向量与四桨真实增益。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Iterable, Sequence

import numpy as np

from .features import (
    FEATURE_DIM,
    GRAVITY,
    ROTOR_COUNT,
    build_step_features,
    make_windows,
    specific_force,
)

#: 故障类型 -> 中文名（写文档和终端表格用）
FAULT_CN = {
    "healthy": "健康",
    "loss": "单桨完全失效",
    "degradation": "单桨推力衰减",
    "stuck": "桨卡死",
    "dual_loss": "双桨失效",
    "progressive": "渐进退化",
    "imu_bias": "IMU 零偏",
    "imu_noise": "IMU 噪声放大",
}

ROTOR_FAULT_KINDS = ("loss", "degradation", "stuck", "dual_loss", "progressive")
SENSOR_FAULT_KINDS = ("imu_bias", "imu_noise")
FAULT_KINDS = ("healthy",) + ROTOR_FAULT_KINDS + SENSOR_FAULT_KINDS

PHYSICAL_SCENES = ("open_ground", "corridor", "obstacle_field", "urban_block")

#: 不同故障类型在随机采样里的权重（健康样本要占一定比例，否则检测器只会报警）
_KIND_WEIGHTS = {
    "healthy": 0.22,
    "loss": 0.16,
    "degradation": 0.16,
    "stuck": 0.14,
    "dual_loss": 0.10,
    "progressive": 0.14,
    "imu_bias": 0.04,
    "imu_noise": 0.04,
}

#: 增益下限：完全失效用 0，渐进退化保留一点推力（与 #576 的 max(0.05, ...) 一致）
GAIN_FLOOR_PROGRESSIVE = 0.05


@dataclass(frozen=True)
class FaultPlan:
    """一次飞行的故障脚本。"""

    kind: str
    rotor: int = 0
    rotor2: int = 1
    onset: float = 2.0
    severity: float = 1.0
    ramp: float = 0.0
    gyro_bias: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if self.kind not in FAULT_KINDS:
            raise ValueError(f"未知故障类型 {self.kind!r}，可选：{FAULT_KINDS}")
        for index in (self.rotor, self.rotor2):
            if not 0 <= index < ROTOR_COUNT:
                raise ValueError(f"桨序号必须在 0..{ROTOR_COUNT - 1}，收到 {index}")
        if self.onset < 0.0:
            raise ValueError(f"onset 必须 >= 0，收到 {self.onset}")
        if self.ramp < 0.0:
            raise ValueError(f"ramp 必须 >= 0，收到 {self.ramp}")
        if self.kind == "progressive" and self.severity <= 0.0:
            raise ValueError("progressive 的 severity 是衰减速率（1/s），必须 > 0")

    # ------------------------------------------------------------------ 增益
    def _alpha(self, t: float) -> float:
        if t < self.onset:
            return 0.0
        if self.ramp <= 0.0:
            return 1.0
        return min(1.0, (t - self.onset) / self.ramp)

    def _gain_one(self, t: float) -> float:
        alpha = self._alpha(t)
        if alpha <= 0.0:
            return 1.0
        if self.kind == "loss":
            return max(0.0, 1.0 - self.severity * alpha)
        if self.kind == "degradation":
            return max(0.0, 1.0 - self.severity * alpha)
        if self.kind == "stuck":
            return max(0.0, 1.0 - alpha)
        if self.kind == "dual_loss":
            return max(0.0, 1.0 - alpha)
        if self.kind == "progressive":
            # severity 是每秒损失的推力比例，例如 0.25 表示 4 s 掉光
            return max(GAIN_FLOOR_PROGRESSIVE, 1.0 - self.severity * (t - self.onset))
        return 1.0

    def gains(self, t: float) -> np.ndarray:
        """当前时刻四桨的真实推力增益 ``g(t)``，形状 ``(4,)``。"""
        gains = np.ones(ROTOR_COUNT, dtype=np.float64)
        if self.kind in ROTOR_FAULT_KINDS:
            gains[self.rotor] = self._gain_one(t)
            if self.kind == "dual_loss":
                gains[self.rotor2] = self._gain_one(t)
        return gains

    def sensor_factor(self) -> tuple[tuple[float, float, float], float]:
        """返回 (陀螺零偏, 噪声放大系数)，用于 ``imu_bias`` / ``imu_noise``。"""
        if self.kind == "imu_bias":
            return self.gyro_bias, 1.0
        if self.kind == "imu_noise":
            return self.gyro_bias, 6.0
        return (0.0, 0.0, 0.0), 1.0

    def describe(self) -> str:
        name = FAULT_CN.get(self.kind, self.kind)
        if self.kind in ("healthy",) + SENSOR_FAULT_KINDS:
            return f"{name}@{self.onset:.1f}s"
        if self.kind == "dual_loss":
            return f"{name}(桨{self.rotor},{self.rotor2})@{self.onset:.1f}s"
        return f"{name}(桨{self.rotor})@{self.onset:.1f}s"


def random_plan(rng: np.random.Generator) -> FaultPlan:
    """按权重随机抽一个故障脚本。"""
    kinds = list(_KIND_WEIGHTS)
    weights = np.array([_KIND_WEIGHTS[k] for k in kinds], dtype=np.float64)
    kind = str(rng.choice(kinds, p=weights / weights.sum()))

    rotor = int(rng.integers(0, ROTOR_COUNT))
    rotor2 = int((rotor + int(rng.integers(1, ROTOR_COUNT))) % ROTOR_COUNT)
    onset = float(rng.uniform(1.5, 4.0))

    if kind == "loss":
        severity = 1.0
        ramp = float(rng.choice([0.0, 0.3, 0.8]))
    elif kind == "degradation":
        severity = float(rng.uniform(0.2, 0.85))
        ramp = float(rng.choice([0.0, 0.5]))
    elif kind == "stuck":
        severity = 1.0
        ramp = float(rng.choice([0.0, 0.5]))
    elif kind == "dual_loss":
        severity = 1.0
        ramp = float(rng.choice([0.0, 0.6]))
    elif kind == "progressive":
        severity = float(rng.uniform(0.1, 0.4))
        ramp = 0.0
    else:
        severity = float(rng.uniform(0.3, 1.0))
        ramp = 0.0

    bias = (
        float(rng.uniform(-0.15, 0.15)),
        float(rng.uniform(-0.15, 0.15)),
        float(rng.uniform(-0.15, 0.15)),
    )
    return FaultPlan(
        kind=kind,
        rotor=rotor,
        rotor2=rotor2,
        onset=onset,
        severity=severity,
        ramp=ramp,
        gyro_bias=bias,
    )


@dataclass
class Episode:
    """一次飞行的采样结果。"""

    features: np.ndarray
    gains: np.ndarray
    altitude: np.ndarray
    plan: FaultPlan
    scene: str
    seed: int
    crashed: bool
    crash_time: float
    meta: dict = field(default_factory=dict)

    @property
    def length(self) -> int:
        return int(self.features.shape[0])


def make_quad(api, scene: str, seed: int, dt: float, substeps: int):
    """按场景名构造底座 ``Quadrotor``（``make_scene`` 不一定接受 ``seed``）。"""
    try:
        scene_obj = api.make_scene(scene, seed=seed)
    except TypeError:
        scene_obj = api.make_scene(scene)
    return api.Quadrotor(scene=scene_obj, dt=dt, substeps=substeps)


def simulate_episode(
    api,
    plan: FaultPlan,
    scene: str = "open_ground",
    seed: int = 0,
    duration: float = 8.0,
    dt: float = 0.01,
    substeps: int = 4,
    imu_noise: float = 0.02,
    hover_altitude: float = 1.5,
    waypoint_count: int = 3,
    max_waypoint_radius: float = 2.5,
    stop_on_crash: bool = True,
) -> Episode:
    """跑一次仿真飞行并记录特征序列。"""
    rng = np.random.default_rng(seed)

    quad = make_quad(api, scene, seed, dt, substeps)
    ctrl = api.GeometricController(quad.params)
    max_thrust = float(quad.params.max_thrust)

    quad.reset(pos=(0.0, 0.0, hover_altitude))
    target = np.array([0.0, 0.0, hover_altitude], dtype=np.float64)
    v_d = np.zeros(3)
    a_d = np.zeros(3)
    yaw_d = float(rng.uniform(-0.6, 0.6))

    # 目标点序列：每次切点都会让控制指令出现一段明显变化，便于网络看到"指令-响应"关系
    waypoints = []
    for _ in range(waypoint_count):
        radius = rng.uniform(0.8, max_waypoint_radius)
        angle = rng.uniform(0.0, 2.0 * np.pi)
        waypoints.append(
            np.array(
                [
                    radius * np.cos(angle),
                    radius * np.sin(angle),
                    float(np.clip(rng.normal(hover_altitude + 0.5, 0.6), 0.8, 3.0)),
                ]
            )
        )
    segment = max(duration / max(len(waypoints), 1), 0.8)

    steps = int(round(duration / dt))
    bias, noise_scale = plan.sensor_factor()

    features = np.zeros((steps, FEATURE_DIM), dtype=np.float32)
    gains = np.zeros((steps, ROTOR_COUNT), dtype=np.float32)
    altitude = np.zeros((steps,), dtype=np.float32)

    prev_vel = np.zeros(3)
    crashed = False
    crash_time = float("nan")
    written = 0

    for k in range(steps):
        t = k * dt

        # 目标点按段切换
        index = min(int(t // segment), len(waypoints) - 1) if waypoints else 0
        if waypoints:
            target = waypoints[index]

        u_cmd = np.asarray(ctrl.act(quad, target, v_d, a_d, yaw_d), dtype=np.float64)
        u_cmd = np.clip(u_cmd, 0.0, max_thrust)

        g = plan.gains(t)
        u_act = np.clip(g * u_cmd, 0.0, max_thrust)
        quad.apply_rotor_thrust(u_act)

        prev_vel = np.asarray(quad.state().vel, dtype=np.float64).copy()
        quad.step(1)
        state = quad.state()

        acc = specific_force(state.vel, prev_vel, dt, state.quat)
        gyro = np.asarray(state.omega, dtype=np.float64)

        if imu_noise > 0.0:
            acc = acc + rng.normal(0.0, imu_noise * noise_scale, 3)
            gyro = gyro + rng.normal(0.0, imu_noise * noise_scale, 3)
        if any(bias):
            gyro = gyro + np.asarray(bias, dtype=np.float64)

        features[k] = build_step_features(acc, gyro, state.vel, u_cmd, max_thrust)
        gains[k] = g
        written = k + 1

        altitude[k] = float(state.pos[2])
        if altitude[k] < 0.15:
            crashed = True
            crash_time = t
            if stop_on_crash:
                break

    return Episode(
        features=features[:written],
        gains=gains[:written],
        altitude=altitude[:written],
        plan=plan,
        scene=scene,
        seed=seed,
        crashed=crashed,
        crash_time=crash_time,
        meta={"duration": float(written * dt), "dt": dt, "max_thrust": max_thrust},
    )


def build_dataset(
    api,
    n_episodes: int,
    seed: int = 0,
    scenes: Sequence[str] = ("open_ground", "corridor", "obstacle_field"),
    duration: float = 8.0,
    dt: float = 0.01,
    substeps: int = 4,
    window: int = 100,
    stride: int = 10,
    plans: Iterable[FaultPlan] | None = None,
    progress=None,
) -> dict:
    """批量生成滑窗数据集。

    ``plans`` 给定时按给定顺序使用（用于可复现的对照实验），否则随机采样。
    """
    rng = np.random.default_rng(seed + 977)
    plan_list = list(plans) if plans is not None else None
    if plan_list is not None and len(plan_list) < n_episodes:
        raise ValueError(f"plans 只有 {len(plan_list)} 条，不足以覆盖 {n_episodes} 个 episode")

    xs: list[np.ndarray] = []
    ys: list[np.ndarray] = []
    ep_index: list[np.ndarray] = []
    end_index: list[np.ndarray] = []
    kinds: list[str] = []
    scene_list: list[str] = []
    crashed: list[bool] = []
    crash_times: list[float] = []
    descriptions: list[str] = []

    for i in range(n_episodes):
        plan = plan_list[i] if plan_list is not None else random_plan(rng)
        scene = str(rng.choice(list(scenes)))
        episode_seed = int(seed * 100_000 + i)
        episode = simulate_episode(
            api,
            plan,
            scene=scene,
            seed=episode_seed,
            duration=duration,
            dt=dt,
            substeps=substeps,
        )
        windows, targets, ends = make_windows(episode.features, episode.gains, window, stride)
        if windows.shape[0] == 0:
            continue
        xs.append(windows)
        ys.append(targets)
        ep_index.append(np.full(windows.shape[0], i, dtype=np.int64))
        end_index.append(ends)
        kinds.append(plan.kind)
        scene_list.append(scene)
        crashed.append(episode.crashed)
        crash_times.append(episode.crash_time)
        descriptions.append(plan.describe())
        if progress is not None:
            progress(i + 1, n_episodes, plan)

    if not xs:
        raise RuntimeError("没有生成任何样本，检查 duration/window/stride 设置")

    return {
        "X": np.concatenate(xs, axis=0),
        "Y": np.concatenate(ys, axis=0),
        "episode": np.concatenate(ep_index, axis=0),
        "end": np.concatenate(end_index, axis=0),
        "kinds": np.array(kinds, dtype=object),
        "scenes": np.array(scene_list, dtype=object),
        "crashed": np.array(crashed, dtype=bool),
        "crash_time": np.array(crash_times, dtype=np.float64),
        "descriptions": np.array(descriptions, dtype=object),
        "window": np.array(window, dtype=np.int64),
        "stride": np.array(stride, dtype=np.int64),
        "dt": np.array(dt, dtype=np.float64),
    }


def save_dataset(path, dataset: dict) -> None:
    """把数据集存成 ``.npz``（文本字段序列化成 JSON 字符串）。"""
    payload = dict(dataset)
    for key in ("kinds", "scenes", "descriptions"):
        payload[key] = json.dumps([str(v) for v in dataset[key]], ensure_ascii=False)
    np.savez_compressed(path, **payload)


def load_dataset(path) -> dict:
    """读回 ``save_dataset`` 写出的 ``.npz``。"""
    with np.load(path, allow_pickle=False) as handle:
        data = {key: handle[key] for key in handle.files}
    for key in ("kinds", "scenes", "descriptions"):
        if key in data:
            data[key] = np.array(json.loads(str(data[key])), dtype=object)
    return data


def split_by_episode(dataset: dict, fractions=(0.8, 0.1, 0.1)) -> dict[str, np.ndarray]:
    """按 **episode** 而不是按样本切分，避免同一段飞行的相邻窗口同时落进训练和测试。

    返回 ``{"train": mask, "val": mask, "test": mask}`` 三个布尔掩码。
    """
    if abs(sum(fractions) - 1.0) > 1e-6:
        raise ValueError(f"fractions 之和必须为 1，收到 {fractions}")
    n_episodes = int(dataset["episode"].max()) + 1
    rng = np.random.default_rng(20241026)
    order = rng.permutation(n_episodes)

    n_train = int(round(n_episodes * fractions[0]))
    n_val = int(round(n_episodes * fractions[1]))
    train_eps = set(order[:n_train].tolist())
    val_eps = set(order[n_train : n_train + n_val].tolist())

    episode = dataset["episode"]
    masks = {
        "train": np.isin(episode, list(train_eps)),
        "val": np.isin(episode, list(val_eps)),
        "test": np.isin(episode, list(order[n_train + n_val :].tolist())),
    }
    return masks
