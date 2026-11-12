"""仿真与数据生成：跑一次飞行、按时间采集"分配器看到的量 / 精确解"配对。

一次飞行里同时记录：

* 几何控制器给出的期望 ``(T, τ)``；
* 当前健康度 ``g``、卡死偏差 ``b``；
* 近似分配（加权最小二乘）给出的指令 ``u_ls``；
* 精确箱约束解（枚举有效集）给出的指令 ``u_star``。

后两者之差就是神经网络的监督标签，所以数据集是**离线**生成的：训练时不需要
仿真器，推理时也不需要精确 QP 的 81 次枚举。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import allocation as A
from . import features as F
from . import faults as FL
from . import trajectory as TJ


@dataclass
class EpisodeResult:
    """一次飞行的结果与轨迹。"""

    times: np.ndarray
    positions: np.ndarray
    altitudes: np.ndarray
    desired: np.ndarray
    u_ls: np.ndarray
    u_star: np.ndarray
    features: np.ndarray
    targets: np.ndarray
    gains: np.ndarray
    plan: FL.FaultPlan = FL.FaultPlan()
    trajectory: str = "hover"
    crashed: bool = False
    crash_time: float | None = None
    error: np.ndarray = field(default_factory=lambda: np.zeros(0))

    def __len__(self) -> int:
        return int(self.times.size)

    @property
    def min_altitude(self) -> float:
        return float(np.min(self.altitudes)) if self.altitudes.size else float("nan")

    @property
    def final_altitude(self) -> float:
        return float(self.altitudes[-1]) if self.altitudes.size else float("nan")


def build_context(api):
    """把一次飞行需要的东西打包（机体参数、效率矩阵、参考控制器）。"""
    quad = api.Quadrotor(scene=api.make_scene("open_ground", seed=0),
                         dt=0.01, substeps=4)
    params = quad.params
    effectiveness = A.build_effectiveness(api.mixer, params)
    return params, effectiveness


def run_episode(api, plan: FL.FaultPlan, trajectory: str = "hover",
                seconds: float = 8.0, dt: float = 0.01, substeps: int = 4,
                seed: int = 0, collect: bool = False, sample_stride: int = 1,
                params=None, effectiveness=None) -> EpisodeResult:
    """跑一次飞行。

    ``collect=True`` 时额外记录特征/标签（用精确 QP 求解，较慢），
    ``sample_stride`` 控制每隔多少步求一次标签——数据不必每步都有。
    """
    from . import controller as C

    if params is None or effectiveness is None:
        params, effectiveness = build_context(api)

    quad = api.Quadrotor(scene=api.make_scene("open_ground", seed=int(seed)),
                         dt=dt, substeps=substeps)
    quad.reset(np.array([0.0, 0.0, 1.5]), 0.0)
    base = api.GeometricController(params)
    ctrl = C.FaultTolerantController(params, api.mixer, base, effectiveness,
                                     mode="ls")
    n_steps = int(round(seconds / dt))

    times, positions, altitudes, desired_log, u_ls_log = [], [], [], [], []
    feats, targets, gains_log, u_star_log = [], [], [], []
    crashed, crash_time = False, None

    for k in range(n_steps):
        t = k * dt
        gains = plan.gains(t)
        bias = plan.bias(t, params.hover_thrust)
        ctrl.set_health(gains, bias)

        p_d, v_d, a_d, yaw_d = TJ.reference(trajectory, t)
        u = ctrl.act(quad, p_d, v_d, a_d, yaw_d)

        desired_log.append(np.array(ctrl.last_desired))
        u_ls_log.append(np.array(ctrl.last_u_ls))
        gains_log.append(np.array(gains))

        if collect and (k % max(1, int(sample_stride)) == 0):
            u_star = A.solve_box_qp(effectiveness, ctrl.last_desired, params,
                                    gains, v_ref=None, bias=bias,
                                    row_weights=ctrl.row_weights())
            feats.append(F.build_features(ctrl.last_desired, gains,
                                          ctrl.last_u_ls, params))
            targets.append(F.build_target(u_star, ctrl.last_u_ls, params))
            u_star_log.append(np.array(u_star))

        u_act = np.clip(gains * u + bias, 0.0, float(params.max_thrust))
        quad.apply_rotor_thrust(u_act)
        quad.step(1)
        state = quad.state()
        times.append(t)
        positions.append(np.array(state.pos))
        altitudes.append(float(state.pos[2]))
        if not crashed and state.pos[2] < 0.15:
            crashed, crash_time = True, t
            break

    times_arr = np.asarray(times, dtype=float)
    positions_arr = np.asarray(positions, dtype=float)
    alt_arr = np.asarray(altitudes, dtype=float)
    desired_arr = np.asarray(desired_log, dtype=float)
    u_ls_arr = np.asarray(u_ls_log, dtype=float)
    gains_arr = np.asarray(gains_log, dtype=float)

    # 跟踪误差：只统计位置（用解析参考轨迹对齐）
    err = np.zeros(alt_arr.size)
    for i, t in enumerate(times_arr):
        p_d, _, _, _ = TJ.reference(trajectory, float(t))
        err[i] = float(np.linalg.norm(positions_arr[i] - p_d))

    return EpisodeResult(
        times=times_arr, positions=positions_arr, altitudes=alt_arr,
        desired=desired_arr, u_ls=u_ls_arr,
        u_star=(np.asarray(u_star_log, dtype=float) if u_star_log
                else np.zeros((0, 4), dtype=float)),
        features=np.asarray(feats, dtype=np.float32),
        targets=np.asarray(targets, dtype=np.float32), gains=gains_arr,
        plan=plan, trajectory=trajectory, crashed=crashed,
        crash_time=crash_time, error=err,
    )


#: 生成数据集时用的工况（故障 × 轨迹），刻意覆盖"接近饱和"的情形
DATASET_COMBOS = (
    (FL.FaultPlan(kind="healthy"), "hover"),
    (FL.FaultPlan(kind="healthy"), "zigzag"),
    (FL.FaultPlan(kind="loss", rotor=0, onset=2.0), "hover"),
    (FL.FaultPlan(kind="loss", rotor=2, onset=1.5), "circle"),
    (FL.FaultPlan(kind="degradation", rotor=1, onset=2.0, severity=0.5), "zigzag"),
    (FL.FaultPlan(kind="degradation", rotor=3, onset=1.5, severity=0.3), "circle"),
    (FL.FaultPlan(kind="stuck", rotor=1, onset=2.0), "hover"),
    (FL.FaultPlan(kind="stuck", rotor=3, onset=1.5), "zigzag"),
    (FL.FaultPlan(kind="dual_loss", rotor=0, rotor2=2, onset=2.0), "hover"),
    (FL.FaultPlan(kind="progressive", rotor=2, onset=1.0, severity=0.4,
                  ramp=0.0), "climb"),
)


def build_dataset(api, n_episodes: int = 30, seconds: float = 6.0,
                  sample_stride: int = 4, seed: int = 0,
                  progress=None) -> dict:
    """跑 ``n_episodes`` 次飞行，收集 (特征, 标签)。"""
    params, effectiveness = build_context(api)
    rng = np.random.default_rng(int(seed))
    feats, targets, kinds, scenes, episode_ids = [], [], [], [], []
    n_crash = 0

    for i in range(int(n_episodes)):
        plan, traj = DATASET_COMBOS[int(rng.integers(len(DATASET_COMBOS)))]
        # 同一个组合也抖一下 onset，避免网络只认某个固定时刻
        jittered = FL.FaultPlan(**{**plan.__dict__,
                                   "onset": max(0.5, plan.onset + float(rng.uniform(-0.3, 0.3)))})
        ep = run_episode(api, jittered, trajectory=traj, seconds=seconds,
                         seed=int(rng.integers(1000)), collect=True,
                         sample_stride=sample_stride,
                         params=params, effectiveness=effectiveness)
        if ep.crashed:
            n_crash += 1
        if len(ep.features):
            feats.append(ep.features)
            targets.append(ep.targets)
            kinds.append(np.full(len(ep.features), jittered.kind, dtype=object))
            scenes.append(np.full(len(ep.features), traj, dtype=object))
            episode_ids.append(np.full(len(ep.features), i, dtype=np.int32))
        if progress is not None:
            progress(i + 1, n_episodes, jittered.kind, traj, ep.crashed)

    if not feats:
        raise RuntimeError("没有采到任何样本，检查故障/轨迹配置")

    x = np.concatenate(feats).astype(np.float32)
    y = np.concatenate(targets).astype(np.float32)
    return {
        "X": x,
        "Y": y,
        "kind": np.concatenate(kinds),
        "trajectory": np.concatenate(scenes),
        "episode": np.concatenate(episode_ids),
        "episodes": int(n_episodes),
        "crashed_episodes": int(n_crash),
        "feature_names": list(F.FEATURE_NAMES),
        "seconds": float(seconds),
        "sample_stride": int(sample_stride),
        "max_thrust": float(params.max_thrust),
        "hover_thrust": float(params.hover_thrust),
    }


def save_dataset(path, dataset: dict) -> None:
    """存成 npz；字符串数组与元信息单独打包，读回来还能用。"""
    np.savez_compressed(
        path,
        X=dataset["X"], Y=dataset["Y"],
        kind=np.asarray(dataset["kind"], dtype=str),
        trajectory=np.asarray(dataset["trajectory"], dtype=str),
        episode=np.asarray(dataset.get("episode", []), dtype=np.int32),
        meta=np.array([str(dataset.get("episodes", 0)),
                       str(dataset.get("crashed_episodes", 0)),
                       str(dataset.get("seconds", 0.0)),
                       str(dataset.get("sample_stride", 1)),
                       ",".join(dataset.get("feature_names", [])),
                       str(dataset.get("max_thrust", 6.0)),
                       str(dataset.get("hover_thrust", 0.0))]),
    )


def load_dataset(path) -> dict:
    with np.load(path, allow_pickle=False) as data:
        meta = [str(v) for v in data["meta"]]
        episode = (np.asarray(data["episode"], dtype=np.int32)
                   if "episode" in data.files else np.arange(data["X"].shape[0]))
        return {
            "X": np.asarray(data["X"], dtype=np.float32),
            "Y": np.asarray(data["Y"], dtype=np.float32),
            "kind": np.asarray(data["kind"], dtype=str),
            "trajectory": np.asarray(data["trajectory"], dtype=str),
            "episode": episode,
            "episodes": int(meta[0]),
            "crashed_episodes": int(meta[1]),
            "seconds": float(meta[2]),
            "sample_stride": int(meta[3]),
            "feature_names": meta[4].split(",") if meta[4] else [],
            "max_thrust": float(meta[5]) if len(meta) > 5 else 6.0,
            "hover_thrust": float(meta[6]) if len(meta) > 6 else 0.0,
        }


def split_by_episode(dataset: dict, fractions=(0.8, 0.1, 0.1), seed: int = 0):
    """按**飞行批次**切分，而不是按样本随机切。

    同一次飞行里相邻样本几乎一样，按样本切会让测试集和训练集共享同一条轨迹，
    指标会虚高。这里先把 episode 打乱，再按 80/10/10 分配 episode。
    """
    ids = np.asarray(dataset.get("episode", np.arange(dataset["X"].shape[0])))
    unique = np.unique(ids)
    rng = np.random.default_rng(int(seed))
    order = rng.permutation(unique.size)
    n_train = int(round(fractions[0] * unique.size))
    n_val = int(round(fractions[1] * unique.size))
    groups = {
        "train": set(unique[order[:n_train]].tolist()),
        "val": set(unique[order[n_train:n_train + n_val]].tolist()),
        "test": set(unique[order[n_train + n_val:]].tolist()),
    }
    if not groups["test"] and n_train > 0:      # episode 太少时至少留一个给测试
        groups["test"] = {int(unique[order[-1]])}
    masks = {name: np.isin(ids, np.fromiter(vals, dtype=ids.dtype, count=len(vals)))
             for name, vals in groups.items()}
    return masks


#: 旧名字保留，语义已改为"按批次切分"
split_by_kind = split_by_episode
