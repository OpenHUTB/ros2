"""闭环：专家 / 延迟线 / 四种延迟补偿 / 指标统计。

延迟在本模块里的定义是**执行延迟**：第 t 帧算出的 (v,ω) 要等 d 帧才真正落到车上。
这是端到端系统里最常见的一类延迟（感知推理 + 通信 + 执行器排队），也是纯神经网络
策略最怕的东西——策略看到的永远是"过去的场景"。
"""
from __future__ import annotations

import time

import numpy as np

from .env import DT, LIDAR_RANGE, SUBSTEPS, LatencyEnv, _wrap

V_REF = 1.0
W_MAX = 1.6
LD = 1.0                 # 纯追踪预瞄距离
COLLIDE_CLEARANCE = 0.22  # ROBOT_RADIUS 0.18 m + 4 cm 余量（与 env.ROBOT_RADIUS 同步）


class DelayLine:
    """长度为 d 的执行延迟队列：入队 d 帧后才会出现在出水端。"""

    def __init__(self, ticks: int, init: tuple[float, float] = (0.0, 0.0)):
        self.n = int(ticks)
        self.buf = [init] * self.n if self.n > 0 else []

    def push(self, cmd: tuple[float, float]) -> tuple[float, float]:
        if self.n <= 0:
            return cmd
        self.buf.append(cmd)
        return self.buf.pop(0)

    def pending(self) -> list[tuple[float, float]]:
        """还在飞行中的指令（先入先出顺序）。"""
        return list(self.buf)


def pure_pursuit(pose: np.ndarray, path: np.ndarray, idx: int, ld: float = LD,
                 v_ref: float = V_REF) -> tuple[float, float]:
    """按弧长找预瞄点，回一个 (v, ω)。"""
    n = len(path)
    step = float(np.mean(np.linalg.norm(np.diff(path, axis=0), axis=1)))
    k = max(1, int(round(ld / max(step, 1e-6))))
    tgt = path[(idx + k) % n]
    alpha = _wrap(float(np.arctan2(tgt[1] - pose[1], tgt[0] - pose[0])) - pose[2])
    w = 2.0 * v_ref * np.sin(alpha) / max(ld, 1e-6)
    return v_ref, float(np.clip(w, -W_MAX, W_MAX))


def integrate(pose: np.ndarray, v: float, w: float, dt: float) -> np.ndarray:
    """单轨模型前向积分一步（补偿器的内部模型）。"""
    yaw = pose[2] + w * dt
    x = pose[0] + v * np.cos(pose[2] + 0.5 * w * dt) * dt
    y = pose[1] + v * np.sin(pose[2] + 0.5 * w * dt) * dt
    return np.array([x, y, _wrap(yaw)])


def predict_pose(pose: np.ndarray, pending: list[tuple[float, float]], dt: float) -> np.ndarray:
    """模型前向 d 步：把"还在飞行中的指令"依次施加到内部模型上。

    第 t 帧下的指令会在 t+d 帧生效，所以用 x_t 加上排队中的 d 条指令，就得到
    指令真正落到车上时车会在哪里——这正是 Smith 预测器的核心。
    """
    x = pose.copy()
    for v, w in pending:
        x = integrate(x, v, w, dt)
    return x


def warp_scan(scan: np.ndarray, pose_from: np.ndarray, pose_to: np.ndarray,
              beam_angles: np.ndarray, max_range: float = LIDAR_RANGE) -> np.ndarray:
    """把 t 时刻的扫描按位姿差**刚体变换**到预测位姿上（真车做得到的做法）。

    做法：把每束的量程还原成世界系点云，再变换到目标位姿的极坐标里重新分箱，
    每个扇区取最小距离。没有点落进来的扇区填 `max_range`——这正是这种做法的
    两个已知缺陷：**看不到新露出来的几何**（遮挡关系被忽略，偏乐观），
    以及扇区边缘的重采样误差。它不需要地图、也不需要仿真器，因此是可上车的。
    """
    ang = pose_from[2] + beam_angles
    pts = pose_from[:2] + scan[:, None] * np.stack([np.cos(ang), np.sin(ang)], axis=1)

    rel = pts - pose_to[:2]
    c, s = np.cos(-pose_to[2]), np.sin(-pose_to[2])
    x = c * rel[:, 0] - s * rel[:, 1]
    y = s * rel[:, 0] + c * rel[:, 1]

    rng = np.hypot(x, y)
    bearing = np.arctan2(y, x)

    # 只有 24 个点：直接按扇区分箱会漏掉一半扇区（平移 0.3 m 就能让相邻束错开 17°，
    # 大于 15° 的角分辨率）。所以把 r(θ) 看成连续曲线，按极角线性插值再采样——
    # 这也是真车做扫描变换时的标准做法（会导致"把角点插成斜面"的误差，写进文档）。
    order = np.argsort(bearing)
    b_sorted = bearing[order]
    r_sorted = rng[order]
    b_ext = np.concatenate([b_sorted, b_sorted[:1] + 2.0 * np.pi])
    r_ext = np.concatenate([r_sorted, r_sorted[:1]])

    target = (beam_angles - b_sorted[0]) % (2.0 * np.pi) + b_sorted[0]
    return np.clip(np.interp(target, b_ext, r_ext), 0.02, max_range)


def query_pose(env: LatencyEnv, pose: np.ndarray, v: float, w: float,
               pending: list[tuple[float, float]], dt: float, comp: str) -> np.ndarray:
    """策略"以为自己在哪"的那个位姿。"""
    if comp in ("none",):
        return pose
    if comp == "extrap":
        # 匀速外推：假设 v、ω 在 d 帧内不变
        d = len(pending)
        return integrate(pose, v, w, d * dt)
    if comp in ("model", "oracle"):
        # model 与 oracle 用同一个预测位姿，区别只在"在哪儿取观测"，见 observation_at()
        return predict_pose(pose, pending, dt)
    raise ValueError(f"未知补偿方式 {comp}")


def scan_at(env: LatencyEnv, pose: np.ndarray,
            rng: np.random.Generator | None = None) -> np.ndarray:
    """在给定位姿上打一圈射线（不推进物理）。"""
    ang = pose[2] + env.beam_angles
    r = env.raycast(pose, ang)
    if rng is not None:
        r = r + rng.normal(0.0, env.sensor_noise, size=r.shape)
    return np.clip(r, 0.02, LIDAR_RANGE)


def features(scan: np.ndarray, v: float, w_prev: float) -> np.ndarray:
    """24 束归一化距离 + 当前速度 + 上一帧角速度 = 26 维。"""
    return np.concatenate([scan / LIDAR_RANGE, [v / 2.0, w_prev / W_MAX]]).astype(np.float32)


def run_episode(env: LatencyEnv, policy=None, comp: str = "none", delay_ms: float = 0.0,
                rate_hz: float = 20.0, max_time: float = 60.0, seed: int = 0,
                record: bool = False) -> dict:
    """跑一圈。policy=None 时跑真值纯追踪（作为上界参考）。"""
    env.reset(seed=seed)
    dt = 1.0 / rate_hz
    sub = max(1, int(round(dt / (DT / SUBSTEPS))))
    ticks = int(round(delay_ms / 1000.0 * rate_hz))
    delay = DelayLine(ticks, init=(0.0, 0.0))
    rng = np.random.default_rng(seed + 7)

    n = len(env.path)
    idx = env.nearest_index(env.pose()[:2])
    start_idx = idx
    errs, clear, frames = [], [], []
    w_prev = 0.0
    steps = int(round(max_time / dt))
    collide = 0
    completed = False
    t0 = time.perf_counter()
    cpu = 0.0

    for _ in range(steps):
        pose = env.pose()
        v, w = env.velocity()
        scan_now = scan_at(env, pose, rng)
        clear.append(float(scan_now.min()))
        if scan_now.min() < COLLIDE_CLEARANCE:
            collide += 1

        if policy is None:
            idx = env.nearest_index(pose[:2], idx)
            v_cmd, w_cmd = pure_pursuit(pose, env.path, idx)
        else:
            q = query_pose(env, pose, v, w, delay.pending(), dt, comp)
            if comp == "oracle":
                # 上界：直接问仿真器"那一刻那个位置的完整扫描"（需要精确地图/仿真器，真车做不到）
                scan_q = scan_at(env, q, rng)
            elif comp == "none":
                scan_q = scan_now
            else:
                # extrap / model：手上只有当前帧扫描，按位姿差做刚体变换（真车做得到）
                scan_q = warp_scan(scan_now, pose, q, env.beam_angles)
            feat = features(scan_q, v, w_prev)
            t1 = time.perf_counter()
            v_cmd, w_cmd = policy.act(feat)
            cpu += time.perf_counter() - t1

        v_app, w_app = delay.push((float(v_cmd), float(w_cmd)))
        if record:
            frames.append(pose.copy())
        env.step_period(v_app, w_app, sub)
        w_prev = w_cmd

        e, _ = env.errors()
        errs.append(abs(e))
        idx = env.nearest_index(env.pose()[:2], idx)
        travelled = (idx - start_idx) % n
        if travelled >= 0.97 * n:
            completed = True
            break

    elapsed = time.perf_counter() - t0
    errs = np.array(errs)
    return {
        "rms": float(np.sqrt(np.mean(errs**2))),
        "max": float(errs.max()),
        "completed": bool(completed),
        "progress": float(((idx - start_idx) % n) / n),
        "collide_frames": int(collide),
        "min_clear": float(np.min(clear)),
        "steps": len(errs),
        "mean_cpu_ms": 1000.0 * cpu / max(1, len(errs)),
        "wall_s": elapsed,
        "frames": frames,
    }
