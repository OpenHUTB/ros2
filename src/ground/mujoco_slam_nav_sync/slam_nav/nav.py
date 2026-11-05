"""闭环导航：SLAM 位姿 -> 栅格 -> A* -> 神经网络局部规划 -> 小车。

两种模式（这就是"导航与 SLAM 同步仿真"的对照）：

* `use_slam=True`：位姿来自 EKF-SLAM（里程计预测 + 路标观测更新），地图用 SLAM 位姿构建；
* `use_slam=False`：纯里程计航迹推算（dead reckoning），地图用漂移位姿构建。

里程计轮滑率 `slip` 可调，用来量化"定位漂移对导航成功率的影响"。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .env import DiffDriveEnv, LIDAR_RANGE, wrap
from .net import features
from .planner import astar, lookahead_point, lookahead_points, pure_pursuit
from .slam import GRID_X0, GRID_Y0, OccupancyGrid, detect_landmarks, EKFSLAM


@dataclass
class NavConfig:
    v_max: float = 0.7
    w_max: float = 1.5
    ld: float = 0.9                 # 前视距离
    goal_tol: float = 0.45
    steps: int = 1200               # 20 Hz -> 60 s
    replan_every: int = 10
    safe_dist: float = 0.7         # 正前方小于此距离就降速


@dataclass
class NavResult:
    mode: str
    slip: float
    success: bool
    crashed: bool
    steps: int
    ate_rmse: float
    final_dist: float
    path_length: float
    map_known: float
    frames: list = field(default_factory=list)
    trace: list = field(default_factory=list)
    grid: object = None                 # 在线构建的占据栅格（用于绘图/评价）


def _front_clearance(scan: np.ndarray) -> float:
    n = len(scan)
    k = max(1, n // 18)                       # 正前方 ±20°
    idx = np.concatenate([np.arange(0, k), np.arange(n - k, n)])
    return float(np.min(scan[idx]))


def govern(scan, v, w, cfg: NavConfig):
    """安全护栏：按正前方 60° 扇区的最短距离做速度/转向修正（TTC 思路）。

    神经网络只学"专家怎么走"，不保证不撞；这里给它套一个只依赖雷达的护栏，
    既保留网络输出，又保证碰撞率不被策略噪声放大。
    """
    n = len(scan)
    k = max(2, n // 12)                    # ±30°
    left, right = scan[1:k + 1], scan[n - k:]
    fmin = float(min(left.min(), right.min()))
    if fmin >= cfg.safe_dist:
        return v, w
    turn = 0.9 if left.min() > right.min() else -0.9
    if fmin < 0.45:
        return 0.0, float(turn)
    scale = max(0.2, fmin / cfg.safe_dist)
    return float(v * scale * 0.6), float(np.clip(0.6 * w + 0.5 * turn, -cfg.w_max, cfg.w_max))


def expert_action(scan, pose, path, cfg: NavConfig, goal):
    target = lookahead_point(path, pose, cfg.ld) or goal
    v, w = pure_pursuit(pose, target, cfg.v_max, cfg.w_max)
    front = _front_clearance(scan)
    if front < cfg.safe_dist:                 # 前方贴脸：降速并让纯追踪接管转向
        v *= max(0.0, front / cfg.safe_dist)
    return float(v), float(w), target


def build_truth_grid(env: DiffDriveEnv, res: float = 0.1, w: int = 140, h: int = 140):
    """把墙与柱子栅格化成"真值占据图"（只用于生成专家数据与评价建图质量）。"""
    occ = np.zeros((h, w), dtype=bool)
    for name in ("wall_n", "wall_s", "wall_w", "wall_e", "wall_mid_a", "wall_mid_b", "wall_mid_c"):
        gid = env.model.geom(name).id
        px, py, _ = env.model.geom_pos[gid]
        sx, sy = env.model.geom_size[gid][:2]
        x0, x1 = px - sx, px + sx
        y0, y1 = py - sy, py + sy
        cx0, cx1 = int((x0 - GRID_X0) / res), int((x1 - GRID_X0) / res)
        cy0, cy1 = int((y0 - GRID_Y0) / res), int((y1 - GRID_Y0) / res)
        cx0, cx1 = max(0, cx0), min(w - 1, cx1)
        cy0, cy1 = max(0, cy0), min(h - 1, cy1)
        if cy1 >= cy0 and cx1 >= cx0:
            occ[cy0:cy1 + 1, cx0:cx1 + 1] = True
    for gid in env.lm_ids:
        px, py, _ = env.model.geom_pos[gid]
        r = env.model.geom_size[gid][0]
        cx, cy = int((px - GRID_X0) / res), int((py - GRID_Y0) / res)
        rr = int(np.ceil(r / res))
        for dy in range(-rr, rr + 1):
            for dx in range(-rr, rr + 1):
                if dx * dx + dy * dy <= rr * rr:
                    x, y = cx + dx, cy + dy
                    if 0 <= x < w and 0 <= y < h:
                        occ[y, x] = True
    return occ


def inflate(occ: np.ndarray, cells: int = 2) -> np.ndarray:
    out = occ.copy()
    for dy in range(-cells, cells + 1):
        for dx in range(-cells, cells + 1):
            if dx * dx + dy * dy > cells * cells:
                continue
            out |= np.roll(np.roll(occ, dy, axis=0), dx, axis=1)
    return out


def make_expert_dataset(env: DiffDriveEnv, n: int = 1200, seed: int = 0,
                        cfg: NavConfig | None = None, log=print):
    """随机位姿 -> A* 路径 -> 纯追踪专家动作，得到 (X, Y) 供神经网络模仿。"""
    cfg = cfg or NavConfig()
    scene0, seed0 = env.scene, env.seed
    pose0 = env.state().pos.copy()
    yaw0 = env.state().yaw
    rng = np.random.default_rng(seed)
    truth = inflate(build_truth_grid(env), 2)
    xs, ys = [], []
    tries = 0
    while len(xs) < n and tries < n * 20:
        tries += 1
        px = float(rng.uniform(0.8, 11.2))
        py = float(rng.uniform(-5.2, 5.2))
        yaw = float(rng.uniform(-np.pi, np.pi))
        env.reset(scene=env.scene, seed=env.seed, pos=(px, py), yaw=yaw)
        scan = env.scan()
        if np.min(scan) < 0.45:               # 太贴墙的位置丢掉
            continue
        path = astar(truth, (px, py), DiffDriveEnv.GOAL)
        if not path:
            continue
        pose = np.array([px, py, wrap(yaw)])
        # 目标用"路径前视点"，让网络学到局部反应而不是背地图
        targets = lookahead_points(path, pose, cfg.ld + 0.4)
        if not targets:
            continue
        v, w, _ = expert_action(scan, pose, path, cfg, DiffDriveEnv.GOAL)
        xs.append(features(scan, pose, targets, DiffDriveEnv.GOAL, cfg.v_max))
        ys.append([v / cfg.v_max, w / cfg.w_max])
    log(f"专家数据: {len(xs)} 条（尝试 {tries} 次）")
    env.reset(scene=scene0, seed=seed0, pos=pose0, yaw=yaw0)   # 采样会改位姿，必须还原
    return np.stack(xs).astype(np.float32), np.stack(ys).astype(np.float32)


def run_episode(env: DiffDriveEnv, net=None, use_slam: bool = True, slip: float = 0.0,
                odom_noise: float = 0.003, seed: int = 0, scene: str = "maze",
                cfg: NavConfig | None = None, record: bool = False,
                collect: list | None = None) -> NavResult:
    cfg = cfg or NavConfig()
    env.reset(scene=scene, seed=seed)
    # 每个回合用独立随机源：同 (scene, seed, slip) 必须复现同一条轨迹
    env.rng = np.random.default_rng(10007 * (seed + 1) + int(slip * 1000))
    truth0 = env.state().pos.copy()
    grid = OccupancyGrid()
    ekf = EKFSLAM(init_pose=(truth0[0], truth0[1], 0.0))
    dead_pose = np.array([truth0[0], truth0[1], 0.0])

    path = None
    ate, traces, frames = [], [], []
    path_len, step = 0.0, 0
    prev = truth0.copy()
    stuck = 0
    w_filt = 0.0                      # 角速度一阶低通，抑制网络输出的抖动

    for step in range(cfg.steps):
        scan = env.scan()
        truth = env.state()

        if use_slam:
            dets = detect_landmarks(scan)
            ekf.observe(dets)
            pose = ekf.pose
        else:
            pose = dead_pose.copy()

        grid.update(pose, scan)

        if step % cfg.replan_every == 0 or path is None:
            path = astar(grid.occupancy_for_planning(3), pose[:2], DiffDriveEnv.GOAL)

        if path:
            targets = lookahead_points(path, pose, cfg.ld)
            target = targets[0] if targets else path[-1]
            if net is not None:
                v, w = net.act(scan, pose, targets, cfg.v_max, cfg.w_max,
                               goal=DiffDriveEnv.GOAL)
                w_filt = 0.5 * w_filt + 0.5 * w
                w = w_filt
                v, w = govern(scan, v, w, cfg)      # 网络不管安全，护栏兜底
            else:
                v, w, _ = expert_action(scan, pose, path, cfg, DiffDriveEnv.GOAL)
                v, w = govern(scan, v, w, cfg)
            if collect is not None:
                # DAgger：在"网络自己走出来的状态"上问专家该怎么做
                ev, ew, _ = expert_action(scan, pose, path, cfg, DiffDriveEnv.GOAL)
                collect.append((features(scan, pose, targets, DiffDriveEnv.GOAL, cfg.v_max),
                                [ev / cfg.v_max, ew / cfg.w_max]))
        else:
            v, w, target = 0.0, 0.6, None       # 暂时没路径：原地转找视野

        if record and step % 2 == 0 and target is not None:
            frames.append((env.rgb("chase_cam"), scan.copy(), pose.copy(),
                           float(np.linalg.norm(truth.pos - DiffDriveEnv.GOAL)),
                           grid.logodds.copy(), truth.pos.copy()))

        # 卡死恢复：网络可能输出过小的 v，连续 15 步几乎不动就短暂推一下
        if abs(v) < 0.05:
            stuck += 1
        else:
            stuck = 0
        if stuck > 15 and _front_clearance(scan) > 0.6:
            v, w = 0.35, 0.0
            stuck = 0

        state = env.step(v, w)
        # 里程计推进（两种模式都用同一个漂移里程计）
        v_o, w_o = env.odom(v, w, slip, odom_noise)
        dead_pose[0] += v_o * np.cos(dead_pose[2]) * cfg_steps_dt()
        dead_pose[1] += v_o * np.sin(dead_pose[2]) * cfg_steps_dt()
        dead_pose[2] = wrap(dead_pose[2] + w_o * cfg_steps_dt())
        if use_slam:
            ekf.predict(v_o, w_o, cfg_steps_dt())

        ate.append(float(np.linalg.norm(pose[:2] - state.pos)))
        path_len += float(np.linalg.norm(state.pos - prev))
        prev = state.pos.copy()
        traces.append((state.pos.copy(), pose[:2].copy()))

        if state.crashed:
            break
        if np.linalg.norm(state.pos - DiffDriveEnv.GOAL) < cfg.goal_tol:
            break

    final_dist = float(np.linalg.norm(env.state().pos - DiffDriveEnv.GOAL))
    return NavResult(
        mode="slam" if use_slam else "dead-reckoning",
        slip=slip, success=final_dist < cfg.goal_tol and not env.state().crashed,
        crashed=bool(env.state().crashed), steps=step + 1,
        ate_rmse=round(float(np.sqrt(np.mean(np.square(ate)))), 3),
        final_dist=round(final_dist, 2),
        path_length=round(path_len, 2),
        map_known=round(grid.known_ratio(), 3),
        frames=frames, trace=traces, grid=grid,
    )


def cfg_steps_dt() -> float:
    from .env import DT
    return DT
