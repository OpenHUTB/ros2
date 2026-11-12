"""离线闭环仿真：rollout / evaluate / 数据采集（BC + DAgger）。

离线闭环与 ROS 闭环用的是同一套模块，区别只在于谁来推进仿真时间：
这里是本进程的循环，ROS 里是 sim_node 的定时器。
"""

import math
import time

import numpy as np

from . import camera, detector
from . import params as P
from .controllers import goal_reached
from .gridmap import truth_grid
from .kinematics import DiffDrive, wrap
from .policies import E2EPolicy, goal_channel, odom_delta
from .world import make_scene

FRAME_W, FRAME_H = E2EPolicy.IN_W, E2EPolicy.IN_H


def _obs(world, drive, rgb, depth, lms, wall, prev_odom, t):
    return {
        'rgb': rgb,
        'depth': depth,
        'landmarks': lms,
        'wall_pts': wall,
        'odom': drive.odom.copy(),
        'odom_delta': odom_delta(prev_odom, drive.odom),
        'true_pose': drive.pose.copy(),
        'goal': world.goal.copy(),
        't': t,
        'scene': getattr(world, 'key', ''),
        'dyn_center': world.dynamic_center(),
        'dyn_radius': float(world.dynamic[2]) if world.dynamic is not None else 0.0,
    }


def rollout(scene_key, policy, seed=0, max_steps=None, record=False,
            dagger_oracle=None, beta=0.0, on_frame=None):
    """跑一个 episode，返回指标字典（可选返回逐步记录）。"""
    world = make_scene(scene_key, seed)
    world.key = scene_key
    rng = np.random.default_rng(seed * 7919 + 13)
    drive = DiffDrive(world.start)
    policy.reset(world)
    if dagger_oracle is not None:
        dagger_oracle.reset(world)

    night = scene_key == 'night'
    max_steps = max_steps or int(P.ARRIVE_TIME_LIMIT / P.DT)
    prev_odom = drive.odom.copy()

    traj, actions, frames, labels, depths, odoms = [], [], [], [], [], []
    collided = False
    n_est_err = []
    est_valid = 0

    t = 0.0
    for step in range(max_steps):
        rgb, depth = camera.render(world, drive.pose, rng, night=night)
        lms, wall = detector.detect(rgb, depth)
        obs = _obs(world, drive, rgb, depth, lms, wall, prev_odom, t)

        v, w = policy.act(obs)

        if dagger_oracle is not None:
            # DAgger：标签永远是专家动作，执行的是 student 与专家的混合动作
            ev, ew = dagger_oracle.act(obs)
            if record:
                frames.append(camera.resize_gray(camera.to_gray(rgb), FRAME_W, FRAME_H))
                depths.append(camera.resize_gray(
                    np.clip(depth / P.CAM_RANGE_MAX, 0.0, 1.0), FRAME_W, FRAME_H))
                odoms.append(drive.odom.copy())
                labels.append([ev / P.MAX_V, ew / P.MAX_W])
            v = beta * ev + (1.0 - beta) * v
            w = beta * ew + (1.0 - beta) * w
        elif record:
            # 行为克隆：策略本身就是专家，动作即标签
            frames.append(camera.resize_gray(camera.to_gray(rgb), FRAME_W, FRAME_H))
            depths.append(camera.resize_gray(
                np.clip(depth / P.CAM_RANGE_MAX, 0.0, 1.0), FRAME_W, FRAME_H))
            odoms.append(drive.odom.copy())
            labels.append([v / P.MAX_V, w / P.MAX_W])

        if on_frame is not None:
            on_frame(world, drive.pose.copy(), rgb, policy, t)

        prev_odom = drive.odom.copy()
        drive.step(v, w, P.DT, rng, slip_scale=1.10 if scene_key == 'night' else 1.0)
        world.step(P.DT)
        t += P.DT

        traj.append(drive.pose.copy())
        actions.append([v, w])

        if not world.point_free(drive.pose[0], drive.pose[1],
                                margin=P.ROBOT_RADIUS):
            collided = True
            break

        # 定位误差（只有带 slam 属性的策略才有）
        est = getattr(policy, 'slam', None)
        if est is not None:
            e = math.hypot(est.mu[0] - drive.pose[0], est.mu[1] - drive.pose[1])
            n_est_err.append(e)
            est_valid += int(e < P.ERR_OK)

        if goal_reached(drive.pose, world.goal):
            break

    goal = goal_reached(drive.pose, world.goal)
    traj = np.asarray(traj).reshape(-1, 3)
    path_len = float(np.sum(np.hypot(np.diff(traj[:, 0]), np.diff(traj[:, 1])))) if len(traj) > 1 else 0.0

    out = {
        'scene': scene_key,
        'policy': getattr(policy, 'name', 'unknown'),
        'seed': seed,
        'success': bool(goal and not collided),
        'collided': bool(collided),
        'time_s': t,
        'path_len': path_len,
        'n_steps': len(traj),
        'final_dist': float(math.hypot(drive.pose[0] - world.goal[0],
                                       drive.pose[1] - world.goal[1])),
        'ate_rmse': float(np.sqrt(np.mean(np.square(n_est_err)))) if n_est_err else float('nan'),
        'loc_valid': float(est_valid / max(1, len(n_est_err))) if n_est_err else float('nan'),
        'mean_speed': float(np.mean([a[0] for a in actions])) if actions else 0.0,
    }

    gm = getattr(policy, 'grid', None)
    if gm is not None:
        out['map_iou'] = gm.iou(truth_grid(world))
        out['coverage'] = gm.coverage()
        out['n_landmarks'] = float(len(policy.slam.ids)) if hasattr(policy, 'slam') else 0.0

    if record:
        out['traj'] = traj
        out['frames'] = np.asarray(frames, dtype=np.float32)
        out['depths'] = np.asarray(depths, dtype=np.float32)
        out['odoms'] = np.asarray(odoms, dtype=np.float64).reshape(-1, 3)
        out['goal'] = np.asarray(world.goal, dtype=np.float64)
        out['labels'] = np.asarray(labels, dtype=np.float32)
        out['actions'] = np.asarray(actions, dtype=np.float32)
        out['world'] = world
        out['slam_map'] = gm
    return out


def evaluate(kind, scenes=P.SCENES, seeds=(0, 1, 2), net=None):
    """多场景 × 多种子评测，返回逐条记录。"""
    rows = []
    for sc in scenes:
        for sd in seeds:
            from .policies import make_policy
            pol = make_policy(kind, make_scene(sc, sd), net=net)
            r = rollout(sc, pol, seed=sd)
            rows.append(r)
    return rows


def collect_demo(scene_keys, seeds, steps_limit=None):
    """用 Oracle 专家采集行为克隆数据（按 episode 返回帧序列与动作）。"""
    from .policies import make_policy
    eps = []
    for sc in scene_keys:
        for sd in seeds:
            world = make_scene(sc, sd)
            pol = make_policy('oracle', world)
            r = rollout(sc, pol, seed=sd, record=True,
                        max_steps=steps_limit or int(P.ARRIVE_TIME_LIMIT / P.DT))
            if len(r['frames']) < 5:
                continue
            eps.append({'frames': r['frames'], 'depths': r['depths'],
                        'odoms': r['odoms'], 'goal': r['goal'],
                        'actions': r['actions'],
                        'scene': sc, 'seed': sd,
                        'success': r['success']})
    return eps


def stack_dataset(eps, frames_n=3):
    """把 episode 序列展开成 (x, y) 监督对：N 帧灰度 + 当前帧深度 + 目标方位。"""
    xs, ys = [], []
    for ep in eps:
        f = ep['frames']
        a = ep['actions']
        d = ep.get('depths')
        od = ep.get('odoms')
        gl = ep.get('goal')
        T = len(f)
        for i in range(T):
            idx = [max(0, i - k) for k in range(frames_n - 1, -1, -1)]
            gray = (f[idx] - 0.45) / 0.25
            dep = (d[i][None] if d is not None else np.zeros((1, FRAME_H, FRAME_W), np.float32))
            gch = goal_channel(od[i] if od is not None and i < len(od) else None,
                               gl, FRAME_W, FRAME_H)
            xs.append(np.concatenate([gray, dep, gch[None]], axis=0))
            ys.append([a[i, 0] / P.MAX_V, a[i, 1] / P.MAX_W])
    if not xs:
        return (np.zeros((0, frames_n + 2, FRAME_H, FRAME_W), np.float32),
                np.zeros((0, 2), np.float32))
    return np.asarray(xs, dtype=np.float32), np.asarray(ys, dtype=np.float32)
