"""离线闭环仿真：rollout / evaluate / 数据采集（BC + DAgger）。

离线闭环与 ROS 闭环用的是同一套模块，区别只在于谁来推进仿真时间：
这里是本进程的循环，ROS 里是 sim_node 的定时器。

任务是往返巡检：A→B→A。结束条件：回到 A（成功）、压管/撞应答器
（碰撞）或超时。
"""

import math
import time

import numpy as np

from . import camera, detector
from . import params as P
from .controllers import goal_reached
from .gridmap import pipeline_mask, truth_grid
from .kinematics import RovDrive, wrap
from .policies import (E2EPolicy, depth_feature, frame_gray, goal_channel,
                       net_gray, odom_delta)
from .world import make_scene

FRAME_W, FRAME_H = E2EPolicy.IN_W, E2EPolicy.IN_H


def _obs(world, rov, rgb, depth, lms, pipe, prev_odom, t):
    return {
        'rgb': rgb,
        'depth': depth,
        'landmarks': lms,
        'pipe_pts': pipe,
        'odom': rov.odom.copy(),
        'odom_delta': odom_delta(prev_odom, rov.odom),
        'true_pose': rov.pose.copy(),
        'goal': world.goal.copy(),
        't': t,
        'scene': getattr(world, 'key', ''),
    }


def rollout(scene_key, policy, seed=0, max_steps=None, record=False,
            dagger_oracle=None, beta=0.0, on_frame=None, start_offset=None):
    """跑一个 episode，返回指标字典（可选返回逐步记录）。

    start_offset=(dlat, dth) 给起点叠加横向/艏向偏置，只用于数据采集
    造"非标称状态"样本（评测一律用 None，保证与标称起点可比）。
    """
    world = make_scene(scene_key, seed)
    world.key = scene_key
    rng = np.random.default_rng(seed * 7919 + 13)
    rov = RovDrive(world.start)
    if start_offset is not None:
        dlat, dth = start_offset
        start = world.start_pose(dlat, dth)
        rov.pose = start.copy()
        rov.odom = start.copy()
    policy.reset(world)
    if dagger_oracle is not None:
        dagger_oracle.reset(world)

    max_steps = max_steps or int(P.ARRIVE_TIME_LIMIT / P.DT)
    prev_odom = rov.odom.copy()

    traj, actions, frames, depths, odoms, labels, goals = [], [], [], [], [], [], []
    percep_labels = []
    collided = False
    n_est_err = []
    est_valid = 0
    lat_errs = []
    loop_err = float('nan')

    t = 0.0
    for step in range(max_steps):
        rgb, depth = camera.render(world, rov.pose, rng)
        lms, pipe = detector.detect(rgb, depth)
        obs = _obs(world, rov, rgb, depth, lms, pipe, prev_odom, t)

        # 真值感知量（任务系）：既当专家动作，也当感知网络的训练标签。
        # 横向误差按巡检方向换算 —— 返程艏向反 180°，符号必须翻转，
        # 否则 CNN 学到的是一组自相矛盾的标签。
        pose = rov.pose.copy()
        s, lat, (tx, ty) = world.progress_of(pose[0], pose[1])
        mdir = world.mission_direction(pose[0], pose[1])
        gt_dx = mdir * (P.FOLLOW_OFFSET - lat)
        gt_dth = wrap(math.atan2(mdir * ty, mdir * tx) - pose[2])

        v, w = policy.act(obs)

        if dagger_oracle is not None:
            # DAgger：标签永远是专家动作，执行的是 student 与专家的混合动作
            ev, ew = dagger_oracle.act(obs)
            if record:
                frames.append(frame_gray(rgb, FRAME_W, FRAME_H))
                depths.append(depth_feature(depth, FRAME_W, FRAME_H))
                odoms.append(rov.odom.copy())
                goals.append(world.goal.copy())
                labels.append([ev / P.MAX_V, ew / P.MAX_W])
                percep_labels.append([float(np.clip(gt_dx / 1.2, -1.0, 1.0)),
                                  float(np.clip(gt_dth / 1.5, -1.0, 1.0))])
            v = beta * ev + (1.0 - beta) * v
            w = beta * ew + (1.0 - beta) * w
        elif record:
            # 行为克隆：策略本身就是专家，动作即标签
            frames.append(frame_gray(rgb, FRAME_W, FRAME_H))
            depths.append(depth_feature(depth, FRAME_W, FRAME_H))
            odoms.append(rov.odom.copy())
            goals.append(world.goal.copy())
            labels.append([v / P.MAX_V, w / P.MAX_W])
            percep_labels.append([float(np.clip(gt_dx / 1.2, -1.0, 1.0)),
                                  float(np.clip(gt_dth / 1.5, -1.0, 1.0))])

        if on_frame is not None:
            on_frame(world, rov.pose.copy(), rgb, policy, t)

        prev_odom = rov.odom.copy()
        cur = world.current_vector()
        rov.step(v, w, P.DT, rng, current=cur,
                 slip_scale=1.25 if world.turbid else 1.0)
        world.step(P.DT)
        t += P.DT

        traj.append(rov.pose.copy())
        actions.append([v, w])

        # 横向跟踪误差：相对巡检偏置目标的偏差（不是到管道的距离），
        # 只统计管道本体上方（去掉两端转弯区）
        if 0.5 < s < world.length - 0.5:
            lat_errs.append(abs(lat - P.FOLLOW_OFFSET))

        if not world.point_free(rov.pose[0], rov.pose[1], margin=P.ROBOT_RADIUS * 0.8):
            collided = True
            break

        # 定位误差（只有带 slam 属性的策略才有）
        est = getattr(policy, 'slam', None)
        if est is not None:
            e = math.hypot(est.mu[0] - rov.pose[0], est.mu[1] - rov.pose[1])
            n_est_err.append(e)
            est_valid += int(e < P.ERR_OK)

        # 到 B 换回程；回到 A 结束（按弧长进度判定）
        if world.advance(rov.pose):
            world.reached_b = True
            if est is not None:
                loop_err = float(math.hypot(est.mu[0] - rov.pose[0],
                                            est.mu[1] - rov.pose[1]))
            break

    success = bool(world.done and world.reached_b and not collided)
    traj = np.asarray(traj).reshape(-1, 3)
    path_len = float(np.sum(np.hypot(np.diff(traj[:, 0]), np.diff(traj[:, 1])))) \
        if len(traj) > 1 else 0.0

    out = {
        'scene': scene_key,
        'policy': getattr(policy, 'name', 'unknown'),
        'seed': seed,
        'success': success,
        'collided': bool(collided),
        'reached_b': bool(world.reached_b),
        'time_s': t,
        'path_len': path_len,
        'n_steps': len(traj),
        'final_dist': float(math.hypot(rov.pose[0] - world.end_a[0],
                                       rov.pose[1] - world.end_a[1])),
        'ate_rmse': float(np.sqrt(np.mean(np.square(n_est_err)))) if n_est_err else float('nan'),
        'loc_valid': float(est_valid / max(1, len(n_est_err))) if n_est_err else float('nan'),
        'lat_rmse': float(np.sqrt(np.mean(np.square(lat_errs)))) if lat_errs else float('nan'),
        'loop_err': loop_err,
        'mean_speed': float(np.mean([a[0] for a in actions])) if actions else 0.0,
    }

    gm = getattr(policy, 'grid', None)
    if gm is not None:
        out['map_iou'] = gm.iou(truth_grid(world))
        pm = pipeline_mask(world)
        occ = gm.occupied(th=0.62)
        out['pipe_cov'] = float((occ & pm).sum()) / max(1, int(pm.sum()))
        out['coverage'] = gm.coverage()
        out['n_landmarks'] = float(len(policy.slam.ids))

    if record:
        out['traj'] = traj
        out['frames'] = np.asarray(frames, dtype=np.float32)
        out['depths'] = np.asarray(depths, dtype=np.float32)
        out['odoms'] = np.asarray(odoms, dtype=np.float64).reshape(-1, 3)
        # 注意 goal 是 (x, y) 二维向量（当前任务端点），不要 reshape 成 3 列
        out['goals'] = np.asarray(goals, dtype=np.float64).reshape(-1, 2)
        out['labels'] = np.asarray(labels, dtype=np.float32)
        out['percep_labels'] = np.asarray(percep_labels, dtype=np.float32)
        out['actions'] = np.asarray(actions, dtype=np.float32)
        out['world'] = world
        out['slam_map'] = gm
    return out


def evaluate(kind, scenes=P.SCENES, seeds=(0, 1, 2), net=None, percep_net=None):
    """多场景 × 多种子评测，返回逐条记录。"""
    rows = []
    for sc in scenes:
        for sd in seeds:
            from .policies import make_policy
            pol = make_policy(kind, make_scene(sc, sd), net=net,
                              percep_net=percep_net)
            r = rollout(sc, pol, seed=sd)
            rows.append(r)
    return rows


def collect_demo(scene_keys, seeds, steps_limit=None, jitter=None):
    """用 Oracle 专家采集行为克隆数据（按 episode 返回帧序列与动作）。

    jitter = (背离管道侧, 朝向管道侧) 会额外制造**非标称起始状态**：
    专家轨迹的横向误差几乎恒为 0，只用它训出的横向回归器会均值坍缩成
    "恒输出 0"。但把扰动混进行为克隆数据也会让端到端策略退化（实测
    9/12 → 0/12，学生被大量"救偏"样本带偏），所以默认不扰动 —— 需要
    时显式传 jitter，用于验证感知回归的数据覆盖问题。

    两侧幅值不能对称：巡检偏置只有 0.9 m，朝管道那侧偏移超过 0.40 m
    就会让起点落进"管道半径 0.18 + ROV 0.25"的接触区，第 0 步就判碰撞。
    """
    from .policies import make_policy
    eps = []
    for i, sc in enumerate(scene_keys):
        for j, sd in enumerate(seeds):
            # 正负交替，两个方向都要有；奇偶按 (场景, 种子) 下标定
            sign = 1.0 if (i + j) % 2 == 0 else -1.0
            mag = 0.0 if jitter is None else (jitter[0] if sign > 0 else jitter[1])
            r = None
            for off in (mag, 0.5 * mag, 0.0):
                pol = make_policy('oracle', make_scene(sc, sd))
                r = rollout(sc, pol, seed=sd, record=True,
                            start_offset=(sign * off, 0.0) if off else None,
                            max_steps=steps_limit or int(P.ARRIVE_TIME_LIMIT / P.DT))
                if r['reached_b'] and len(r['frames']) >= 5:
                    break
            if r is None or len(r['frames']) < 5 or not r['reached_b']:
                continue
            eps.append({'frames': r['frames'], 'depths': r['depths'],
                        'odoms': r['odoms'], 'goals': r['goals'],
                        'actions': r['actions'],
                        'percep_labels': r['percep_labels'],
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
        gls = ep.get('goals')
        T = len(f)
        for i in range(T):
            idx = [max(0, i - k) for k in range(frames_n - 1, -1, -1)]
            gray = net_gray(f[idx])
            dep = (d[i][None] if d is not None else np.zeros((1, FRAME_H, FRAME_W), np.float32))
            gl = gls[i] if gls is not None and i < len(gls) else None
            gch = goal_channel(od[i] if od is not None and i < len(od) else None,
                               gl, FRAME_W, FRAME_H)
            xs.append(np.concatenate([gray, dep, gch[None]], axis=0))
            ys.append([a[i, 0] / P.MAX_V, a[i, 1] / P.MAX_W])
    if not xs:
        return (np.zeros((0, frames_n + 2, FRAME_H, FRAME_W), np.float32),
                np.zeros((0, 2), np.float32))
    return np.asarray(xs, dtype=np.float32), np.asarray(ys, dtype=np.float32)


def stack_percep(eps):
    """感知网络的监督对：单帧（灰度+深度）→ (dx_err, dtheta) 归一化。"""
    xs, ys = [], []
    for ep in eps:
        f = ep['frames']
        d = ep.get('depths')
        pl = ep.get('percep_labels')
        if pl is None or len(pl) != len(f):
            continue
        for i in range(len(f)):
            g = f[i][None]
            dep = d[i][None] if d is not None else np.zeros((1, FRAME_H, FRAME_W), np.float32)
            xs.append(np.concatenate([g, dep], axis=0))
            ys.append(pl[i])
    if not xs:
        return (np.zeros((0, 2, FRAME_H, FRAME_W), np.float32),
                np.zeros((0, 2), np.float32))
    return np.asarray(xs, dtype=np.float32), np.asarray(ys, dtype=np.float32)
