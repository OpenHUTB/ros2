#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
神经网络控制器多场景闭环评估（任务 2）

固定一组不同的起点/终点对（方位、距离、障碍布局都不同），逐对飞一遍，
统计成功率、到达误差分布、耗时、碰撞次数、安全层介入次数，以及闭环过程中
网络与势场法专家的动作偏差。

用法：
    python3 eval_multi_target.py --pairs 10 --out eval/multi_target.csv
    python3 eval_multi_target.py --pairs 10 --no-shield      # 对照：关闭安全层
    python3 eval_multi_target.py --start-index 5             # 从第 5 对继续

脚本直接通过 AirSim RPC 控制无人机，不依赖 ROS 话题，便于在没有 ROS 的
机器上复跑；控制律、特征提取与部署节点完全一致（共用 uav_features）。
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np
import torch

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

# 被 import 的模块在 devel 空间里不在 sys.path 上（CMake 用的是 install(FILES)），
# 通过 rospkg 找到源码包目录补进来，rosrun 走 relay 脚本时也能正常 import
try:
    import rospkg as _rospkg
    _pkg_scripts = os.path.join(
        _rospkg.RosPack().get_path('uav_nn_perception_navigation'), 'scripts')
    if os.path.isdir(_pkg_scripts) and _pkg_scripts not in sys.path:
        sys.path.insert(0, _pkg_scripts)
except Exception:
    pass

import safety_shield  # noqa: E402
from uav_features import (  # noqa: E402
    MAX_RANGE, build_feature_vector, yaw_from_quaternion,
)

_client_path = os.environ.get('AIRSIM_PYTHON_CLIENT')
if _client_path and _client_path not in sys.path:
    sys.path.insert(0, _client_path)

import airsim  # noqa: E402

ACTION_SCALE = np.array([3.0, 2.0, 1.0], dtype=np.float32)

# 10 组起终点（NED，米）：覆盖四个象限、10~40 m 距离，部分航线会经过障碍区
DEFAULT_PAIRS = [
    ((-4.0, -4.0), (16.0, 12.0)),
    ((-16.0, 6.0), (14.0, -10.0)),
    ((-10.0, 18.0), (12.0, -18.0)),
    ((-18.0, -16.0), (18.0, 16.0)),
    ((6.0, -20.0), (-14.0, 14.0)),
    ((18.0, -12.0), (-16.0, 12.0)),
    ((-20.0, 14.0), (20.0, 12.0)),
    ((4.0, 20.0), (-18.0, -12.0)),
    ((-14.0, -6.0), (14.0, 4.0)),
    ((10.0, 16.0), (-12.0, -16.0)),
]

SUCCESS_DIST = 1.0        # 米，平面距离小于此值算到达
CTRL_HZ = 10.0


class MLPPolicy(torch.nn.Module):
    def __init__(self, in_dim=16, hidden=64):
        super(MLPPolicy, self).__init__()
        self.net = torch.nn.Sequential(
            torch.nn.Linear(in_dim, hidden),
            torch.nn.ReLU(),
            torch.nn.Linear(hidden, hidden),
            torch.nn.ReLU(),
            torch.nn.Linear(hidden, 3),
        )

    def forward(self, x):
        return self.net(x)


def resolve_model_path(model_path):
    if os.path.isabs(model_path) and os.path.exists(model_path):
        return model_path
    try:
        import rospkg
        pkg_dir = rospkg.RosPack().get_path('uav_nn_perception_navigation')
        candidate = os.path.join(pkg_dir, 'models', os.path.basename(model_path))
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    return os.path.abspath(os.path.join(_SCRIPT_DIR, '..', 'models',
                                        os.path.basename(model_path)))


def connect(ip, vehicle):
    """建立 AirSim 连接并接管无人机。

    timeout_value 留得比较宽：仿真器刚起或场景复杂时单次调用可能超过 10 s，
    超时会让 msgpack 流错位，后续所有调用都失败。
    """
    client = airsim.MultirotorClient(ip=ip, timeout_value=30)
    client.confirmConnection()
    client.enableApiControl(True, vehicle)
    client.armDisarm(True, vehicle)
    return client


class Policy(object):
    """与 trajectory_control.py 完全一致的推理封装。"""

    def __init__(self, model_path):
        checkpoint = torch.load(resolve_model_path(model_path),
                                map_location='cpu')
        self.model = MLPPolicy(in_dim=checkpoint.get('in_dim', 16))
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model.eval()
        self.scale = np.array(checkpoint.get('action_scale', ACTION_SCALE.tolist()),
                              dtype=np.float32)
        self.feat_mean = checkpoint.get('feat_mean')
        self.feat_std = checkpoint.get('feat_std')

    def act(self, feat):
        value = feat
        if self.feat_mean is not None and self.feat_std is not None:
            value = (feat - self.feat_mean) / self.feat_std
        with torch.no_grad():
            out = self.model(torch.from_numpy(value.astype(np.float32))
                             .unsqueeze(0)).squeeze(0).numpy()
        return out * self.scale


def expert_action(feat):
    """与 train_control.py 的势场法专家一致，用于闭环偏差对比。"""
    sectors = feat[:12]
    dist_norm, sin_b, cos_b, _dz = feat[12:]
    bearing = math.atan2(float(sin_b), float(cos_b))
    goal_dist = float(dist_norm) * 20.0

    vx = 0.45 * goal_dist * float(cos_b)
    vy = 0.45 * goal_dist * float(sin_b)
    for i, d_norm in enumerate(sectors):
        d = float(d_norm) * MAX_RANGE
        if d >= 6.0:
            continue
        angle = math.radians(-30.0 + (i + 0.5) * 5.0)
        strength = 4.0 * (1.0 - d / 6.0)
        vx -= strength * math.cos(angle)
        vy -= strength * math.sin(angle)

    speed = math.hypot(vx, vy)
    if speed > 3.0:
        vx, vy = vx / speed * 3.0, vy / speed * 3.0
    yaw_rate = max(-1.0, min(1.0, 1.2 * bearing))
    return np.array([vx, vy, yaw_rate], dtype=np.float32)


def read_lidar(client, vehicle):
    lidar = client.getLidarData(lidar_name='lidar_1', vehicle_name=vehicle)
    if len(lidar.point_cloud):
        return np.array(lidar.point_cloud, dtype=np.float32).reshape(-1, 3)
    return np.zeros((0, 3), dtype=np.float32)


def place(client, vehicle, pos, yaw, altitude, tries=6):
    """放到指定起点；若落点撞进几何体，按小圆环重掷。"""
    offsets = [(0.0, 0.0)] + [(1.5 * math.cos(a), 1.5 * math.sin(a))
                              for a in np.linspace(0, 2 * math.pi, 6)[:-1]]
    for dx, dy in offsets[:tries]:
        client.simSetVehiclePose(
            airsim.Pose(airsim.Vector3r(pos[0] + dx, pos[1] + dy, -altitude),
                        airsim.to_quaternion(0, 0, yaw)), True)
        time.sleep(0.4)
        if not client.simGetCollisionInfo().has_collided:
            return True
    return False


def run_episode(client, policy, vehicle, start, goal, altitude, timeout,
                use_safety, trace_fh=None, inject_step=-1):
    yaw = math.atan2(goal[1] - start[1], goal[0] - start[0])
    placed = place(client, vehicle, start, yaw, altitude)
    osc = safety_shield.OscillationDetector()
    goal_arr = np.array([goal[0], goal[1], -altitude], dtype=np.float64)

    reasons = {}
    collisions = 0
    shield_steps = 0
    dev_history = []
    path_length = 0.0
    prev_pos = None
    min_error = float('inf')
    final_error = float('inf')
    steps = 0
    success = False
    fault_step = -1
    fault_handled = False
    t0 = time.time()

    while time.time() - t0 < timeout:
        state = client.getMultirotorState(vehicle_name=vehicle)
        pos = state.kinematics_estimated.position
        cur = np.array([pos.x_val, pos.y_val, pos.z_val], dtype=np.float64)
        yaw_now = yaw_from_quaternion(state.kinematics_estimated.orientation)

        if client.simGetCollisionInfo().has_collided:
            collisions += 1

        err = float(np.linalg.norm(cur[:2] - goal_arr[:2]))
        min_error = min(min_error, err)
        if err < SUCCESS_DIST:
            success = True
            final_error = err
            break

        points = read_lidar(client, vehicle)
        feat = build_feature_vector(points, cur, goal_arr, yaw_now)
        raw = policy.act(feat)

        # 故障注入：在第 N 步把网络输出换成 NaN，检验安全层能否接管
        if inject_step >= 0 and steps == inject_step:
            raw = np.full(3, np.nan, dtype=np.float32)
            fault_step = steps

        expert = expert_action(feat)
        dev_history.append(float(np.linalg.norm(raw - expert)))

        vx, vy, yaw_rate = (float(raw[0]), float(raw[1]), float(raw[2]))
        reason = ''
        if use_safety:
            result = safety_shield.apply_shield(vx, vy, yaw_rate, feat[:12])
            result.yaw_rate = osc.update(result.yaw_rate)
            if result.active:
                shield_steps += 1
                reasons[result.reason] = reasons.get(result.reason, 0) + 1
            if fault_step >= 0 and result.reason == 'non_finite':
                fault_handled = True
            vx, vy, yaw_rate = result.as_tuple()
            reason = result.reason

        # 网络输出是机体系速度，而 moveByVelocityZ 收的是世界系（NED）；
        # 必须和 airsim_bridge.py 一样先转再发，否则朝向一变航向就歪
        psi = float(yaw_now)
        world_vx = vx * math.cos(psi) - vy * math.sin(psi)
        world_vy = vx * math.sin(psi) + vy * math.cos(psi)
        client.moveByVelocityZAsync(
            world_vx, world_vy, -altitude, 1.0 / CTRL_HZ,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=True,
                                    yaw_or_rate=math.degrees(yaw_rate)),
            vehicle_name=vehicle,
        )

        if prev_pos is not None:
            path_length += float(np.linalg.norm(cur[:2] - prev_pos[:2]))
        prev_pos = cur
        steps += 1

        if trace_fh is not None:
            trace_fh.write('%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%s\n'
                           % (time.time() - t0, cur[0], cur[1], err,
                              vx, vy, yaw_rate, reason))

        time.sleep(1.0 / CTRL_HZ)

    duration = time.time() - t0
    if not success:
        final_error = min_error if min_error < float('inf') else float('nan')
    else:
        final_error = err

    # 结束前悬停 0.5 s，避免留下速度指令
    client.moveByVelocityZAsync(0.0, 0.0, -altitude, 0.5,
                                yaw_mode=airsim.YawMode(is_rate=True,
                                                        yaw_or_rate=0.0),
                                vehicle_name=vehicle)
    time.sleep(0.5)

    return {
        'placed': placed,
        'success': success,
        'final_error': round(float(final_error), 3),
        'min_error': round(float(min_error), 3),
        'duration': round(duration, 2),
        'steps': steps,
        'path_length': round(path_length, 2),
        'collided': 1 if collisions else 0,
        'collision_steps': collisions,
        'shield_steps': shield_steps,
        'shield_reasons': ';'.join('%s:%d' % kv for kv in sorted(reasons.items())),
        'fault_step': fault_step,
        'fault_handled': 1 if fault_handled else 0,
        'expert_dev_mean': round(float(np.mean(_finite(dev_history))), 3)
        if _finite(dev_history) else float('nan'),
        'expert_dev_max': round(float(np.max(_finite(dev_history))), 3)
        if _finite(dev_history) else float('nan'),
    }


def _finite(values):
    """去掉故障注入产生的 NaN，避免污染统计。"""
    return [v for v in values if np.isfinite(v)]


def summarize(rows, use_safety):
    ok = [r for r in rows if r['success']]
    errs = np.array([r['final_error'] for r in ok], dtype=np.float32)
    durations = np.array([r['duration'] for r in rows], dtype=np.float32)
    dev_mean = np.array([r['expert_dev_mean'] for r in rows
                         if np.isfinite(r['expert_dev_mean'])], dtype=np.float32)
    summary = {
        'runs': len(rows),
        'success_count': len(ok),
        'success_rate': round(len(ok) / float(len(rows)), 4) if rows else 0.0,
        'error_mean': round(float(errs.mean()), 3) if len(errs) else None,
        'error_std': round(float(errs.std()), 3) if len(errs) else None,
        'error_min': round(float(errs.min()), 3) if len(errs) else None,
        'error_max': round(float(errs.max()), 3) if len(errs) else None,
        'duration_mean': round(float(durations.mean()), 2) if len(durations) else None,
        'collision_episodes': int(sum(r['collided'] for r in rows)),
        'collision_steps_total': int(sum(r['collision_steps'] for r in rows)),
        'shield_steps_total': int(sum(r['shield_steps'] for r in rows)),
        'expert_dev_mean': round(float(dev_mean.mean()), 3) if len(dev_mean) else None,
        'expert_dev_max': round(float(dev_mean.max()), 3) if len(dev_mean) else None,
        'safety_enabled': bool(use_safety),
        'success_dist': SUCCESS_DIST,
    }
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', default=os.environ.get('AIRSIM_IP', '172.25.176.1'))
    parser.add_argument('--vehicle', default='Drone_1')
    parser.add_argument('--model', default='controller.pt')
    parser.add_argument('--pairs', type=int, default=len(DEFAULT_PAIRS))
    # 自定义起终点：每行 sx,sy,gx,gy（NED，米），用于障碍/故障注入场景
    parser.add_argument('--pairs-file', default=None)
    parser.add_argument('--start-index', type=int, default=0)
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--altitude', type=float, default=4.0)
    parser.add_argument('--no-shield', action='store_true')
    # 在第 N 个控制步注入 NaN，检验安全层接管（-1 表示不注入）
    parser.add_argument('--inject-fault', type=int, default=-1)
    parser.add_argument('--out', default=os.path.join('eval', 'multi_target.csv'))
    parser.add_argument('--summary', default=None)
    parser.add_argument('--trace-dir', default=None)
    args = parser.parse_args()

    use_safety = not args.no_shield
    policy = Policy(args.model)

    client = connect(args.ip, args.vehicle)

    if args.pairs_file:
        pairs = []
        with open(args.pairs_file, 'r', encoding='utf-8') as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                sx, sy, gx, gy = [float(v) for v in line.split(',')[:4]]
                pairs.append(((sx, sy), (gx, gy)))
        if not pairs:
            raise SystemExit('--pairs-file 里没有有效航线')
    else:
        pairs = DEFAULT_PAIRS[:max(1, min(args.pairs, len(DEFAULT_PAIRS)))]
    rows = []

    for index in range(args.start_index, len(pairs)):
        start, goal = pairs[index]
        dist = math.hypot(goal[0] - start[0], goal[1] - start[1])
        timeout = max(args.timeout, dist / 1.0 + 15.0)
        print('[%d/%d] 起点 (%.1f, %.1f) -> 终点 (%.1f, %.1f)，直线距离 %.1f m'
              % (index + 1, len(pairs), start[0], start[1], goal[0], goal[1], dist))

        trace_fh = None
        if args.trace_dir:
            os.makedirs(args.trace_dir, exist_ok=True)
            trace_fh = open(os.path.join(args.trace_dir,
                                         'pair_%02d.csv' % (index + 1)),
                            'w', encoding='utf-8')
            trace_fh.write('t,x,y,error,vx,vy,yaw_rate,shield\n')

        try:
            result = run_episode(client, policy, args.vehicle, start, goal,
                                 args.altitude, timeout, use_safety, trace_fh,
                                 inject_step=args.inject_fault)
        except Exception as exc:
            # 仿真器偶发超时会让 msgpack 流错位，重建连接后继续下一组
            print('    运行异常，重建连接后继续: %s' % exc)
            result = {'placed': False, 'success': False, 'final_error': float('nan'),
                      'min_error': float('nan'), 'duration': 0.0, 'steps': 0,
                      'path_length': 0.0, 'collided': 1, 'collision_steps': 0,
                      'shield_steps': 0, 'fault_step': -1, 'fault_handled': 0,
                      'shield_reasons': 'aborted', 'expert_dev_mean': float('nan'),
                      'expert_dev_max': float('nan')}
            try:
                client = connect(args.ip, args.vehicle)
            except Exception as reconnect_exc:  # pragma: no cover
                print('    重连失败: %s' % reconnect_exc)
        finally:
            if trace_fh is not None:
                trace_fh.close()
        time.sleep(1.0)

        result.update({'index': index + 1,
                       'start_x': start[0], 'start_y': start[1],
                       'goal_x': goal[0], 'goal_y': goal[1],
                       'distance': round(dist, 2)})
        rows.append(result)
        print('    成功=%s 误差=%.2f m 耗时=%.1f s 碰撞步数=%d 安全层=%d 步'
              % (result['success'], result['final_error'], result['duration'],
                 result['collision_steps'], result['shield_steps']))

    fields = ['index', 'start_x', 'start_y', 'goal_x', 'goal_y', 'distance',
              'placed', 'success', 'final_error', 'min_error', 'duration',
              'steps', 'path_length', 'collided', 'collision_steps',
              'shield_steps', 'shield_reasons', 'fault_step', 'fault_handled',
              'expert_dev_mean', 'expert_dev_max']
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w', newline='', encoding='utf-8') as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    summary = summarize(rows, use_safety)
    summary_path = args.summary or os.path.splitext(args.out)[0] + '_summary.json'
    with open(summary_path, 'w', encoding='utf-8') as fh:
        json.dump(summary, fh, ensure_ascii=False, indent=2)

    print('\n汇总: 成功 %d/%d (%.0f%%)，误差 均值 %.2f ± %.2f m，'
          '发生过碰撞的回合 %d 个，安全层介入 %d 步，专家偏差 均值 %.2f'
          % (summary['success_count'], summary['runs'],
             summary['success_rate'] * 100,
             summary['error_mean'] or 0, summary['error_std'] or 0,
             summary['collision_episodes'], summary['shield_steps_total'],
             summary['expert_dev_mean'] or 0))
    print('明细: %s\n汇总: %s' % (args.out, summary_path))

    client.armDisarm(False, args.vehicle)
    client.enableApiControl(False, args.vehicle)


if __name__ == '__main__':
    main()
