"""无 ROS 的闭环代理评测。

把现有的 NN / LOS 控制器与轨迹生成器接到一个一阶速度响应代理上，
用于「没有 ROS、没有 MuJoCo」的机器上做快速回归与对比。
代理模型：

    m * dv/dt = R(yaw) @ [fx, fy, 0] + [0, 0, fz] - c * v
    I * dw/dt = tau_z - c_yaw * w

它是**运动学近似**，不代表 MuJoCo 里推进器布置、附加质量与水流剪切下的真实响应，
因此结果只用于回归和相对比较，不能替代仿真结论。
"""
import math

import numpy as np

from .trajectory_generator import TrajectoryGenerator


class FirstOrderROV:
    """一阶速度响应代理（世界系位置 + 偏航角）。"""

    def __init__(self, mass=40.0, drag=6.0, inertia=1.2, yaw_drag=1.5, max_speed=2.0):
        self.mass = float(mass)
        self.drag = float(drag)
        self.inertia = float(inertia)
        self.yaw_drag = float(yaw_drag)
        self.max_speed = float(max_speed)

    def initial_state(self, position):
        return {'pos': np.asarray(position, dtype=float).copy(),
                'vel': np.zeros(3), 'yaw': 0.0, 'yaw_rate': 0.0}

    def step(self, state, forces, dt):
        """forces: {'fx','fy','fz','tau_z'}，fx/fy 为机体系，fz 为世界系垂直力。"""
        cos_y, sin_y = math.cos(state['yaw']), math.sin(state['yaw'])
        body = np.array([forces['fx'], forces['fy']], dtype=float)
        world_xy = np.array([cos_y * body[0] - sin_y * body[1],
                             sin_y * body[0] + cos_y * body[1]])
        acc = np.array([world_xy[0], world_xy[1], forces['fz']]) / self.mass - self.drag * state['vel'] / self.mass
        state['vel'] = state['vel'] + acc * dt
        speed = float(np.linalg.norm(state['vel']))
        if speed > self.max_speed:
            state['vel'] *= self.max_speed / speed
        state['pos'] = state['pos'] + state['vel'] * dt
        yaw_acc = forces['tau_z'] / self.inertia - self.yaw_drag * state['yaw_rate']
        state['yaw_rate'] += yaw_acc * dt
        state['yaw'] = (state['yaw'] + state['yaw_rate'] * dt + math.pi) % (2 * math.pi) - math.pi
        return state


def run_episode(controller, trajectory_type='3d_helix', duration=60.0, dt=0.02,
                vehicle=None, start_offset=(0.0, 0.0, 0.0)):
    """跑一条航迹，返回指标与轨迹数组。"""
    generator = TrajectoryGenerator(trajectory_type=trajectory_type)
    vehicle = vehicle or FirstOrderROV()
    first = generator.get_state(0.0)
    state = vehicle.initial_state(first['pos'] + np.asarray(start_offset, dtype=float))

    times, errors, yaw_errors, commands = [], [], [], []
    for step in range(int(round(duration / dt))):
        t = step * dt
        ref = generator.get_state(t)
        out = controller.compute(state['pos'], state['vel'], state['yaw'],
                                 ref['pos'], ref['vel'], ref['yaw'])
        state = vehicle.step(state, out, dt)
        errors.append(float(np.linalg.norm(state['pos'] - ref['pos'])))
        yaw_errors.append(float(abs((ref['yaw'] - state['yaw'] + math.pi) % (2 * math.pi) - math.pi)))
        # 注意：这里是力(N)与力矩(N·m)的混合量，只作为控制代价的相对指标，不是物理功
        commands.append(math.hypot(out['fx'], out['fy']) + abs(out['fz']) + abs(out['tau_z']))
        times.append(t)

    errors = np.asarray(errors)
    return {
        'trajectory': trajectory_type,
        'duration': duration,
        'rmse': float(np.sqrt(np.mean(errors ** 2))),
        'max_error': float(errors.max()),
        'mean_error': float(errors.mean()),
        'final_error': float(errors[-1]),
        'yaw_rmse': float(np.sqrt(np.mean(np.square(yaw_errors)))),
        'effort': float(np.mean(commands)),  # 力与力矩混合的相对量，不可当物理能量解读
        'times': times,
        'errors': errors.tolist(),
    }

def check_gate(summary, max_rmse):
    """返回未通过门禁的 (组合, RMSE) 列表，空列表表示通过。

    RMSE 为 NaN/inf 时 ``rmse > max_rmse`` 是 False，会被静默放行——控制器发散
    却显示 GATE PASS 是门禁最不该有的行为，所以非有限值一律算失败。
    """
    failures = []
    for key, value in summary.items():
        if key == 'config':
            continue
        rmse = float(value['rmse'])
        if not math.isfinite(rmse) or rmse > max_rmse:
            failures.append((key, rmse))
    return failures
