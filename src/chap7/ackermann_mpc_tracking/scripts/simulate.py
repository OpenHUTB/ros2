"""闭环仿真：同一条 S 弯参考路径上对比 MPC 与 Stanley。

参考路径由「直线 - 圆弧 - 反向圆弧 - 直线」组成，曲率在三个点突变
（0 -> -0.25 -> +0.25 -> 0）。这类曲率突变正是「有没有预瞄」的分水岭：
Stanley 要等误差出现才修正，MPC 在进入弯道之前就开始打方向。

车辆初速 2 m/s，初始横向偏差 0.8 m（压在路径左侧），两个控制器使用
完全相同的限幅与步长，只比控制律本身。
"""

import math

from mpc import KinematicBicycle, LinearMPC, PathTracker, wrap_pi
from stanley_baseline import StanleyController

DT = 0.05              # 仿真步长 (s)
MPC_PERIOD = 0.05      # MPC 求解周期 (s)
V_CRUISE = 5.0         # 纵向速度 (m/s)，18 km/h 的园区小车
WHEELBASE = 2.5        # 轴距 (m)
DELTA_MAX = 0.6        # 前轮转角上限 (rad)


class RefPath(object):
    """分段常曲率参考路径：S 直线、L 左转弧、R 右转弧。"""

    def __init__(self, start, segments, radius=4.0):
        self.start = tuple(start)
        self.radius = float(radius)
        self.segments = list(segments)          # [(kind, length), ...]
        self._starts = [tuple(start)]
        pose = tuple(start)
        for kind, seg_len in self.segments:
            pose = self._advance(pose, kind, seg_len)
            self._starts.append(pose)

    @property
    def length(self):
        return sum(s[1] for s in self.segments)

    def _kappa(self, kind):
        if kind == 'L':
            return 1.0 / self.radius
        if kind == 'R':
            return -1.0 / self.radius
        return 0.0

    def _advance(self, pose, kind, seg_len):
        x, y, th = pose
        kappa = self._kappa(kind)
        if kappa == 0.0:
            return (x + seg_len * math.cos(th), y + seg_len * math.sin(th), th)
        dth = kappa * seg_len
        r = 1.0 / abs(kappa)
        sign = 1.0 if kappa > 0 else -1.0
        return (x + sign * r * (math.sin(th + dth) - math.sin(th)),
                y + sign * r * (math.cos(th) - math.cos(th + dth)),
                th + dth)

    def pose_at(self, s):
        s = min(max(s, 0.0), self.length)
        acc = 0.0
        pose = self.start
        for i, (kind, seg_len) in enumerate(self.segments):
            if seg_len <= 0.0:
                continue
            if s <= acc + seg_len + 1e-12:
                local = s - acc
                return self._advance(pose, kind, local) + (self._kappa(kind),)
            acc += seg_len
            pose = self._advance(pose, kind, seg_len)
        return pose + (0.0,)

    def sample(self, step=0.1):
        n = max(2, int(math.ceil(self.length / step)))
        return [self.pose_at(self.length * i / n) for i in range(n + 1)]


def build_s_curve(radius=6.0, lead=6.0, bend_angle=math.pi / 2.0, tail=8.0):
    """默认场景：直线 -> 右转 -> 左转 -> 直线。"""
    arc = radius * bend_angle
    return RefPath((0.0, 0.0, 0.0),
                   [('S', lead), ('R', arc), ('L', arc), ('S', tail)],
                   radius=radius)


def _metrics(trace, path_length):
    """从轨迹里算误差与执行器负担。"""
    errs = [abs(p['e_y']) for p in trace]
    deltas = [p['delta'] for p in trace]
    rates = [abs(deltas[i] - deltas[i - 1]) / DT for i in range(1, len(deltas))]
    rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
    return {
        'rmse': rmse,
        'max_err': max(errs),
        'final_err': errs[-1],
        'max_rate': max(rates) if rates else 0.0,
        'rms_rate': math.sqrt(sum(r * r for r in rates) / len(rates)) if rates else 0.0,
        'max_delta': max(abs(d) for d in deltas),
        'duration': trace[-1]['t'],
        's_final': trace[-1]['s'],
        'completed': trace[-1]['s'] >= path_length - 0.5,
    }


def run_mpc(path, init_pose, e_y0=0.8, v=V_CRUISE, horizon=15,
            delta_max=DELTA_MAX, q=(10.0, 5.0), r=0.5, r_rate=40.0,
            rate_max=1.6, max_time=40.0):
    """用 MPC 跟踪参考路径。"""
    model = KinematicBicycle(WHEELBASE, delta_max)
    mpc = LinearMPC(dt=MPC_PERIOD, horizon=horizon, wheelbase=WHEELBASE,
                    q=q, r=r, r_rate=r_rate, delta_max=delta_max, v_ref=v)
    tracker = PathTracker(path, s0=0.0)
    state = tuple(init_pose)
    trace = []
    t = 0.0
    step = 0
    delta = 0.0
    n_steps = int(round(MPC_PERIOD / DT))
    while t < max_time:
        info = tracker.update(state[0], state[1])
        e_y = info['e_y']
        e_psi = wrap_pi(state[2] - info['th_ref'])
        if step % n_steps == 0:
            # 前瞻时域内每个预测点的参考转角
            delta_ref = []
            for k in range(horizon):
                s_k = info['s'] + v * MPC_PERIOD * (k + 1)
                kap = path.pose_at(s_k)[3]
                delta_ref.append(model.delta_of_curvature(kap))
            d_seq, _info = mpc.solve([e_y, e_psi], delta_ref, delta_prev=delta)
            # 只执行序列里的第一个增量，下个周期重新规划（滚动优化）
            delta = mpc.delta_sequence([d_seq[0]], delta)[0]
        trace.append({'t': t, 'x': state[0], 'y': state[1], 'psi': state[2],
                      'e_y': e_y, 'e_psi': e_psi, 'delta': delta,
                      's': info['s']})
        if info['s'] >= path.length - 0.3:
            break
        state = model.step(state, v, delta, DT)
        step += 1
        t += DT
    return trace, _metrics(trace, path.length)


def run_stanley(path, init_pose, e_y0=0.8, v=V_CRUISE, k=1.5,
                delta_max=DELTA_MAX, rate_max=1.6, max_time=40.0):
    """用 Stanley 跟踪同一条参考路径。"""
    model = KinematicBicycle(WHEELBASE, delta_max)
    ctrl = StanleyController(WHEELBASE, k=k, delta_max=delta_max,
                             rate_max=rate_max)
    tracker = PathTracker(path, s0=0.0)
    state = tuple(init_pose)
    trace = []
    t = 0.0
    while t < max_time:
        info = tracker.update(state[0], state[1])
        e_y = info['e_y']
        e_psi = wrap_pi(state[2] - info['th_ref'])
        delta = ctrl.control(e_y, e_psi, info['kappa'], v, DT)
        trace.append({'t': t, 'x': state[0], 'y': state[1], 'psi': state[2],
                      'e_y': e_y, 'e_psi': e_psi, 'delta': delta,
                      's': info['s']})
        if info['s'] >= path.length - 0.3:
            break
        state = model.step(state, v, delta, DT)
        t += DT
    return trace, _metrics(trace, path.length)


def sweep_initial_offset(offsets=(0.0, 0.3, 0.6, 0.9, 1.2), **kwargs):
    """初始横向偏差扫描：看两个控制器从多大偏差开始出现明显差异。"""
    rows = []
    for e0 in offsets:
        path = build_s_curve()
        pose = (0.0, e0, 0.0)
        _tm, m_m = run_mpc(path, pose, **kwargs)
        _ts, m_s = run_stanley(path, pose, **kwargs)
        rows.append((e0, m_m, m_s))
    return rows


def benchmark(**mpc_kwargs):
    """默认场景，返回两条轨迹与各自的指标（kwargs 只调 MPC）。"""
    path = build_s_curve()
    pose = (0.0, 0.8, 0.0)
    trace_m, mets_m = run_mpc(path, pose, **mpc_kwargs)
    trace_s, mets_s = run_stanley(path, pose)
    return path, trace_m, mets_m, trace_s, mets_s
