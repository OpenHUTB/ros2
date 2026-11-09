"""由 Ku / Tu 生成 PID 参数，并在候选规则里按闭环指标挑最优。"""

import math

from .metrics import analyze_response
from .pid_controller import PIDController
from .plant import HeadingPlant

# 各整定规则的系数：kp_scale、ti_scale、td_scale，均相对于 Ku / Tu
RULES = {
    'ziegler_nichols_classic': (0.60, 0.50, 0.125),
    'ziegler_nichols_no_overshoot': (0.20, 0.50, 0.330),
    'tyreus_luyben': (0.45, 2.20, 0.159),
}


# 网格搜索范围（上界扩到 1.75Ku / 4.5Tu：1.2Ku 时最优仍压边界，扩到 1.75Ku 后最优点进入内部）
KP_SCALES = (0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95, 1.05, 1.2, 1.35, 1.5, 1.75)
TI_SCALES = (0.4, 0.6, 0.9, 1.3, 1.9, 2.6, 3.2, 4.0, 4.5)
TD_SCALES = (0.0, 0.08, 0.16, 0.25, 0.33)


def gains_from_rule(ku, tu, rule):
    kp_scale, ti_scale, td_scale = rule
    kp = kp_scale * ku
    ti = ti_scale * tu
    ki = kp / ti if ti > 0 else 0.0
    kd = kp * td_scale * tu
    return kp, ki, kd


def simulate_closed_loop(gains, plant_factory=None, setpoint=math.pi / 2.0,
                         t_end=8.0, dt=0.01, u_limit=2.5):
    if plant_factory is None:
        plant_factory = lambda: HeadingPlant(dt=dt, u_max=u_limit)
    kp, ki, kd = gains
    plant = plant_factory()
    pid = PIDController(kp, ki, kd, dt, u_limit=u_limit)

    t, y, u = [], [], []
    time = 0.0
    for _ in range(int(round(t_end / dt))):
        error = setpoint - plant.theta
        control = pid.update(error)
        theta = plant.step(control)
        time += dt
        t.append(time)
        y.append(theta)
        u.append(control)
        if abs(theta) > 10.0:
            break

    metrics = analyze_response(t, y, setpoint, dt)
    metrics['gains'] = gains
    metrics['t'] = t
    metrics['y'] = y
    metrics['u'] = u
    return metrics


def score(metrics):
    # 发散或仿真结束时仍未进入误差带的候选直接淘汰
    if metrics['diverged'] or metrics['settle_time'] is None:
        return 1e6
    return (0.05 * metrics['overshoot_pct']
            + metrics['settle_time']
            + 20.0 * metrics['steady_state_error']
            + 0.5 * metrics['iae'])


def select_best_gains(ku, tu, plant_factory=None, setpoint=math.pi / 2.0,
                      dt=0.01, u_limit=2.5):
    """先按三条经验规则试，再在 Ku/Tu 附近网格搜索。

    返回 (gains, metrics, info)；info['hit_boundary'] 表示网格最优是否压在
    搜索范围边界上，为 True 时应扩大 KP_SCALES/TI_SCALES/TD_SCALES 再确认。
    """
    best = None
    for name, rule in RULES.items():
        gains = gains_from_rule(ku, tu, rule)
        metrics = simulate_closed_loop(gains, plant_factory, setpoint,
                                       dt=dt, u_limit=u_limit)
        info = {'source': name, 'scales': rule, 'hit_boundary': False}
        if best is None or score(metrics) < score(best[1]):
            best = (gains, metrics, info)

    for kp_scale in KP_SCALES:
        for ti_scale in TI_SCALES:
            for td_scale in TD_SCALES:
                gains = gains_from_rule(ku, tu, (kp_scale, ti_scale, td_scale))
                metrics = simulate_closed_loop(gains, plant_factory, setpoint,
                                               dt=dt, u_limit=u_limit)
                hit_boundary = (kp_scale in (KP_SCALES[0], KP_SCALES[-1])
                                or ti_scale in (TI_SCALES[0], TI_SCALES[-1])
                                or td_scale in (TD_SCALES[0], TD_SCALES[-1]))
                info = {'source': 'grid',
                        'scales': (kp_scale, ti_scale, td_scale),
                        'hit_boundary': hit_boundary}
                if score(metrics) < score(best[1]):
                    best = (gains, metrics, info)

    return best
