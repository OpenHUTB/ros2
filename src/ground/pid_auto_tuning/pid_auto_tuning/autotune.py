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
        plant_factory = lambda: HeadingPlant(dt=dt)
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
    if metrics['diverged']:
        return 1e6
    return (0.05 * metrics['overshoot_pct']
            + metrics['settle_time']
            + 20.0 * metrics['steady_state_error']
            + 0.5 * metrics['iae'])


def select_best_gains(ku, tu, plant_factory=None, setpoint=math.pi / 2.0):
    """先按三条经验规则试，再在 Ku/Tu 附近小范围网格搜索。"""
    best = None
    for rule in RULES.values():
        gains = gains_from_rule(ku, tu, rule)
        metrics = simulate_closed_loop(gains, plant_factory, setpoint)
        if best is None or score(metrics) < score(best[1]):
            best = (gains, metrics)

    for kp_scale in (0.15, 0.25, 0.35, 0.45, 0.55, 0.65):
        for ti_scale in (0.4, 0.6, 0.9, 1.3, 1.9, 2.6):
            for td_scale in (0.0, 0.08, 0.16, 0.25):
                gains = gains_from_rule(ku, tu, (kp_scale, ti_scale, td_scale))
                metrics = simulate_closed_loop(gains, plant_factory, setpoint)
                if score(metrics) < score(best[1]):
                    best = (gains, metrics)

    return best
