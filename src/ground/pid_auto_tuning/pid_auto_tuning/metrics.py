"""阶跃响应指标：超调量、上升时间、调节时间、稳态误差、IAE。"""


def analyze_response(t, y, target, dt, settle_band=0.02):
    n = len(y)
    if n == 0:
        return {'overshoot_pct': 0.0, 'rise_time': 0.0, 'settle_time': 0.0,
                'steady_state_error': 0.0, 'iae': 0.0, 'diverged': True}

    tail = max(3, int(0.05 * n))
    y_final = sum(y[-tail:]) / tail
    y_peak = max(y) if target >= 0 else min(y)

    if target != 0:
        overshoot = max(0.0, (y_peak - target) / abs(target) * 100.0)
    else:
        overshoot = 0.0

    rise_start = _first_cross(t, y, 0.1 * target)
    rise_end = _first_cross(t, y, 0.9 * target)
    rise_time = (rise_end - rise_start) if (rise_start is not None and rise_end is not None) else 0.0

    band = settle_band * abs(target)
    last_out = -1
    for i in range(n):
        if abs(y[i] - target) > band:
            last_out = i
    settle_time = t[min(last_out + 1, n - 1)] if last_out >= 0 else 0.0

    iae = sum(abs(y[i] - target) for i in range(n)) * dt

    return {
        'overshoot_pct': overshoot,
        'rise_time': rise_time,
        'settle_time': settle_time,
        'steady_state_error': abs(y_final - target),
        'iae': iae,
        'diverged': abs(y_peak) > 10.0,
    }


def _first_cross(t, y, level):
    for i in range(len(y)):
        if y[i] >= level:
            return t[i]
    return None
