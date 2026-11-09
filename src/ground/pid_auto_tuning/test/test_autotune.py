"""离线测试：被控对象、指标、继电器整定、参数生成。"""

import math

from pid_auto_tuning.autotune import gains_from_rule, select_best_gains
from pid_auto_tuning.metrics import analyze_response
from pid_auto_tuning.plant import HeadingPlant
from pid_auto_tuning.relay_tune import run_relay_feedback


def test_plant_reaches_steady_speed():
    plant = HeadingPlant(dt=0.01)
    for _ in range(2000):
        plant.step(1.0)
    assert abs(plant.omega - 1.0) < 0.02
    assert plant.theta > 15.0


def test_metrics_detect_overshoot_and_settle():
    target = 1.0
    t = [i * 0.01 for i in range(500)]
    y = [1.0 - math.exp(-2.5 * ti) * math.cos(6.0 * ti) for ti in t]
    metrics = analyze_response(t, y, target, 0.01)
    assert metrics['overshoot_pct'] > 15.0
    assert metrics['steady_state_error'] < 0.05
    assert metrics['settle_time'] is not None


def test_metrics_settle_none_when_never_converged():
    # 响应一直为零、从未进入 ±2% 误差带：settle_time 与稳态误差应为 None
    target = 1.0
    t = [i * 0.01 for i in range(800)]
    y = [0.5] * 800
    metrics = analyze_response(t, y, target, 0.01)
    assert metrics['settle_time'] is None
    assert metrics['steady_state_error'] is None


def test_metrics_rise_none_when_no_crossing():
    target = 1.0
    t = [i * 0.01 for i in range(200)]
    y = [0.05] * 200
    metrics = analyze_response(t, y, target, 0.01)
    assert metrics['rise_time'] is None


def test_metrics_support_negative_target():
    # 目标为负时响应向反方向变化，上升时间不能退化为 0
    target = -1.0
    t = [i * 0.01 for i in range(1000)]
    y = [-(1.0 - math.exp(-2.5 * ti) * math.cos(6.0 * ti)) for ti in t]
    metrics = analyze_response(t, y, target, 0.01)
    assert metrics['rise_time'] > 0.05
    assert metrics['overshoot_pct'] > 0.0
    assert metrics['settle_time'] is not None


def test_relay_feedback_gives_positive_ku_and_tu():
    relay = run_relay_feedback(setpoint=math.pi / 2.0)
    assert relay['ku'] > 0.0
    assert relay['tu'] > 0.0
    assert relay['amplitude'] > 0.0


def test_gains_are_positive_and_stable():
    relay = run_relay_feedback(setpoint=math.pi / 2.0)
    kp, ki, kd = gains_from_rule(relay['ku'], relay['tu'], (0.2, 0.5, 0.33))
    assert kp > 0.0 and ki > 0.0 and kd > 0.0

    gains, metrics, info = select_best_gains(relay['ku'], relay['tu'], setpoint=math.pi / 2.0)
    assert not metrics['diverged']
    assert metrics['steady_state_error'] < 0.05
    assert 'hit_boundary' in info
