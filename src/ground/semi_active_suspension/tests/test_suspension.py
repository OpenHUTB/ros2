"""半主动悬架模块的离线测试：静平衡、载荷转移、控制律方向、阀门滞后与四个工况。

这里只测不需要 ROS 的部分。ros_node 需要 rclpy，不在单元测试覆盖范围内。
"""
import math
import os

import pytest

from semi_active_suspension import main, render, road, scenarios, scene, sim
from semi_active_suspension import suspension
from semi_active_suspension.params import (CORNERS, DamperConfig, Disturbance,
                                           VehicleSpec, VehicleState)

DEG = math.pi / 180.0
SPEC = VehicleSpec()


@pytest.fixture(scope='module')
def pairs():
    """四个工况各跑被动 / 半主动一遍。"""
    return [(sc.name,) + sim.run_pair(sc) for sc in scenarios.all_scenarios()]


def _pair(pairs, name):
    for item in pairs:
        if item[0] == name:
            return item[1], item[2]
    raise KeyError(name)


# ---------------------------------------------------------------- 静平衡
def test_静态轮荷总和等于整车重量():
    total = sum(SPEC.static_load(c) for c in range(4))
    expected = SPEC.sprung_mass * 9.81 + 4 * SPEC.unsprung_mass * 9.81
    assert total == pytest.approx(expected, rel=1e-12)


def test_质心靠前所以前轴静载大于后轴():
    front = SPEC.static_load(0) + SPEC.static_load(1)
    rear = SPEC.static_load(2) + SPEC.static_load(3)
    assert front > rear
    # 左右对称
    assert SPEC.static_load(0) == pytest.approx(SPEC.static_load(1))


def test_平衡位形上悬架合力为零():
    state = sim.static_state(SPEC)
    forces = suspension.spring_forces(state, SPEC, suspension.passive_damping(SPEC))
    assert sum(forces) == pytest.approx(0.0, abs=1e-9)
    for force in forces:
        assert force == pytest.approx(0.0, abs=1e-9)


def test_平路静止时不会自己动起来():
    flat = scenarios.Scenario(name='flat', title='FLAT', subtitle='',
                              road=road.FlatRoad(), speed=10.0, duration=0.4)
    curve = sim.run(flat, SPEC, semi_active=True)
    for frame in curve.frames:
        assert abs(frame.state.z_s) < 1e-9
        assert abs(frame.state.pitch) < 1e-9
        assert abs(frame.state.roll) < 1e-9


# ---------------------------------------------------------------- 载荷转移
def test_制动时前轴加载后轴减载():
    state = sim.static_state(SPEC)
    dist = Disturbance(accel_x=-5.0)
    damping = suspension.passive_damping(SPEC)
    # 直接看惯性力矩的方向：制动应当产生正俯仰角（车头下沉）
    deriv = suspension.derivative(state, dist, SPEC, damping)
    assert deriv[3] > 0.0
    assert deriv[2] == 0.0  # 初始俯仰角为 0，俯仰角速度也为 0

    moved = suspension.rk4_step(state, dist, SPEC, damping, 0.02)
    assert moved.pitch > 0.0


def test_左转时右侧车轮加载左侧减载():
    state = sim.static_state(SPEC)
    # 左转 = 向心加速度指向左侧，车身向外（右）倾
    dist = Disturbance(accel_y=3.0)
    damping = suspension.passive_damping(SPEC)
    moved = suspension.rk4_step(state, dist, SPEC, damping, 0.02)
    assert moved.roll > 0.0

    body = suspension.body_velocities(moved, SPEC)
    assert body[0] > body[1]  # FL 比 FR 抬得高


def test_侧倾产生的轮荷转移左右反号且总量守恒():
    base = sim.static_state(SPEC)
    dist = Disturbance(accel_y=3.0)
    damping = suspension.passive_damping(SPEC)
    moved = suspension.rk4_step(base, dist, SPEC, damping, 0.05)
    loads = suspension.tire_load(moved, dist, SPEC)
    left = loads[0] + loads[2]
    right = loads[1] + loads[3]
    assert right > left
    static_total = sum(SPEC.static_load(c) for c in range(4))
    assert left + right == pytest.approx(static_total, rel=0.02)


# ---------------------------------------------------------------- 轮胎
def test_轮胎只能受压不能受拉():
    state = VehicleState(z_u=(0.08, 0.08, 0.08, 0.08))
    dist = Disturbance()
    for corner in range(4):
        dynamic = suspension.dynamic_tire_load(state, dist, SPEC, corner)
        # 抬得比静态压缩量还高，轮胎离地，动载停在 -静载
        assert dynamic == pytest.approx(-SPEC.static_load(corner), rel=1e-12)
        assert suspension.tire_load(state, dist, SPEC)[corner] == pytest.approx(0.0)


def test_轮胎压缩时动载为正():
    state = VehicleState(z_u=(-0.01, -0.01, -0.01, -0.01))
    dist = Disturbance()
    dynamic = suspension.dynamic_tire_load(state, dist, SPEC, 0)
    assert dynamic == pytest.approx(SPEC.tire_stiffness * 0.01, rel=1e-9)


def test_路面抬起等于轮胎压缩():
    state = sim.static_state(SPEC)
    dist = Disturbance(road=(0.02, 0.02, 0.02, 0.02))
    dynamic = suspension.dynamic_tire_load(state, dist, SPEC, 0)
    assert dynamic == pytest.approx(SPEC.tire_stiffness * 0.02, rel=1e-9)


# ---------------------------------------------------------------- 控制律
def test_天棚律在车身与车轮反向运动时介入():
    cfg = DamperConfig()
    # 车身向上、车轮不动：相对速度向下，二者反向，天棚应当加硬
    state = VehicleState(v_s=0.3)
    damping = suspension.semi_active_damping(state, SPEC, cfg)
    for corner in range(4):
        assert damping[corner] > SPEC.passive_damping(corner)


def test_车身与车轮同向且同步时退到最软():
    cfg = DamperConfig()
    # 车身和四个轮子一起向上、相对速度为零，减振器无从出力
    state = VehicleState(v_s=0.3, v_u=(0.3, 0.3, 0.3, 0.3))
    damping = suspension.semi_active_damping(state, SPEC, cfg)
    for corner in range(4):
        assert damping[corner] == pytest.approx(SPEC.damper_min)


def test_阻尼系数永远落在可调区间内():
    cfg = DamperConfig()
    worst = VehicleState(v_s=50.0, v_u=(-50.0, 50.0, -50.0, 50.0))
    for state in (worst, VehicleState(v_s=-50.0, v_u=(50.0,) * 4),
                  sim.static_state(SPEC)):
        for value in suspension.semi_active_damping(state, SPEC, cfg):
            assert SPEC.damper_min <= value <= SPEC.damper_max


def test_地棚律在车轮相对车身向上运动时介入():
    cfg = DamperConfig(groundhook_gain=4000.0)
    # 仅车轮向上，车身不动：相对速度向上，地棚条件成立
    state = VehicleState(v_u=(0.4, 0.4, 0.4, 0.4))
    damping = suspension.semi_active_damping(state, SPEC, cfg)
    for corner in range(4):
        assert damping[corner] > SPEC.damper_min


# ---------------------------------------------------------------- 阀门滞后
def test_阀门不是瞬间到位():
    """时间常数越大，第一帧离被动阻尼越近。"""
    scenario = scenarios.brake_dive()
    fast = sim.run(scenario, SPEC, semi_active=True,
                   damper=DamperConfig(valve_tau=0.001))
    slow = sim.run(scenario, SPEC, semi_active=True,
                   damper=DamperConfig(valve_tau=0.2))
    passive = tuple(SPEC.passive_damping(i) for i in range(4))

    def gap(result, count=40):
        return sum(abs(result.frames[i].damping[c] - passive[c])
                   for i in range(count) for c in range(4))

    assert gap(fast) > gap(slow) * 2.0


def test_阀门最终会追到目标阻尼():
    scenario = scenarios.brake_dive()
    result = sim.run(scenario, SPEC, semi_active=True)
    last = result.frames[-1].damping
    target = suspension.semi_active_damping(
        result.frames[-1].state, SPEC, DamperConfig())
    for corner in range(4):
        assert last[corner] == pytest.approx(target[corner], rel=0.05, abs=20.0)


# ---------------------------------------------------------------- 路面
def test_减速带以外路面为零():
    bump = road.BumpRoad(height=0.05, length=0.4, start=3.0)
    assert bump.height(0.0) == 0.0
    assert bump.height(2.4) == 0.0
    assert bump.height(3.6) == 0.0


def test_减速带中点等于设计高度():
    bump = road.BumpRoad(height=0.05, length=0.4, start=3.0)
    assert bump.height(3.2) == pytest.approx(0.05, rel=1e-9)
    assert bump.slope(3.2) == pytest.approx(0.0, abs=1e-6)


def test_ISO路面同一随机种子可复现():
    first = road.IsoRoad(grade='C')
    second = road.IsoRoad(grade='C')
    other = road.IsoRoad(grade='C', seed=7)
    xs = [i * 0.25 for i in range(40)]
    assert [first.height(x) for x in xs] == [second.height(x) for x in xs]
    assert [first.height(x) for x in xs] != [other.height(x) for x in xs]


def test_ISO路面等级越差起伏越大():
    xs = [i * 0.1 for i in range(2000)]

    def rms(grade):
        profile = road.IsoRoad(grade=grade)
        values = [profile.height(x) for x in xs]
        mean = sum(values) / len(values)
        return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))

    assert rms('A') < rms('B') < rms('C')


# ---------------------------------------------------------------- 工况
def test_工况标题只由点阵字支持的字符组成():
    for scenario in scenarios.all_scenarios():
        assert scenario.title == scenario.title.upper()
        assert all(ch.isalnum() or ch in ' -./' for ch in scenario.title)


def test_制动工况的车速不会变成负数():
    scenario = scenarios.brake_dive()
    for t in [i * 0.1 for i in range(40)]:
        assert scenario.velocity(t) >= 0.0


def test_行驶里程单调递增():
    for scenario in scenarios.all_scenarios():
        values = [scenario.travel(t) for t in [i * 0.05 for i in range(60)]]
        assert all(b >= a for a, b in zip(values, values[1:]))


def test_前后轮路面输入相差一个轴距():
    scenario = scenarios.speed_bump()
    heights, _rates = scenarios.road_inputs(scenario, SPEC, 0.7)
    travel = scenario.travel(0.7)
    front = scenario.road.height(travel)
    rear = scenario.road.height(travel - SPEC.wheelbase)
    assert heights[0] == pytest.approx(front)
    assert heights[2] == pytest.approx(rear)


# ---------------------------------------------------------------- 闭环结果
def test_减速带工况半主动把车身加速度压下来(pairs):
    passive, active = _pair(pairs, 'speed_bump')
    assert active.metrics['accel_peak'] < passive.metrics['accel_peak']
    assert active.metrics['accel_rms'] < passive.metrics['accel_rms']


def test_制动工况半主动减小点头(pairs):
    passive, active = _pair(pairs, 'brake_dive')
    assert active.metrics['pitch_deg'] < passive.metrics['pitch_deg']
    assert active.metrics['pitch_deg'] < 0.9 * passive.metrics['pitch_deg']


def test_过弯工况半主动减小侧倾(pairs):
    passive, active = _pair(pairs, 'cornering_roll')
    assert active.metrics['roll_deg'] < passive.metrics['roll_deg']


def test_随机路面半主动兼顾舒适性与接地性(pairs):
    passive, active = _pair(pairs, 'iso_rough')
    assert active.metrics['accel_rms'] < passive.metrics['accel_rms']
    assert active.metrics['load_min'] > passive.metrics['load_min']


def test_稳态姿态由弹簧决定与减振器无关(pairs):
    """半主动只能削减过渡过程，稳态俯仰角两边应当一样。"""
    passive, active = _pair(pairs, 'brake_dive')
    assert active.frames[-1].pitch_deg == pytest.approx(
        passive.frames[-1].pitch_deg, rel=0.02)


def test_稳态俯仰角等于串联刚度下的解析解(pairs):
    """弹簧和轮胎串联，拐角刚度是两者串联值，稳态俯仰角可以手算出来对账。"""
    passive, _active = _pair(pairs, 'brake_dive')

    def series(wheel: float, tire: float) -> float:
        return wheel * tire / (wheel + tire)

    stiffness = [series(SPEC.spring_rate(i), SPEC.tire_stiffness) for i in range(4)]
    total = sum(stiffness)
    first = sum(stiffness[i] * SPEC.axle_position(i) for i in range(4))
    second = sum(stiffness[i] * SPEC.axle_position(i) ** 2 for i in range(4))
    moment = SPEC.sprung_mass * 0.6 * 9.81 * SPEC.cg_height
    theta = moment / (second - first * first / total)

    assert passive.frames[-1].state.pitch == pytest.approx(theta, rel=0.02)


def test_四个工况都能跑完且指标有限(pairs):
    assert len(pairs) == 4
    for name, passive, active in pairs:
        for result in (passive, active):
            assert len(result.frames) > 100
            for key, value in result.metrics.items():
                assert math.isfinite(value), '%s %s' % (name, key)


def test_悬架行程始终在合理范围内(pairs):
    for name, passive, active in pairs:
        for result in (passive, active):
            assert result.metrics['travel_max'] < 0.09, name
            assert result.metrics['load_min'] >= 0.0, name


def test_半主动平均阻尼高于被动():
    """天棚律大多数时候在加硬，平均阻尼理应更大。"""
    for scenario in scenarios.all_scenarios():
        passive, active = sim.run_pair(scenario)
        assert active.metrics['damper_mean'] > passive.metrics['damper_mean']


def test_积分步长减半后结果基本不变(monkeypatch):
    scenario = scenarios.speed_bump()
    coarse = sim.run(scenario, SPEC, semi_active=True).metrics['accel_peak']
    monkeypatch.setattr(sim, 'DT', 0.00025)
    fine = sim.run(scenario, SPEC, semi_active=True).metrics['accel_peak']
    assert fine == pytest.approx(coarse, rel=0.02)


# ---------------------------------------------------------------- 出图
def test_画布能写出合法的PNG(tmp_path):
    canvas = render.Canvas(320, 200)
    canvas.fill_rect(0, 0, 320, 200, render.BG)
    canvas.text(20, 20, 'SEMI ACTIVE SUSPENSION', render.TEXT)
    path = os.path.join(str(tmp_path), 'probe.png')
    render.write_png(path, canvas)
    with open(path, 'rb') as handle:
        header = handle.read(8)
    assert header == b'\x89PNG\r\n\x1a\n'
    assert os.path.getsize(path) > 200


def test_四张图都能生成(pairs):
    built = [
        scene.bump_figure(pairs[0][1], pairs[0][2]),
        scene.dive_figure(pairs[1][1], pairs[1][2]),
        scene.roll_figure(pairs[2][1], pairs[2][2]),
        scene.summary_figure([(p[1], p[2]) for p in pairs]),
    ]
    for canvas in built:
        assert canvas.w > 0 and canvas.h > 0


def test_命令行只打印指标时正常退出(capsys):
    assert main.main(['--no-images', '--scenario', 'speed_bump']) == 0
    assert '车身加速度' in capsys.readouterr().out


def test_四个车轮的编号与名称一致():
    assert CORNERS == ('FL', 'FR', 'RL', 'RR')
    assert suspension.corner_names() == CORNERS
