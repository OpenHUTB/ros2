"""ESC 模块的离线测试：轮胎力、载荷转移、控制律与四个工况的闭环结果。"""
from __future__ import annotations

import math
import struct

import pytest

from vehicle_esc_stability import render, scenarios, scene, sim
from vehicle_esc_stability.dynamics import RoadSpec, _load_transfer, rk4_step, wheel_forces
from vehicle_esc_stability.driver import PurePursuitDriver, ReferencePath
from vehicle_esc_stability.esc import Esc
from vehicle_esc_stability.params import (
    ControlInput,
    EscConfig,
    G,
    TireSpec,
    VehicleSpec,
    VehicleState,
)

DEG = math.pi / 180.0


@pytest.fixture(scope='module')
def double_lane_change():
    scenario = scenarios.lane_change()
    return sim.run(scenario, esc_enabled=False), sim.run(scenario, esc_enabled=True)


@pytest.fixture(scope='module')
def split_mu():
    scenario = scenarios.mu_split()
    return sim.run(scenario, esc_enabled=False), sim.run(scenario, esc_enabled=True)


@pytest.fixture(scope='module')
def wet_lane_change():
    scenario = scenarios.wet_lane_change()
    return sim.run(scenario, esc_enabled=False), sim.run(scenario, esc_enabled=True)


# ------------------------------------------------------------------ 轮胎与载荷
def test_静止时前后轴载荷按轴距分配():
    spec = VehicleSpec()
    fl, fr, rl, rr = _load_transfer(spec, 0.0, 0.0)
    assert fl + fr + rl + rr == pytest.approx(spec.weight, rel=1e-9)
    # 质心离前轴更近（a < b），所以前轴载荷更大
    assert fl + fr > rl + rr
    assert fl == pytest.approx(fr)


def test_纵向加速度把载荷压向后轴():
    spec = VehicleSpec()
    fl, fr, rl, rr = _load_transfer(spec, 3.0, 0.0)
    fl0, fr0, rl0, rr0 = _load_transfer(spec, 0.0, 0.0)
    assert rl > rl0  # 加速时后轴加载
    assert fl < fl0  # 前轴同时减载
    # 转移总量守恒
    assert (rl - rl0) + (rr - rr0) == pytest.approx((fl0 - fl) + (fr0 - fr), rel=1e-9)


def test_侧向加速度把载荷压向弯道外侧():
    spec = VehicleSpec()
    fl, fr, rl, rr = _load_transfer(spec, 0.0, 5.0)
    # ay 为正表示向左加速，载荷压向右侧车轮
    assert fr > fl
    assert rr > rl


def test_侧偏力在峰值侧偏角处取到附着极限():
    tire = TireSpec()
    fz = 4000.0
    peak = tire.peak_slip_angle
    peak_force = abs(tire.lateral(peak, 0.9, fz))
    assert peak_force == pytest.approx(0.9 * fz, rel=0.02)
    # 小于峰值时斜率随角度增加而变缓
    small = abs(tire.lateral(0.1 * peak, 0.9, fz))
    assert small < peak_force


def test_侧偏力符号与侧偏角相反():
    tire = TireSpec()
    assert tire.lateral(0.02, 0.9, 4000.0) < 0.0
    assert tire.lateral(-0.02, 0.9, 4000.0) > 0.0


def test_制动会挤占侧向附着():
    """同一侧偏角下踩制动，附着椭圆把侧向力压小。"""
    spec, tire = VehicleSpec(), TireSpec()
    state = VehicleState(u=20.0, v=1.0, r=0.2)
    road = RoadSpec(0.9, 0.9)

    free = ControlInput(steer=0.0, driver_brake=0.0, wheel_brake=[0.0] * 4)
    braked = ControlInput(steer=0.0, driver_brake=1.0, wheel_brake=[1.0] * 4)

    fy_free = abs(wheel_forces(state, free, spec, tire, road).wheels[2].lateral_force)
    fy_brake = abs(wheel_forces(state, braked, spec, tire, road).wheels[2].lateral_force)
    assert fy_brake < fy_free


def test_制动力不超过附着上限():
    spec, tire = VehicleSpec(), TireSpec()
    state = VehicleState(u=20.0)
    control = ControlInput(steer=0.0, driver_brake=1.0, wheel_brake=[1.0] * 4)
    bundle = wheel_forces(state, control, spec, tire, RoadSpec(0.9, 0.9))
    for wheel in bundle.wheels:
        assert abs(wheel.longitudinal_force) <= wheel.mu * wheel.normal_force + 1e-6


def test_倒车时制动力方向反向():
    """车轮反转时制动力必须跟着翻号，否则越刹越快。"""
    spec, tire = VehicleSpec(), TireSpec()
    state = VehicleState(u=-2.0)
    control = ControlInput(steer=0.0, driver_brake=1.0, wheel_brake=[1.0] * 4)
    bundle = wheel_forces(state, control, spec, tire, RoadSpec(0.9, 0.9))
    assert all(wheel.longitudinal_force > 0.0 for wheel in bundle.wheels)


# ------------------------------------------------------------------ 参考量
def test_参考横摆角速度受附着限制():
    esc = Esc(EscConfig(), VehicleSpec())
    state = VehicleState(u=30.0)
    r_ref = esc.reference_yaw_rate(state, 20.0 * DEG, RoadSpec(0.9, 0.9))
    limit = 0.9 * 0.9 * G / 30.0
    assert r_ref == pytest.approx(limit, rel=1e-9)


def test_参考横摆角速度在低速时按运动学计算():
    esc = Esc(EscConfig(), VehicleSpec())
    state = VehicleState(u=10.0)
    steer = 2.0 * DEG
    r_ref = esc.reference_yaw_rate(state, steer, RoadSpec(0.9, 0.9))
    kinematic = 10.0 * math.tan(steer) / 2.5
    assert r_ref == pytest.approx(kinematic, rel=1e-6)
    assert r_ref < 0.9 * 0.9 * G / 10.0


def test_静止时参考横摆角速度为零():
    esc = Esc(EscConfig(), VehicleSpec())
    assert esc.reference_yaw_rate(VehicleState(u=0.0), 10.0 * DEG, RoadSpec()) == 0.0


# ------------------------------------------------------------------ 控制律
def test_esc_关闭时不改制动指令():
    esc = Esc(EscConfig(enabled=False), VehicleSpec())
    state = VehicleState(u=20.0, r=0.5)
    control = ControlInput(steer=0.05, driver_brake=0.3, wheel_brake=[0.3] * 4)
    out = esc.step(state, control, RoadSpec(0.9, 0.9), 0.02)
    assert out.wheel_brake == [0.3] * 4
    assert out.esc_active is False


def test_稳态直行时不介入():
    esc = Esc(EscConfig(), VehicleSpec())
    state = VehicleState(u=20.0, r=0.0)
    control = ControlInput(steer=0.0, driver_brake=0.0, wheel_brake=[0.0] * 4)
    out = esc.step(state, control, RoadSpec(0.9, 0.9), 0.02)
    assert out.wheel_brake == [0.0] * 4
    assert out.esc_intensity == pytest.approx(0.0)


def test_转向不足时给左侧车轮加压():
    """实际横摆角速度小于期望时，需要一个左转力矩。"""
    esc = Esc(EscConfig(), VehicleSpec())
    state = VehicleState(u=20.0, r=0.0)
    control = ControlInput(steer=4.0 * DEG, driver_brake=0.0, wheel_brake=[0.0] * 4)
    out = esc.step(state, control, RoadSpec(0.9, 0.9), 0.02)
    assert out.wheel_brake[0] > 0.0
    assert out.wheel_brake[2] > 0.0
    assert out.wheel_brake[0] == pytest.approx(out.wheel_brake[2])
    assert esc.status.direction == 1


def test_转向过度时给右侧车轮加压():
    esc = Esc(EscConfig(), VehicleSpec())
    state = VehicleState(u=20.0, r=0.6)
    control = ControlInput(steer=0.0, driver_brake=0.0, wheel_brake=[0.0] * 4)
    out = esc.step(state, control, RoadSpec(0.9, 0.9), 0.02)
    assert out.wheel_brake[1] > 0.0
    assert out.wheel_brake[3] > 0.0
    assert esc.status.direction == -1


def test_侧偏角超限时四轮一起补制动():
    esc = Esc(EscConfig(), VehicleSpec())
    state = VehicleState(u=20.0, v=6.0, r=0.1)  # β 约 17 deg
    control = ControlInput(steer=0.0, driver_brake=0.0, wheel_brake=[0.0] * 4)
    out = esc.step(state, control, RoadSpec(0.9, 0.9), 0.02)
    assert all(value > 0.0 for value in out.wheel_brake)


def _only_sideslip_config(enabled: bool) -> EscConfig:
    """只留侧偏角项，把横摆误差、积分、微分、附着差前馈都关掉，隔离出这一项的作用。"""
    return EscConfig(
        enabled=enabled,
        yaw_gain=0.0,
        yaw_integral_gain=0.0,
        yaw_rate_gain=0.0,
        split_mu_feedforward=0.0,
        speed_brake_gain=0.0,
    )


def _drop_lateral(state: VehicleState, enabled: bool) -> tuple[float, float]:
    """无转向、无制动，让车自己衰减侧偏角，返回 0.5 s 后的 (|β|, r)。"""
    spec, tire = VehicleSpec(), TireSpec()
    road = RoadSpec(0.9, 0.9)
    esc = Esc(_only_sideslip_config(enabled), spec)
    ax = ay = 0.0
    for _ in range(int(0.5 / sim.DT)):
        control = ControlInput(steer=0.0, driver_brake=0.0, wheel_brake=[0.0] * 4)
        control = esc.step(state, control, road, sim.CONTROL_DT)
        state, bundle = rk4_step(state, control, spec, tire, road, sim.DT, ax, ay)
        ax, ay = bundle.ax, bundle.ay
    return abs(state.sideslip), state.r


def test_侧偏角为正时压左轮把车头往左带():
    """β>0 表示速度矢量偏在车头左侧，要减小 β 得给正（左转）力矩。

    第一版这里取了负号（认为「车尾向左甩要用负力矩」），结果这一项和横摆误差项
    互相抵消：同一初始条件下 β 从 5.7° 只衰减到 1.16°，关掉 ESC 反而能到 0.48°。
    """
    esc = Esc(_only_sideslip_config(True), VehicleSpec())
    state = VehicleState(u=20.0, v=3.0, r=0.0)  # β 约 8.5 deg，超过 5.7 deg 软阈值
    control = ControlInput(steer=0.0, driver_brake=0.0, wheel_brake=[0.0] * 4)
    out = esc.step(state, control, RoadSpec(0.9, 0.9), 0.02)
    assert esc.status.direction == 1
    assert out.wheel_brake[0] > out.wheel_brake[1]
    assert out.wheel_brake[2] > out.wheel_brake[3]


def test_侧偏角为负时压右轮():
    esc = Esc(_only_sideslip_config(True), VehicleSpec())
    state = VehicleState(u=20.0, v=-3.0, r=0.0)
    control = ControlInput(steer=0.0, driver_brake=0.0, wheel_brake=[0.0] * 4)
    out = esc.step(state, control, RoadSpec(0.9, 0.9), 0.02)
    assert esc.status.direction == -1
    assert out.wheel_brake[1] > out.wheel_brake[0]


def test_开_esc_后侧偏角衰减不得慢于关_esc():
    """方向性单测：只留侧偏角项时，介入后的 β 必须比不介入掉得更快。"""
    off_beta, off_r = _drop_lateral(VehicleState(u=20.0, v=3.0), False)
    on_beta, on_r = _drop_lateral(VehicleState(u=20.0, v=3.0), True)
    assert on_beta < off_beta
    # 车辆自稳定会产生负的横摆角速度（回弹），介入后这个回弹应被明显削弱
    assert on_r > off_r


def test_驾驶员全力制动时只减压不加压():
    """低附着侧再加压只会让它先锁死，μ-split 工况靠释放高附着侧。"""
    esc = Esc(EscConfig(), VehicleSpec())
    state = VehicleState(u=25.0)
    control = ControlInput(steer=0.0, driver_brake=0.85, wheel_brake=[0.85] * 4)
    out = esc.step(state, control, RoadSpec(0.95, 0.30), 0.02)
    left = out.wheel_brake[0]
    right = out.wheel_brake[1]
    assert left < right
    assert right == pytest.approx(0.85)


def test_强度一阶低通会削弱突变():
    """`intensity_tau` 默认关，这里只验证它打开后确实把第一拍削弱了。"""
    state = VehicleState(u=20.0, r=0.0)
    road = RoadSpec(0.9, 0.9)

    def first_step(config):
        # step() 会原地改写 control 并返回它，所以每次都得给一个新的
        control = ControlInput(steer=4.0 * DEG, driver_brake=0.0, wheel_brake=[0.0] * 4)
        return Esc(config, VehicleSpec()).step(state, control, road, 0.02)

    plain = first_step(EscConfig())
    filtered = first_step(EscConfig(intensity_tau=0.05))
    assert plain.esc_intensity == pytest.approx(1.0)
    assert 0.0 < filtered.esc_intensity < plain.esc_intensity * 0.5


def test_介入时长与峰值会被统计():
    esc = Esc(EscConfig(), VehicleSpec())
    state = VehicleState(u=20.0, r=0.0)
    for _ in range(10):
        control = ControlInput(steer=3.0 * DEG, wheel_brake=[0.0] * 4)
        esc.step(state, control, RoadSpec(0.9, 0.9), 0.02)
    stats = esc.stats
    assert stats['active_time'] == pytest.approx(0.2, rel=0.05)
    assert 0.0 < stats['peak_intensity'] <= 1.0


def test_reset_清空积分与统计():
    esc = Esc(EscConfig(), VehicleSpec())
    state = VehicleState(u=20.0)
    control = ControlInput(steer=3.0 * DEG, wheel_brake=[0.0] * 4)
    esc.step(state, control, RoadSpec(0.9, 0.9), 0.02)
    esc.reset()
    assert esc.stats['active_time'] == 0.0
    assert esc.status.intensity == 0.0


# ------------------------------------------------------------------ 驾驶员
def test_路径投影给出横向误差与弧长():
    path = ReferencePath.double_lane_change(offset=3.5, x_start=10.0, length=40.0)
    s, cross, heading = path.project(0.0, 0.0)
    assert s == pytest.approx(0.0, abs=1e-6)
    assert cross == pytest.approx(0.0, abs=1e-6)

    s_mid, cross_mid, heading = path.project(30.0, 3.75)
    assert cross_mid == pytest.approx(2.0, abs=0.1)  # 在路径左侧 2 m
    assert s_mid == pytest.approx(30.0, abs=1.0)
    assert heading > 0.0  # 换道段路径朝左偏


def test_路径末端按最后方向外推():
    path = ReferencePath.double_lane_change(offset=3.5, x_start=10.0, length=40.0)
    x, y = path.sample(500.0)
    assert x > 210.0
    assert y == pytest.approx(3.5, abs=0.2)


def test_纯跟踪在直路上方向盘回正():
    path = ReferencePath([(x * 0.5, 0.0) for x in range(0, 400)])
    driver = PurePursuitDriver(path, wheelbase=2.5)
    for _ in range(20):
        steer = driver.steer(20.0, 0.0, 0.0, 20.0, 0.02)
    assert abs(steer) < 1e-6


def test_纯跟踪会朝参考线打方向():
    path = ReferencePath([(x * 0.5, 0.0) for x in range(0, 400)])
    driver = PurePursuitDriver(path, wheelbase=2.5)
    steer = 0.0
    for _ in range(10):
        steer = driver.steer(20.0, 1.0, 0.0, 20.0, 0.02)  # 偏在参考线左侧
    assert steer < 0.0  # 往右打，回到参考线


def test_前视距离随车速增大():
    driver = PurePursuitDriver(ReferencePath([(0.0, 0.0), (10.0, 0.0)]))
    assert driver.lookahead(30.0) > driver.lookahead(10.0)


# ------------------------------------------------------------------ 闭环结果
def test_双移线无_esc_会甩尾(double_lane_change):
    off, on = double_lane_change
    assert off.metrics['spin'] == 1.0
    assert off.metrics['max_beta_deg'] > 45.0
    assert abs(off.metrics['final_offset_m']) > 5.0


def test_双移线有_esc_守住路径(double_lane_change):
    _, on = double_lane_change
    assert on.metrics['spin'] == 0.0
    assert on.metrics['max_beta_deg'] < 10.0
    assert on.metrics['max_cross_track_m'] < 1.5


def test_正弦停滞靠_esc_降低跟踪误差():
    scenario = scenarios.sine_dwell()
    off = sim.run(scenario, esc_enabled=False)
    on = sim.run(scenario, esc_enabled=True)
    assert on.metrics['yaw_error_rms_dps'] < off.metrics['yaw_error_rms_dps'] * 0.5
    # 这个工况本身不甩尾，ESC 只是把误差压小
    assert off.metrics['spin'] == 0.0
    assert on.metrics['spin'] == 0.0


def test_对开路面无_esc_会被拽偏甩尾(split_mu):
    off, on = split_mu
    assert off.metrics['spin'] == 1.0
    assert off.metrics['max_beta_deg'] > 45.0
    assert on.metrics['spin'] == 0.0
    assert on.metrics['max_beta_deg'] < 3.0


def test_对开路面_esc_的制动距离变长(split_mu):
    """select-low 的代价：车不甩了，但制动力被低附着侧拖住。

    注意两侧的停车姿态不同：没 ESC 那次是横滑过去、被侧向阻力拽停的（57.6 m），
    有 ESC 那次是直线停住（169.5 m）。所以这里的倍数只能说明「保住方向要付多少
    纵向距离」，不能当成纯制动力差异。
    """
    off, on = split_mu
    assert off.metrics['stopped'] == 1.0
    assert on.metrics['stopped'] == 1.0
    assert on.metrics['stop_distance_m'] > off.metrics['stop_distance_m'] * 2.5
    assert on.metrics['final_speed_kph'] == pytest.approx(0.0, abs=1.0)


def test_停车判定用合速度而不是纵向分量(split_mu):
    """甩尾横滑时 u 会先过零而车身还在几十 km/h 侧滑。

    早先用 `state.u < 0.5` 判定，把 36.6 m 记成「制动距离」，而那一刻车其实还在
    61 km/h 横滑、最终停在 57.6 m。这条测试守住口径。
    """
    off, _ = split_mu
    assert off.metrics['stop_distance_m'] > 50.0
    assert off.metrics['stop_time_s'] > 4.0
    assert off.metrics['spin'] == 1.0


def test_死区减少左右轮交替加压():
    """0.02 rad/s 的死区：方向翻转次数明显减少，侧偏角几乎不变。"""
    scenario = scenarios.lane_change()

    def flips(result):
        count, previous = 0, 0
        for frame in result.frames:
            direction = frame.esc.direction
            if direction and previous and direction != previous:
                count += 1
            if direction:
                previous = direction
        return count

    noisy = sim.run(scenario, esc_config=EscConfig(yaw_rate_deadband=0.0))
    smooth = sim.run(scenario, esc_config=EscConfig())
    assert flips(smooth) < flips(noisy)
    assert smooth.metrics['max_beta_deg'] < noisy.metrics['max_beta_deg'] + 0.2


def test_湿滑路面_esc_把车拉回来(wet_lane_change):
    off, on = wet_lane_change
    assert off.metrics['spin'] == 1.0
    assert on.metrics['spin'] == 0.0
    assert on.metrics['max_beta_deg'] < off.metrics['max_beta_deg'] * 0.2
    assert on.metrics['max_cross_track_m'] < 1.0


def test_四个工况的_esc_都不甩尾():
    for scenario in scenarios.all_scenarios():
        on = sim.run(scenario, esc_enabled=True)
        assert on.metrics['spin'] == 0.0, scenario.name
        assert on.metrics['esc_active_s'] > 0.0, scenario.name


def test_按名字取工况():
    assert scenarios.by_name('mu_split').road.mu_right == pytest.approx(0.30)
    with pytest.raises(KeyError):
        scenarios.by_name('not_a_scenario')


# ------------------------------------------------------------------ 出图
def test_trajectory_figure_尺寸正确(double_lane_change):
    off, on = double_lane_change
    canvas = scene.trajectory_figure(off, on)
    assert (canvas.w, canvas.h) == (1180, 660)


def test_写出的_png_是合法文件(tmp_path, double_lane_change):
    off, on = double_lane_change
    path = tmp_path / 'esc.png'
    render.write_png(str(path), scene.summary_figure([(off, on)]))
    data = path.read_bytes()
    assert data[:8] == b'\x89PNG\r\n\x1a\n'
    width, height = struct.unpack('>II', data[16:24])
    assert (width, height) == (1180, 548)


def test_动画帧数受上限约束(double_lane_change):
    off, on = double_lane_change
    frames = scene.animation_frames(off, on, max_frames=12)
    assert 0 < len(frames) <= 14
    assert frames[0].w == 900
    # 两栏的高度按等比例反推，不该出现大片空白
    assert 200 < frames[0].h < 320
