"""检查悬停抗风核算: 倾角-风速关系、推重比余量、空气密度与蒲福风级.

期望值都能手算或由公式直接核对: tan(theta) = 0.5*rho*CdA*V^2/(m*g),
最大抗风由 theta_max 反解, 推重比 = 1/cos(theta_max), 密度按 ISA 近似.
"""
import io
import json
import math
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uav_hover_wind_limit import cli  # noqa: E402
from uav_hover_wind_limit.core import (  # noqa: E402
    RHO0,
    HoverWindLimit,
    air_density,
    beaufort_level,
    build_limits,
)


@pytest.fixture
def limits():
    return HoverWindLimit(mass=2.0, drag_area=0.15, max_tilt_deg=25.0)


def test_drag_and_tilt_identity(limits):
    """拖曳力与倾角必须满足 tan(theta) = D / (m*g)."""
    for wind in (0.0, 2.0, 5.0, 10.0):
        drag = 0.5 * RHO0 * 0.15 * wind ** 2
        assert limits.drag_n(wind) == pytest.approx(drag)
        assert math.tan(math.radians(limits.tilt_for_wind(wind))) == pytest.approx(
            drag / limits.weight_n)


def test_wind_and_tilt_are_inverse(limits):
    for tilt in (0.0, 5.0, 12.5, 25.0, 40.0):
        wind = limits.wind_for_tilt(tilt)
        assert limits.tilt_for_wind(wind) == pytest.approx(tilt, abs=1e-9)
    for wind in (0.0, 3.0, 9.9):
        assert limits.wind_for_tilt(limits.tilt_for_wind(wind)) == pytest.approx(wind, abs=1e-9)


def test_max_wind_closed_form(limits):
    expected = math.sqrt(2.0 * limits.mass * limits.gravity * math.tan(math.radians(25.0))
                         / (RHO0 * 0.15))
    assert limits.max_wind_m_s == pytest.approx(expected, rel=1e-12)
    # 2 kg / 0.15 m^2 / 25 度 -> 约 9.98 m/s
    assert limits.max_wind_m_s == pytest.approx(9.976, abs=0.01)


def test_thrust_margin(limits):
    """最大倾角处所需推重比 = 1/cos(theta_max)."""
    assert limits.thrust_to_weight == pytest.approx(1.0 / math.cos(math.radians(25.0)))
    assert limits.thrust_to_weight == pytest.approx(1.1034, abs=1e-4)
    assert limits.thrust_increase_percent == pytest.approx(10.34, abs=0.02)
    for wind in (0.0, 4.0, 8.0):
        expected = 1.0 / math.cos(math.radians(limits.tilt_for_wind(wind)))
        assert limits.thrust_to_weight_at(wind) == pytest.approx(expected)


def test_scaling_laws(limits):
    """抗风上限与 sqrt(m / (rho * CdA)) 成正比."""
    heavier = HoverWindLimit(mass=limits.mass * 4.0, drag_area=limits.drag_area,
                             max_tilt_deg=limits.max_tilt_deg)
    assert heavier.max_wind_m_s == pytest.approx(2.0 * limits.max_wind_m_s, rel=1e-12)

    bigger = HoverWindLimit(mass=limits.mass, drag_area=limits.drag_area * 4.0,
                            max_tilt_deg=limits.max_tilt_deg)
    assert bigger.max_wind_m_s == pytest.approx(0.5 * limits.max_wind_m_s, rel=1e-12)


def test_altitude_lowers_density_and_raises_capability(limits):
    sea = air_density(0.0)
    high = air_density(2000.0)
    assert sea == pytest.approx(RHO0)
    assert high < sea
    high_limits = HoverWindLimit(mass=limits.mass, drag_area=limits.drag_area,
                                 max_tilt_deg=limits.max_tilt_deg, rho=high)
    assert high_limits.max_wind_m_s == pytest.approx(
        limits.max_wind_m_s * math.sqrt(sea / high), rel=1e-12)


def test_beaufort_boundaries():
    cases = [(0.0, 0), (0.2, 0), (0.3, 1), (1.5, 1), (1.6, 2), (3.3, 2), (3.4, 3),
             (5.4, 3), (5.5, 4), (8.0, 5), (10.8, 6), (13.9, 7), (17.2, 8),
             (20.8, 9), (24.5, 10), (28.5, 11), (32.7, 12), (40.0, 12)]
    for speed, level in cases:
        assert beaufort_level(speed) == level, speed


def test_gust_requires_extra_tilt(limits):
    extra = limits.gust_tilt_deg(limits.max_wind_m_s, 3.0)
    assert 0.0 < extra < 90.0 - limits.max_tilt_deg
    assert limits.gust_tilt_deg(limits.max_wind_m_s, 0.0) == pytest.approx(0.0)
    with pytest.raises(ValueError):
        limits.gust_tilt_deg(limits.max_wind_m_s, -1.0)


def test_report_is_consistent(limits):
    report = limits.report()
    assert report['max_wind_beaufort'] == beaufort_level(report['max_wind_m_s'])
    assert report['thrust_to_weight'] == pytest.approx(limits.thrust_to_weight, abs=1e-4)
    sweep = report['tilt_sweep']
    assert len(sweep) == 7
    assert sweep[0]['tilt_deg'] == pytest.approx(0.0)
    assert sweep[-1]['tilt_deg'] == pytest.approx(25.0)
    assert sweep[-1]['wind_m_s'] == pytest.approx(limits.max_wind_m_s, abs=1e-4)
    assert sweep[0]['wind_m_s'] == pytest.approx(0.0)
    winds = [row['wind_m_s'] for row in sweep]
    assert winds == sorted(winds)


def test_invalid_parameters_are_rejected(limits):
    for kwargs in ({'mass': 0.0}, {'mass': -1.0}, {'drag_area': 0.0},
                   {'max_tilt_deg': 0.0}, {'max_tilt_deg': 90.0},
                   {'rho': 0.0}, {'gravity': -1.0}):
        base = {'mass': 2.0, 'drag_area': 0.15, 'max_tilt_deg': 25.0}
        base.update(kwargs)
        with pytest.raises(ValueError):
            HoverWindLimit(**base)
    with pytest.raises(ValueError):
        limits.wind_for_tilt(-1.0)
    with pytest.raises(ValueError):
        limits.tilt_for_wind(-0.5)
    with pytest.raises(ValueError):
        limits.drag_n(float('nan'))
    with pytest.raises(ValueError):
        beaufort_level(-0.1)
    with pytest.raises(ValueError):
        air_density(12000.0)
    with pytest.raises(ValueError):
        air_density(-600.0)


def test_print_report_is_gbk_safe(limits):
    """Windows 默认 GBK 控制台下打印不能抛 UnicodeEncodeError."""
    buffer = io.BytesIO()
    wrapper = io.TextIOWrapper(buffer, encoding='gbk', errors='strict')
    original = sys.stdout
    sys.stdout = wrapper
    try:
        cli.print_report(limits.report(), altitude=0.0)
        wrapper.flush()
    finally:
        sys.stdout = original
    text = buffer.getvalue().decode('gbk')
    assert 'uav_hover_wind_limit' in text
    assert '蒲福' in text
    assert 'm^2' in text


def test_cli_smoke_writes_outputs(tmp_path):
    """跑真实入口: 出 2 张 PNG 与 report.json (无显示环境也能跑)."""
    pytest.importorskip('matplotlib')
    output = tmp_path / 'out'
    assert cli.main(['--output', str(output)]) == 0
    names = sorted(path.name for path in output.iterdir())
    assert names == ['capability.png', 'report.json', 'wind_vs_tilt.png']
    report = json.loads((output / 'report.json').read_text(encoding='utf-8'))
    assert report['max_wind_beaufort'] == 5
    assert report['thrust_to_weight'] == pytest.approx(1.1034, abs=1e-4)


def test_configure_console_tolerates_streams_without_reconfigure():
    original = sys.stdout
    sys.stdout = io.StringIO()
    try:
        assert cli._configure_console() is None
    finally:
        sys.stdout = original


# ---------------------------------------------------------------- 倾角与推力余量耦合


def test_thrust_margin_limits_effective_tilt():
    """实际 T/W=1.05 时可达倾角只有 acos(1/1.05)≈17.75 度，抗风随之下降."""
    limited = HoverWindLimit(mass=2.0, drag_area=0.15, max_tilt_deg=25.0, available_tw=1.05)
    assert limited.achievable_tilt_deg == pytest.approx(17.753, abs=1e-3)
    assert limited.effective_tilt_deg == pytest.approx(17.753, abs=1e-3)
    assert limited.tilt_limited_by == 'thrust_margin'
    # 抗风按 tan(17.75)/tan(25) 的平方根缩放
    ratio = math.sqrt(math.tan(math.radians(17.753)) / math.tan(math.radians(25.0)))
    assert limited.max_wind_m_s == pytest.approx(9.9773 * ratio, rel=1e-3)
    assert limited.max_wind_m_s < 9.98
    # 姿态上限本身要求的推重比仍按 25 度算
    assert limited.tilt_limit_thrust_to_weight == pytest.approx(1.0 / math.cos(math.radians(25.0)))
    assert limited.thrust_to_weight == pytest.approx(1.05, abs=1e-3)


def test_tilt_limit_wins_when_thrust_margin_is_enough():
    generous = HoverWindLimit(mass=2.0, drag_area=0.15, max_tilt_deg=25.0, available_tw=1.30)
    assert generous.tilt_limited_by == 'tilt_limit'
    assert generous.effective_tilt_deg == pytest.approx(25.0)
    assert generous.max_wind_m_s == pytest.approx(9.9773, abs=0.01)


def test_report_exposes_tilt_budget_and_notes():
    limited = HoverWindLimit(mass=2.0, drag_area=0.15, max_tilt_deg=25.0, available_tw=1.05)
    report = limited.report()
    budget = report['tilt_budget']
    assert budget['limited_by'] == 'thrust_margin'
    assert budget['achievable_tilt_deg'] == pytest.approx(17.753, abs=1e-2)
    assert budget['tilt_limit_thrust_to_weight'] == pytest.approx(1.1034, abs=1e-4)
    assert report['parameters']['available_thrust_to_weight'] == pytest.approx(1.05)
    assert report['tilt_sweep'][-1]['tilt_deg'] == pytest.approx(17.753, abs=1e-2)
    assert any('密度' in note for note in report['notes'])


def test_invalid_thrust_margin_is_rejected():
    for bad in (1.0, 0.95, -1.0, float('nan')):
        with pytest.raises(ValueError):
            HoverWindLimit(mass=2.0, drag_area=0.15, max_tilt_deg=25.0, available_tw=bad)


def test_build_limits_helper_maps_altitude_and_tw():
    sea = build_limits(2.0, 0.15, 25.0)
    high = build_limits(2.0, 0.15, 25.0, altitude_m=2000.0)
    assert sea.rho == pytest.approx(RHO0)
    assert high.rho == pytest.approx(air_density(2000.0))
    assert high.max_wind_m_s > sea.max_wind_m_s
    coupled = build_limits(2.0, 0.15, 25.0, 0.0, 1.05)
    assert coupled.tilt_limited_by == 'thrust_margin'


# ---------------------------------------------------------------- ROS 节点（用假 rclpy 验证接线）


def _install_fake_ros():
    """把最小 rclpy/std_msgs 替身塞进 sys.modules, 便于在没有 ROS 的机器上测节点."""
    class _Node:
        def __init__(self, name):
            self.name = name
            self.parameters = {}
            self.logs = []
            self.publisher = None

        def declare_parameter(self, name, value):
            self.parameters[name] = value

        def get_parameter(self, name):
            return types.SimpleNamespace(value=self.parameters[name])

        def create_publisher(self, msg_type, topic, qos):
            node = self

            class _Publisher:
                def __init__(self):
                    self.topic = topic
                    self.messages = []

                def publish(self, message):
                    node.logs.append(('publish', message.data))
                    self.messages.append(message)

            self.publisher = _Publisher()
            return self.publisher

        def create_timer(self, period, callback):
            self.timer_period = period
            self.timer_callback = callback
            return object()

        def get_logger(self):
            node = self

            class _Logger:
                def info(self, text):
                    node.logs.append(('info', text))

            return _Logger()

    class _String:
        def __init__(self, data=''):
            self.data = data

    rclpy = types.ModuleType('rclpy')
    node_module = types.ModuleType('rclpy.node')
    node_module.Node = _Node
    qos_module = types.ModuleType('rclpy.qos')
    qos_module.QoSProfile = lambda **kwargs: dict(kwargs)
    qos_module.QoSReliabilityPolicy = types.SimpleNamespace(RELIABLE='reliable')
    qos_module.QoSDurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL='transient_local')
    qos_module.QoSHistoryPolicy = types.SimpleNamespace(KEEP_LAST='keep_last')
    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')
    std_msgs_msg.String = _String
    modules = {
        'rclpy': rclpy,
        'rclpy.node': node_module,
        'rclpy.qos': qos_module,
        'std_msgs': std_msgs,
        'std_msgs.msg': std_msgs_msg,
    }
    sys.modules.update(modules)
    sys.modules.pop('uav_hover_wind_limit.ros_node', None)
    return modules


def test_ros_node_publishes_report_without_ros(monkeypatch):
    """节点构造时不连 ROS: 参数 → 计算 → 发布 JSON, 用假 rclpy 验证这条链路."""
    modules = _install_fake_ros()
    try:
        from uav_hover_wind_limit import ros_node

        node = ros_node.WindLimitNode()
        assert node.name == 'uav_hover_wind_limit'
        assert node.publisher.topic == 'uav/wind_limit'
        assert node.publisher.messages, '构造时应先发一次'
        payload = json.loads(node.publisher.messages[-1].data)
        assert payload['max_wind_beaufort'] == 5
        assert payload['max_wind_m_s'] == pytest.approx(9.9773, abs=0.01)
        # 参数里给了实际推重比时按耦合口径输出
        limited = ros_node.build_from_parameters({
            'mass': 2.0, 'drag_area': 0.15, 'max_tilt_deg': 25.0,
            'altitude_m': 0.0, 'available_thrust_to_weight': 1.05,
        })
        assert limited.tilt_limited_by == 'thrust_margin'
        # 定时器周期来自参数, 且发布一次即可由 callback 复现
        assert node.timer_period == pytest.approx(5.0)
        before = len(node.publisher.messages)
        node.timer_callback()
        assert len(node.publisher.messages) == before + 1
    finally:
        for name in modules:
            sys.modules.pop(name, None)
        sys.modules.pop('uav_hover_wind_limit.ros_node', None)
