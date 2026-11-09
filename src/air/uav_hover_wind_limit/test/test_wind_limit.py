"""检查悬停抗风核算: 倾角-风速关系、推重比余量、空气密度与蒲福风级.

期望值都能手算或由公式直接核对: tan(theta) = 0.5*rho*CdA*V^2/(m*g),
最大抗风由 theta_max 反解, 推重比 = 1/cos(theta_max), 密度按 ISA 近似.
"""
import io
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uav_hover_wind_limit import cli  # noqa: E402
from uav_hover_wind_limit.core import (  # noqa: E402
    RHO0,
    HoverWindLimit,
    air_density,
    beaufort_level,
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
