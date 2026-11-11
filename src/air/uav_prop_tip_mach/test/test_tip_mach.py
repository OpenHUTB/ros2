"""检查桨尖马赫数、桨盘载荷与诱导速度的闭式解与边界.

期望值都能手算: a = sqrt(gamma*R*T), v_tip = pi*D*rpm/60, M = v_tip/a,
A = pi*(D/2)^2*n, DL = W/A, v_i = sqrt(DL/(2*rho)).
"""
import io
import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uav_prop_tip_mach import cli  # noqa: E402
from uav_prop_tip_mach.core import (  # noqa: E402
    DISK_LOADING_LOW,
    MACH_MARGIN,
    GAMMA,
    RHO0,
    R_SPECIFIC,
    PropCheck,
    disk_area,
    disk_loading,
    disk_loading_level,
    induced_velocity,
    mach_level,
    speed_of_sound,
    tip_mach,
    tip_speed,
)


@pytest.fixture
def check():
    return PropCheck(mass=2.0, diameter_m=0.254, rotor_count=4,
                     hover_rpm=6500.0, max_rpm=9000.0, temperature_c=15.0)


def test_speed_of_sound():
    """15 摄氏度声速约 340.29 m/s；温度升高声速变大."""
    assert speed_of_sound(15.0) == pytest.approx(math.sqrt(GAMMA * R_SPECIFIC * 288.15))
    assert speed_of_sound(15.0) == pytest.approx(340.29, abs=0.01)
    assert speed_of_sound(30.0) > speed_of_sound(0.0)


def test_tip_speed_and_mach():
    """v_tip = pi*D*rpm/60，M = v_tip/a."""
    expected_tip = math.pi * 0.254 * 6500.0 / 60.0
    assert tip_speed(6500.0, 0.254) == pytest.approx(expected_tip)
    assert tip_speed(6500.0, 0.254) == pytest.approx(86.45, abs=0.01)
    assert tip_mach(6500.0, 0.254, 15.0) == pytest.approx(expected_tip / speed_of_sound(15.0))
    assert tip_mach(6500.0, 0.254, 15.0) == pytest.approx(0.2540, abs=1e-3)


def test_tip_mach_scales_linearly():
    assert tip_mach(9000.0, 0.254) / tip_mach(6500.0, 0.254) == pytest.approx(9000 / 6500)
    assert tip_mach(6500.0, 0.508) == pytest.approx(2.0 * tip_mach(6500.0, 0.254), rel=1e-12)


def test_disk_area_and_loading():
    area = math.pi * 0.127 ** 2 * 4
    assert disk_area(0.254, 4) == pytest.approx(area)
    assert disk_area(0.254, 4) == pytest.approx(0.2027, abs=1e-3)
    loading = 2.0 * 9.80665 / area
    assert disk_loading(2.0, 0.254, 4) == pytest.approx(loading)
    assert disk_loading(2.0, 0.254, 4) == pytest.approx(96.79, abs=0.05)
    # 重量翻倍 -> 载荷翻倍；桨径翻倍 -> 载荷降到 1/4
    assert disk_loading(4.0, 0.254, 4) == pytest.approx(2.0 * loading)
    assert disk_loading(2.0, 0.508, 4) == pytest.approx(loading / 4.0)


def test_induced_velocity_from_loading():
    loading = disk_loading(2.0, 0.254, 4)
    expected = math.sqrt(loading / (2.0 * RHO0))
    assert induced_velocity(2.0, 0.254, 4) == pytest.approx(expected)
    assert induced_velocity(2.0, 0.254, 4) == pytest.approx(6.285, abs=0.01)
    # 空气密度降低 -> 诱导速度变大
    assert induced_velocity(2.0, 0.254, 4, rho=1.0) > induced_velocity(2.0, 0.254, 4)


def test_mach_and_loading_levels():
    assert mach_level(0.0) == 'ok'
    assert mach_level(0.599) == 'ok'
    assert mach_level(MACH_MARGIN) == 'caution'
    assert mach_level(0.699) == 'caution'
    assert mach_level(0.7) == 'too_high'
    assert mach_level(0.95) == 'too_high'
    assert disk_loading_level(DISK_LOADING_LOW - 0.1) == 'low'
    assert disk_loading_level(DISK_LOADING_LOW) == 'medium'
    assert disk_loading_level(250.0) == 'medium'
    assert disk_loading_level(250.1) == 'high'


def test_prop_check_defaults(check):
    assert check.weight_n == pytest.approx(19.6133, abs=1e-3)
    assert check.thrust_per_rotor_n == pytest.approx(4.9033, abs=1e-3)
    assert check.area_m2 == pytest.approx(0.2027, abs=1e-3)
    assert check.hover_tip_speed_m_s == pytest.approx(86.45, abs=0.01)
    assert check.hover_tip_mach == pytest.approx(0.2540, abs=1e-3)
    assert check.max_tip_mach == pytest.approx(0.3518, abs=1e-3)
    assert check.max_tip_mach > check.hover_tip_mach
    assert check.disk_loading_n_m2 == pytest.approx(96.79, abs=0.05)
    assert check.disk_loading_kg_m2 == pytest.approx(
        check.disk_loading_n_m2 / 9.80665, rel=1e-9)


def test_rpm_for_mach_is_inverse(check):
    rpm = check.rpm_for_mach(0.6)
    assert tip_mach(rpm, check.diameter_m) == pytest.approx(0.6, rel=1e-12)
    assert rpm == pytest.approx(15352.2, abs=1.0)


def test_diameter_for_disk_loading_is_inverse(check):
    target = 100.0
    diameter = check.diameter_for_disk_loading(target)
    assert disk_loading(check.mass, diameter, check.rotor_count) == pytest.approx(
        target, rel=1e-12)
    assert diameter == pytest.approx(0.2499, abs=1e-3)


def test_report_is_consistent(check):
    report = check.report()
    assert report['tip_mach_level'] == mach_level(report['tip_mach_max'])
    assert report['disk_loading_level'] == disk_loading_level(report['disk_loading_n_m2'])
    assert report['induced_velocity_m_s'] == pytest.approx(check.induced_velocity_m_s, abs=1e-4)
    assert report['limits']['rpm_for_mach_0_7'] > report['limits']['rpm_for_mach_0_6']
    assert any('静' in note or '静态' in note for note in report['notes'])


def test_invalid_parameters_are_rejected(check):
    for kwargs in ({'mass': 0.0}, {'diameter_m': 0.0}, {'hover_rpm': -1.0},
                   {'max_rpm': -1.0}, {'temperature_c': 100.0}, {'rho': 0.0},
                   {'gravity': 0.0}):
        base = {'mass': 2.0, 'diameter_m': 0.254, 'rotor_count': 4,
                'hover_rpm': 6500.0, 'max_rpm': 9000.0}
        base.update(kwargs)
        with pytest.raises(ValueError):
            PropCheck(**base)
    with pytest.raises(ValueError):
        PropCheck(mass=2.0, diameter_m=0.254, hover_rpm=9000.0, max_rpm=6500.0)
    with pytest.raises(ValueError):
        disk_area(0.254, 0)
    with pytest.raises(ValueError):
        check.rpm_for_mach(1.0)
    with pytest.raises(ValueError):
        check.diameter_for_disk_loading(0.0)
    with pytest.raises(ValueError):
        mach_level(float('nan'))
    with pytest.raises(ValueError):
        speed_of_sound(100.0)


def test_print_report_is_gbk_safe(check):
    """Windows 默认 GBK 控制台下打印不能抛 UnicodeEncodeError."""
    buffer = io.BytesIO()
    wrapper = io.TextIOWrapper(buffer, encoding='gbk', errors='strict')
    original = sys.stdout
    sys.stdout = wrapper
    try:
        cli.print_report(check.report())
        wrapper.flush()
    finally:
        sys.stdout = original
    text = buffer.getvalue().decode('gbk')
    assert 'uav_prop_tip_mach' in text
    assert '桨尖马赫数' in text
    assert 'm^2' in text


def test_cli_smoke_writes_report(tmp_path):
    """跑真实入口: 写出 report.json 并返回 0（不需要绘图库）."""
    output = tmp_path / 'out'
    assert cli.main(['--output', str(output)]) == 0
    assert sorted(path.name for path in output.iterdir()) == ['report.json']
    report = json.loads((output / 'report.json').read_text(encoding='utf-8'))
    assert report['tip_mach_level'] == 'ok'
    assert report['disk_loading_level'] == 'low'
    assert report['parameters']['rotor_count'] == 4
