"""uav_hover_throttle 的离线单元测试（不依赖 ROS 2）。"""

import math

import pytest

from uav_hover_throttle.rotor import G, RHO0, Rotor, air_density


def test_sea_level_density():
    assert air_density(0.0) == pytest.approx(RHO0, rel=1e-12)


def test_density_decreases_with_altitude():
    values = [air_density(altitude) for altitude in (0.0, 1000.0, 2000.0, 4000.0)]
    assert all(values[i] > values[i + 1] for i in range(len(values) - 1))
    assert air_density(4000.0) == pytest.approx(0.8191, abs=5e-4)


def test_density_range_guard():
    with pytest.raises(ValueError):
        air_density(-1000.0)
    with pytest.raises(ValueError):
        air_density(12000.0)


def test_thrust_scales_with_square_of_rpm():
    rotor = Rotor()
    low = rotor.thrust(3000.0)
    high = rotor.thrust(6000.0)
    assert high == pytest.approx(4.0 * low, rel=1e-9)


def test_hover_thrust_balances_weight():
    rotor = Rotor()
    mass = 1.6
    rpm = rotor.hover_rpm(mass)
    assert rotor.thrust(rpm) == pytest.approx(mass * G, rel=1e-9)


def test_hover_throttle_is_reasonable():
    rotor = Rotor()
    throttle, rpm_hover, rpm_max = rotor.hover_throttle(1.6, rotor.nominal_voltage)
    assert 0.3 < throttle < 0.8
    assert rpm_hover < rpm_max


def test_altitude_increases_throttle():
    rotor = Rotor()
    low = rotor.hover_throttle(1.6, rotor.nominal_voltage, 0.0)[0]
    high = rotor.hover_throttle(1.6, rotor.nominal_voltage, 4000.0)[0]
    assert high > low


def test_lower_voltage_increases_throttle():
    rotor = Rotor()
    full = rotor.hover_throttle(1.6, 16.8)[0]
    weak = rotor.hover_throttle(1.6, 14.0)[0]
    assert weak > full


def test_heavier_mass_needs_more_rpm_and_power():
    rotor = Rotor()
    light = rotor.hover_report(1.3, rotor.nominal_voltage)
    heavy = rotor.hover_report(2.2, rotor.nominal_voltage)
    assert heavy['rpm_hover'] > light['rpm_hover']
    assert heavy['power_w'] > light['power_w']


def test_power_is_monotonic_and_zero_at_rest():
    rotor = Rotor()
    assert rotor.electrical_power(0.0) == pytest.approx(0.0)
    low = rotor.electrical_power(3000.0)
    high = rotor.electrical_power(6000.0)
    assert high > low > 0.0


def test_hover_power_per_kg_is_plausible():
    rotor = Rotor()
    report = rotor.hover_report(1.6, rotor.nominal_voltage)
    assert 50.0 < report['power_per_kg_w'] < 200.0
    assert report['feasible'] is True


def test_max_rpm_is_linear_in_voltage():
    rotor = Rotor()
    half = rotor.max_rpm(rotor.nominal_voltage / 2.0)
    assert half == pytest.approx(rotor.nominal_rpm / 2.0)
    with pytest.raises(ValueError):
        rotor.max_rpm(0.0)


def test_invalid_rotor_parameters_raise():
    with pytest.raises(ValueError):
        Rotor(diameter_m=0.0)
    with pytest.raises(ValueError):
        Rotor(thrust_coefficient=-0.1)
    with pytest.raises(ValueError):
        Rotor(figure_of_merit=1.2)
    with pytest.raises(ValueError):
        Rotor().hover_rpm(0.0)


def test_disk_area_formula():
    rotor = Rotor(diameter_m=0.5)
    assert rotor.disk_area == pytest.approx(math.pi * 0.25 / 4.0)
