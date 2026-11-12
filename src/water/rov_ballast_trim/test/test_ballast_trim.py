"""检查浮力、配平与初稳性的闭式解与边界.

期望值都能手算：V = L*B*H，W = m*g，B = rho*V*g，
GM = z_B - z_G，tan(phi) = (y_G-y_B)/GM，T = 2*pi*sqrt(I_xx/(m*g*GM))。

测试不依赖 pytest 专有接口：`pytest test -v` 与 `python test/test_ballast_trim.py`
两种方式都能跑（后者用文件末尾的 __main__ 运行器）。
"""
import io
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rov_ballast_trim import cli  # noqa: E402
from rov_ballast_trim.core import (  # noqa: E402
    ADDED_MASS_FACTOR,
    DEFAULT_COMPONENTS,
    GM_MIN,
    GM_RECOMMENDED,
    G,
    RHO_NOMINAL,
    Component,
    RovTrim,
    build_components,
    cog,
    gm_level,
    roll_period_s,
    trim_angle_deg,
    water_density,
)
from rov_ballast_trim.cli import print_report  # noqa: E402


def close(value, expected, tol=1e-9):
    """相对/绝对混合比较，替代 pytest.approx，避免测试依赖 pytest."""
    return abs(value - expected) <= max(tol, abs(expected) * 1e-9)


def raises(callable_, exception=ValueError):
    """断言调用抛异常，返回 True 便于 assert."""
    try:
        callable_()
    except exception:
        return True
    return False


def make_trim(**kwargs):
    params = {'length_m': 0.90, 'beam_m': 0.70, 'height_m': 0.50}
    params.update(kwargs)
    return RovTrim(**params)


def test_water_density_monotonic():
    """表层 10 C/35 psu 取 1027；温度升高变轻，盐度与深度增加变重."""
    assert close(water_density(), RHO_NOMINAL)
    assert close(water_density(temp_c=10.0, salinity_psu=35.0, depth_m=0.0), 1027.0)
    assert water_density(temp_c=20.0) < water_density(temp_c=10.0)
    assert water_density(salinity_psu=38.0) > water_density(salinity_psu=35.0)
    assert water_density(depth_m=3000.0) > water_density(depth_m=0.0)
    # 3000 m：1027 + 0.0046*3000 = 1040.8
    assert close(water_density(depth_m=3000.0), 1040.8, 1e-9)
    assert raises(lambda: water_density(temp_c=100.0))
    assert raises(lambda: water_density(salinity_psu=-1.0))
    assert raises(lambda: water_density(depth_m=-5.0))


def test_component_validation():
    assert raises(lambda: Component('', 1.0))
    assert raises(lambda: Component('x', 0.0))
    assert raises(lambda: Component('x', 1.0, z_m=float('nan')))
    item = Component('battery', 12.0, -0.1, 0.0, -0.05)
    assert close(item.mass_kg, 12.0) and close(item.z_m, -0.05)


def test_cog_of_symmetric_parts():
    """四个对称部件 -> 重心在原点; 单边偏置 -> 按质量比偏移."""
    rows = [('a', 10.0, 0.2, 0.0, 0.0), ('b', 10.0, -0.2, 0.0, 0.0),
            ('c', 10.0, 0.0, 0.3, 0.0), ('d', 10.0, 0.0, -0.3, 0.0)]
    x, y, z, mass = cog(build_components(rows))
    assert close(mass, 40.0) and close(x, 0.0) and close(y, 0.0) and close(z, 0.0)
    x2, _, _, _ = cog(build_components([('a', 10.0, 0.2, 0.0, 0.0),
                                        ('b', 30.0, -0.2, 0.0, 0.0)]))
    assert close(x2, 0.25 * 0.2 - 0.75 * 0.2)
    assert raises(lambda: cog([]))


def test_trim_angle_formula():
    """tan(phi) = offset/GM：offset=GM 时正好 45 deg."""
    assert close(trim_angle_deg(0.0, 0.05), 0.0)
    assert close(trim_angle_deg(0.05, 0.05), 45.0, 1e-9)
    assert close(trim_angle_deg(-0.05, 0.05), -45.0, 1e-9)
    assert close(trim_angle_deg(0.001, 0.04), math.degrees(math.atan(0.025)), 1e-9)
    # GM 不为正 -> 视为失稳，倾角给 +-90 deg
    assert close(trim_angle_deg(0.01, 0.0), 90.0)
    assert close(trim_angle_deg(-0.01, -0.02), -90.0)


def test_gm_level_thresholds():
    assert gm_level(-0.01) == 'unstable'
    assert gm_level(0.0) == 'unstable'
    assert gm_level(GM_MIN - 0.001) == 'unstable'
    assert gm_level(GM_MIN) == 'marginal'
    assert gm_level(GM_RECOMMENDED - 0.001) == 'marginal'
    assert gm_level(GM_RECOMMENDED) == 'ok'
    assert raises(lambda: gm_level(float('nan')))


def test_default_hydrostatics():
    """默认外壳 0.90x0.70x0.50 m、285 kg 干重：排水 323.505 kg，缺 38.505 kg 压载."""
    trim = make_trim()
    assert close(trim.volume_m3, 0.315)
    assert close(trim.displacement_mass_kg, 323.505)
    assert close(trim.dry_mass_kg, 285.0)
    assert close(trim.required_ballast_kg, 38.505)
    assert close(trim.buoyancy_n, 323.505 * G)
    assert close(trim.weight_dry_n, 285.0 * G)
    assert close(trim.net_buoyancy_n, 38.505 * G)
    x, y, z = trim.cog_dry
    assert close(x, 9.25 / 285.0, 1e-12)
    assert close(y, 1.0 / 285.0, 1e-12)
    assert close(z, -6.7 / 285.0, 1e-12)


def test_dry_trim_is_large_because_gm_is_small():
    """空载 GM 只有 23.5 mm，32 mm 的纵向偏心就换来 54 deg 纵倾."""
    trim = make_trim()
    state = trim.state(0.0)
    assert close(state['gm_m'], 6.7 / 285.0, 1e-12)
    assert close(state['roll_deg'], math.degrees(math.atan2(1.0 / 285.0, 6.7 / 285.0)), 1e-9)
    assert close(state['pitch_deg'],
                 math.degrees(math.atan2(9.25 / 285.0, 6.7 / 285.0)), 1e-9)
    assert abs(state['pitch_deg']) > 50.0
    assert state['gm_level'] == 'marginal'


def test_ballast_plan_makes_it_neutral_and_level():
    """压载 38.505 kg 放在 (-0.240, -0.026, -0.180) m：净浮力归零、倾角归零."""
    trim = make_trim()
    plan = trim.ballast_plan()
    assert plan['feasible'] is True
    assert close(plan['mass_kg'], 38.505)
    assert close(plan['x_required_m'], -9.25 / 38.505, 1e-12)
    assert close(plan['y_required_m'], -1.0 / 38.505, 1e-12)
    assert close(plan['z_required_m'], -0.18)
    after = plan['state']
    assert close(after['net_buoyancy_n'], 0.0, 1e-6)
    assert close(after['roll_deg'], 0.0, 1e-9)
    assert close(after['pitch_deg'], 0.0, 1e-9)
    assert close(after['mass_kg'], 323.505)


def test_gm_after_ballast_matches_hand_calc():
    """z_G = (285*(-0.023509) + 38.505*(-0.18)) / 323.505 = -0.04214 m -> GM 42.1 mm."""
    trim = make_trim()
    after = trim.ballast_plan()['state']
    z_g = (285.0 * (-6.7 / 285.0) + 38.505 * (-0.18)) / 323.505
    assert close(after['cog']['z_m'], z_g, 1e-12)
    assert close(after['gm_m'], -z_g, 1e-12)
    assert close(after['gm_m'], 0.042135, 1e-6)
    assert after['gm_level'] == 'marginal'


def test_target_gm_inverse():
    """反解安装高度：目标 GM=0.05 m 需要把压载放到 z=-0.246 m（离船底 4 mm）."""
    trim = make_trim()
    plan = trim.ballast_plan(target_gm_m=0.05)
    assert plan['feasible'] is True
    assert close(plan['z_required_m'], -0.246079, 1e-5)
    assert close(plan['state']['gm_m'], 0.05, 1e-12)
    assert plan['state']['gm_level'] == 'ok'
    # 目标 GM 超过外壳能提供的范围 -> 判不可行
    bad = trim.ballast_plan(target_gm_m=0.30)
    assert bad['feasible'] is False
    assert '超出外壳范围' in bad['reason']
    assert raises(lambda: trim.ballast_plan(target_gm_m=0.0))


def test_infeasible_when_ballast_leaves_the_tray():
    """机械臂前移 0.35 m 后需要 -0.69 m 的纵向位置，超出 +-0.40 m 托盘限位."""
    rows = [list(row) for row in DEFAULT_COMPONENTS]
    rows[4][2] = 0.60
    trim = make_trim(components=rows)
    plan = trim.ballast_plan()
    assert plan['feasible'] is False
    assert '超出托盘限位' in plan['reason']
    assert close(plan['x_m'], -0.40)
    assert plan['residual_trim_deg'] > 30.0


def test_unstable_case_has_no_roll_period():
    """所有质量都堆到顶部，重心高于浮心 -> GM<0，横摇周期无定义."""
    rows = [('top', 100.0, 0.0, 0.0, 0.40)]
    trim = make_trim(components=rows)
    state = trim.state(0.0)
    assert state['gm_m'] < 0.0
    assert state['gm_level'] == 'unstable'
    assert state['roll_period_s'] is None
    assert close(state['roll_deg'], 90.0)
    assert raises(lambda: roll_period_s(100.0, -0.01, 0.7, 0.5))
    assert raises(lambda: trim.max_disturbance_moment(state))


def test_roll_period_scaling():
    """T = 2*pi*sqrt(I/(m*g*GM))：T 与 1/sqrt(GM) 成正比，与 sqrt(附加惯量) 成正比."""
    base = roll_period_s(323.505, 0.04, 0.70, 0.50)
    assert close(base, 2.0 * math.pi * math.sqrt(
        323.505 * (0.70 ** 2 + 0.50 ** 2) / 12.0 * ADDED_MASS_FACTOR
        / (323.505 * G * 0.04)))
    doubled = roll_period_s(323.505, 0.08, 0.70, 0.50)
    assert close(doubled, base / math.sqrt(2.0), 1e-12)
    heavier_added = roll_period_s(323.505, 0.04, 0.70, 0.50, added_mass_factor=2.6)
    assert close(heavier_added, base * math.sqrt(2.0), 1e-12)
    assert raises(lambda: roll_period_s(0.0, 0.04, 0.7, 0.5))


def test_tolerance_and_disturbance():
    """2 deg 门限对应 1.47 mm 横向偏心；5 deg 对应 11.65 N*m 扰动力矩."""
    trim = make_trim()
    after = trim.ballast_plan()['state']
    gm = after['gm_m']
    assert close(trim.offset_for_trim(gm), gm * math.tan(math.radians(2.0)), 1e-12)
    assert close(trim.offset_for_trim(gm), 0.001471, 1e-6)
    moment = trim.max_disturbance_moment(after)
    assert close(moment, 323.505 * G * gm * math.sin(math.radians(5.0)), 1e-12)
    assert close(moment, 11.65, 1e-2)
    assert close(trim.max_disturbance_moment(after, angle_deg=2.0), 4.66, 1e-2)
    assert raises(lambda: trim.offset_for_trim(0.0))


def test_depth_sweep():
    """深水变轻：3000 m 处浮力多 42.6 N，等效 4.35 kg."""
    trim = make_trim()
    rows = trim.depth_sweep()
    assert [row['depth_m'] for row in rows] == [0.0, 100.0, 1000.0, 3000.0, 6000.0]
    assert close(rows[0]['net_buoyancy_n'], 0.0, 1e-6)
    assert rows[3]['net_buoyancy_n'] > rows[2]['net_buoyancy_n'] > 0.0
    assert close(rows[3]['extra_ballast_kg'], 4.347, 1e-3)
    assert close(rows[0]['density_kg_m3'], 1027.0)
    assert raises(lambda: trim.depth_sweep((-1.0,)))


def test_report_is_consistent():
    trim = make_trim()
    report = trim.report()
    plan = report['ballast_plan']
    after = report['state_after_ballast']
    assert close(report['mass']['required_ballast_kg'], 38.505)
    assert plan['feasible'] is True
    assert close(after['gm_m'], 0.042135, 1e-6)
    assert report['stability_dry']['gm_level'] == 'marginal'
    assert len(report['warnings']) >= 3
    assert any('压载' in text for text in report['warnings'])
    assert any('BM' in text for text in report['notes'])
    assert close(report['parameters']['volume_m3'], 0.315)
    json.dumps(report)  # 报告必须可序列化


def test_print_report_is_gbk_safe():
    """Windows 默认 GBK 控制台下打印不能抛 UnicodeEncodeError."""
    buffer = io.BytesIO()
    wrapper = io.TextIOWrapper(buffer, encoding='gbk', errors='strict')
    original = sys.stdout
    sys.stdout = wrapper
    try:
        print_report(make_trim().report())
        wrapper.flush()
    finally:
        sys.stdout = original
    text = buffer.getvalue().decode('gbk')
    assert 'rov_ballast_trim' in text
    assert '初稳心高' in text
    assert 'm^3' in text


def test_cli_smoke_writes_report(tmp_path=None):
    """跑真实入口: 写出 report.json 并返回 0."""
    output = Path(tmp_path) if tmp_path else Path(__file__).resolve().parent / '_out'
    assert cli.main(['--output', str(output)]) == 0
    assert (output / 'report.json').is_file()
    report = json.loads((output / 'report.json').read_text(encoding='utf-8'))
    assert report['ballast_plan']['feasible'] is True
    assert close(report['mass']['dry_kg'], 285.0)


def main():
    """不装 pytest 也能跑：逐个调用 test_* 并汇总."""
    failures = []
    tests = [(name, obj) for name, obj in sorted(globals().items())
             if name.startswith('test_') and callable(obj)]
    for name, func in tests:
        try:
            func()
        except AssertionError as error:
            failures.append('%s: %s' % (name, error))
            print('FAIL %s' % name)
        else:
            print('ok   %s' % name)
    print()
    print('%d passed, %d failed' % (len(tests) - len(failures), len(failures)))
    for item in failures:
        print('  - ' + item)
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
