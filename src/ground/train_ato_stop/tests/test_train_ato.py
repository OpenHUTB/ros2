"""ATO 模块的离线测试：物理量、速度曲线、ATP 层次与四个场景的闭环结果。"""
import math

import pytest

from train_ato_stop import scenarios, sim
from train_ato_stop.ato import AtpState, Ato, AtoConfig
from train_ato_stop.profile import atp_permit_curve, build_profile
from train_ato_stop.train import Segment, TrainSpec, braking_distance


# ------------------------------------------------------------------ 物理量
def test_牵引力低速受上限约束():
    spec = TrainSpec()
    assert spec.traction_force(0.0) == pytest.approx(spec.max_tractive_effort)
    assert spec.traction_force(1.0) == pytest.approx(spec.max_tractive_effort)


def test_牵引力高速转为恒功率():
    spec = TrainSpec()
    v = 40.0
    assert spec.traction_force(v) == pytest.approx(spec.rated_power / v)
    assert spec.traction_force(v) < spec.max_tractive_effort


def test_基本阻力随速度单调增加():
    spec = TrainSpec()
    forces = [spec.resistance(v) for v in (0.0, 10.0, 20.0, 40.0)]
    assert all(b > a for a, b in zip(forces, forces[1:]))


def test_坡道力上坡为正下坡为负():
    spec = TrainSpec()
    assert spec.grade_force(8.0) > 0.0
    assert spec.grade_force(0.0) == pytest.approx(0.0)
    assert spec.grade_force(-12.0) < 0.0
    # 300 t 车在 12‰ 坡上的分力约 35 kN
    assert spec.grade_force(-12.0) == pytest.approx(-35.3e3, rel=0.05)


def test_下坡的可用制动减速度更小():
    """同样踩制动，下坡停得比平道远，这是坡道场景的理论依据。"""
    spec = TrainSpec()
    seg_flat = [Segment(0, 1000, 160, grade=0.0)]
    seg_down = [Segment(0, 1000, 160, grade=-12.0)]
    p_flat = build_profile(seg_flat, 1000.0, spec, a_plan=0.8)
    p_down = build_profile(seg_down, 1000.0, spec, a_plan=0.8)
    # 同一位置，下坡的允许速度更低（说明必须更早开始减速）
    assert p_down.envelope_at(700.0) < p_flat.envelope_at(700.0)


def test_制动距离公式():
    assert braking_distance(40.0, 1.0) == pytest.approx(800.0)
    assert braking_distance(0.0, 1.0) == pytest.approx(0.0)


# ------------------------------------------------------------------ 速度曲线
def test_曲线不超过限速():
    spec = TrainSpec()
    sc = scenarios.restriction()
    p = build_profile(sc.segments, sc.stop_x, spec, limit_margin=2.0 / 3.6)
    for i, x in enumerate(p.x):
        assert p.v_ref[i] <= sc.limit_at(x) + 1e-6


def test_停车点速度为零():
    spec = TrainSpec()
    sc = scenarios.flat()
    p = build_profile(sc.segments, sc.stop_x, spec)
    assert p.v_ref[-1] == pytest.approx(0.0)
    assert p.v_envelope[-1] == pytest.approx(0.0)


def test_曲线在制动段单调下降():
    spec = TrainSpec()
    sc = scenarios.flat()
    p = build_profile(sc.segments, sc.stop_x, spec)
    tail = p.v_ref[-200:]
    assert all(b <= a + 1e-6 for a, b in zip(tail, tail[1:]))


def test_包络永不高于限速():
    spec = TrainSpec()
    sc = scenarios.restriction()
    p = build_profile(sc.segments, sc.stop_x, spec)
    for i, x in enumerate(p.x):
        assert p.v_envelope[i] <= sc.limit_at(x) + 1e-6


# ------------------------------------------------------------------ ATP 层次
def test_ATP允许速度高于ATO规划速度():
    """ATP 用满紧急制动，ATO 用打折的规划减速度，前者应当更宽松。"""
    spec = TrainSpec()
    sc = scenarios.flat()
    cfg = AtoConfig()
    p_ato = build_profile(sc.segments, sc.stop_x, spec, a_plan=cfg.a_plan)
    p_atp = atp_permit_curve(sc.segments, sc.stop_x, spec)
    ato = Ato(p_ato, spec, cfg, atp_profile=p_atp)
    for x in range(100, int(sc.stop_x) - 100, 50):
        assert p_atp.envelope_at(x) >= p_ato.envelope_at(x) - 1e-6
        assert ato.supervise(x, p_ato.envelope_at(x)) is not AtpState.INTERVENTION


def test_超速时ATP报警并介入():
    spec = TrainSpec()
    sc = scenarios.flat()
    p_atp = atp_permit_curve(sc.segments, sc.stop_x, spec)
    ato = Ato(p_atp, spec, AtoConfig(), atp_profile=p_atp)
    permit = p_atp.envelope_at(1000.0)
    assert ato.supervise(1000.0, permit + 1.0 / 3.6) is AtpState.NORMAL
    assert ato.supervise(1000.0, permit + 3.0 / 3.6) is AtpState.WARNING
    assert ato.supervise(1000.0, permit + 6.0 / 3.6) is AtpState.INTERVENTION


# ------------------------------------------------------------------ 闭环结果
def test_平直道停车在标尺以内():
    cfg = AtoConfig()
    r = sim.run(scenarios.flat(), config=cfg)
    assert abs(r.metrics['stop_error']) <= cfg.stop_tolerance
    assert r.metrics['atp_events'] == 0


def test_限速场景不超速():
    r = sim.run(scenarios.restriction())
    assert r.metrics['overspeed_time'] == pytest.approx(0.0, abs=0.05)
    assert r.metrics['max_speed_kph'] <= 160.0


def test_坡道场景仍在标尺以内():
    cfg = AtoConfig()
    r = sim.run(scenarios.grade(), config=cfg)
    assert abs(r.metrics['stop_error']) <= cfg.stop_tolerance


def test_限速场景确实降到了80以下():
    r = sim.run(scenarios.restriction())
    in_zone = [f.v * 3.6 for f in r.frames if 820.0 < f.x < 1290.0]
    assert in_zone, '应当有帧落在限速区内'
    assert max(in_zone) <= 80.0


def test_晚制动对照必然越位且触发ATP():
    r = sim.run(scenarios.late_brake())
    assert r.metrics['stop_error'] > 10.0
    assert r.metrics['atp_events'] >= 1


def test_能耗为正且量级合理():
    r = sim.run(scenarios.flat())
    assert 20.0 < r.metrics['energy_kwh'] < 200.0


def test_所有场景最高速度不超过限速():
    for sc in scenarios.all_scenarios():
        r = sim.run(sc)
        assert r.metrics['max_speed_kph'] <= sc.max_limit() + 0.5, sc.name


def test_基础阻力在高速段不超过牵引力():
    """如果阻力超过牵引力，车根本跑不到 160 km/h，场景就失去意义。"""
    spec = TrainSpec()
    v = 160.0 / 3.6
    assert spec.resistance(v) < spec.traction_force(v)
