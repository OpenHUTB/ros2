# -*- coding: utf-8 -*-
"""故障场景 / 用例矩阵的单元测试。"""

import pytest

from uav_actuator_fault_sim import scenario as S


def test_all_physical_scenes_have_duration():
    assert len(S.PHYSICAL_SCENES) == 5
    for name in S.PHYSICAL_SCENES:
        assert name in S.SCENE_DURATION
        assert S.SCENE_DURATION[name] > 0.0


def test_seven_scenarios_and_six_fault_only():
    assert len(S.FAULT_SCENARIOS) == 7
    assert len(S.FAULT_ONLY) == 6
    assert "healthy" in S.FAULT_SCENARIOS
    assert "healthy" not in S.FAULT_ONLY


def test_every_scenario_can_build_a_spec():
    for name, sc in S.FAULT_SCENARIOS.items():
        spec = sc.build(rotor=1, onset=2.5)
        assert spec is not None, name
        assert isinstance(sc.label(), str) and sc.label()


def test_healthy_scenario_has_no_rotor_fault():
    assert S.FAULT_SCENARIOS["healthy"].build(rotor=0, onset=2.0).rotor_faults == ()


def test_severity_defaults_by_scenario():
    """不传 severity 时也要能构造（各场景自带默认值）。"""
    for name in S.FAULT_ONLY:
        spec = S.FAULT_SCENARIOS[name].build(rotor=0, onset=1.0, severity=None)
        assert spec is not None, name


def test_learnable_flag_marks_faults_worth_learning():
    """至少有一半故障被标为"值得学习型方法处理"。"""
    learnable = [n for n, sc in S.FAULT_SCENARIOS.items() if sc.learnable]
    assert len(learnable) >= 3


def test_full_matrix_shape():
    full = S.full_matrix(seeds=(0,))
    assert len(full) == len(S.PHYSICAL_SCENES) * len(S.FAULT_SCENARIOS)
    assert len({c.tag() for c in full}) == len(full)


def test_training_matrix_skips_healthy():
    train = S.training_matrix(seeds=(0, 1))
    assert len(train) == len(S.PHYSICAL_SCENES) * len(S.FAULT_ONLY) * 2
    assert all(c.fault_scenario != "healthy" for c in train)


def test_make_case_defaults_and_build_spec():
    case = S.make_case("corridor", "single_loss", seed=3)
    assert case.seed == 3
    # rotor / onset 缺省时由 seed 决定：随机桨位 + 1.5~3.5 s 的注入时刻
    assert 0 <= case.rotor < 4
    assert 1.5 <= case.onset <= 3.5
    assert case.severity is None
    spec = case.build_spec()
    assert len(spec.rotor_faults) == 1
    assert spec.rotor_faults[0].kind == "loss"


def test_make_case_is_deterministic_for_same_seed():
    a = S.make_case("corridor", "single_loss", seed=11)
    b = S.make_case("corridor", "single_loss", seed=11)
    assert (a.rotor, a.onset) == (b.rotor, b.onset)


def test_explicit_rotor_and_onset_override_randomisation():
    case = S.make_case("corridor", "single_loss", seed=3, rotor=2, onset=1.0)
    assert (case.rotor, case.onset) == (2, 1.0)


def test_case_duration_comes_from_scene():
    case = S.make_case("open_ground", "healthy")
    assert case.duration() == pytest.approx(S.SCENE_DURATION["open_ground"])


def test_unknown_names_raise():
    with pytest.raises((KeyError, ValueError)):
        S.make_case("no_such_scene", "single_loss")
    with pytest.raises((KeyError, ValueError)):
        S.make_case("open_ground", "no_such_fault")
