"""闭环集成测试：七个场景的期望行为必须可复现。"""
from __future__ import annotations

import pytest

from sim.experiments import POST_ESCAPE_TRAVEL, run_episode
from sim.scenarios import ALL_SCENARIOS, SCENARIOS


@pytest.mark.parametrize("name", ALL_SCENARIOS)
def test_scenario_meets_expectation(name):
    ep = run_episode(name, seed=0, use_recovery=True)
    assert ep.success, f"{name} 未达成期望行为，记录：{ep.notes}"


@pytest.mark.parametrize("name", ["blocked_wall", "corner", "slip", "one_wheel"])
def test_without_recovery_every_escape_scenario_fails(name):
    """对照实验：没有脱困层时，四类卡死都会让机器人停在原地。"""
    ep = run_episode(name, seed=0, use_recovery=False)
    assert ep.triggers == 0
    assert ep.post_travel < POST_ESCAPE_TRAVEL
    assert not ep.success


def test_detected_cause_matches_injected_fault():
    expected = {
        "blocked_wall": "blocked",
        "corner": "blocked",
        "slip": "slip",
        "one_wheel": "one_wheel",
    }
    for name, cause in expected.items():
        ep = run_episode(name, seed=0, use_recovery=True)
        assert next(c for c in ep.causes if c != "none") == cause


def test_detection_is_faster_than_collision_fix():
    """检测应当发生在机器人真正被顶住后的 2 s 内。"""
    ep = run_episode("blocked_wall", seed=0, use_recovery=True)
    first = next(ep.t[i] for i, c in enumerate(ep.causes) if c != "none")
    assert first < 10.0


def test_permanent_fault_aborts_without_false_escape():
    ep = run_episode("one_wheel_permanent", seed=0, use_recovery=True)
    assert ep.aborted
    assert ep.escapes == 0


def test_no_false_alarm_in_normal_scenarios():
    for name in ("free", "wall_hug"):
        for seed in range(3):
            ep = run_episode(name, seed=seed, use_recovery=True)
            assert ep.triggers == 0, f"{name} seed={seed} 出现误报"


def test_seed_robustness_on_hard_scenario():
    """墙角场景对噪声最敏感：多个种子下都要能脱困。"""
    for seed in range(5):
        ep = run_episode("corner", seed=seed, use_recovery=True)
        assert ep.success, f"corner seed={seed} 失败"


def test_turning_is_what_actually_frees_the_robot():
    """参数敏感性自检：把转向角设成 0（只倒车不换方向）时，正面顶墙场景应当失败。"""
    from ros2_stuck_recovery.recovery import RecoveryConfig

    no_turn = [run_episode("blocked_wall", seed=s, use_recovery=True,
                           config=RecoveryConfig(turn_angles=(0.0, 0.0, 0.0))) for s in range(3)]
    assert not any(e.success for e in no_turn)


def test_scenario_library_is_complete():
    assert set(ALL_SCENARIOS) == set(SCENARIOS)
    for sc in SCENARIOS.values():
        assert sc.expect in {"escape", "abort", "no_trigger"}
        assert sc.duration > 0
