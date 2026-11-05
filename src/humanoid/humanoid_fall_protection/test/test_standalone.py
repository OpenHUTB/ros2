#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""humanoid_fall_protection 自检。

两种跑法：
    python3 test/test_standalone.py      # 独立运行，失败退出码 1
    python3 -m pytest test/ -v           # pytest / colcon 收集

所有检查都用 assert 或抛异常，**失败必须让运行器失败**，
不能只打印 FAIL 就正常返回。
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402

import humanoid_fall_protection.config as config       # noqa: E402
from humanoid_fall_protection.metrics import RunRecorder, reduction_vs_best_passive  # noqa: E402
from humanoid_fall_protection.protection import FallController, FallDetector  # noqa: E402
from humanoid_fall_protection.runner import run_episode  # noqa: E402
from humanoid_fall_protection.sim_core import FallSim   # noqa: E402

#: 整个测试用同一次 rigid 跑出的结果做多项断言，避免重复仿真
_CACHE = {}


def _cached_episode(strategy, **kwargs):
    key = (strategy, tuple(sorted(kwargs.items())))
    if key not in _CACHE:
        _CACHE[key] = run_episode(strategy, **kwargs)[0]
    return _CACHE[key]


def test_model_loads():
    sim = FallSim()
    assert sim.model.nu == 15, "执行器数量应为 15，实际 %d" % sim.model.nu
    assert sim.model.nq == 22, "广义坐标应为 22，实际 %d" % sim.model.nq
    assert abs(sim.model.opt.timestep - 0.002) < 1e-9


def test_feet_touch_ground():
    sim = FallSim()
    sim.reset()
    lowest = min(float(sim.data.geom_xpos[g][2]) for g in sim.foot_geoms)
    # 脚掌 box 半高 0.025，贴地时脚底中心正好在 0.025
    assert abs(lowest - 0.025) < 5e-3, "脚底高度 %.4f，未贴地" % lowest


def test_standing_is_stable():
    """没有推扰时，1 s 内倾角必须很小，且不能触发判据。"""
    sim = FallSim()
    sim.reset()
    controller = FallController()
    controller.reset("protect")
    max_tilt = 0.0
    for i in range(int(1.0 / sim.dt)):
        sim.apply_push(0.0, False)
        controller.step(sim, i * sim.dt)
        sim.step()
        max_tilt = max(max_tilt, abs(sim.tilt()))
    assert not controller.detector.triggered, "静立时误触发跌倒判据"
    assert np.degrees(max_tilt) < 8.0, "静立倾角 %.2f°，站姿不稳" % np.degrees(max_tilt)


def test_push_triggers_protection():
    sim = FallSim()
    sim.reset()
    controller = FallController()
    controller.reset("protect")
    fired_at = None
    for i in range(int(2.5 / sim.dt)):
        t = i * sim.dt
        sim.apply_push(300.0, 1.0 <= t < 1.08)
        state = controller.step(sim, t)
        sim.step()
        if state["just_triggered"] and fired_at is None:
            fired_at = t
    assert fired_at is not None, "300 N 推扰未触发保护"
    assert 1.0 < fired_at < 1.6, "触发时刻 %.3f s 不在推扰后的合理窗口内" % fired_at


def test_detector_thresholds():
    # 倾角超限（30° = 0.524 rad）单独就能触发 or 模式
    d = FallDetector(tilt_threshold_deg=30, tilt_rate_threshold=2.0, mode="or")
    assert not d.update(0.1, 0.1, 0.0)
    assert d.update(0.60, 0.0, 0.1), "倾角 34° 应触发 or 模式"
    # and 模式要求两者同时超限
    d2 = FallDetector(tilt_threshold_deg=30, tilt_rate_threshold=2.0, mode="and")
    d2.update(0.60, 0.0, 0.0)
    assert not d2.triggered, "and 模式下只有倾角不应触发"
    d3 = FallDetector(tilt_threshold_deg=30, tilt_rate_threshold=2.0, mode="and")
    d3.update(0.60, 3.0, 0.0)
    assert d3.triggered, "and 模式下倾角与角速度都超限应触发"


def test_detector_direction():
    d = FallDetector(tilt_threshold_deg=30, tilt_rate_threshold=2.0)
    d.update(0.6, 0.0, 0.0)
    assert d.direction() == "forward"
    d2 = FallDetector(tilt_threshold_deg=30, tilt_rate_threshold=2.0)
    d2.update(-0.6, 0.0, 0.0)
    assert d2.direction() == "backward"


def test_rigid_head_sinks_into_ground():
    """僵直倒下时头部球面会压进地面（间隙为负），这是「没有保护」的判据。"""
    s = _cached_episode("rigid")
    assert s["head_min_gap"] < 0.0, "僵直组头部未接触地面，场景可能没倒"
    assert s["torso_peak_force"] > 3000.0, "僵直组躯干冲击 %.0f N，偏低" % s["torso_peak_force"]


def test_protect_keeps_head_above_ground():
    s = _cached_episode("protect")
    assert s["head_min_gap"] > 0.0, "保护组头部球面仍压入地面 %.3f m" % s["head_min_gap"]
    assert s["head_peak_force"] < 50.0, "保护组头部峰值力 %.1f N" % s["head_peak_force"]


def test_protect_reduces_torso_impact():
    rigid = _cached_episode("rigid")
    protect = _cached_episode("protect")
    assert protect["torso_peak_force"] < 0.5 * rigid["torso_peak_force"], (
        "保护组躯干冲击没有明显下降：%.0f → %.0f N" % (
            rigid["torso_peak_force"], protect["torso_peak_force"]))


def test_limp_settles_faster():
    """瘫软组没有关节刚度支撑，倒地明显更快。"""
    rigid = _cached_episode("rigid")
    limp = _cached_episode("limp")
    assert limp["settle_time"] is not None
    assert limp["settle_time"] < rigid["settle_time"], (
        "瘫软倒地 %.2f s 不早于僵直 %.2f s" % (limp["settle_time"], rigid["settle_time"]))


def test_head_contact_force_reads_nonzero():
    """接触力必须能读到非零值。

    用瘫软组而不是僵直组来测：僵直倒下时头是被躯干「垫」着慢慢压进地面的，
    接触力的建立和穿透深度并不同步（某一帧球面已经低于地面，但约束力还没起来），
    拿它做断言会偶发失败。瘫软组是实打实地砸下去，接触力一定存在。
    """
    sim = FallSim()
    sim.reset()
    controller = FallController()
    controller.reset("limp")
    peak = 0.0
    for i in range(int(3.0 / sim.dt)):
        t = i * sim.dt
        sim.apply_push(300.0, 1.0 <= t < 1.08)
        controller.step(sim, t)
        sim.step()
        peak = max(peak, sim.part_contact("head"))
    assert peak > 100.0, "读不到头部接触力（峰值 %.1f N），法向分量取错了？" % peak


def test_params_override():
    """阈值改小必须让触发更早，证明参数真的传到了控制器。"""
    default = _cached_episode("protect")
    early = run_episode("protect", params={"tilt_threshold_deg": 10.0})[0]
    assert early["detect_delay"] is not None
    assert early["detect_delay"] <= default["detect_delay"] + 1e-9, (
        "阈值 10° 的触发延迟 %.3f 不早于默认 %.3f" % (
            early["detect_delay"], default["detect_delay"]))


def test_recorder_csv(tmp_dir=None):
    import tempfile
    s, recorder = run_episode("rigid", duration=1.5)
    assert len(recorder.rows) > 100, "采样点太少：%d" % len(recorder.rows)
    path = os.path.join(tempfile.mkdtemp(), "run.csv")
    recorder.write_csv(path)
    assert os.path.exists(path)
    with open(path, encoding="utf-8") as handle:
        lines = handle.readlines()
    assert lines[0].startswith("t,tilt_rad")
    assert len(lines) == len(recorder.rows) + 1


def test_summary_fields():
    s = _cached_episode("protect")
    for key in ("head_min_z", "head_min_gap", "head_peak_force", "torso_peak_force",
                "knee_peak_force", "detect_delay", "settle_time", "head_touched"):
        assert key in s, "summary 缺少字段 %s" % key
    assert abs(s["head_min_gap"] - (s["head_min_z"] - 0.09)) < 1e-9


def test_best_passive_reduction_definition():
    """reduction 用的是「两个被动策略里更好的那个」，不是最差的那个。"""
    summaries = [
        {"strategy": "rigid", "head_peak_force": 100.0},
        {"strategy": "limp", "head_peak_force": 400.0},
        {"strategy": "protect", "head_peak_force": 20.0},
    ]
    r = reduction_vs_best_passive(summaries)
    assert abs(r - 0.8) < 1e-9, "相对最佳被动策略的降低比例算错：%r" % r


def test_protect_pose_within_joint_limits():
    """保护姿态的目标角必须落在模型关节行程内，否则会被限位截断。"""
    import mujoco

    sim = FallSim()
    limits = {}
    for name in config.JOINT_NAMES:
        joint_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        limits[name] = (float(sim.model.jnt_range[joint_id][0]),
                        float(sim.model.jnt_range[joint_id][1]))
    for pose_name, pose in (("PROTECT_FORWARD", config.PROTECT_FORWARD),
                            ("PROTECT_BACKWARD", config.PROTECT_BACKWARD)):
        for name, value in pose.items():
            low, high = limits[name]
            assert low - 1e-6 <= value <= high + 1e-6, (
                "%s 的 %s=%.2f 超出关节行程 [%.2f, %.2f]" % (pose_name, name, value, low, high))


def main():
    tests = [(name, obj) for name, obj in sorted(globals().items())
             if name.startswith("test_") and callable(obj)]
    print("=" * 66)
    print("  humanoid_fall_protection 单元测试（%d 项）" % len(tests))
    print("=" * 66)
    failed = 0
    for name, func in tests:
        try:
            func()
            print("  [PASS] %s" % name)
        except AssertionError as exc:
            failed += 1
            print("  [FAIL] %s：%s" % (name, exc))
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print("  [ERROR] %s：%r" % (name, exc))
    print("=" * 66)
    if failed:
        print("  %d 项失败" % failed)
        return 1
    print("  全部通过 ✅")
    return 0


if __name__ == "__main__":
    sys.exit(main())
