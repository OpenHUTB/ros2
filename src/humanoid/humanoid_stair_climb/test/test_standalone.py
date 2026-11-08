#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""humanoid_stair_climb 自检。

两种跑法：
    python3 test/test_standalone.py      # 独立运行，失败退出码 1
    python3 -m pytest test/ -v           # pytest / colcon 收集

所有检查都用 assert 或抛异常，失败必须让运行器失败。
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import humanoid_stair_climb.config as config                    # noqa: E402
from humanoid_stair_climb.gait import (                          # noqa: E402
    StairGait, ground_height, step_index, smootherstep, SWING_WINDOW)
from humanoid_stair_climb.metrics import RunRecorder             # noqa: E402
from humanoid_stair_climb.sim_core import StairSim               # noqa: E402

SIM = None


def sim():
    global SIM
    if SIM is None:
        SIM = StairSim()
    return SIM


try:
    import pytest

    @pytest.fixture(name="sim")
    def _sim_fixture():
        return StairSim()


except ImportError:                                              # pragma: no cover
    pass


# ---------------------------------------------------------------- 地形
def test_ground_height_levels():
    assert ground_height(-0.5) == 0.0
    assert abs(ground_height(0.10) - config.STEP_HEIGHT) < 1e-9
    assert abs(ground_height(0.40) - 2 * config.STEP_HEIGHT) < 1e-9
    assert abs(ground_height(0.70) - 3 * config.STEP_HEIGHT) < 1e-9
    assert abs(ground_height(config.LANDING_X + 0.1) - config.LANDING_TOP) < 1e-9


def test_step_index_monotonic():
    assert step_index(-0.1) == 0
    assert step_index(0.0) == 1
    assert step_index(0.35) == 2
    assert step_index(0.35 + config.STEP_DEPTH * (config.STEP_COUNT - 2)) == config.STEP_COUNT
    assert step_index(config.LANDING_X + 0.1) == config.STEP_COUNT + 1


def test_joint_limits_match_xml():
    """config.JOINT_LIMITS 必须与模型里各 joint 的 range 完全一致。"""
    import xml.etree.ElementTree as ET
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "models", "stair_scene.xml")
    root = ET.parse(path).getroot()
    seen = {}
    for joint in root.iter("joint"):
        name = joint.get("name") or ""
        key = name.split("_")[0]
        assert key in config.JOINT_LIMITS, "模型里有 config 未登记的关节：%s" % name
        lo, hi = [float(v) for v in joint.get("range").split()]
        if key in seen:
            assert seen[key] == (lo, hi), "%s 两条腿的限位不一致：%s vs %s" % (
                key, seen[key], (lo, hi))
        seen[key] = (lo, hi)
    assert set(seen) == set(config.JOINT_LIMITS), (set(seen), set(config.JOINT_LIMITS))
    for key, limits in config.JOINT_LIMITS.items():
        assert seen[key] == limits, "%s: xml=%s config=%s" % (key, seen[key], limits)


def test_smootherstep_endpoints():
    assert smootherstep(0.0) == 0.0
    assert abs(smootherstep(1.0) - 1.0) < 1e-12
    assert abs(smootherstep(0.5) - 0.5) < 1e-12
    # 单调
    prev = -1.0
    for i in range(21):
        v = smootherstep(i / 20.0)
        assert v >= prev - 1e-12
        prev = v


# ---------------------------------------------------------------- IK
def test_ik_roundtrip_zero_error(sim):
    """IK 解出的关节角写回 MuJoCo 后，踝相对髋的位置必须与目标一致。"""
    import numpy as np
    mj = sim.mujoco
    thigh = mj.mj_name2id(sim.model, mj.mjtObj.mjOBJ_BODY, "thigh_l")
    foot = mj.mj_name2id(sim.model, mj.mjtObj.mjOBJ_BODY, "foot_l")
    for dx, dz in [(0.0, -0.80), (0.18, -0.78), (-0.18, -0.78), (0.0, -0.55)]:
        hip, knee = StairGait.leg_ik(dx, dz)
        ankle = StairGait.ankle_angle(hip, knee)
        sim.set_state(0.0, config.STAND_Z, [hip, knee, ankle] * 2)
        h = np.array(sim.data.xpos[thigh])
        a = np.array(sim.data.xpos[foot])
        assert abs((a[0] - h[0]) - dx) < 1e-6, (dx, a[0] - h[0])
        assert abs((a[2] - h[2]) - dz) < 1e-6, (dz, a[2] - h[2])


def test_ik_never_exceeds_leg_length():
    """全程扫描：髋到踝距离不得超过腿长（否则 IK 会被 clamp，脚被拖走）。"""
    gait = StairGait()
    worst = 0.0
    t = 0.0
    dt = 1.0 / config.CONTROL_HZ
    base_z = config.STAND_Z
    while t < config.TIME_LIMIT:
        bx = gait.base_x(t)
        base_z += (config.STAND_Z + gait.support_height(t) - base_z) * min(
            1.0, dt / config.BASE_Z_TAU)
        for side in ("l", "r"):
            fx, fz, _ = gait.foot_state(t, side)
            dx = fx - bx
            dz = (fz + config.FOOT_DROP) - (base_z + config.HIP_Z)
            worst = max(worst, math.hypot(dx, dz))
        t += dt
    assert worst <= config.THIGH + config.SHANK, "最大伸展 %.3f 超过腿长 %.3f" % (
        worst, config.THIGH + config.SHANK)


# ---------------------------------------------------------------- 步态
def test_swing_feet_never_below_safe_height():
    """摆动相脚底必须高于所跨越的最高台阶面（脚厚 0.04）。"""
    gait = StairGait()
    for side in ("l", "r"):
        a, b = SWING_WINDOW[side]
        for i in range(200):
            s = a + (b - a) * i / 200.0
            # 反解出该相位对应的 t
            k = 0
            t = (k + s) * gait.cycle_time
            x, z, swing = gait.foot_state(t, side)
            assert swing
            # 脚跨度 [x-0.06, x+0.18] 覆盖的地面里，最高那级不能高于脚底
            probes = [x - 0.06 + 0.24 * i / 20.0 for i in range(21)]
            gmax = max(ground_height(p) for p in probes)
            assert z >= gmax - 1e-9, (
                side, "s=%.3f x=%.3f z=%.4f 脚下方最高地面=%.3f" % (s, x, z, gmax))


def test_stance_foot_is_fixed():
    """支撑相脚的世界坐标不随时间变化。"""
    gait = StairGait()
    dt = 1.0 / config.CONTROL_HZ
    for side in ("l", "r"):
        prev = None
        t = 0.0
        while t < config.CYCLE_TIME * 3:
            x, z, swing = gait.foot_state(t, side)
            if not swing:
                if prev is not None:
                    assert abs(x - prev[0]) < 1e-9 and abs(z - prev[1]) < 1e-9
                prev = (x, z)
            else:
                prev = None
            t += dt


def test_only_one_foot_swings():
    """两脚不同时摆动（swing_ratio < 0.5 的必然结果）。"""
    assert config.SWING_RATIO < 0.5
    gait = StairGait()
    t = 0.0
    while t < config.CYCLE_TIME * 3:
        lsw = gait.foot_state(t, "l")[2]
        rsw = gait.foot_state(t, "r")[2]
        assert not (lsw and rsw), "t=%.3f 两脚同时在摆动相" % t
        t += 1.0 / config.CONTROL_HZ


def test_landing_points_on_step_centers():
    """落脚点必须落在台阶面上，且不能跨在立面边缘。"""
    gait = StairGait()
    for side in ("l", "r"):
        a, b = SWING_WINDOW[side]
        k = 0
        while k < 5:
            t = (k + b + 1e-6) * gait.cycle_time
            x, z, swing = gait.foot_state(t, side)
            assert not swing
            assert abs(z - ground_height(x)) < 1e-9
            # 脚几何跨度 [x-0.06, x+0.18] 不能跨立面
            lo, hi = x - 0.06, x + 0.18
            assert ground_height(lo) == ground_height(hi), (
                side, "k=%d 脚跨在立面边缘：x=%.3f [%.3f, %.3f]" % (k, x, lo, hi))
            k += 1


# ---------------------------------------------------------------- 端到端
def test_run_reaches_landing():
    """完整跑一遍必须站上平台，且脚不穿台阶。"""
    import subprocess
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    r = subprocess.run([sys.executable, os.path.join(here, "main.py"), "--headless"],
                       capture_output=True, text=True, cwd=here, timeout=180)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "到达=True" in r.stdout, r.stdout
    assert "最大穿透=0.000m" in r.stdout, r.stdout
    assert "关节越界=0" in r.stdout, r.stdout


def test_first_sample_is_initial_state():
    """CSV 第一行必须是 t=0 的初始位姿，不是走了一步之后的状态。"""
    from main import build_parser, run_one
    args = build_parser().parse_args(["--headless", "--control-hz", "50"])
    rec, _ = run_one(args)
    first = rec.samples[0]
    assert first["t"] == 0.0
    assert abs(first["x"] - config.BASE_X0) < 1e-9
    assert abs(first["z"] - config.STAND_Z) < 1e-6


def test_empty_summary_has_all_keys():
    s = RunRecorder().summary()
    for k in ("reached", "time", "steps", "climb", "max_base_z", "final_x",
              "min_clearance", "max_penetration", "landing_err_max",
              "joint_violations", "samples"):
        assert k in s, "summary 缺键 %s" % k


def test_control_hz_must_be_positive():
    from main import build_parser
    for bad in ("0", "-1"):
        try:
            build_parser().parse_args(["--control-hz", bad])
        except SystemExit as e:
            assert e.code == 2
        else:
            raise AssertionError("--control-hz %s 应该被拒绝" % bad)


if __name__ == "__main__":
    fails = 0
    names = [n for n in sorted(globals()) if n.startswith("test_")]
    for name in names:
        fn = globals()[name]
        try:
            import inspect
            if "sim" in inspect.signature(fn).parameters:
                fn(sim())
            else:
                fn()
        except Exception as e:                                   # noqa: BLE001
            fails += 1
            print("FAIL  %-45s %s" % (name, e))
        else:
            print("ok    %s" % name)
    print("\n%d/%d 通过" % (len(names) - fails, len(names)))
    sys.exit(1 if fails else 0)
