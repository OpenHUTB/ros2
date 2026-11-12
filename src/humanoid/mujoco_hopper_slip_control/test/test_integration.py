#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MuJoCo 单足弹跳模块的离线集成测试（需要 mujoco，不需要 ROS）。

直接运行：``python3 src/humanoid/mujoco_hopper_slip_control/test/test_integration.py``
CI 里就是这么跑的，见 ``.github/workflows/mujoco_hopper_slip_control_tests.yml``。
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

import numpy as np                                             # noqa: E402

import mujoco_hopper_slip_control.config as config             # noqa: E402
from mujoco_hopper_slip_control import metrics                 # noqa: E402
from mujoco_hopper_slip_control.sim import HopperSim, find_model  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    ok = bool(cond)
    if not ok:
        FAILS.append(name)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))
    return ok


def test_model():
    """模型文件与 config 常量必须一致——改一处忘另一处是这类模块最常见的错。"""
    path = find_model()
    check("模型文件存在", path.is_file(), path.name)

    tree = ET.parse(path)
    root = tree.getroot()
    # 质量都挂在 geom 上（body 只负责运动学），按 geom 名取
    geom_mass = {g.get("name"): float(g.get("mass", 0.0))
                 for g in root.iter("geom") if g.get("mass")}
    check("模型质量总和等于 TOTAL_MASS",
          abs(sum(geom_mass.values()) - config.TOTAL_MASS) < 1e-9,
          f"xml {sum(geom_mass.values()):.2f} kg vs config {config.TOTAL_MASS:.2f} kg")
    check("躯干质量与配置一致",
          abs(geom_mass.get("torso_geom", 0.0) - config.MASS_TORSO) < 1e-9,
          f"xml {geom_mass.get('torso_geom', 0.0):.2f} kg vs config {config.MASS_TORSO:.2f} kg")

    leg = next(j for j in root.iter("joint") if j.get("name") == "leg")
    lo, hi = (float(v) for v in leg.get("range").split())
    check("腿关节 range 与配置一致",
          abs(lo - config.LEG_Q_MIN) < 1e-9 and abs(hi - config.LEG_Q_MAX) < 1e-9,
          f"[{lo}, {hi}]")

    hip = next(j for j in root.iter("joint") if j.get("name") == "hip")
    lo, hi = (float(v) for v in hip.get("range").split())
    check("髋关节 range 覆盖落脚角限幅",
          lo <= -config.RAIBERT_ANGLE_LIMIT and hi >= config.RAIBERT_ANGLE_LIMIT,
          f"[{lo}, {hi}] vs ±{config.RAIBERT_ANGLE_LIMIT}")

    stiff = float(leg.get("stiffness"))
    check("弹簧刚度与配置一致", abs(stiff - config.LEG_STIFFNESS) < 1e-9, f"{stiff:.0f} N/m")

    sim = HopperSim()
    check("MuJoCo 模型质量与 TOTAL_MASS 一致",
          abs(sim.mass - config.TOTAL_MASS) < 1e-9, f"{sim.mass:.2f} kg")
    sim.close()


def test_hopping():
    """跑 10 s：应稳定弹跳、顶点接近目标、不摔倒。"""
    sim = HopperSim(target_height=config.TARGET_HEIGHT)
    log = sim.run(10.0)
    sim.close()

    hops = metrics.extract_hops(log)
    st = metrics.steady_stats(hops)

    check("10 s 内至少 6 跳", st["count"] >= config.MIN_HOPS, f"{st['count']} hops")
    check("顶点高度接近目标",
          abs(st["apex_mean"] - config.TARGET_HEIGHT) < 0.08,
          f"{st['apex_mean']:.3f} m vs {config.TARGET_HEIGHT:.2f}")
    check("顶点波动小", st["apex_std"] < 0.03, f"std={st['apex_std']:.3f} m")
    check("支撑相与飞行相都非空",
          st["stance_mean"] > 0.05 and st["flight_mean"] > 0.10,
          f"{st['stance_mean'] * 1e3:.0f} ms / {st['flight_mean'] * 1e3:.0f} ms")
    check("占空比合理（0.15~0.55）", 0.15 < st["duty"] < 0.55, f"{st['duty']:.2f}")
    check("全程未摔倒", log["z"].min() > config.FALL_Z, f"z_min={log['z'].min():.3f} m")
    check("躯干最高点超过目标 80%",
          log["z"].max() > config.TARGET_HEIGHT * 0.85, f"z_max={log['z'].max():.3f} m")

    # 起飞速度应该为正、触地速度为负
    vzs = [h.vz_liftoff for h in hops]
    check("起飞竖直速度为正", all(v > 0 for v in vzs), f"min={min(vzs):.2f} m/s")


def test_determinism():
    """同一配置下两次运行必须逐位一致（没有隐藏的随机源）。"""
    a = HopperSim(target_height=0.55)
    la = a.run(4.0)
    a.close()
    b = HopperSim(target_height=0.55)
    lb = b.run(4.0)
    b.close()
    check("两次运行结果一致",
          np.array_equal(la["z"], lb["z"]), f"{len(la['z'])} steps")


def test_target_tracking():
    """换目标高度应能在 10 s 内收敛到新目标。"""
    for h in (0.45, 0.65):
        sim = HopperSim(target_height=h)
        log = sim.run(10.0)
        sim.close()
        st = metrics.steady_stats(metrics.extract_hops(log))
        check(f"目标 {h:.2f} m 收敛", abs(st.get("apex_mean", 0) - h) < 0.08,
              f"{st.get('apex_mean', float('nan')):.3f} m")


def test_metrics_io():
    """CSV 往返与空列表边界。"""
    import tempfile
    sim = HopperSim(target_height=0.55)
    hops = metrics.extract_hops(sim.run(6.0))
    sim.close()
    rows = metrics.hops_to_rows(hops)
    check("hop 指标行非空", len(rows) == len(hops) and rows, f"{len(rows)} rows")
    with tempfile.TemporaryDirectory() as td:
        p = metrics.write_csv(Path(td) / "hops.csv", rows)
        check("CSV 已写出且非空", p.is_file() and p.stat().st_size > 0, p.name)
        p2 = metrics.write_csv(Path(td) / "empty.csv", [])
        check("空列表也能写出（空文件）", p2.is_file() and p2.stat().st_size == 0)


def main():
    for fn in (test_model, test_hopping, test_determinism,
               test_target_tracking, test_metrics_io):
        print(f"\n--- {fn.__name__} ---")
        fn()
    print()
    if FAILS:
        print(f"{len(FAILS)} 项失败：{', '.join(FAILS)}")
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
