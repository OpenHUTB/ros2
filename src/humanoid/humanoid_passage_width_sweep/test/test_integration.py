#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集成测试（需要 mujoco，不需要 ROS）：场景加载、激光、碰撞判定、2x2 结论。

直接运行：python3 src/humanoid/humanoid_passage_width_sweep/test/test_integration.py
"""

import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

import numpy as np                                              # noqa: E402

import humanoid_passage_width_sweep.config as config            # noqa: E402
import humanoid_passage_width_sweep.planners as planners        # noqa: E402
import humanoid_passage_width_sweep.scene as scene              # noqa: E402
from humanoid_passage_width_sweep.sim import PassageSim, find_model  # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    ok = bool(cond)
    if not ok:
        FAILS.append(name)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


def test_model():
    path = find_model()
    check("随包场景 XML 存在", path.is_file(), path.name)
    shipped = path.read_text(encoding="utf-8")
    generated = scene.build_scene_xml(config.DEFAULT_WIDTH)
    check("XML 与代码生成的一致（防止两边漂移）", shipped.strip() == generated.strip())

    sim = PassageSim(width=0.90, variant="mppi_footprint")
    check("场景加载", sim.model.ngeom > 10, f"{sim.model.ngeom} geoms")
    check("起点不与墙接触", not sim.in_contact())
    pts = sim.scan()
    hit = int(np.isfinite(pts).all(axis=1).sum())
    check("激光有命中", hit > 10, f"{hit}/{config.N_RAYS}")
    sim.close()


def test_collision_excludes_floor():
    """足迹底面贴地，若把地面算进接触，所有实验会在 0.1 s 就终止。"""
    sim = PassageSim(width=0.90, variant="mppi_footprint")
    log = sim.run(duration=2.0)
    check("起步 2 s 内不误判碰撞", log["contact"].sum() == 0.0,
          f"contact={int(log['contact'].sum())}")
    check("确实在动", float(log["x"][-1]) > 0.3, f"x={log['x'][-1]:.2f} m")
    sim.close()


def test_ablation():
    """核心结论：同一算法，足迹感知能过 0.90 m 门，质点假设过不去。"""
    for variant, expect in (("mppi_footprint", True), ("mppi_point", False)):
        sim = PassageSim(width=0.90, variant=variant)
        sim.run(duration=40.0)
        reached = sim.goal_distance() <= config.GOAL_RADIUS and not sim.failed
        check(f"{variant} 在 0.90 m 门口{'通过' if expect else '失败'}",
              reached == expect, f"reached={reached}, failed={sim.failed}")
        sim.close()


def test_narrow_gate_limit():
    """0.62 m 单侧净空仅 1 cm，不宣称可过；0.86 m 应当稳定通过。"""
    sim = PassageSim(width=0.86, variant="mppi_footprint")
    sim.run(duration=40.0)
    ok = sim.goal_distance() <= config.GOAL_RADIUS and not sim.failed
    check("0.86 m 门口通过", ok, f"reached={ok}")
    sim.close()


def test_determinism():
    a = PassageSim(width=0.86, variant="mppi_footprint")
    la = a.run(duration=3.0)
    a.close()
    b = PassageSim(width=0.86, variant="mppi_footprint")
    lb = b.run(duration=3.0)
    b.close()
    check("两次运行逐位一致（固定随机种子）",
          np.array_equal(la["x"], lb["x"]), f"{len(la['x'])} steps")


def main():
    for fn in (test_model, test_collision_excludes_floor, test_ablation,
               test_narrow_gate_limit, test_determinism):
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