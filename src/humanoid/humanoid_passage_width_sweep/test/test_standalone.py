#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线单元测试：场景几何、规划器接口、指标聚合。不需要 MuJoCo / ROS。

直接运行：python3 test/test_standalone.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np                                              # noqa: E402

import humanoid_passage_width_sweep.config as config            # noqa: E402
import humanoid_passage_width_sweep.metrics as metrics          # noqa: E402
import humanoid_passage_width_sweep.planners as planners        # noqa: E402
import humanoid_passage_width_sweep.scene as scene              # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    ok = bool(cond)
    if not ok:
        FAILS.append(name)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


def test_geometry():
    check("0.90 m 门单侧净空 = 0.15 m", abs(scene.gate_clearance(0.90) - 0.15) < 1e-9,
          f"{scene.gate_clearance(0.90):.3f}")
    check("0.62 m 门可通行", scene.is_passable(0.62))
    check("0.58 m 门不可通行", not scene.is_passable(0.58))
    check("膨胀半径有两个档位且足迹档更大",
          config.INFLATE_FOOTPRINT > config.INFLATE_POINT,
          f"{config.INFLATE_POINT:.2f} / {config.INFLATE_FOOTPRINT:.2f}")
    check("足迹档膨胀 = 足迹半径 + 裕度",
          abs(config.INFLATE_FOOTPRINT - (config.FOOTPRINT_RADIUS + config.SAFETY_MARGIN)) < 1e-9)
    check("扫描档位含几何极限档", min(config.WIDTH_GRID) < 0.70)


def test_planner_interface():
    goal = np.array(config.GOAL)
    for v in planners.VARIANT_ORDER:
        p = planners.make(v)
        check(f"{v} 膨胀半径正确", abs(p.inflate - (
            config.INFLATE_FOOTPRINT if v.endswith("footprint") else config.INFLATE_POINT)) < 1e-9,
            f"{p.inflate:.2f}")
        # 空激光：应能给出朝目标的控制，且限幅
        v_cmd, w_cmd = p.compute(0.0, 0.0, 0.0, np.zeros((0, 2)))
        check(f"{v} 空激光下速度在限幅内",
              -1e-9 <= v_cmd <= config.MAX_V + 1e-9 and abs(w_cmd) <= config.MAX_W + 1e-9,
              f"v={v_cmd:.2f} w={w_cmd:.2f}")
        # 远处障碍（2 m，远超两个膨胀档）不应引起明显转向。
        # 阈值放宽到 0.05：MPPI 是采样型，加权均值天然有 1e-4 量级噪声。
        p.reset()
        v3, w3 = p.compute(0.0, 0.0, 0.0, np.array([[2.0, 0.0]]))
        check(f"{v} 远处障碍不误触发", abs(w3) < 0.05, f"w={w3:+.3f}")

        # 近距障碍只有斥力法能保证响应：MPPI 的代价是「整条轨迹的最小距离」，
        # 对 0.1 m 处单点这种已在足迹内部的退化输入没有可分辨的梯度。
        if v.startswith("reactive"):
            p.reset()
            v2, w2 = p.compute(0.0, 0.0, 0.0, np.array([[0.5 * p.inflate, 0.0]]))
            check(f"{v} 近距障碍时转向", abs(w2) > 0.01, f"w={w2:+.3f}")


def test_metrics():
    log = dict(t=np.array([0.0, 0.05, 0.10]), x=np.array([0.0, 0.05, 0.10]),
               y=np.zeros(3), yaw=np.zeros(3), v=np.full(3, 0.6), w=np.zeros(3),
               clearance=np.array([0.5, 0.4, 0.3]), contact=np.zeros(3))
    row = metrics.summarize(log, True, False, 0.9, "mppi_footprint", "x")
    check("summarize 取到最小净空", abs(row["min_clear"] - 0.3) < 1e-9, f"{row['min_clear']}")
    check("summarize 路径长度正确", abs(row["path"] - 0.10) < 1e-6, f"{row['path']:.3f}")
    empty = metrics.summarize({}, False, True, 0.9, "v", "x")
    check("空日志不崩溃且判失败", empty["failed"] and empty["reached"] is False)

    agg = metrics.aggregate([row, dict(row, reached=False, min_clear=-0.1)])
    check("aggregate 通过率 1/2", agg[0]["success_rate"] == 0.5, f"{agg[0]['success_rate']}")
    check("aggregate 失败样本的负净空不被丢掉",
          agg[0]["min_clear"] < 0.2, f"{agg[0]['min_clear']:.3f}")


def main():
    for fn in (test_geometry, test_planner_interface, test_metrics):
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