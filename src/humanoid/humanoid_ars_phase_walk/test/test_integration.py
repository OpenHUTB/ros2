#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""集成测试：CPG 基线可走、MLP 可训练、结果可复现。

直接运行：python3 src/humanoid/humanoid_ars_phase_walk/test/test_integration.py
"""
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))

import numpy as np                                              # noqa: E402

import humanoid_ars_phase_walk.config as config                 # noqa: E402
import humanoid_ars_phase_walk.env as env                      # noqa: E402
from humanoid_ars_phase_walk import learners                    # noqa: E402

FAILS = []


def check(name, cond, detail=""):
    ok = bool(cond)
    if not ok:
        FAILS.append(name)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))


def main():
    base = env.rollout(env.make_env(0))
    check("CPG 基线前进 > 0.8 m", base["dx"] > 0.8, f"{base['dx']:+.2f} m")
    check("CPG 基线走满全程", base["steps"] >= config.SUCCESS_STEPS, f"{base['steps']} 步")

    again = env.rollout(env.make_env(0))
    check("同一配置可复现", abs(again["dx"] - base["dx"]) < 1e-6,
          f"差 {abs(again['dx']-base['dx'])*1000:.3f} mm")

    X, Y = learners.collect_cpg(2)
    check("采样维度正确", X.shape[1] == env.OBS_DIM and Y.shape[1] == env.ACT_DIM,
          f"{X.shape} -> {Y.shape}")

    net = learners.MLPPolicy(n_hidden=32).fit(X, Y, epochs=60)
    pred = net.predict(X)
    rmse = float(np.sqrt(np.mean((pred - Y) ** 2)))
    check("行为克隆可训练（RMSE < 0.2 rad）", rmse < 0.2, f"{rmse:.4f} rad")

    lin = learners.LinearPolicy()
    check("线性策略输出维度正确",
          np.asarray(lin(None, np.zeros(env.OBS_DIM))).shape == (env.ACT_DIM,))

    print()
    print(f"{len(FAILS)} 项失败" if FAILS else "全部通过")
    return 1 if FAILS else 0


if __name__ == "__main__":
    raise SystemExit(main())