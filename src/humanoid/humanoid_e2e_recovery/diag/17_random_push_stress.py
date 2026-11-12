#!/usr/bin/env python3
"""17_random_push_stress.py —— 连续随机推扰下的长时站立统计（闭环，需 MuJoCo）

回答的问题：`main.py --check` 只覆盖几个离散工况（0 / 30 N / 50 N，0.02 / 0.08 s），
"网络在连续不断、幅值和时长都随机的推扰下能站多久"没有数据。本脚本补上这一项。

口径（与 --check 保持一致，便于横向对比）：
  * 仿真步长 0.002 s；网络每 `--refresh` 步刷新一次（默认 2，与 --check 一致）；
  * 推扰期间由专家控制器维持平衡（`balance=expert_action`），推扰结束后交回网络
    —— 这与 --check 里 30 N / 50 N 的施加方式相同，否则几十毫秒内就会失稳；
  * 推力在水平面内随机方向（前后/左右都在），幅值 U(force_min, force_max)，
    时长 U(dur_min, dur_max)，两次推扰间隔 U(0.8, 1.6)/rate 秒；
  * 判据 root_z < 0.6 视为跌倒，窗口 `--seconds`。

每个种子打印一行，末尾汇总存活率、平均最大倾角、最差种子与平均跌倒时间。
"""
from __future__ import annotations
import argparse
import os
import statistics
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MOD = os.path.dirname(HERE)
sys.path.insert(0, MOD)

from humanoid_e2e_recovery.sim import E2ESim, DEFAULT_STAND_TARGET   # noqa: E402
from humanoid_e2e_recovery.predictor import E2EPredictor            # noqa: E402
from humanoid_e2e_recovery.expert import expert_action               # noqa: E402

MODEL = os.path.join(MOD, "models", "humanoid_camera.xml")
WEIGHTS = os.path.join(MOD, "models_trained", "e2e_policy.npz")


def run_seed(seed, seconds, rate, fmin, fmax, dmin, dmax, refresh,
             dir_deg=180.0, warmup=4.0, quiet=False):
    rng = np.random.default_rng(seed)
    sim = E2ESim(MODEL)
    pred = E2EPredictor(WEIGHTS)
    pred.reset()
    sim.reset()

    u = np.zeros(15)
    max_tilt = 0.0
    max_drift = 0.0
    n_push = 0
    fell_at = None
    ankle_sum, ankle_n = 0.0, 0

    # 先空站 warmup 秒再开始推：README 实测 2 s 相位（25/30 N）会失稳，
    # --check 与演示动图都用 4 s 相位，这里默认对齐 4 s。
    next_push = float(warmup)
    step_i = 0

    while sim.data.time < seconds:
        t = float(sim.data.time)
        if t >= next_push:
            force = float(rng.uniform(fmin, fmax))
            dur = float(rng.uniform(dmin, dmax))
            # 方向：在水平面内随机；dir_deg 限制偏离 +x 的最大角度，
            # 用于区分"前后向推力超出能力"与"侧向推力从未被训练"
            ang = float(np.radians(rng.uniform(-dir_deg, dir_deg)))
            steps = max(1, int(round(dur / 0.002)))
            sim.apply_push((force * np.cos(ang), force * np.sin(ang), 0.0),
                           steps=steps, target=DEFAULT_STAND_TARGET,
                           balance=expert_action)
            n_push += 1
            next_push = t + max(0.3, rng.uniform(0.8, 1.6) / rate)
            continue

        if step_i % refresh == 0:
            u = pred.predict_from_image(sim.render_front(gray=True), t=t)
        sim.step(target=DEFAULT_STAND_TARGET, torque_extra=u)
        step_i += 1

        tilt = abs(sim.torso_tilt())
        max_tilt = max(max_tilt, tilt)
        max_drift = max(max_drift, abs(sim.pelvis_x))
        ankle_sum += abs(float(u[3]))
        ankle_n += 1

        if sim.fallen():
            fell_at = float(sim.data.time)
            break

    res = dict(seed=seed, pushes=n_push, survived=(fell_at is None),
               fell_at=fell_at, max_tilt_deg=float(np.degrees(max_tilt)),
               max_drift_m=max_drift, mean_abs_ankle=(ankle_sum / ankle_n if ankle_n else 0.0))
    sim.close()
    if not quiet:
        status = "站满 %.0f s" % seconds if res["survived"] else "跌倒于 %.2f s" % fell_at
        print("[seed %d] %-14s 推力 %2d 次 | 最大倾角 %5.1f° | 最大水平漂移 %.3f m | 踝力矩均值 %.1f N·m"
              % (seed, status, n_push, res["max_tilt_deg"], max_drift, res["mean_abs_ankle"]))
    return res


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seconds", type=float, default=15.0)
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--seed0", type=int, default=0)
    p.add_argument("--rate", type=float, default=0.3, help="每秒推扰次数（平均间隔 1/rate s）")
    p.add_argument("--force-min", type=float, default=15.0)
    p.add_argument("--force-max", type=float, default=30.0)
    p.add_argument("--dur-min", type=float, default=0.02)
    p.add_argument("--dur-max", type=float, default=0.08)
    p.add_argument("--refresh", type=int, default=2, help="网络每 N 个仿真步刷新一次")
    p.add_argument("--dir-deg", type=float, default=180.0,
                   help="推力方向偏离 +x 的最大角度；180=全向，25=近前后向")
    p.add_argument("--warmup", type=float, default=4.0,
                   help="首次推扰前的空站时长（2 s 相位已被证明脆弱，默认 4 s）")
    a = p.parse_args()

    print("=" * 78)
    print("连续随机推扰长时统计：每回合约 %.0f s，推力 U(%.0f, %.0f) N，时长 U(%.0f, %.0f) ms，"
          "平均每 %.1f s 推一次，方向 ±%.0f°，网络每 %d 步刷新"
          % (a.seconds, a.force_min, a.force_max, a.dur_min * 1000, a.dur_max * 1000,
             1.0 / a.rate, a.dir_deg, a.refresh))
    print("首次推扰在 %.1f s 之后（避开 2 s 脆弱相位）" % a.warmup)
    print("=" * 78)

    rows = [run_seed(a.seed0 + i, a.seconds, a.rate, a.force_min, a.force_max,
                     a.dur_min, a.dur_max, a.refresh, a.dir_deg, a.warmup)
            for i in range(a.seeds)]

    n_ok = sum(1 for r in rows if r["survived"])
    falls = [r["fell_at"] for r in rows if r["fell_at"] is not None]
    print("-" * 78)
    print("存活 %d/%d | 最大倾角 均值 %.1f° / 最差 %.1f° | 最大水平漂移 最差 %.3f m"
          % (n_ok, len(rows),
             statistics.mean(r["max_tilt_deg"] for r in rows),
             max(r["max_tilt_deg"] for r in rows),
             max(r["max_drift_m"] for r in rows)))
    if falls:
        print("跌倒时间：%s（均值 %.2f s）" % ("/".join("%.2f" % f for f in falls),
                                          statistics.mean(falls)))
    print("[判定] %s" % ("PASS：全部回合站满窗口" if n_ok == len(rows)
                        else "FAIL：%d 个回合在窗口内跌倒" % (len(rows) - n_ok)))
    return 0 if n_ok == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
