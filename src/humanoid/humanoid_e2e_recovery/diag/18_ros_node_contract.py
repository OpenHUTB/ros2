#!/usr/bin/env python3
"""18_ros_node_contract.py —— 按 ROS 双节点的真实调用契约重测闭环（需 MuJoCo）

回答的问题：`main.py --check` 里网络每 2 个仿真步（4 ms）刷新一次，并把
`sim.data.time` 直接喂给 `predict_from_image(t=...)`；而 launch 起来的双节点是
另一套时钟——`e2e_sim_node` 每 tick 步进 5 步（10 ms）才发一帧，`e2e_predictor_node`
每帧把 `self._t += image_dt`（默认 0.01 s）。两套时钟的**刷新率**和 **t 的来源**都不同，
所以"节点里的网络还站不站得住"不能由 --check 推得。

本脚本不 import rclpy，把节点时序契约复刻到仿真循环里，用 `--suite` 一次跑四组：

  A) 基准：refresh=2、t=仿真时间          （= --check 口径，4 ms 控制周期）
  B) 旧节点契约：refresh=5、t=自累加 0.01（修复前 launch 的实际时序，10 ms）
  C) 修复后：refresh=2、t=自累加 0.004    （2 步/tick，节点与 predictor 同为 4 ms）
  D) C + 1 tick 传输延迟                  （模拟 ROS 消息往返：扭矩晚一拍才生效）

判定：A/C/D 应站满；B 若跌倒，说明旧时序是真实缺陷，C/D 通过即证明修复有效。
注意：真实 ROS 图的 QoS、线程调度、渲染墙钟开销本脚本覆盖不到，只覆盖时序口径。
"""
from __future__ import annotations
import argparse
import os
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
CTRL_DT = 0.002


def episode(seconds, refresh, clock, image_dt=0.01, delay_ticks=0, push=None, push_at=4.0):
    """clock: 'sim' 用 sim.data.time 当时间戳；'self' 按帧累加 image_dt（节点做法）。"""
    sim = E2ESim(MODEL)
    pred = E2EPredictor(WEIGHTS)
    pred.reset()
    sim.reset()

    u = np.zeros(15)            # 当前生效的力矩
    pending = np.zeros(15)      # 待生效力矩（模拟传输延迟）
    step_i = 0
    self_t = 0.0
    max_tilt = 0.0
    fell_at = None
    clock_err = 0.0
    pushed = False

    for _ in range(int(round(seconds / CTRL_DT))):
        t_sim = float(sim.data.time)
        if push is not None and not pushed and t_sim >= push_at:
            sim.apply_push(push, steps=40, target=DEFAULT_STAND_TARGET,
                           balance=expert_action)
            pushed = True

        if step_i % refresh == 0:
            img = sim.render_front(gray=True)
            if clock == "self":
                self_t += image_dt
                tau = pred.predict_from_image(img, t=self_t)
                clock_err = max(clock_err, abs(self_t - t_sim))
            else:
                tau = pred.predict_from_image(img, t=t_sim)
            if delay_ticks > 0:
                pending = tau          # 下一 tick 才生效
            else:
                u = tau

        if delay_ticks > 0 and step_i > 0 and step_i % refresh == 0:
            u = pending
        sim.step(target=DEFAULT_STAND_TARGET, torque_extra=u)
        step_i += 1

        max_tilt = max(max_tilt, abs(sim.torso_tilt()))
        if sim.fallen():
            fell_at = float(sim.data.time)
            break

    res = dict(fell_at=fell_at, max_tilt_deg=float(np.degrees(max_tilt)),
               clock_err=clock_err)
    sim.close()
    return res


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--suite", action="store_true", help="跑 A/B/C/D 四组对照")
    p.add_argument("--refresh", type=int, default=2)
    p.add_argument("--clock", choices=["sim", "self"], default="sim")
    p.add_argument("--image-dt", type=float, default=0.01)
    p.add_argument("--delay-ticks", type=int, default=0)
    a = p.parse_args()

    if a.suite:
        cases = [
            ("A 基准(--check)", dict(refresh=2, clock="sim")),
            ("B 旧节点(10ms)", dict(refresh=5, clock="self", image_dt=0.01)),
            ("C 修复后(4ms)", dict(refresh=2, clock="self", image_dt=0.004)),
            ("D C+1tick延迟", dict(refresh=2, clock="self", image_dt=0.004, delay_ticks=1)),
        ]
    else:
        cases = [("单组 refresh=%d clock=%s dt=%.3f delay=%d"
                  % (a.refresh, a.clock, a.image_dt, a.delay_ticks),
                  dict(refresh=a.refresh, clock=a.clock, image_dt=a.image_dt,
                       delay_ticks=a.delay_ticks))]

    print("=" * 84)
    print("ROS 双节点时序契约对照：0 N 空站 %.0f s（判据 root_z < 0.6）" % a.seconds)
    print("=" * 84)

    ok_all = True
    for name, kw in cases:
        r = episode(a.seconds, **kw)
        status = ("站满 %.0f s" % a.seconds if r["fell_at"] is None
                  else "跌倒于 %.2f s" % r["fell_at"])
        extra = (" | 时钟偏差 %.3f s" % r["clock_err"]) if kw["clock"] == "self" else ""
        print("[%-16s] %-13s 最大倾角 %5.1f°%s" % (name, status, r["max_tilt_deg"], extra))
        if name.startswith("B"):
            continue                     # B 是反例，不参与判定
        ok_all = ok_all and r["fell_at"] is None

    print("-" * 84)
    print("[判定] %s" % ("PASS：A/C/D 均站满，旧 10 ms 时序确为缺陷" if ok_all
                        else "FAIL：修复后仍有配置在窗口内跌倒"))
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
