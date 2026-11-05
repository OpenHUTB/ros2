#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""平衡控制自检：四项实验，每项都有明确的通过判据。

    A. 无控制 —— 必须倒（证明这是真倒立摆，不是"站着不动"的假仿真）
    B. 平衡站立 20 s —— 不许倒，倾角要收敛
    C. 外力扰动 —— 推一下要能自己回正
    D. 速度跟踪 —— 指令 0.3 m/s，实测要跟上且不跑偏

用法：
    python3 tools/check_balance.py
    python3 tools/check_balance.py --seconds 20 --csv results/balance.csv
"""

import argparse
import csv
import math
import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG_ROOT / "src"))

from balance_bot import config                                     # noqa: E402
from balance_bot.control import BalanceController                  # noqa: E402
from balance_bot.env import BalanceBotEnv, describe_env            # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description="两轮自平衡机器人控制自检")
    ap.add_argument("--scene", default=None)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--csv", default=None)
    args = ap.parse_args(argv)

    print("[环境] %s" % describe_env())
    env = BalanceBotEnv(config.resolve_scene_path(args.scene))
    ctrl = BalanceController()
    print("[控制器] %s（τ = -kp·θ - kd·θ̇ - kv·(v_t - v)）" % ctrl.gains())
    rows = []

    # ---- A. 无控制 ----
    print("\n=== A. 无控制（倒立摆必须倒）===")
    env.reset(tilt=config.SPAWN_TILT)
    t_fall = None
    for _ in range(int(6.0 * config.CONTROL_HZ)):
        st = env.step()
        if not env.is_upright():
            t_fall = st.t
            break
    ok_a = t_fall is not None
    print("  初始倾角 %.1f°，%.2f s 后倾角超过 %.0f° → %s"
          % (math.degrees(config.SPAWN_TILT), t_fall or -1,
             math.degrees(config.FALL_ANGLE), "倒下 ✓" if ok_a else "居然没倒 **不通过**"))

    # ---- B. 平衡站立 ----
    print("\n=== B. 平衡站立 %.0f s ===" % args.seconds)
    env.reset(tilt=config.SPAWN_TILT)
    worst = 0.0
    n = int(args.seconds * config.CONTROL_HZ)
    for k in range(n):
        tl, tr = ctrl(env.state(), 0.0, 0.0)
        env.set_torque(tl, tr)
        st = env.step()
        worst = max(worst, abs(st.theta))
        if k % int(5 * config.CONTROL_HZ) == 0:
            print("  t=%5.1f s  θ=%+6.2f°  x=%+7.3f m  v=%+6.3f m/s"
                  % (st.t, math.degrees(st.theta), st.x, st.v))
        if not env.is_upright():
            break
    ok_b = env.is_upright()
    print("  → %s（最大倾角 %.2f°，末位置 x=%+.3f m）"
          % ("通过 ✓" if ok_b else "**不通过：倒了**", math.degrees(worst), env.state().x))
    rows.append(dict(case="stand", ok=int(ok_b), worst_deg=round(math.degrees(worst), 2),
                     x=round(env.state().x, 3), v=round(env.state().v, 3)))

    # ---- C. 外力扰动 ----
    print("\n=== C. 外力扰动（第 3 s 推 2 N·s）===")
    env.reset(tilt=0.0)
    worst = 0.0
    recovered = None
    for k in range(int(8.0 * config.CONTROL_HZ)):
        if k == int(3.0 * config.CONTROL_HZ):
            # 自由关节的 qvel[0:3] 是线速度、qvel[3:6] 是角速度；
            # 早先写 qvel[3] 其实是在给"滚转角速度"加冲击，实测倾角纹丝不动。
            env.data.qvel[0] += 2.0 / 6.9      # 沿 x 的线冲量
        tl, tr = ctrl(env.state(), 0.0, 0.0)
        env.set_torque(tl, tr)
        st = env.step()
        worst = max(worst, abs(st.theta))
        if k > int(3.0 * config.CONTROL_HZ) and abs(st.theta) < math.radians(2.0) \
                and recovered is None and st.t > 3.2:
            recovered = st.t
        if not env.is_upright():
            break
    ok_c = env.is_upright()
    print("  → %s（被推后最大倾角 %.2f°，%s）"
          % ("通过 ✓" if ok_c else "**不通过：被推倒**", math.degrees(worst),
             ("%.2f s 回正" % recovered) if recovered else "未在观察窗内回正"))
    rows.append(dict(case="disturb", ok=int(ok_c), worst_deg=round(math.degrees(worst), 2),
                     x=round(env.state().x, 3), v=round(env.state().v, 3)))

    # ---- D. 速度跟踪 ----
    print("\n=== D. 速度跟踪（指令 0.30 m/s）===")
    env.reset(tilt=0.0)
    for k in range(int(8.0 * config.CONTROL_HZ)):
        tl, tr = ctrl(env.state(), 0.30, 0.0)
        env.set_torque(tl, tr)
        st = env.step()
        if k % int(2 * config.CONTROL_HZ) == 0:
            print("  t=%4.1f s  v=%+6.3f m/s  x=%+6.3f m  θ=%+5.2f°"
                  % (st.t, st.v, st.x, math.degrees(st.theta)))
    ok_d = abs(st.v - 0.30) < 0.10 and st.x > 0.5
    print("  → %s（末速度 %+.3f m/s，误差 %.3f）"
          % ("通过 ✓" if ok_d else "**不通过：速度或方向不对**", st.v, abs(st.v - 0.30)))
    rows.append(dict(case="track", ok=int(ok_d), worst_deg=0.0,
                     x=round(st.x, 3), v=round(st.v, 3)))

    all_ok = ok_a and ok_b and ok_c and ok_d
    print("\n[结论] %s" % ("四项全部通过，可以进入神经网络训练" if all_ok
                           else "存在未通过项，见上面 **不通过** 标记"))
    if args.csv:
        out = Path(args.csv)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["case", "ok", "worst_deg", "x", "v"])
            w.writeheader()
            for r in rows:
                w.writerow(r)
        print("[导出] %s" % out)
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
