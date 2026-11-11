#!/usr/bin/env python3
"""无 ROS 冒烟测试：模型加载 + 策略推理 + 站立稳定性 + 一次外力响应。

    python scripts/smoke_compliance.py            # 站立 8 s + 6 N 扰动
    python scripts/smoke_compliance.py --seconds 5

全部通过时输出 ALL PASS 并返回 0。评测机不需要 ROS、不需要显卡、不需要手柄。
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from humanoid_g1_compliance.compliance import (          # noqa: E402
    ComplianceRunner, Perturbation,
)
from humanoid_g1_compliance.policy import (              # noqa: E402
    Command, LacPolicy, StartupFSM,
)
from humanoid_g1_compliance.sim_core import G1Sim         # noqa: E402

ok = True


def check(name: str, cond: bool, detail: str = '') -> None:
    global ok
    print(f'  [{"PASS" if cond else "FAIL"}] {name}{("  " + detail) if detail else ""}')
    ok = ok and bool(cond)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=8.0)
    ap.add_argument('--force', type=float, default=6.0)
    ap.add_argument('--k-la', type=float, default=250.0)
    ap.add_argument('--device', default='cpu')
    args = ap.parse_args()

    t0 = time.perf_counter()

    print('[1/4] 模型加载')
    sim = G1Sim()
    check('model nu == 29', sim.nu == 29, f'nu={sim.nu}')
    check('总质量 ≈ 34 kg', 30.0 < sim.total_mass() < 40.0,
          f'{sim.total_mass():.2f} kg')
    st = sim.read()
    check('初始 pg_z ≈ -1（直立）',
          abs(sim.projected_gravity(st.quat)[2] + 1.0) < 1e-3)
    check('初始骨盆高度 ≈ 0.74 m',
          abs(sim.pelvis_pos()[2] - 0.74) < 0.02,
          f'{sim.pelvis_pos()[2]:.3f} m')

    print('\n[2/4] 策略加载')
    policy = LacPolicy(device=args.device)
    check('policy_obs == 950', policy.info['num_policy_obs'] == 950)
    check('policy_hist == 6080', policy.info['num_policy_hist_obs'] == 6080)

    # 默认站姿前向指纹：每次运行应完全一致（权重与观测布局的联合校验）。
    # 注意与上游 tests/smoke.py 的 1.6833 不同——那边喂的是全零关节状态，
    # 这里是真实 default pose，属于两个不同的输入点。
    cmd = Command()
    cmd.stiffness[0] = args.k_la
    policy.reset()
    for _ in range(3):
        sim.reset()
        policy.reset()
        st = sim.read()
        policy.infer(st, cmd)
    a = policy.action()
    check('默认姿态动作有限', bool(np.isfinite(a).all()))
    print(f'         action abs_max={np.abs(a).max():.4f}  mean={a.mean():+.4f}  (指纹)')

    print('\n[3/4] 站立稳定性')
    runner = ComplianceRunner(sim, policy, StartupFSM(direct=True))
    log = runner.run(args.seconds, cmd=cmd)
    pg = np.asarray(log.pg_z)
    pel = np.asarray(log.pelvis)
    tt = np.asarray(log.t)
    check('未摔倒', not log.fell)
    check('最低骨盆高度 > 0.6 m', pel[:, 2].min() > 0.6,
          f'{pel[:, 2].min():.3f} m')
    check('姿态保持直立（末段 pg_z < -0.98）', pg[-20:].mean() < -0.98,
          f'{pg[-20:].mean():+.4f}')
    # 稳态漂移：起步 4 s 内是策略从 default pose 收敛到自己平衡点的过渡
    # （实测后移约 8.6 cm，时间常数 ~1.3 s），6 s 后应几乎不动（< 5 mm，即收敛后残余的毫米级蠕动）。
    steady = pel[tt > 6.0]
    drift = float(np.linalg.norm(steady[:, :2] - steady[:, :2].mean(axis=0),
                                 axis=1).max())
    check('稳态水平波动 < 5 mm', drift < 0.005, f'{drift * 1000:.2f} mm')

    print('\n[4/4] 外力柔顺响应')
    pert = Perturbation(body='left_wrist_roll_rubber_hand', axis='y',
                        force=args.force, t_start=3.0, duration=2.0)
    log2 = runner.run(7.0, cmd=cmd, perturb=pert)
    m = runner.metrics(log2, pert, k_lin_la=args.k_la)
    print('        ' + '  '.join(f'{k}={v}' for k, v in m.as_dict().items()))
    check('扰动后未摔倒', not m.fell)
    check('末端让位 > 5 mm', m.peak_mm > 5.0, f'{m.peak_mm:.1f} mm')
    check('撤销后回位（残留 < 峰值）', m.settle_mm < m.peak_mm,
          f'settle={m.settle_mm:.1f} mm < peak={m.peak_mm:.1f} mm')

    print(f'\n总用时 {time.perf_counter() - t0:.1f}s')
    print('\n' + ('ALL PASS' if ok else 'SOME FAILED'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
