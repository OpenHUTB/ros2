"""离线演示与刚度说明（不需要 ROS）。

由包根目录的 ``main.py`` 与 ``ros2 run humanoid_g1_compliance demo`` 共用。
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

from .compliance import ComplianceRunner, Perturbation
from .policy import (
    DEFAULT_STIFFNESS_5, STIFF_BAND, STIFF_NAMES, Command, LacPolicy, StartupFSM,
)
from .sim_core import G1Sim


def list_stiffness() -> int:
    print('5 个刚度通道（策略的观测输入，决定外力下的柔顺程度）：')
    for i, name in enumerate(STIFF_NAMES):
        band = STIFF_BAND['K_ang' if 'ang' in name else 'K_lin']
        unit = 'N·m/rad' if 'ang' in name else 'N/m'
        print(f'  [{i}] {name:10s} 默认 {DEFAULT_STIFFNESS_5[i]:6.0f} {unit:8s} '
              f'范围 [{band[0]:.0f}, {band[1]:.0f}]')
    print('\nla/ra/to = 左臂/右臂/躯干；lin = 线刚度，ang = 角刚度')
    return 0


def run_demo(args: argparse.Namespace) -> int:
    sim = G1Sim()
    policy = LacPolicy(device=args.device)
    runner = ComplianceRunner(sim, policy, StartupFSM(direct=True))

    cmd = Command()
    cmd.stiffness = np.asarray(args.stiffness, dtype=np.float64)

    print(f'\n[1/2] 站立 {args.settle:.0f} s（刚度 K_lin_la={cmd.stiffness[0]:.0f}）')
    log = runner.run(args.settle, cmd=cmd)
    pel = np.asarray(log.pelvis)
    print(f'      骨盆高度 {pel[:, 2].min():.3f} ~ {pel[:, 2].max():.3f} m  '
          f'姿态 pg_z={np.asarray(log.pg_z)[-1]:+.4f}  '
          f'{"摔倒" if log.fell else "稳定"}')

    pert = Perturbation(body=args.body, axis=args.axis, force=args.force,
                        t_start=args.settle, duration=args.push_dur)
    total = args.settle + args.push_dur + args.after
    print(f'\n[2/2] 手腕沿 {args.axis} 轴受 {args.force:+.1f} N 外力 {args.push_dur:.1f} s')
    log2 = runner.run(total, cmd=cmd, perturb=pert)
    m = runner.metrics(log2, pert, k_lin_la=float(cmd.stiffness[0]))

    rec = '—' if np.isnan(m.recover_s) else f'{m.recover_s:.2f} s'
    print(f'      峰值让位   {m.peak_mm:6.1f} mm')
    print(f'      稳态让位   {m.steady_mm:6.1f} mm   -> 等效末端刚度 '
          f'{m.effective_stiffness():.0f} N/m')
    print(f'      撤销残留   {m.settle_mm:6.1f} mm')
    print(f'      恢复时间   {rec}')
    print(f'      力矩峰值   {m.tau_peak:6.2f} N·m')
    print(f'      躯干偏移   {m.pelvis_shift_mm:6.1f} mm')
    print(f'      结果       {"摔倒" if m.fell else "OK"}')
    return 0 if not m.fell else 1


def main() -> int:
    ap = argparse.ArgumentParser(description='G1 全身柔顺控制（LAC）离线演示')
    ap.add_argument('--stiffness', type=float, nargs=5,
                    default=list(DEFAULT_STIFFNESS_5))
    ap.add_argument('--force', type=float, default=6.0)
    ap.add_argument('--axis', default='y', choices=['x', 'y', 'z'])
    ap.add_argument('--body', default='left_wrist_roll_rubber_hand')
    ap.add_argument('--settle', type=float, default=3.0)
    ap.add_argument('--push-dur', type=float, default=2.0)
    ap.add_argument('--after', type=float, default=2.5)
    ap.add_argument('--device', default='cpu')
    return run_demo(ap.parse_args())


if __name__ == '__main__':
    sys.exit(main())
