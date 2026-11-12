#!/usr/bin/env python3
"""刚度扫描基准：同一外力下，不同刚度指令的柔顺表现。

对每个 K_lin_la 跑一次「站稳 3 s -> 手腕受恒定力 2 s -> 撤销观察 2 s」，
记录末端让位、关节力矩峰值、躯干偏移、恢复时间与等效末端刚度。

    python scripts/bench_compliance.py                       # 默认 7 个刚度点
    python scripts/bench_compliance.py --stiffness 30 250 500
    python scripts/bench_compliance.py --force 6 --json out.json

输出可直接喂给 ``scripts/plot_compliance.py`` 画图。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from humanoid_g1_compliance.compliance import (          # noqa: E402
    ComplianceRunner, Perturbation,
)
from humanoid_g1_compliance.policy import Command, LacPolicy, StartupFSM  # noqa: E402
from humanoid_g1_compliance.sim_core import G1Sim         # noqa: E402

DEFAULT_STIFFNESS = [30, 55, 80, 110, 150, 250, 500]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--stiffness', type=float, nargs='+', default=DEFAULT_STIFFNESS)
    ap.add_argument('--force', type=float, default=6.0, help='手腕外力 N')
    ap.add_argument('--axis', default='y', choices=['x', 'y', 'z'])
    ap.add_argument('--body', default='left_wrist_roll_rubber_hand')
    ap.add_argument('--settle', type=float, default=3.0, help='施力前稳定时长 s')
    ap.add_argument('--push-dur', type=float, default=2.0)
    ap.add_argument('--after', type=float, default=2.5, help='撤销后观察时长 s')
    ap.add_argument('--json', default=None)
    ap.add_argument('--device', default='cpu')
    args = ap.parse_args()

    sim = G1Sim()
    policy = LacPolicy(device=args.device, verbose=False)
    runner = ComplianceRunner(sim, policy, StartupFSM(direct=True))

    total_s = args.settle + args.push_dur + args.after
    rows = []
    t0 = time.perf_counter()

    print(f'外力 {args.force:+.1f} N 沿 {args.axis} 轴作用于 {args.body}，'
          f'共 {len(args.stiffness)} 个刚度点\n')
    header = (f'{"K_lin_la":>9} {"peak":>8} {"steady":>8} {"settle":>8} '
              f'{"tau_pk":>8} {"pelvis":>8} {"K_eff":>9} {"recover":>8} 状态')
    print(header)
    print('-' * len(header))

    for k in args.stiffness:
        cmd = Command()
        cmd.stiffness[0] = float(k)
        pert = Perturbation(body=args.body, axis=args.axis, force=args.force,
                            t_start=args.settle, duration=args.push_dur)
        log = runner.run(total_s, cmd=cmd, perturb=pert)
        m = runner.metrics(log, pert, k_lin_la=float(k))
        rows.append(m.as_dict())
        rec = '—' if np.isnan(m.recover_s) else f'{m.recover_s:.2f}s'
        print(f'{k:9.0f} {m.peak_mm:7.1f}mm {m.steady_mm:7.1f}mm '
              f'{m.settle_mm:7.1f}mm {m.tau_peak:7.2f}N {m.pelvis_shift_mm:7.1f}mm '
              f'{m.effective_stiffness():8.0f}N/m {rec:>8} '
              f'{"摔倒" if m.fell else "OK"}')

    print(f'\n总用时 {time.perf_counter() - t0:.1f}s')

    ok_rows = [r for r in rows if not r['fell']]
    if len(ok_rows) >= 2:
        peaks = [r['steady_mm'] for r in ok_rows]
        print(f'稳态让位范围 {min(peaks):.1f} ~ {max(peaks):.1f} mm'
              f'（低刚度比高刚度多让位 {max(peaks) / min(peaks):.2f} 倍）')

    if args.json:
        payload = {
            'force_N': args.force,
            'axis': args.axis,
            'body': args.body,
            'policy': 'LAC G1-23DOF (lac_g1_23dof.pt)',
            'rows': rows,
        }
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2)
        print(f'已写入 {args.json}')

    return 0 if all(not r['fell'] for r in rows) else 1


if __name__ == '__main__':
    sys.exit(main())
