"""无 ROS 的轨迹跟踪离线评测：NN 控制器 vs LOS+PID 基准。

用法（在模块目录下）::

    python eval_offline.py --duration 60
    python eval_offline.py --trajectories 3d_helix --controllers nn

结果写入 results/offline_eval.json（指标）与 results/offline_tracking.png（误差曲线）。
代理模型说明见 rov_mujoco/offline_loop.py，只能用于回归和相对比较。
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from rov_mujoco.nn_trajectory_controller import LOSTrajectoryController, NNTrajectoryController
from rov_mujoco.offline_loop import run_episode

ROOT = Path(__file__).resolve().parent
CONTROLLERS = {'nn': NNTrajectoryController, 'los': LOSTrajectoryController}


def evaluate(controllers, trajectories, duration, dt, start_offset):
    results = {}
    for trajectory in trajectories:
        for name in controllers:
            controller = CONTROLLERS[name]()
            episode = run_episode(controller, trajectory_type=trajectory, duration=duration, dt=dt,
                                  start_offset=start_offset)
            results[f'{trajectory}/{name}'] = episode
            print(f"{trajectory:12s} {name:3s}  RMSE={episode['rmse']:.3f} m  "
                  f"max={episode['max_error']:.3f} m  final={episode['final_error']:.3f} m  "
                  f"yaw_rmse={episode['yaw_rmse']:.3f} rad  effort={episode['effort']:.0f}", flush=True)
    return results


def figure(results, path):
    keys = list(results)
    fig, axes = plt.subplots(1, len(set(k.split('/')[0] for k in keys)) or 1, figsize=(11, 3.8), squeeze=False)
    for ax, trajectory in zip(axes[0], sorted(set(k.split('/')[0] for k in keys))):
        for key in keys:
            if not key.startswith(trajectory + '/'):
                continue
            episode = results[key]
            ax.plot(episode['times'], episode['errors'], label=f"{key.split('/')[1].upper()}  "
                    f"RMSE {episode['rmse']:.2f} m", alpha=.85)
        ax.set(title=trajectory, xlabel='Simulation seconds', ylabel='Tracking error (m)')
        ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.suptitle('Offline surrogate loop (not MuJoCo dynamics)')
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def check_gate(summary, max_rmse):
    """返回超过门限的 (组合, RMSE) 列表，空列表表示通过。"""
    return [(key, value['rmse']) for key, value in summary.items()
            if key != 'config' and value['rmse'] > max_rmse]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--controllers', default='nn,los')
    parser.add_argument('--trajectories', default='3d_helix,lawnmower')
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--start-offset', default='1.0,-0.8,0.5',
                        help='初始位置相对参考点的偏移，逗号分隔 x,y,z')
    parser.add_argument('--dt', type=float, default=0.02)
    parser.add_argument('--output', default='results/offline_eval.json')
    parser.add_argument('--figure', default='results/offline_tracking.png')
    parser.add_argument('--max-rmse', type=float, default=None,
                        help='回归门禁：任一组合 RMSE 超过该值则退出码为 1（可放进 CI）')
    args = parser.parse_args()

    controllers = [c.strip() for c in args.controllers.split(',') if c.strip()]
    trajectories = [t.strip() for t in args.trajectories.split(',') if t.strip()]
    unknown = [c for c in controllers if c not in CONTROLLERS]
    if unknown:
        parser.error(f'unknown controllers: {unknown}')

    start_offset = tuple(float(v) for v in args.start_offset.split(','))
    results = evaluate(controllers, trajectories, args.duration, args.dt, start_offset)
    output = ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    summary = {key: {k: v for k, v in value.items() if k not in ('times', 'errors')}
               for key, value in results.items()}
    summary['config'] = {'duration': args.duration, 'dt': args.dt, 'start_offset': list(start_offset),
                         'controllers': controllers, 'trajectories': trajectories,
                         'vehicle': 'first-order surrogate, not MuJoCo dynamics'}
    output.write_text(json.dumps(summary, indent=2), encoding='utf-8')
    figure(results, ROOT / args.figure)
    print(f'wrote {output} and {args.figure}')
    if args.max_rmse is not None:
        failures = check_gate(summary, args.max_rmse)
        if failures:
            for key, rmse in failures:
                print(f'GATE FAIL  {key}: RMSE {rmse:.3f} m > {args.max_rmse:.3f} m')
            raise SystemExit(1)
        print(f'GATE PASS  all combinations RMSE <= {args.max_rmse:.3f} m')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
