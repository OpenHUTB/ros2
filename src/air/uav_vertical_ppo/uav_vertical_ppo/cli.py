"""训练 PPO 策略并保存多场景定高结果."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from .environment import VerticalHoverEnv
from .paths import resource


def train(output, steps=65536, seed=17):
    """以固定种子训练双层 MLP 策略，保存真实交互回报."""
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.monitor import Monitor

    torch.set_num_threads(1)
    output.mkdir(parents=True, exist_ok=True)
    env = Monitor(VerticalHoverEnv(), str(output / 'training'))
    model = PPO('MlpPolicy', env, seed=seed, device='cpu', verbose=0,
                n_steps=1024, batch_size=64, n_epochs=10, learning_rate=3e-4,
                gamma=0.99, policy_kwargs={'net_arch': [32, 32]})
    try:
        model.learn(total_timesteps=steps)
        model.save(output / 'hover_ppo')
        metadata = {'seed': seed, 'requested_steps': steps,
                    'actual_steps': model.num_timesteps, 'network': [2, 32, 32, 1]}
        (output / 'training_config.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    finally:
        env.close()


def evaluate(model_path, output, gif=False):
    """用未参与训练的种子比较 PPO、PD 与固定重力补偿."""
    from stable_baselines3 import PPO
    from PIL import Image
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    from .policy import load_policy
    reference = VerticalHoverEnv()
    try:
        model = load_policy(model_path, reference)
    finally:
        reference.close()
    metrics, trajectories, frames = [], {}, []
    for scenario in ('calm', 'gust', 'payload'):
        for controller in ('ppo', 'pd', 'constant'):
            errors, successes = [], []
            for seed in (101, 202, 303):
                env = VerticalHoverEnv(scenario=scenario)
                rows = []
                try:
                    obs, _ = env.reset(seed=seed)
                    for step in range(400):
                        if controller == 'ppo':
                            action, _ = model.predict(obs, deterministic=True)
                        elif controller == 'pd':
                            action = np.array([(-4.0 * obs[0] - 3.0 * obs[1]) / 6.0])
                        else:
                            action = np.zeros(1)
                        obs, _, terminated, truncated, info = env.step(action)
                        rows.append(info)
                        if gif and scenario == 'gust' and controller == 'ppo' and seed == 101 and step % 4 == 0:
                            frames.append(Image.fromarray(env.render()))
                        if terminated or truncated:
                            break
                    errors.extend([r['height_m'] - r['target_m'] for r in rows])
                    successes.append(len(rows) == 400 and all(abs(r['height_m'] - r['target_m']) < 0.15 for r in rows[-50:]))
                    with (output / f'{scenario}_{controller}_{seed}.csv').open('w', newline='', encoding='utf-8') as stream:
                        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                        writer.writeheader()
                        writer.writerows(rows)
                    if seed == 101:
                        trajectories[(scenario, controller)] = rows
                finally:
                    env.close()
            metrics.append({'scenario': scenario, 'controller': controller,
                            'rmse_m': float(np.sqrt(np.mean(np.square(errors)))),
                            'max_error_m': float(np.max(np.abs(errors))),
                            'success_rate': float(np.mean(successes)), 'episodes': 3})
    (output / 'metrics.json').write_text(json.dumps(metrics, indent=2), encoding='utf-8')
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.5))
    for ax, scenario in zip(axes, ('calm', 'gust', 'payload')):
        for controller in ('ppo', 'pd', 'constant'):
            rows = trajectories[(scenario, controller)]
            ax.plot([r['time_s'] for r in rows], [r['height_m'] for r in rows], label=controller)
        ax.axhline(1.5, color='black', linestyle='--', label='target')
        ax.set(title=scenario, xlabel='Time (s)', ylabel='Height (m)', ylim=(0, 4))
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output / 'comparison.png', dpi=140)
    plt.close(fig)
    if frames:
        frames[0].save(output / 'hover.gif', save_all=True, append_images=frames[1:], duration=80, loop=0)
    print(json.dumps(metrics, indent=2))


def main():
    parser = argparse.ArgumentParser(description='四旋翼垂直通道 PPO 定高控制')
    parser.add_argument('mode', nargs='?', choices=['train', 'evaluate', 'ros'], default='evaluate')
    parser.add_argument('--output', type=Path, default=Path('outputs'))
    parser.add_argument('--model', type=Path)
    parser.add_argument('--steps', type=int, default=65536)
    parser.add_argument('--gif', action='store_true')
    args = parser.parse_args()
    if args.mode == 'train':
        if args.steps <= 0:
            parser.error('训练步数必须大于零')
        train(args.output, args.steps)
    elif args.mode == 'ros':
        from .ros_node import main as ros_main
        ros_main()
    else:
        evaluate(args.model or resource('data/hover_ppo.zip'), args.output, args.gif)


if __name__ == '__main__':
    main()
