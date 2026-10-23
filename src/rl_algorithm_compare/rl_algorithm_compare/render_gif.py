#!/usr/bin/env python3
"""渲染训练好的神经网络策略在 MuJoCo CartPole 上的推理 gif。

优先用 MuJoCo 离屏渲染（需 OpenGL/Xvfb）；若环境不支持 GL，
自动退回用 matplotlib 根据状态绘制倒立摆动画，保证在无显示器
的服务器/虚拟机上也能生成演示动图。

用法：
  python3 render_gif.py --model results/dqn_mlp1.pkl --algo dqn --episodes 1
"""
import argparse
import os
import pickle

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image

from .cartpole_env import CartPoleEnv
from .dqn import DQN
from .reinforce import REINFORCE


def _draw_state(obs):
    """根据 4 维状态画一帧倒立摆（matplotlib 兜底渲染）。"""
    x, xd, th, thd = obs
    fig, ax = plt.subplots(figsize=(4, 3), dpi=90)
    ax.add_patch(Rectangle((-2.6, -0.06), 5.2, 0.12, fc='0.75', ec='none'))
    ax.add_patch(Rectangle((x - 0.4, 0.06), 0.8, 0.2, fc='#3366cc', ec='k'))
    px = x + np.sin(th)
    py = 0.26 + np.cos(th)
    ax.plot([x, px], [0.26, py], color='#cc3333', lw=4,
            solid_capstyle='round')
    ax.scatter([px], [py], c='k', s=40, zorder=5)
    ax.set_xlim(-2.6, 2.6)
    ax.set_ylim(-0.25, 1.7)
    ax.set_aspect('equal')
    ax.axis('off')
    fig.tight_layout(pad=0)
    fig.canvas.draw()
    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8).reshape(
        fig.canvas.get_width_height()[::-1] + (4,))
    plt.close(fig)
    return buf[..., :3].copy()


def _try_mujoco_frame(env, use_mujoco):
    if not use_mujoco:
        return None, False
    try:
        return env.render_frame(), True
    except Exception:
        return None, False


def load_agent(path, algo):
    with open(path, 'rb') as f:
        d = pickle.load(f)
    sizes = d['sizes']
    layers = sizes[1:-1]
    obs_dim, act_dim = sizes[0], sizes[-1]
    if algo == 'dqn':
        agent = DQN(obs_dim, act_dim, layers)
    else:
        agent = REINFORCE(obs_dim, act_dim, layers)
    return agent.load(path)


def render(agent, env, path, episodes=1, fps=30, use_mujoco=True):
    frames = []
    for _ in range(episodes):
        s = env.reset()
        done, traj = False, []
        while not done:
            a = agent.act(s, greedy=True)
            s2, r, done, _ = env.step(a)
            traj.append(s2)
            s = s2
        # 降采样控制 gif 帧数（<10MB）
        stride = max(1, len(traj) // 150)
        for obs in traj[::stride]:
            frame, ok = _try_mujoco_frame(env, use_mujoco)
            if ok:
                frames.append(frame)
            else:
                frames.append(_draw_state(obs))
                use_mujoco = False
    if not frames:
        print('[gif] 没有帧可渲染')
        return
    imgs = [Image.fromarray(f) for f in frames]
    imgs[0].save(path, save_all=True, append_images=imgs[1:],
                 duration=int(1000 / fps), loop=0)
    print('[gif] 已保存 %s (%d 帧)' % (path, len(frames)))


def main():
    ap = argparse.ArgumentParser(description='渲染 RL 推理 gif')
    ap.add_argument('--model', required=True, help='模型 pkl 路径')
    ap.add_argument('--algo', default='dqn', choices=['dqn', 'reinforce'])
    ap.add_argument('--episodes', type=int, default=1)
    ap.add_argument('--out', default='results')
    args = ap.parse_args()

    env = CartPoleEnv()
    agent = load_agent(args.model, args.algo)
    out = os.path.join(args.out, os.path.basename(args.model)[:-4] + '.gif')
    render(agent, env, out, episodes=args.episodes)


if __name__ == '__main__':
    main()
