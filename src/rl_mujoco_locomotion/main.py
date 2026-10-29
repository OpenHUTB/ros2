#!/usr/bin/env python3
"""Unified entry point for MuJoCo locomotion RL module."""

import os
import sys
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 子命令内部再导入各模块：doctor 需要在缺少 matplotlib / imageio / mujoco 时也能给出诊断，
# 因此这里不做顶层导入，避免缺依赖时直接 ImportError 崩溃。


def cmd_doctor(args):
    print("=== Environment Doctor ===")
    try:
        import mujoco
        print(f"[OK] mujoco {mujoco.__version__}")
    except Exception as e:
        print(f"[FAIL] mujoco: {e}")
    try:
        import numpy
        print(f"[OK] numpy {numpy.__version__}")
    except Exception as e:
        print(f"[FAIL] numpy: {e}")
    try:
        import matplotlib
        print(f"[OK] matplotlib {matplotlib.__version__}")
    except Exception as e:
        print(f"[FAIL] matplotlib: {e}")
    try:
        import imageio
        print(f"[OK] imageio {imageio.__version__}")
    except Exception as e:
        print(f"[FAIL] imageio: {e}")
    try:
        from rl_mujoco_locomotion.env import ensure_assets
        ensure_assets()
        print("[OK] assets 已就绪")
    except Exception as e:
        print(f"[FAIL] assets: {e}")
    print("Doctor finished.")


def cmd_test(args):
    print("=== Smoke Test ===")
    from rl_mujoco_locomotion.env import make_env
    for name in ("halfcheetah", "ant", "walker2d"):
        env = make_env(name)
        obs = env.reset()
        print(f"[{name}] obs_dim={obs.shape[0]} act_dim={env.action_space_dim}")
        for _ in range(10):
            obs, rew, term, trunc, _ = env.step(env.action_space_dim * [0.0])
            if term or trunc:
                break
        print(f"[{name}] step OK")
    print("All smoke tests passed.")


def cmd_train(args):
    from rl_mujoco_locomotion.train import train
    train(args.env, alg=args.alg, episodes=args.episodes, seed=args.seed,
          max_total_steps=args.max_steps)


def cmd_evaluate(args):
    from rl_mujoco_locomotion.evaluate import evaluate
    evaluate(args.env, alg=args.alg, episodes=args.episodes, seed=args.seed)


def cmd_render(args):
    from rl_mujoco_locomotion.render_gif import render_gif
    render_gif(args.env, alg=args.alg, episodes=args.episodes, seed=args.seed,
               frame_stride=args.stride)


def cmd_compare(args):
    from rl_mujoco_locomotion.compare import plot_all
    plot_all()


def main():
    parser = argparse.ArgumentParser(description="MuJoCo Locomotion RL")
    sub = parser.add_subparsers(dest="cmd")

    p = sub.add_parser("doctor", help="Check environment")
    p.set_defaults(func=cmd_doctor)

    p = sub.add_parser("test", help="Run smoke tests")
    p.set_defaults(func=cmd_test)

    p = sub.add_parser("train", help="Train RL agent")
    p.add_argument("--env", required=True, choices=["halfcheetah", "ant", "walker2d"])
    p.add_argument("--alg", default="ppo", choices=["ppo", "sac"])
    p.add_argument("--episodes", type=int, default=300)
    p.add_argument("--max-steps", type=int, default=None,
                   help="Total environment step budget (stops early once reached)")
    p.add_argument("--seed", type=int, default=0)
    p.set_defaults(func=cmd_train)

    p = sub.add_parser("evaluate", help="Evaluate trained agent")
    p.add_argument("--env", required=True, choices=["halfcheetah", "ant", "walker2d"])
    p.add_argument("--alg", default="ppo", choices=["ppo", "sac"])
    p.add_argument("--episodes", type=int, default=10)
    p.add_argument("--seed", type=int, default=42)
    p.set_defaults(func=cmd_evaluate)

    p = sub.add_parser("render", help="Render trained policy to GIF")
    p.add_argument("--env", required=True, choices=["halfcheetah", "ant", "walker2d"])
    p.add_argument("--alg", default="ppo", choices=["ppo", "sac"])
    p.add_argument("--episodes", type=int, default=3)
    p.add_argument("--stride", type=int, default=5,
                   help="Keep one frame every N environment steps")
    p.add_argument("--seed", type=int, default=42)
    p.set_defaults(func=cmd_render)

    p = sub.add_parser("compare", help="Plot comparison curves")
    p.set_defaults(func=cmd_compare)

    args = parser.parse_args()
    if args.cmd is None:
        parser.print_help()
        return
    args.func(args)


if __name__ == "__main__":
    main()
