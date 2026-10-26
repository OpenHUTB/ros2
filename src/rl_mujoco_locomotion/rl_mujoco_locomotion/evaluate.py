"""Evaluation script for trained PPO and SAC models."""

import os
import numpy as np
from .env import make_env
from .ppo import PPO
from .sac import SAC

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "..", "results")
MAX_FRAMES = 200


def evaluate(env_name, alg="ppo", episodes=3, seed=42, render=False, frame_stride=2):
    env = make_env(env_name, render_mode="rgb_array" if render else None)
    obs_dim, act_dim = env.observation_space_dim, env.action_space_dim
    if alg.lower() == "ppo":
        agent = PPO(obs_dim, act_dim, seed=seed)
    elif alg.lower() == "sac":
        agent = SAC(obs_dim, act_dim, seed=seed)
    else:
        raise ValueError(f"Unknown algorithm: {alg}")

    model_path = os.path.join(RESULTS_DIR, f"{env_name}_{alg}_final")
    if not os.path.exists(model_path + "_actor.npz"):
        raise FileNotFoundError(f"Trained model not found: {model_path}_actor.npz. Train first.")
    agent.load(model_path)

    returns, frames = [], []
    for ep in range(episodes):
        obs = env.reset(seed=seed + ep)
        done, ep_ret, step = False, 0.0, 0
        while not done:
            act = agent.act(obs, deterministic=True)
            if isinstance(act, tuple):
                act = act[0]
            act = np.nan_to_num(np.asarray(act, dtype=np.float64), nan=0.0,
                                posinf=1.0, neginf=-1.0)
            obs, rew, terminated, truncated, _ = env.step(act)
            done = terminated or truncated
            ep_ret += rew
            if render and step % frame_stride == 0 and len(frames) < MAX_FRAMES:
                img = env.render()
                if img is not None:
                    frames.append(img)
            step += 1
        returns.append(ep_ret)
        print(f"[Eval:{env_name}/{alg}] Ep {ep+1}/{episodes} return={ep_ret:.3f} len={step}")

    mean_ret = float(np.mean(returns))
    std_ret = float(np.std(returns))
    print(f"[Eval:{env_name}/{alg}] Mean={mean_ret:.3f} Std={std_ret:.3f}")
    return {"mean": mean_ret, "std": std_ret, "frames": frames}
