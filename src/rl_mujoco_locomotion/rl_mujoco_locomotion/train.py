"""Training loops for PPO and SAC on MuJoCo locomotion."""

import os
import csv
import numpy as np

from .env import make_env, ensure_assets
from .ppo import PPO, RolloutBuffer
from .sac import SAC, ReplayBuffer

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "..", "results")


def _open_csv(tag):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    path = os.path.join(RESULTS_DIR, f"{tag}.csv")
    f = open(path, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["episode", "episode_return", "episode_length"])
    return f, writer, path


def train_ppo(env_name, episodes=300, steps_per_epoch=4000, seed=0, max_total_steps=None):
    ensure_assets()
    env = make_env(env_name)
    obs_dim, act_dim = env.observation_space_dim, env.action_space_dim
    agent = PPO(obs_dim, act_dim, seed=seed)
    buf = RolloutBuffer(obs_dim, act_dim, steps_per_epoch)
    f, writer, csv_path = _open_csv(f"{env_name}_ppo")

    obs = env.reset(seed=seed)
    ep_ret, ep_len, ep_count = 0.0, 0, 0
    total_steps = 0
    ep_return_hist = []

    while ep_count < episodes and (max_total_steps is None or total_steps < max_total_steps):
        for _ in range(steps_per_epoch):
            act, logp = agent.act(obs)
            val = float(agent.critic.forward(np.atleast_2d(obs))[0, 0])
            next_obs, rew, terminated, truncated, _ = env.step(act)
            next_obs = np.nan_to_num(next_obs, nan=0.0, posinf=0.0, neginf=0.0)
            rew = float(np.nan_to_num(rew, nan=0.0, posinf=0.0, neginf=0.0))
            done = terminated or truncated
            buf.store(obs, act, rew, val, logp, done)
            obs = next_obs
            ep_ret += rew
            ep_len += 1
            total_steps += 1

            if done:
                buf.finish_path(last_val=0.0)
                ep_count += 1
                ep_return_hist.append(ep_ret)
                writer.writerow([ep_count, round(ep_ret, 4), ep_len])
                if ep_count % 10 == 0 or ep_count == 1:
                    recent = np.mean(ep_return_hist[-10:])
                    print(f"[PPO:{env_name}] ep {ep_count}/{episodes}  "
                          f"ret={ep_ret:.2f}  avg10={recent:.2f}  len={ep_len}  "
                          f"steps={total_steps}")
                    f.flush()
                obs = env.reset()
                ep_ret, ep_len = 0.0, 0
                if ep_count >= episodes:
                    break
            if max_total_steps is not None and total_steps >= max_total_steps:
                break

        if buf.ptr > 0:
            last_val = float(agent.critic.forward(np.atleast_2d(obs))[0, 0])
            buf.finish_path(last_val=last_val)
            agent.update(buf)

    agent.save(os.path.join(RESULTS_DIR, f"{env_name}_ppo_final"))
    f.close()
    print(f"[PPO:{env_name}] done -> {csv_path}")
    return csv_path


def train_sac(env_name, episodes=300, max_steps=1000, seed=0,
              start_steps=1000, update_every=4, batch_size=128, max_total_steps=None):
    ensure_assets()
    env = make_env(env_name)
    obs_dim, act_dim = env.observation_space_dim, env.action_space_dim
    agent = SAC(obs_dim, act_dim, seed=seed)
    buf = ReplayBuffer(obs_dim, act_dim, capacity=300000)
    f, writer, csv_path = _open_csv(f"{env_name}_sac")

    total_steps = 0
    ep_return_hist = []

    for ep in range(episodes):
        if max_total_steps is not None and total_steps >= max_total_steps:
            break
        obs = env.reset(seed=seed + ep)
        ep_ret, ep_len = 0.0, 0
        for t in range(max_steps):
            if total_steps < start_steps:
                act = np.random.uniform(-1.0, 1.0, size=act_dim)
            else:
                act = agent.act(obs, deterministic=False)
            next_obs, rew, terminated, truncated, _ = env.step(act)
            next_obs = np.nan_to_num(next_obs, nan=0.0, posinf=0.0, neginf=0.0)
            rew = float(np.nan_to_num(rew, nan=0.0, posinf=0.0, neginf=0.0))
            done = terminated or truncated
            buf.store(obs, act, rew, next_obs, done)
            obs = next_obs
            ep_ret += rew
            ep_len += 1
            total_steps += 1

            if total_steps >= start_steps and total_steps % update_every == 0:
                agent.update(buf, batch_size=batch_size)

            if done:
                break

        ep_return_hist.append(ep_ret)
        writer.writerow([ep + 1, round(ep_ret, 4), ep_len])
        if (ep + 1) % 10 == 0 or ep == 0:
            recent = np.mean(ep_return_hist[-10:])
            alpha = float(np.exp(agent.log_alpha))
            print(f"[SAC:{env_name}] ep {ep+1}/{episodes}  ret={ep_ret:.2f}  "
                  f"avg10={recent:.2f}  len={ep_len}  alpha={alpha:.3f}  "
                  f"updates={agent.update_count}")
            f.flush()

    agent.save(os.path.join(RESULTS_DIR, f"{env_name}_sac_final"))
    f.close()
    print(f"[SAC:{env_name}] done -> {csv_path}")
    return csv_path


def train(env_name, alg="ppo", episodes=300, seed=0, max_total_steps=None):
    if alg.lower() == "ppo":
        return train_ppo(env_name, episodes=episodes, seed=seed,
                         max_total_steps=max_total_steps)
    if alg.lower() == "sac":
        return train_sac(env_name, episodes=episodes, seed=seed,
                         max_total_steps=max_total_steps)
    raise ValueError(f"Unknown algorithm: {alg}")
