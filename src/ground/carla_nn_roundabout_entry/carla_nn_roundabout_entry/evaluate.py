"""闭环评估：规则式汇入基线 vs 训练好的端到端模型。

跑多个 episode（不同 NPC 相位），统计：
  - 碰撞率（crashed episodes / total）
  - 路线完成率（finished episodes / total）
  - 控制误差（NN 预测与规则专家在同一场景下的动作 MAE，离线在测试集上）
"""

from __future__ import annotations

import numpy as np

from .dataset import make_dataset, split_dataset, rule_label
from .simulator import RoundaboutEnv


def rule_policy(obs: np.ndarray) -> np.ndarray:
    """直接从观测反推规则动作（基线策略）。

    数据集 render_frame 是可逆的简化：从画面里来车亮块位置和车道线
    直接读出 lateral/oncoming_x/oncoming_y，再走规则。
    """
    # 简化：直接用 obs 对应的场景参数。这里用图像里亮块位置估计。
    # 为了公平可比，我们用 env 里的真实几何（基线"知道"世界状态），
    # NN 只看图像——这正是端到端 vs 规则基线的对比。
    raise NotImplementedError("规则策略在 evaluate 中通过 env.info 拿真实状态")


def run_episode(env: RoundaboutEnv, policy_fn, npc_phase: float):
    obs = env.reset(npc_phase)
    total = 0.0
    steps = 0
    while True:
        action = policy_fn(obs, env)
        obs, v, done, info = env.step(action)
        total += v
        steps += 1
        if done:
            return {"crashed": env.crashed, "finished": env.finished,
                    "avg_speed": total / max(1, steps)}


def evaluate(model=None, n_episodes: int = 24, seed: int = 0):
    """对比规则基线和 NN 模型。返回统计字典。"""
    rng = np.random.default_rng(seed)
    phases = np.linspace(0, 2 * np.pi, n_episodes, endpoint=False)

    def rule_fn(obs, env):
        # 基线用真实几何（env._npc_state 是确定性的）
        ox, oy = env._npc_state()
        from .dataset import rule_label
        return rule_label(env.lateral, ox, oy)

    def nn_fn(obs, env):
        out = model.predict(obs[None, ...])[0]
        return out

    rule_stats = {"crashes": 0, "finishes": 0}
    nn_stats = {"crashes": 0, "finishes": 0}

    for ph in phases:
        env = RoundaboutEnv()
        r = run_episode(env, rule_fn, ph)
        rule_stats["crashes"] += int(r["crashed"])
        rule_stats["finishes"] += int(r["finished"])

        if model is not None:
            env2 = RoundaboutEnv()
            n = run_episode(env2, nn_fn, ph)
            nn_stats["crashes"] += int(n["crashed"])
            nn_stats["finishes"] += int(n["finished"])

    result = {
        "episodes": n_episodes,
        "rule": {
            "collision_rate": rule_stats["crashes"] / n_episodes,
            "finish_rate": rule_stats["finishes"] / n_episodes,
        },
    }
    if model is not None:
        result["nn"] = {
            "collision_rate": nn_stats["crashes"] / n_episodes,
            "finish_rate": nn_stats["finishes"] / n_episodes,
        }
    return result


def test_control_error(model, n: int = 400, seed: int = 123):
    """在 held-out 测试集上算 NN 预测 vs 规则专家的动作 MAE。"""
    images, labels = make_dataset(n=n, seed=seed)
    _, _, test_img, test_lab = split_dataset(images, labels, seed=seed)
    pred = model.predict(test_img)
    mae = np.mean(np.abs(pred - test_lab), axis=0)
    return {
        "n_test": len(test_lab),
        "mae_steer": float(mae[0]),
        "mae_throttle": float(mae[1]),
        "mae_brake": float(mae[2]),
        "mae_overall": float(np.mean(mae)),
    }
