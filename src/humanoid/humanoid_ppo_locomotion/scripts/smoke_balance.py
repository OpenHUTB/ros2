"""验证平衡基元：零残差动作下应能站 >10 s。"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from humanoid_ppo_locomotion.g1_env import G1WalkEnv, OBS_DIM, EPISODE_STEPS  # noqa

MODEL = r"C:\g1_assets\g1\scene_29dof.xml"

for use_bal in (False, True):
    env = G1WalkEnv(model_path=MODEL, cmd_vx_range=(0.0, 0.0), cmd_wz_range=(0.0, 0.0),
                    use_balance=use_bal)
    lens, heights, xs = [], [], []
    for k in range(5):
        obs, _ = env.reset(seed=k, options={"cmd": np.array([0.0, 0.0])})
        assert obs.shape == (OBS_DIM,), obs.shape
        steps, done = 0, False
        while not done and steps < EPISODE_STEPS:
            obs, r, term, trunc, info = env.step(np.zeros(12, dtype=np.float32))
            steps += 1
            done = term or trunc
        lens.append(steps)
        heights.append(env.sim.base_height)
        xs.append(env.sim.data.qpos[0])
    tag = "平衡基元+残差" if use_bal else "纯PD（无基元）"
    print(f"{tag:<14} 步数={lens} 平均={np.mean(lens):6.1f} ({np.mean(lens)/50:5.2f}s) "
          f"末高={np.mean(heights):.3f} 位移={np.mean(xs):+.3f}")
    env.close()

# 基元 + 随机残差（模拟训练初期探索）
env = G1WalkEnv(model_path=MODEL, cmd_vx_range=(0.0, 0.0), cmd_wz_range=(0.0, 0.0))
lens = []
np.random.seed(0)
for k in range(5):
    env.reset(seed=k, options={"cmd": np.array([0.0, 0.0])})
    steps, done = 0, False
    while not done and steps < EPISODE_STEPS:
        a = np.random.uniform(-0.3, 0.3, 12).astype(np.float32)
        _, _, term, trunc, _ = env.step(a)
        steps += 1
        done = term or trunc
    lens.append(steps)
print(f"{'基元+随机残差':<14} 步数={lens} 平均={np.mean(lens):6.1f} ({np.mean(lens)/50:5.2f}s)")
print("\n观测维度 =", OBS_DIM, " 动作维度 = 12")
