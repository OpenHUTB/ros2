# 复现实验结果

本目录存放 uav_dqn_nav 训练/评估实验的可复现产物：
- `eval_results.csv` —— 100 回合正式评估汇总（16 障碍 / 6 障碍）
- `training_curve.png`、`trajectory.png` —— 学习曲线与飞行轨迹图
  （绘图脚本见 `../scripts/plot_results.py`，运行时生成）

## 复现步骤（Ubuntu 20.04 / ROS Noetic 虚拟机）

```bash
# 1. 安装依赖（CPU 版 torch 即可）
pip install numpy gymnasium torch --index-url https://download.pytorch.org/whl/cpu

# 2. 训练（16 障碍，40 万步，约 40-60 分钟；6 障碍把 --obstacles 换为 6）
cd ~/catkin_ws/src/uav_dqn_nav
python3 scripts/main.py --train --total 400000 --epsilon-decay 250000 \
    --eval-freq 15000 --eval-episodes 20 --obstacles 16 --out models35

# 3. 评估（100 回合，与 PPO 口径一致）
python3 scripts/main.py --eval --weights models35/policy_weights.npz \
    --episodes 100 --obstacles 16

# 4. 绘图（需 matplotlib）
python3 scripts/plot_results.py
```

## 结果（与 PPO 对比，见 README §7）

| 场景 | DQN 成功率 | PPO 成功率 |
|---|---|---|
| 16 障碍 | 4% | 78% |
| 6 障碍 | 0% | 待补跑 |

> 结论：DQN 离散动作能学会避障但无法精确停靠 0.4 m 半径点目标；
> PPO 连续动作平滑逼近，成功率显著更高——动作空间离散化的影响。
