# mujoco_demo

“人形机器人基础”一页的两个可运行示例。

| 文件 | 内容 |
|---|---|
| `falling_ball.py` | 最小可跑示例：一个球自由落体 10 s（验证 MuJoCo 装好没） |
| `scene_humanoid.py` | 10 关节人形的 MJCF：骨盆 + 躯干 + 头 + 双腿（髋/膝/踝）+ 双臂（肩/肘） |
| `controller.py` | 纯控制逻辑：PD 力矩 + 重力补偿、倾角与站立判定（不依赖 mujoco） |
| `humanoid_stand.py` | 让人形站住：PD 位置控制 + `qfrc_bias` 前馈，支持可视化与无头两种跑法 |

## 运行

```bash
pip install mujoco

# 1) 自由落体（原示例）
python3 src/humanoid/mujoco_demo/falling_ball.py

# 2) 人形站立（可视化）
python3 src/humanoid/mujoco_demo/humanoid_stand.py

# 3) 人形站立（无头，站住返回 0）
python3 src/humanoid/mujoco_demo/humanoid_stand.py --headless --seconds 5

# 4) Linux 无显示服务（WSL2 / 服务器）时，无头模式指定软件渲染
MUJOCO_GL=osmesa python3 src/humanoid/mujoco_demo/humanoid_stand.py --headless --seconds 5
```

无头输出示例（本机实测，Windows + Python 3.13 + MuJoCo 3.17）：

```text
初始骨盆高度 1.042 m，关节 10 个，控制步数 2500
结果: {'height': 1.04, 'base_height': 1.042, 'min_height': 1.039, 'tilt_deg': 0.58, 'upright': True}
```

> 另在 Ubuntu 22.04.5（WSL2）+ Python 3.10.12 + MuJoCo 3.12.0 上复测：9 项测试全过，
> 无头统计与上表一致；无显示服务，可视化模式未跑。

## 控制思路

1. **站立姿态** = 所有关节角为 0（直腿、手臂下垂）；
2. **PD**：`tau = kp * (q_des - q) - kd * qd`，腿的增益比胳膊大；
3. **重力补偿**：把 `mj_rne` 算出的 `qfrc_bias` 前馈加上，否则关节会被自重压出稳态误差（腿"软"下去）；
4. **判定**：骨盆高度 ≥ 初值的 90%、倾角 < 12°，两条件同时满足才算"站住"。

## 测试

```bash
python3 tests/test_humanoid_stand_local.py
```

9 项：PD 方向/阻尼/前馈/饱和、四元数倾角、站立判定，以及一局 5 s 的无头物理（没装 mujoco 时自动跳过）。
