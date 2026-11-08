# carla_nn_roundabout_entry

环岛汇入场景下的相机端到端驾驶：16×16 灰度图像 → (转向, 油门, 制动)。
纯 NumPy 实现，无需 CARLA / ROS / 深度学习框架即可训练与评估。

对应 Issue #929。

## 快速开始

```bash
python3 -m carla_nn_roundabout_entry.main --mode demo
```

## 模式

| 模式 | 作用 |
|---|---|
| `train` | 合成数据集 + 训练 MLP，保存权重 |
| `eval` | 闭环对比规则基线 vs NN，输出碰撞率/完成率/控制误差 |
| `demo` | 一键跑 train + eval |

## 依赖

- Python 3.8+
- NumPy
- （可选）matplotlib / imageio 仅用于生成文档图
