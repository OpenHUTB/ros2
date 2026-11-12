# 四旋翼姿态控制对比：PID vs LQR

对比经典 PID 与现代 LQR 两种姿态控制器在四旋翼简化模型下的性能。

## 运行

```powershell
cd src/classical_control/quadcopter_compare
python main.py
```

## 对比内容

| 指标 | PID | LQR |
|------|-----|-----|
| 最大超调 | 单轴独立，耦合弱 | 全状态反馈，天然解耦 |
| 稳态误差 | 需调积分项 | Riccati 最优解 |
| 抗扰恢复 | 依赖各轴积分 | 全局状态量直接补偿 |
| 调参难度 | 3 参数×4 轴 | Q/R 权重矩阵 |

## 模型

**4 自由度简化模型**：roll、pitch、yaw、height
- 状态向量 8 维：`[角度 ×4, 角速度 ×4]`
- 控制输入 4 维：`[roll力矩, pitch力矩, yaw力矩, 总推力]`
- 积分方式：Runge-Kutta 4

### 与 cart-pole LQR 的区别

| | cart-pole | 四旋翼 |
|--|-----------|--------|
| 自由度 | 1 (欠驱动) | 4 (完全驱动) |
| 控制目标 | 平衡 | 轨迹跟踪 + 稳定 |
| 耦合度 | 弱 | 强 |
| 非线性 | 弱 | 强 |

## 依赖

- Python 3.10+
- NumPy
