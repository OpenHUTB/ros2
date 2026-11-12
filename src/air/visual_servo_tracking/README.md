# 无人机视觉伺服目标跟踪 (IBVS)

## 模块说明

无人机配备俯视单目相机，通过 **基于图像特征的视觉伺服（Image-Based Visual Servo）**
闭环跟踪地面移动目标。

与仓库已有 winter 场景圆周飞行的区别：

| | 圆周飞行 | 视觉伺服 |
|---|---------|---------|
| 目标 | 固定圆周轨迹 | 未知移动目标 |
| 控制 | 轨迹跟踪（前馈+反馈） | 图像 Jacobian 闭环 |
| 传感器 | 无视觉 | 俯视单目相机 |

## 运行环境

- Python 3.10+
- NumPy（`pip install numpy`）

## 运行步骤

```powershell
cd src/air/visual_servo_tracking
python main.py
```

## 核心原理

```
图像误差: e = s_current - s_target       # s* = 图像中心
图像 Jacobian: L = [[-fx/Z, 0,   u/Z],
                    [0, -fy/Z, v/Z]]

期望相机速度: v = -λ · L⁺ · e            # Pseudo-inverse
速度 → 姿态角: roll/pitch 由 vy/vx 反推
```

## 示例输出

```
  📊 跟踪结果 (60s):
  起始水平距离: 18.03 m
  最终水平距离: 0.87 m
  平均水平距离: 3.12 m
  图像平均误差: 12.4 像素
  目标在视野时间: 58.2 s / 60 s (97%)
  🏆 是否收敛: ✅ 是
```

## 参数调整

| 参数 | 默认 | 说明 |
|------|------|------|
| lambda_gain | 0.3 | 伺服增益（收敛速度，过大会振荡） |
| initial_height_m | 12.0 | 无人机初始高度 |
| focal_length_m | 0.016 | 相机焦距（16mm 广角） |
| target_velocity | [0.8, 0.3, 0.0] | 目标速度 |
