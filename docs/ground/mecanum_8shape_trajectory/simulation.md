# 仿真配置说明

## URDF模型

- 底盘：0.5m × 0.4m × 0.1m 白色长方体
- 四轮：半径0.08m，宽度0.04m
- 插件：gazebo_ros_control + velocity_controllers/JointGroupVelocityController

## 8字轨迹参数

- 轨迹类型：李萨如曲线 x=A·sin(ωt), y=(A/2)·sin(2ωt)
- 振幅 A=0.3m
- 角频率 ω=0.15rad/s
- 周期约40s

## PD控制器参数

| 方向 | Kp | Kd | 输出限幅 |
|---|---|---|---|
| x | 0.8 | 0.05 | ±0.05 m/s |
| y | 0.8 | 0.05 | ±0.05 m/s |
| θ | 1.5 | 0.1 | ±0.3 rad/s |

## 调参建议

1. 先调Kp：从小到大，响应快但不超调
2. 再加Kd：抑制振荡
3. 速度太慢就加大限幅，太快就减小
