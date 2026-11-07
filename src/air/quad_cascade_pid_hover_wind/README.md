# 基于串级 PID 的四旋翼航点定点悬停与抗风扰仿真

文档页：`docs/air/quad_cascade_pid_hover_wind.md`。

## 1. 项目简介

本模块实现一个**自包含的四旋翼 6 自由度刚体动力学仿真器**与**串级 PID 控制器**，完成航点飞行、定点悬停与风扰抑制实验。

控制回路（仅使用 `/odom` 反馈）：

```
位置环 PID  ──►  期望加速度  ──►  期望推力 + 期望滚/俯仰
   (世界系)                                │
                                           ▼
姿态环 PID  ──►  期望角速率  ──►  力矩     偏航环
                                            │
                                            ▼
角速率环 PID  ──►  期望力矩  ──►  混控  ──►  4 个旋翼转速
```

仿真器包含：

- 6-DOF 刚体动力学（四元数积分、机体力矩、陀螺耦合项）
- 4 个旋翼推力/反扭矩模型与电机一阶响应
- 风场模型：恒定风 + 半正弦阵风 + 一阶湍流
- 气动阻力（相对速度产生的扰动力）

## 2. 运行环境

- Ubuntu 22.04、ROS 2 Humble、Python 3.8+
- 依赖：`numpy`、`pyyaml`、`matplotlib`（`pip install numpy pyyaml matplotlib`）
- 不依赖 Gazebo/MuJoCo/PyBullet，仿真内核为纯 Python 刚体动力学

## 3. 构建与运行

```bash
cd <workspace>
colcon build --packages-select quad_cascade_pid_hover_wind --symlink-install
source install/setup.bash
```

一键实验（自动构建、启动、记录、生成图表）：

```bash
cd src/air/quad_cascade_pid_hover_wind
python3 main.py --duration 40.0
```

可选参数：`--output <目录>` 指定结果输出路径、`--build` 强制重新构建。

单独启动：

```bash
ros2 launch quad_cascade_pid_hover_wind main.launch.py duration:=40.0
```

## 4. 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/odom` | `nav_msgs/Odometry` | 状态反馈（位置、速度、姿态、角速度） |
| `/ground_truth` | `nav_msgs/Odometry` | 真值（同仿真器状态，用于离线评估） |
| `/cmd_motors` | `std_msgs/Float32MultiArray` | 4 个旋翼目标转速 |
| `/wind` | `geometry_msgs/Vector3Stamped` | 世界系风速 |
| `/target` | `geometry_msgs/Point` | 当前航点 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 手动指定目标（可覆盖航点序列） |

## 5. 算法

### 5.1 动力学

X 型四旋翼，旋翼布置：FR(顺)、FL(逆)、RL(顺)、RR(逆)。

- 推力：$T_i = k_f \omega_i^2$，反扭矩：$Q_i = k_\tau \omega_i^2 \cdot \text{spin}_i$
- 混合矩阵将 $(T, \tau_x, \tau_y, \tau_z)$ 映射到 4 个旋翼推力
- 电机一阶响应：$\dot{\omega} = (\omega_{cmd} - \omega) / \tau_m$
- 风扰通过相对速度产生的气动阻力作用于机体：$F_d = -\frac{1}{2}\rho C_d A |v_r| v_r$

### 5.2 串级 PID

- **位置环**（世界系）：误差积分 + 速度微分反馈 → 期望加速度
- **期望姿态**：由期望加速度与重力合成推力方向，提取期望滚转/俯仰
- **姿态环**：欧拉角误差 → 期望角速率
- **角速率环**：角速率误差 → 期望力矩
- **混控**：推力 + 力矩 → 旋翼推力 → 转速（限幅）

### 5.3 航点管理

按顺序遍历航点，到达（距离 < `acceptance_radius`）后悬停 `hover_time` 秒，再切换到下一航点。

## 6. 真实运行结果

`results/run_01`，单次 40 s 飞行，5 个航点（含返回起点悬停），18~24 s 施加阵风（恒定风 2 m/s，阵风峰值 6.8 m/s）。

| 指标 | 数值 |
| --- | --- |
| 轨迹长度 | 12.17 m |
| 航点切换最大误差 | 2.55 m（目标跳变瞬间，非失控） |
| 稳定悬停最大误差 | 0.30 m |
| 末段悬停 RMS 误差 | 0.067 m（XY 0.066 m / Z 0.012 m） |
| 阵风段悬停 RMS 误差 | 0.174 m |
| 阵风前悬停 RMS 误差 | 0.076 m |
| 最大风速 | 6.76 m/s |
| 最大滚转/俯仰 | 32.1° / 32.3° |
| 终点误差 | 0.060 m |

阵风期间悬停误差由 7.6 cm 升至 17.4 cm，阵风过后恢复至 6.7 cm，体现串级 PID 的风扰抑制能力。

## 7. 测试

```bash
cd src/air/quad_cascade_pid_hover_wind
PYTHONPATH=. python3 -m pytest test/ -q
```

13 项测试覆盖四元数/旋转、动力学悬停平衡、混控、风场、PID、航点切换与闭环收敛。

## 8. 大模型使用声明

使用 AI 编程助手辅助生成代码与文档；作者已逐行核对并实际运行验证，实验数据与图表均由真实仿真产生，未人工篡改。
