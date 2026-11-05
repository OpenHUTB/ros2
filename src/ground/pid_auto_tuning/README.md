# pid_auto_tuning 航向 PID 参数自整定

用**继电器反馈法**（Åström–Hägglund）自动求临界增益 `Ku` 与临界振荡周期 `Tu`，
再由经验规则生成 PID 参数，并在闭环指标上筛选最优，最后与一组手工试凑参数对比。

## 1. 运行环境

- 操作系统：Ubuntu 20.04（实测）
- ROS 版本：ROS 2 Humble
- Python：3.10
- 依赖：`rclpy`、`std_msgs`、`visualization_msgs`（ROS 2 Desktop 自带），绘图需要 `matplotlib`

## 2. 编译

```bash
cd ~/hutb_ws
colcon build --packages-select pid_auto_tuning
source install/setup.bash
```

## 3. 运行

### 3.1 一键跑完整流程（终端打印指标，不需要 RViz）

```bash
ros2 run pid_auto_tuning main
```

实测输出（目标航向 90°）：

```
继电器反馈实验: Ku=9.705, Tu=1.360s, 振荡幅值=0.066rad

方案                 超调      上升      调节        稳态误差     IAE
----------------------------------------------------------
手工试凑            62.7%   1.35s   8.00s   0.374rad    5.11
自动整定             6.8%   0.70s   2.15s   0.006rad    1.26

自动整定参数: Kp=6.308, Ki=1.784, Kd=1.3726
手工试凑参数: Kp=0.500, Ki=0.400, Kd=0.0000
```

同时把两条阶跃曲线写入 `resource/step_response.csv`（列：`t_s`、`manual_theta_rad`、`auto_theta_rad`）。

### 3.2 绘制对比图

```bash
ros2 run pid_auto_tuning main_plot
```

弹窗左侧为航向角阶跃响应，右侧为航向误差收敛过程。

### 3.3 可选：RViz 可视化

环境里有可用的 `rviz2` 时：

```bash
ros2 launch pid_auto_tuning main_launch.py
rviz2 -d src/ground/pid_auto_tuning/resource/pid_tuning.rviz
```

RViz 设置：`Fixed Frame` 为 `map`，`Add` → `By topic` → `/pid_auto_tuning/markers` → `MarkerArray`。
绿色箭头是当前航向，红色箭头是目标航向，目标每 6 s 在 ±90° 之间切换。

其他检查命令：

```bash
ros2 topic list | grep pid_auto_tuning
ros2 topic echo /pid_auto_tuning/heading --once
```

按 `Ctrl+C` 退出节点。

## 4. 参数说明

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `target_deg` | 90.0 | 目标航向角（度） |
| `dt` | 0.02 | 仿真步长（秒） |
| `switch_period` | 6.0 | 目标航向切换周期（秒） |
| `u_limit` | 2.5 | 角速度指令饱和上限（rad/s） |

修改方式：

```bash
ros2 run pid_auto_tuning main_node --ros-args -p target_deg:=60.0 -p switch_period:=4.0
```

## 5. 指标定义

| 指标 | 定义 | 单位 |
| --- | --- | --- |
| 超调量 | `(峰值 - 目标值) / 目标值 × 100%` | % |
| 上升时间 | 响应从 10% 到 90% 目标值所用时间 | s |
| 调节时间 | 最后一次超出 ±2% 误差带之后的时间 | s |
| 稳态误差 | 末段 5% 样本均值与目标值之差 | rad |
| IAE | 绝对误差积分 `Σ|e|·dt` | rad·s |

## 6. 简化假设

- 被控对象是**一阶惯性 + 纯延迟**的航向模型，不含轮胎侧偏、道路坡度和侧风
- 只在仿真与离线计算中验证，没有上真车
- 控制量按角速度指令饱和处理，未建模电机死区

## 7. 大模型声明

本模块由作者编写并实际运行验证；写作过程中使用 AI 大模型辅助检查错误与润色文档表述。作者对提交内容的正确性与完整性负全部责任。
