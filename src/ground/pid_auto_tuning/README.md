# pid_auto_tuning 航向 PID 参数自整定

用**继电器反馈法**（Åström–Hägglund）自动求临界增益 `Ku` 与临界振荡周期 `Tu`，
再由经验规则生成 PID 参数，并在闭环指标上筛选最优，最后与三组不同水平的手工试凑参数对比。

## 1. 运行环境

- 操作系统：Ubuntu 20.04（实测）
- ROS 版本：ROS 2 Humble
- Python：3.8（Ubuntu 20.04 自带版本）
- 依赖：`rclpy`、`std_msgs`、`visualization_msgs`（ROS 2 Desktop 自带）；`main_plot` 另需 `python3-matplotlib`

## 2. 编译

```bash
cd ~/ros2                      # 仓库克隆目录，按实际路径调整
source /opt/ros/humble/setup.bash
colcon build --packages-select pid_auto_tuning
source install/setup.bash
```

## 3. 运行

### 3.1 一键跑完整流程（终端打印指标，不需要 RViz）

```bash
ros2 run pid_auto_tuning main
```

实测输出（目标航向 90°；最后一行为 CSV 的绝对路径，随执行目录不同而变化）：

```
被控对象: 一阶惯性 + 纯延迟 (gain=1.0, tau=0.40s, delay=0.10s)
目标航向: 90.0 deg

继电器反馈实验: Ku=9.705, Tu=1.360s, 振荡幅值=0.066rad
参数筛选: 来源=grid, 比例=(kp:0.95, ti:3.2, td:0.16)（落在网格边界，建议扩大搜索范围复核）

方案                 超调      上升       调节        稳态误差     IAE
----------------------------------------------------------
保守试凑            62.7%   1.35s    未收敛        —    5.11
常规试凑            47.2%   0.73s   6.91s   0.005rad    2.41
积极试凑            29.4%   0.68s   5.80s   0.009rad    1.75
自动整定             3.6%   0.69s   1.61s   0.006rad    1.20

自动整定参数: Kp=9.219, Ki=2.118, Kd=2.0061
保守试凑参数: Kp=0.500, Ki=0.400, Kd=0.0000
常规试凑参数: Kp=1.500, Ki=0.800, Kd=0.0000
积极试凑参数: Kp=3.000, Ki=1.000, Kd=0.0000

阶跃响应数据已写入 /home/user/ros2/resource/step_response.csv
```

同时把四条阶跃曲线写入当前目录下的 `resource/step_response.csv`（列：`t_s`、三组手工参数与自动整定的 `*_theta_rad`）。

「未收敛」表示该组参数在 8 s 仿真内没有进入 ±2% 误差带，此时不输出稳态误差。

### 3.2 绘制对比图

```bash
ros2 run pid_auto_tuning main_plot
```

弹窗左侧为航向角阶跃响应，右侧为航向误差收敛过程。

### 3.3 可选：RViz 可视化

环境里有可用的 `rviz2` 时：

```bash
cd ~/ros2                      # rviz 配置使用相对路径时需在仓库根目录执行
ros2 launch pid_auto_tuning main.launch.py
rviz2 -d src/ground/pid_auto_tuning/resource/pid_tuning.rviz
# 或直接使用安装后的配置：
# rviz2 -d "$(ros2 pkg prefix pid_auto_tuning)/share/pid_auto_tuning/rviz/pid_tuning.rviz"
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

注：第 3.1 节的指标表由 `main` 以 `dt = 0.01` 计算；上面 `dt` 是 RViz 节点的默认步长，同一组参数下节点侧的指标会与此表略有差异。

修改方式：

```bash
ros2 run pid_auto_tuning main_node --ros-args -p target_deg:=60.0 -p switch_period:=4.0
```

## 5. 指标定义

| 指标 | 定义 | 单位 |
| --- | --- | --- |
| 超调量 | `(峰值 - 目标值) / 目标值 × 100%` | % |
| 上升时间 | 响应从 10% 到 90% 目标值所用时间 | s |
| 调节时间 | 最后一次超出 ±2% 误差带之后的时间；仿真结束仍未进入误差带记为「未收敛」 | s |
| 稳态误差 | 末段 5% 样本均值与目标值之差；未收敛时不输出 | rad |
| IAE | 绝对误差积分 `Σ|e|·dt` | rad·s |

## 6. 简化假设

- 被控对象是**一阶惯性 + 纯延迟**的航向模型，不含轮胎侧偏、道路坡度和侧风
- 只在仿真与离线计算中验证，没有上真车
- 控制量按角速度指令饱和处理，未建模电机死区
- 未做对象参数摄动（K、T、τ ±20%）的鲁棒性验证，「换对象后重新整定」的效果未在摄动对象上复核
- 网格搜索的最优参数当前落在搜索边界（kp = 0.95Ku、Ti = 3.2Tu），报告值为网格内最优，换对象后应扩大网格复核
- `main_plot` 弹窗依赖 python3-tk（matplotlib 默认 Tk 后端）；系统无中文字体（如 Noto Sans CJK）时图中中文显示为方块
