# 新增无人机飞行状态识别与轨迹绘制 ROS 2 封装模块
本模块基于 ROS 2 实现了一套无人机飞行状态识别系统。系统通过发布节点模拟无人机速度指令（Twist），接收节点实时订阅数据，自动识别当前的飞行状态（直线、旋转、悬停），并基于固定步长模拟实时绘制飞行轨迹图。

## 新增功能

* **状态细分**：在“直线 / 旋转 / 悬停”的基础上，细分为 9 种飞行状态
  （直线前进、直线后退、原地左转、原地右转、前进左转、前进右转、后退左转、后退右转、悬停）。
* **状态广播**：识别结果同时发布到 `/drone/flight_state`（`std_msgs/String`），方便后续节点直接消费。
* **记录导出**：退出时自动导出逐点轨迹 `drone_trajectory.csv` 与状态统计 `drone_trajectory_summary.json`
  （采样数、时长、折线距离、包围盒、各状态次数/时长/占比）。
* **参数化**：步长、转角、死区、发布频率、指令序列、导出目录等均可通过 ROS 参数或 launch 参数配置，默认值与旧版本一致。
* **可测试**：状态识别 / 位姿积分 / 记录导出拆到纯逻辑模块 `flight_state.py`，不依赖 ROS 即可运行单元测试。

## 运行环境
* 操作系统：Ubuntu 22.04（虚拟机）
* ROS 2 版本：Humble
* 依赖库：rclpy, geometry_msgs, std_msgs, matplotlib
## 核心代码文件清单

本模块包含以下核心文件，均已提交至仓库：

```text
src/drone_trajectory_logger/
├── drone_trajectory_logger/
│   ├── __init__.py
│   ├── flight_state.py      # 状态识别 / 位姿积分 / 记录导出（纯逻辑，无 ROS 依赖）
│   ├── main.py              # 发布节点：按指令序列发送 Twist
│   └── trajectory_analyzer.py  # 识别节点：订阅 Twist、识别状态、绘图与导出
├── resource/
│   └── drone_trajectory_logger
├── launch/
│   └── main.launch.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 快速开始
```bash
colcon build --packages-select drone_trajectory_logger
source install/setup.bash
ros2 launch drone_trajectory_logger main.launch.py

# 无显示器 / 服务器环境：关掉实时绘图窗口
ros2 launch drone_trajectory_logger main.launch.py enable_plot:=false
```

完整参数表见 [主文档](../../docs/air/drone_trajectory_logger.md#参数说明)；
`publish_rate` 与记录时长无关——识别节点按**节点时钟**给采样点打时间戳。

自定义指令序列与导出位置：

```bash
ros2 launch drone_trajectory_logger main.launch.py \
  sequence:=forward,forward,turn_left,hover \
  output_dir:=/tmp/drone_logs \
  export_every:=10
```

退出（`Ctrl+C`）后当前目录或 `output_dir` 下会生成：

* `drone_trajectory.csv`：`index,time_s,x,y,theta_deg,state,linear_x,angular_z`
* `drone_trajectory_summary.json`：采样数、时长、距离、包围盒、各状态统计

## 本地测试（不需要 ROS）

```bash
python3 tests/test_drone_trajectory_logger_local.py
```

> 📖 **详细原理与实验步骤请参阅：** [点击查看主文档](../../docs/air/drone_trajectory_logger.md)
