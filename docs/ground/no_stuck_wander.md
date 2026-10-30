# 防卡死漫游（no_stuck_wander）

## 简介

`no_stuck_wander` 是一个基于激光雷达（`/scan`）的**防卡死漫游**模块，
用于地面移动机器人（如 TurtleBot3）。机器人正常情况下向前随机漫游，
根据正前 / 左前 / 右前三路测距结果自动决策，避免撞墙并防止在墙角等
凹形环境中卡死。

* 源码：`src/ground/no_stuck_wander/`（入口 `main.py`）
* 语言 / 框架：Python3 + ROS 1（`rospy`）
* 话题：订阅 `/scan`（`sensor_msgs/LaserScan`），发布 `/cmd_vel`（`geometry_msgs/Twist`）

## 工作方式（状态机）

把激光扫描前半扇区平均分为 左前（`fl`）、正中（`fm`）、右前（`fr`）三段，
取每段最近有效距离，按最近距离 `minf` 决策：

| 状态 | 触发条件 | 动作 |
| --- | --- | --- |
| GO | `minf > 0.78` | 直行 + 小幅随机转角 |
| AVOID（绕障） | `0.40 < minf < 0.65` | 减速慢行，向空旷侧转向 |
| ESCAPE_BACK（直退） | `minf < 0.40` | 向后直退约 1.4s |
| ESCAPE_TURN（原地转） | 直退结束 | 原地转向约 1.6s，朝空旷侧 |
| AFTER_ESCAPE（脱困前进） | 原地转结束 | 强制前进约 1.0s，脱离墙角引力区 |

脱困链路（直退 → 原地转 → 强制前进）完成后回到 `GO`，可有效避免机器人
在墙角来回蹭、原地空转等卡死现象。

## 文件结构

```
src/ground/no_stuck_wander/
├── main.py                      # 模块入口
├── package.xml                  # ROS1 catkin 包描述
├── CMakeLists.txt               # 构建 / 安装配置
├── launch/
│   └── no_stuck_wander.launch   # launch 启动文件
└── README.md                    # 运行环境与运行步骤
```

## 运行环境与步骤

运行环境与运行步骤详见模块内 [`README.md`](https://github.com/OpenHUTB/ros2/blob/master/src/ground/no_stuck_wander/README.md)。

快速启动：

```bash
# 1) 编译（若尚未编译本包）
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 2) 启动仿真 / 底盘（提供 /scan 与 /cmd_vel），以 TurtleBot3 为例
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# 3) 启动本模块
roslaunch no_stuck_wander no_stuck_wander.launch
```

## 运行效果

机器人在仿真中漫游、绕障与脱困的运行效果（Webots 仿真录制）：

![no_stuck_wander 运行效果](../img/ground/no_stuck_wander.gif)

## 使用的大模型声明

本模块的 `main.py` 为作业原始代码；`package.xml`、`CMakeLists.txt`、`launch`、
`README.md`、本文档由大模型辅助整理生成，作者已逐项核对并对提交内容负全部责任。
