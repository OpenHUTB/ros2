# tb3_lidar_obstacle：ROS 2 TurtleBot3 激光雷达避障实验包

本包是机器人工程 ROS 2 课程的**独立课程作业提交**（作业二：激光雷达避障实验报告）。它与仓库已有的 `src/ground/turtlebot3_lidar_avoid/` 是两份不同作业、不同作者维护的独立示例，不复刻、不替代后者：已有模块是前/左/右扇区 + 三态状态机 + 阈值滞回 + 紧急后退的完整方案，本包保留课程要求的「最小可运行闭环」，并在此基础上补齐评审要求的前方扇区、量程过滤、左右择优与 `/scan` 断流看门狗。**新项目请优先复用 `turtlebot3_lidar_avoid`；本目录只作为该次课程作业的归档与复现材料，不另起一条维护链路。**

实验报告：[docs/ground/ros2_turtlebot3_obstacle.md](../../../docs/ground/ros2_turtlebot3_obstacle.md)

## 运行环境（以实际复现环境为准）

| 项 | 值 |
| --- | --- |
| 操作系统 | Ubuntu 22.04.5 |
| ROS 2 | **Humble Hawksbill** |
| 仿真 | Gazebo 11 + `turtlebot3_gazebo` |
| 模型 | TurtleBot3 `burger`（启动前必须导出） |
| 可视化 | RViz2，`/scan` 的 QoS 选 Best Effort |
| 构建 | `colcon build --packages-select tb3_lidar_obstacle` |

ROS 2 Humble 对应 Ubuntu 22.04；Galactic 对应 Ubuntu 20.04。早先文档里「Ubuntu 22.04 + Galactic」与 PR 描述里「Ubuntu 20.04 + Humble」都是笔误，现统一为本表。评审复现环境同为 Ubuntu 22.04.5 / Humble / Gazebo 11。

## 目录结构

```
src/ground/tb3_lidar_obstacle/
├── tb3_lidar_obstacle/
│   ├── avoid_logic.py      # 纯标准库：扇区取最小值、量程过滤、左右择优
│   └── obstacle_avoid.py   # rclpy 节点 + 0.5 s /scan 断流看门狗
├── launch/tb3_lidar_obstacle.launch.py
├── tests/test_avoid_logic.py   # 不依赖 Gazebo / rclpy 的控制逻辑测试
├── package.xml / setup.py / setup.cfg / resource/
└── README.md
```

`setup.py` 只注册一个入口：

```python
'obstacle_avoid = tb3_lidar_obstacle.obstacle_avoid:main',
```

## 启动步骤

```bash
export TURTLEBOT3_MODEL=burger          # 不设置这一步 Gazebo 起不来
cd <ros2_ws>
colcon build --packages-select tb3_lidar_obstacle
source install/setup.bash

# 终端 1：仿真环境
ros2 launch turtlebot3_gazebo empty_world.launch.py

# 终端 2：避障节点
ros2 run tb3_lidar_obstacle obstacle_avoid

# 终端 3（可选）：RViz2，把 LaserScan 的 Reliability 设为 Best Effort
rviz2
```

可调参数：`--ros-args -p safe_dist:=0.5 -p scan_timeout:=0.5`。

## 控制逻辑与安全性

1. **不再用 `ranges[0]` 当正前方**。按 `angle_min + i * angle_increment` 反算索引，取 `±15°` 前方扇区的最小有效距离；换 LDS 型号 / 改 URDF 雷达朝向也不会失准，细杆落在两条扫描线之间时同样能被扇区抓到。
2. **量程过滤**：`math.isfinite()` 一次覆盖 `inf` / `nan`，并按 `range_min < r < range_max` 裁剪；无效扇区返回 `inf`，按「前方无障碍」处理而不是伪造 10.0 m。
3. **左右择优**：前方小于 `safe_dist` 时再比较 `±30°`（各宽 15°）左右扇区的最近障碍，向更空的一侧转；两侧一样则默认左转。对应实验报告思考题里「比较两侧距离」的答案已落地。
4. **断流看门狗**：0.1 s 定时器检查最后一次 `/scan` 的时间戳，超过 `scan_timeout`（默认 0.5 s）立即发布零速度并只告警一次；节点销毁前也会补一条零速度，避免 Gazebo 卡住 / QoS 不匹配时机器人保持上一个 `cmd_vel` 一直直行或转圈。

## 离线测试

不需要 ROS、Gazebo 或 `rclpy`，Windows / Linux 上都能跑：

```bash
cd src/ground/tb3_lidar_obstacle
python -m unittest discover -s tests -v      # 10 项
python -m pytest tests -q                    # 同一套测试
```

覆盖：`angle_min != 0` 时正前方索引正确、`ranges[0]` 不再误判、`inf/nan/超量程` 过滤、空扇区返回 `inf`、直行 / 左转 / 右转 / 两侧相等、端到端 `scan_to_decision` 以及看门狗常量契约。

## 运行效果

实验报告第六节的两张动图 `docs/img/gazebo_demo.gif`、`docs/img/rviz_demo.gif` 已重新导出并压到 5 MB 以内（原 9.15 MB / 1.39 MB，且 PR head 上曾误提交成 0 字节空 blob）。

## 局限

固定阈值 + 原地转向策略只验证「直行 → 停车 → 向空侧转」的基础闭环，评审的 25 s 单障碍物测试（采样 251 点、路径 1.11 m / 终点位移 0.77 m、前方最小距离 0.165 m、最终朝向 −0.91 rad）也说明场景较简单；复杂墙角、动态障碍物和多目标连续绕行不做保证，这部分请用仓库已有的 `turtlebot3_lidar_avoid` 方案。
