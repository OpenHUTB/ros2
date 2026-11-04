# 高精度定位与重定位（AMCL 式粒子滤波）· TurtleBot3

> 地面载具方向③：机器人知道环境地图、但**不知道自己在哪**（初始位姿未知）时，
> 用激光雷达 + 粒子滤波做全局定位/重定位，并持续修正里程计累积漂移。
> 仿真平台：**Gazebo / TurtleBot3**（渲染版）＋ **2D 轻量版**（本仓库可直接跑、出效果动图）。

![重定位效果](img/ground/turtlebot3_relocalization_demo.gif)

!!! note "与仓库已有模块的区别"

    本模块**不做控制算法**，而是解决机器人定位中的"我在哪"问题：

    * `turtlebot3_lidar_avoid`、`turtlebot3_auto_nav` 假定机器人**已知自身位姿**去做避障/导航；
    * 本模块反向处理——**初始位姿未知**，仅靠地图 + 激光 + 里程计把位置找出来，
      并持续抑制里程计漂移，为后续导航提供可靠 /amcl_pose。

## 1. 解决什么问题

里程计（航位推算）会随时间累积误差——轮子打滑、航向漂移，走得越远越不知道自己在哪。
重定位任务假设手里有**环境地图**但没有初始位姿。本模块用 **AMCL 风格粒子滤波**：

1. 在地图上**均匀撒 N 个粒子**（每个粒子 = 候选位姿 (x, y, θ)），代表"可能在这里"；
2. 每步按里程计增量推进所有粒子（运动模型，含噪声）；
3. 用激光扫描与地图的**匹配程度**给粒子打分（likelihood field 权重）：观测应落在障碍边缘；
4. **低方差重采样**：高分粒子复制、低分粒子淘汰；
5. 粒子逐渐聚拢到真实位姿 —— 重定位完成，输出高精度位姿。

对比基准：同环境跑一条**纯里程计航位推算**轨迹，展示没有地图修正时的累积漂移。

## 2. 文件结构

| 文件 | 说明 |
|------|------|
| `reloc_core.py` | **共享核心**：栅格地图+距离场、激光 raycast、likelihood field 权重、低方差重采样粒子滤波（只依赖 numpy） |
| `sim_2d_relocalization.py` | 2D 轻量重定位仿真，可直接运行并生成效果动图（真实跑出，算法核心与渲染版共用） |
| `turtlebot3_amcl.py` | **Gazebo/TurtleBot3 渲染版**：ROS2 节点，订阅 /map /odom /scan，输出 /amcl_pose 与粒子云 |
| `README.md` / `requirements.txt` | 说明与依赖 |

## 3. 运行

### 3.1 2D 轻量版（可直接运行、出动图）

```
pip install -r requirements.txt
python3 sim_2d_relocalization.py                     # 默认 240 帧 @30fps
python3 sim_2d_relocalization.py --frames 300 --fps 20
```

运行后 stdout 第一行为 GIF 绝对路径，随后打印末帧验收指标
`state` / `err_amcl` / `err_odom` / `particle_spread` / `neff`，便于复现者与 CI 核对。

### 3.2 Gazebo/TurtleBot3 渲染版（需本机 ROS2 + Gazebo）

```bash
# 1) 启动 TurtleBot3 世界 + 地图服务器（发布 /map）
# 2) 让机器人移动（发布 /scan /odom）
python3 turtlebot3_amcl.py
# 3) RViz 查看 /amcl_pose（PoseWithCovariance）与 /particle_cloud（PoseArray）收敛
```

!!! note "沙箱限制声明"

    本仓库沙箱无 GPU / CARLA / ROS / Gazebo，**渲染版无法在沙箱实跑**。
    上文动图由**共用同一算法核心的 2D 轻量版真实运行**产出；Gazebo 渲染效果请在本机验证。

## 4. 结果

12m×12m 带内墙/孤岛环境中，蛇形真值轨迹 + 全图撒粒子重定位，粒子滤波估计与纯里程计对比：

| 指标 | 粒子滤波（AMCL 式） | 纯里程计航位推算 |
|------|--------------------|------------------|
| 稳态定位误差 | **0.20 ~ 0.33 m** | **6.9 ~ 9.1 m**（持续漂移） |
| 定位提升 | 约 **97 ~ 98%** | — |
| 收敛行为 | 前 ~2 s 内从全图分散聚拢（spread 5 m → 0.1 m） | 无地图修正，误差单调累积 |

动图展示：初始粒子全图分散（state=searching）→ 随移动与扫描收敛（relocalized）→
粒子滤波估计贴合真值，而纯里程计轨迹持续漂移远离真值。

## 5. 合规说明

本项目为课程/学习用途的仿真代码，不含真实道路行驶与真车控制，遵循开源许可。
