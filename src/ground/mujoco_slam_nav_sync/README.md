# mujoco_slam_nav_sync — MuJoCo 差速小车：建图与导航同步仿真

用 MuJoCo 做一台 12 m × 12 m 场地里的差速小车，**一边 EKF-SLAM 建图定位、一边用 A\* + 神经网络局部规划导航**，
并定量回答课程任务 (3) 关心的那个问题：**定位漂移到底会把导航拖垮到什么程度？**

对照方式很直接：同一个导航栈、同一条轨迹、同一个随机种子，只换定位来源。

| 模式 | 位姿来源 | 说明 |
| --- | --- | --- |
| `slam` | EKF-SLAM（里程计预测 + 柱子路标观测更新） | 位姿与路标联合估计 |
| `dead-reckoning` | 轮式里程计航迹推算 | 漂移不可观测，用来做对照 |

## 目录结构

```
mujoco_slam_nav_sync/
├── main.py                 # 统一入口：--demo / --sweep / --test / --ros
├── main.sh / main.bat      # 一键运行（等价 --demo）
├── models/
│   ├── arena.xml           # 场地：外围墙 + 3 道隔断 + 8 根路标柱 + 差速小车
│   └── planner.npz         # 训练好的局部规划网络权重（20 KB，随包发布）
├── slam_nav/
│   ├── env.py              # MuJoCo 环境：速度伺服、120 线雷达、带轮滑的里程计
│   ├── slam.py             # EKF-SLAM + 对数几率占据栅格 + 路标聚类检测
│   ├── planner.py          # A* 全局规划、多点前视、纯追踪（专家策略）
│   ├── net.py              # 纯 numpy MLP 局部规划器（前向/反向/Adam/存取）
│   ├── nav.py              # 闭环导航 + 安全护栏 + 专家数据采集
│   ├── train.py            # 训练、曲线、GIF、指标落盘
│   ├── sweep.py            # 漂移 × 模式 × 种子 的对照实验与出图
│   └── node.py             # ROS 2 节点：/scan /odom /slam/pose /map /nav/cmd
├── launch/main.launch.py
├── config/params.yaml
└── test/test_slam_nav.py   # 37 个单元测试
```

## 运行环境

| 项 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 20.04（Humble）或 Windows 10/11 |
| Python | 3.8 / 3.10 |
| 依赖 | numpy、mujoco ≥ 3.1、matplotlib、pillow、pytest |
| ROS（可选） | ROS 2 Humble（`rclpy`、`sensor_msgs`、`nav_msgs`） |

Windows 上 MuJoCo 离屏渲染需要 OpenGL；如果驱动不支持（例如 VMware 的软件 OpenGL），
在 **Linux** 上可以把 `MUJOCO_GL=osmesa` 打开；**Windows 没有 osmesa 后端**（只有 `wgl`/`glfw`），
设了会直接 `RuntimeError`，无显卡时请只跑 `--test`（不渲染）。

## 三步跑通

```bash
cd src/ground/mujoco_slam_nav_sync
pip install -r requirements.txt

# 1) 训练 + 三模式对照 + 漂移扫描 + 出图出 GIF（约 60 s，纯 CPU）
python main.py --demo

# 2) 只要对照实验
python main.py --sweep

# 3) 单元测试
python main.py --test
```

Windows 直接双击 `main.bat`，Linux / macOS 用 `./main.sh`。

产物都在 `results/`：`training_curve.png`、`sweep.png`、`map_slam.png`、`nav_slam.gif`、`metrics.json`。

## ROS 2（Humble）运行

```bash
# 工作空间里编译
colcon build --packages-select mujoco_slam_nav_sync
source install/setup.bash

# 默认：maze 场景 + EKF-SLAM
ros2 launch mujoco_slam_nav_sync main.launch.py

# 对照：关掉 SLAM，注入 20% 轮滑
ros2 launch mujoco_slam_nav_sync main.launch.py use_slam:=false slip:=0.20
```

话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | 120 线平面雷达，`frame_id=laser`，带 `header.stamp` |
| `/odom` | `nav_msgs/Odometry` | 轮式里程计，`frame_id=map`，`child_frame_id=base_link` |
| `/slam/pose` | `geometry_msgs/PoseStamped` | EKF-SLAM 估计位姿 |
| `/map` | `nav_msgs/OccupancyGrid` | 在线构建的占据栅格（-1 未知 / 0 空闲 / 100 占据） |
| `/nav/cmd` | `geometry_msgs/Twist` | 下发给底盘的速度指令 |
| `/nav/status` | `std_msgs/String` | `mode/step/dist/ate/cmd` 一行状态 |

## 实测结果（`python main.py --demo`，CPU）

单场景三种模式：

| 模式 | 到达 | 步数（20 Hz） | ATE 均值 | 终点误差 | 路径长度 |
| --- | --- | --- | --- | --- |
| 专家（A\* + 纯追踪）+ SLAM | 是 | 527 | 0.226 m | 0.45 m | 11.7 m |
| **神经网络 + SLAM** | 是 | 550 | 0.131 m | 0.45 m | 11.5 m |
| 神经网络 + 死推算（10% 轮滑） | 否 | 1200 | 0.967 m | 1.15 m | 11.4 m |

漂移扫描（4 档轮滑 × 2 模式 × 10 种子 = 80 次导航）：

| 配置 | 成功率 | 碰撞 | ATE 均值 |
| --- | --- | --- | --- |
| SLAM @ 0% | 100% | 0 | 0.114 m |
| SLAM @ 5% | 80% | 0 | 0.246 m |
| SLAM @ 10% | 80% | 0 | 0.236 m |
| SLAM @ 20% | 40% | 0 | 0.442 m |
| 死推算 @ 0% | 100% | 0 | 0.025 m |
| 死推算 @ 5% | 0% | 0 | 0.502 m |
| 死推算 @ 10% | 0% | 0 | 0.953 m |
| 死推算 @ 20% | 0% | 7 | 0.878 m |

结论：0% 轮滑时两者都满血（死推算 ATE 甚至更小，因为里程计≈真值）；一旦有 5% 轮滑，
死推算的 ATE（0.50 m）就超过 0.45 m 的到位容差，成功率直接归零，而 EKF-SLAM 仍能到 80%。
20% 轮滑时 SLAM 掉到 40%，死推算 10 次里 7 次撞墙。

## 设计要点

1. **物理与里程计分离**：车按指令真实运动，但 SLAM 只能读被轮滑污染的里程计，漂移来自传感器误差而不是把物理改坏。
2. **安全护栏**：神经网络只学"专家怎么走"，不保证不撞；外加一个只看雷达的 TTC 护栏（正前方 60° 内 < 0.45 m 停车转向）。
3. **确定性**：每个回合按 `(seed, slip)` 重建随机源，同一配置重复运行结果完全一致（有单元测试守着）。
4. 完整推导、算法流程、源码解析和更多实验见 `docs/ground/mujoco_slam_nav_sync.md`。
