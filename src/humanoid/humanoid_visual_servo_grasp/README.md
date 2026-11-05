# humanoid_visual_servo_grasp — 人形机器人视觉伺服抓取

本项目在 MuJoCo 中仿真一台带固定底座、单臂（肩 roll + 肩 pitch + 肘 pitch，3 自由度）和两指夹具的简化人形，通过前视相机感知桌面上的红色目标物，利用平面 IK 反解关节角，以视觉伺服方式驱动手臂伸向目标并在到达后合拢夹具完成抓取。

## 特点

- **视觉伺服**：每个控制周期读取目标物世界坐标，计算末端与目标的误差，IK 反解肩/肘关节角并渐进逼近，末端误差收敛到容差内即判定到达。
- **真实仿真**：MuJoCo 物理引擎驱动，重力场 + 固定底座，目标物静态放置于桌面。
- **双入口**：既可作为独立脚本（`main.py`）运行演示，也可作为 ROS2 节点（`mujoco_sim_node`）周期发布抓取状态。
- **自带测试与动图**：`--test` 自动化回归测试，`--gif`/`--shots` 生成真实演示动图与关键帧截图。

## 依赖

- Ubuntu 22.04 / ROS 2 Humble
- `mujoco>=3.0`、`numpy`、`Pillow`（见 `requirements.txt`）

```bash
sudo apt update
sudo apt install ros-humble-geometry-msgs ros-humble-std-msgs
pip install mujoco numpy Pillow
```

## 独立运行

```bash
# 进入包目录
cd src/humanoid/humanoid_visual_servo_grasp

# 可视化窗口演示抓取
PYTHONPATH=$PWD python3 main.py

# 无窗口，仅命令行输出
PYTHONPATH=$PWD python3 main.py --headless

# 生成演示动图 doc/humanoid_visual_servo_grasp.gif
PYTHONPATH=$PWD python3 main.py --gif

# 渲染关键帧截图（初始 / 伸向中 / 抓取完成）
PYTHONPATH=$PWD python3 main.py --shots

# 自动化回归测试
PYTHONPATH=$PWD python3 main.py --test
# 或
python3 test/test_standalone.py --test
```

## ROS2 运行

```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_visual_servo_grasp
source install/setup.bash

# 启动仿真节点
ros2 launch humanoid_visual_servo_grasp main.launch.py
# 打开可视化窗口
ros2 launch humanoid_visual_servo_grasp main.launch.py visualize:=true

# 查看抓取状态与位置
ros2 topic echo /humanoid/grasp/status   # [末端误差, 是否到达, 是否抓取完成]
ros2 topic echo /humanoid/grasp/target   # 目标物位置
ros2 topic echo /humanoid/grasp/tip      # 末端位置
```

## 话题

| 话题 | 类型 | 内容 |
|------|------|------|
| `/humanoid/grasp/status` | `std_msgs/Float64MultiArray` | `[末端误差(m), 到达标志, 抓取完成标志]` |
| `/humanoid/grasp/target` | `geometry_msgs/PoseStamped` | 目标物世界坐标 |
| `/humanoid/grasp/tip` | `geometry_msgs/PoseStamped` | 末端执行器世界坐标 |

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `control_hz` | `50.0` | 控制频率 (Hz) |
| `visualize` | `false` | 是否打开 MuJoCo 可视化窗口 |

## 工作原理

1. 读取目标物世界坐标 `target`，减去肩基座 `SHOULDER_BASE` 得到局部目标。
2. 平面两杆 IK：由臂长 `L1=0.35`、`L2=0.33` 与目标距离 `dist` 反解肘角 `elbow` 与肩俯仰角 `shoulder_pitch`。
3. 用低增益 `K_IK=0.15` 的关节速度渐进逼近目标，避免末端突变。
4. 末端误差进入 `REACH_TOL=0.06 m` 判定到达，随即合拢两指夹具完成抓取。

## 目录结构

```
humanoid_visual_servo_grasp/
├── main.py                            # 独立入口（演示 / 动图 / 截图 / 测试）
├── humanoid_visual_servo_grasp/
│   ├── sim_core.py                    # 核心：模型 + 视觉伺服 + IK + 夹具
│   └── mujoco_sim_node.py             # ROS2 仿真节点
├── launch/main.launch.py              # Launch 文件
├── config/params.yaml                 # 参数配置
├── models/humanoid_visual_servo_grasp.xml  # MuJoCo 模型
├── resource/humanoid_visual_servo_grasp
├── test/test_standalone.py            # 独立回归测试
├── package.xml / setup.py / setup.cfg
└── requirements.txt
```
