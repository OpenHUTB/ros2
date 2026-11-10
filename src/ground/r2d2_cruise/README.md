# r2d2_cruise

R2D2 四轮小车的**固定轮速差速圆周运动**示例：左轮速度高、右轮速度低，左右轮保持固定速度差，小车沿圆弧持续巡航。

本模块是**开环控制**，只演示「固定轮速差速」，不含轨迹反馈、路径跟踪与避障，不能据此说明能够稳定保持给定巡航轨迹。

## 与仓库已有模块的区别

| 模块 | 平台 | 控制方式 | 是否有反馈 |
|---|---|---|---|
| `ground/rostopic_cruise` | ROS + 话题 | 订阅速度指令 | 有（外部指令） |
| `ground/ros2_pure_pursuit` | ROS 2 | 纯追踪路径跟踪 | 有（路径跟踪） |
| **本模块** | PyBullet 独立仿真 | 左右轮固定速度差 | **无（开环）** |

## 目录结构

```
r2d2_cruise/
├── main.py           入口：建场景、加载 r2d2.urdf、差速驱动、轨迹记录
├── launch/main.launch  roslaunch 统一入口（只调用 main.py，不依赖 ROS 运行时）
├── CMakeLists.txt     catkin 安装规则
├── package.xml        ROS 包描述
└── requirements.txt   Python 依赖（pybullet）
```

## 运行环境

- Python 3.8+
- pybullet 3.2.5+
- 有图形界面时才能看到仿真窗口；无图形环境用 `--direct` 自检

PyPI 上的 pybullet **只提供 Linux 轮子**，Windows 直接 `pip install` 会尝试本地编译（需要 MSVC）。Windows 建议走 conda-forge：

```bash
conda create -n r2d2 -c conda-forge python=3.8 pybullet
conda activate r2d2
```

Ubuntu 20.04 下直接安装即可：

```bash
pip install pybullet
```

## 运行步骤（从仓库根目录执行）

```bash
# 1. 安装依赖
pip install -r src/ground/r2d2_cruise/requirements.txt

# 2. 可视化运行（弹出 PyBullet 窗口，一直巡航到关窗或 Ctrl+C）
python3 src/ground/r2d2_cruise/main.py

# 3. 无窗口自检：跑 5 秒并把轨迹写成 CSV
python3 src/ground/r2d2_cruise/main.py --direct --duration 5 --log run.csv

# 4. 通过 ROS launch 启动（需 ROS 环境）
catkin_make && roslaunch r2d2_cruise main.launch
```

参数说明：

| 参数 | 说明 |
|---|---|
| `--direct` | 不弹窗，用 DIRECT 模式跑，适合服务器/CI |
| `--duration N` | 仿真时长（秒），默认 0 表示一直跑 |
| `--log PATH` | 按 10 Hz 记录 `t,x,y,z,yaw,left_vel,right_vel` |
| `--width/--height` | 可视化窗口尺寸 |

## 运行结果

固定轮速 `left=10 rad/s`、`right=2 rad/s` 时，实测稳态（15–20 s 采样窗口，折线弧长法）绕半径约 **1.02 m** 的圆弧巡航，线速度约 0.26 m/s、角速度约 0.26 rad/s；起步 5 s 内仍在加速，同一算法约 0.80 m。无滑移运动学估算 `R = (L/2)·(ωL+ωR)/(ωL−ωR)` 会明显偏小，差异来自轮胎滑移与关节力矩上限（`force=20`）。

## 自检

```bash
python3 -m pytest tests/test_r2d2_cruise_local.py -v
```

三个用例分别检查：轨迹 CSV 能生成、5 秒内确实走出圆弧（位移/转角/半径）、车体高度稳定不弹飞。

## 说明

本作业的代码与文档在编写过程中使用了 AI 大模型辅助，本人已核对并确认提交内容，对提交内容负责。
