# 简易人形机器人 MuJoCo 物理仿真

---

## 1. 概述与目标

本模块使用 MuJoCo 物理仿真引擎，通过内联 XML 构建一个简易人形机器人，
实现原地踏步 + 交替摆臂的节律运动仿真，并自动渲染输出仿真视频。
不依赖 Gazebo、不需要 URDF，适合作为 MuJoCo 基础建模与运动控制的入门示例。

### 核心任务目标
1. **简易人形建模**：用基础几何体（球 / 盒 / 圆柱）在 XML 中搭建人形机器人。
2. **节律运动控制**：位置伺服执行器驱动双腿交替踏步、双臂交替摆动（相位协调）。
3. **仿真视频自动渲染**：离屏渲染（Offscreen Rendering）自动生成 MP4 视频。

---

## 2. 模型与运动设计

### 2.1 模型结构（内联 XML，`main.py` 内 `XML`）
| 部件 | 几何体 | 说明 |
|------|--------|------|
| 躯干 | 蓝色 box（0.2×0.15×0.28） | 带 free root 关节，可整体平动/转动 |
| 头部 | 橙色 sphere（r=0.15） | 固定于躯干顶部 |
| 双臂 | 绿色 cylinder（长 0.35） | 左右各 1 个 hinge 关节 |
| 双腿 | 紫色 cylinder（长 0.45） | 左右各 1 个 hinge 关节 |

### 2.2 控制方式
| 项目 | 说明 |
|------|------|
| 执行器 | 4 个位置伺服（position actuator，kp=8） |
| 控制律 | `ctrl = 幅度 × sin(2π·freq·t + 相位)` |
| 频率 | 0.5 Hz（2 秒一个完整周期） |

### 2.3 相位安排
| 关节 | 幅度（rad） | 相位 |
|------|------------|------|
| 左腿 `j_lleg` | 0.42 | 0 |
| 右腿 `j_rleg` | 0.42 | π（与左腿反相，交替踏步） |
| 左臂 `j_larm` | 0.55 | π/2 |
| 右臂 `j_rarm` | 0.55 | 3π/2（与左臂反相，且与腿部错相） |

---

## 3. 运行方式

### 3.1 环境依赖
- Ubuntu 20.04 / 22.04 / 24.04（Windows 亦可）
- Python >= 3.10
- MuJoCo 3.x、numpy、imageio + imageio-ffmpeg

### 3.2 安装依赖
```bash
cd src/humanoid/midterm_humanoid
pip install -r requirements.txt
```

### 3.3 运行仿真

方式一（直接运行，不依赖 ROS）：
```bash
cd src/humanoid/midterm_humanoid
python3 main.py
```
运行成功后，仿真视频自动保存至 `output/robot_sim.mp4`；无显示器环境请加 `MUJOCO_GL=osmesa` 前缀。

方式二（catkin 包 + roslaunch，需 ROS 环境）：
```bash
# 将本模块软链到工作空间 src 下，然后
catkin_make
source devel/setup.bash
roslaunch midterm_humanoid main.launch
```

---

## 4. 运行效果

![简易人形机器人动图](https://raw.githubusercontent.com/nanxi666521/ros2/mujoco2/src/humanoid/midterm_humanoid/output/robot_sim.gif)

完整视频见 `src/humanoid/midterm_humanoid/output/robot_sim.mp4`（8 秒，30 fps，960×544）。

---

## 5. 文件结构

```
midterm_humanoid/
├── main.py            # 仿真与渲染主脚本（入口）
├── common.py          # 公共库（渲染后端 / 仿真循环 / 视频导出）
├── requirements.txt
├── package.xml        # catkin 包描述
├── CMakeLists.txt
├── launch/main.launch # roslaunch 启动入口
├── README.md          # 运行环境与步骤说明
└── output/
    ├── robot_sim.mp4   # 生成的仿真视频
    ├── robot_sim.gif   # 运行效果动图
    └── preview_000.png
```
