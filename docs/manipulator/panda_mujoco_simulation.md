# Franka Panda 机械臂 MuJoCo 物理仿真

---

## 1. 概述与目标

本模块基于 MuJoCo 物理仿真引擎与官方 Menagerie 模型库中的 `franka_emika_panda` 七轴机械臂模型，
实现关节空间轨迹运动与夹爪开合的仿真示例，并自动渲染输出仿真视频。

### 核心任务目标
1. **关节轨迹运动**：在关节空间用余弦缓动在多个目标位姿间平滑过渡。
2. **夹爪开合**：通过腱执行器控制二指夹爪周期性开合。
3. **仿真视频自动渲染**：离屏渲染（Offscreen Rendering）自动生成 MP4 视频。

---

## 2. 模型与运动设计

### 2.1 模型来源
- 机器人：`franka_emika_panda`（Franka Emika Panda），来自官方 [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) 模型库。
- 场景：Menagerie 自带 `scene.xml`，首次运行自动下载缓存。

### 2.2 控制方式
| 项目 | 说明 |
|------|------|
| 臂关节 | 7 个位置伺服执行器（增益 2000~4500），直接写目标角 |
| 夹爪 | actuator8 腱执行器（范围 0~255），8 秒开合周期 |
| 轨迹 | home + 3 组增量位姿 × 4 段循环，段间余弦缓动 |

### 2.3 位姿参数（`main.py` 内 `deltas`）
| 段 | 动作 | 增量（弧度） |
|----|------|-------------|
| 0 | 归位（home） | 全 0 |
| 1 | 上抬 | [0, 0.7, 0.15, 1.2, 0, 0.8, 0] |
| 2 | 前伸侧摆 | [0.5, 0.4, 0.25, 0.6, -0.4, 0.3, 0.4] |
| 3 | 归位（home） | 全 0 |

---

## 3. 运行方式

### 3.1 环境依赖
- Ubuntu 20.04 / 22.04 / 24.04（Windows 亦可）
- Python >= 3.10
- MuJoCo 3.x、mujoco-menagerie、imageio + imageio-ffmpeg、numpy

### 3.2 安装依赖
```bash
cd src/manipulator/panda_mujoco
pip install -r requirements.txt
```

### 3.3 运行仿真

方式一（直接运行，不依赖 ROS）：
```bash
cd src/manipulator/panda_mujoco
python3 main.py
```
运行成功后，仿真视频自动保存至 `output/panda_motion.mp4`；无显示器环境请加 `MUJOCO_GL=osmesa` 前缀。

方式二（catkin 包 + roslaunch，需 ROS 环境）：
```bash
# 将本模块软链到工作空间 src 下，然后
catkin_make
source devel/setup.bash
roslaunch panda_mujoco main.launch
```

---

## 4. 运行效果

![Panda 机械臂动图](https://raw.githubusercontent.com/nanxi666521/ros2/mujoco_panda/src/manipulator/panda_mujoco/output/panda_motion.gif)

完整视频见 `src/manipulator/panda_mujoco/output/panda_motion.mp4`（8 秒，30 fps，960×544）。

---

## 5. 文件结构

```
panda_mujoco/
├── main.py            # 仿真与渲染主脚本（入口）
├── common.py          # 公共库（渲染后端 / 相机注入 / 视频导出）
├── requirements.txt
├── package.xml        # catkin 包描述
├── CMakeLists.txt
├── launch/main.launch # roslaunch 启动入口
├── README.md          # 运行环境与步骤说明
└── output/
    ├── panda_motion.mp4   # 生成的仿真视频
    ├── panda_motion.gif   # 运行效果动图
    └── preview_000.png
```
