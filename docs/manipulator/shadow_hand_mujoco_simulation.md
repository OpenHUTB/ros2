# Shadow Hand 灵巧手 MuJoCo 物理仿真

---

## 1. 概述与目标

本模块基于 MuJoCo 物理仿真引擎与官方 Menagerie 模型库中的 `shadow_hand` 右手灵巧手模型，
实现五指 + 手腕的正弦开合摆动仿真示例，并自动渲染输出仿真视频。

### 核心任务目标
1. **多指协调运动**：20 个自由度（16 关节位置伺服 + 4 腱执行器）按不同幅度/相位正弦摆动。
2. **灵巧手开合动作**：小指反相、拇指错相，形成自然抓取-张开循环。
3. **仿真视频自动渲染**：离屏渲染（Offscreen Rendering）自动生成 MP4 视频。

---

## 2. 模型与运动设计

### 2.1 模型来源
- 机器人：`shadow_hand`（Shadow Hand 右手），来自官方 [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) 模型库。
- 场景：Menagerie 自带 `scene_right.xml`（右手平放于地面、手掌托球），首次运行自动下载缓存。

### 2.2 控制方式
| 项目 | 说明 |
|------|------|
| 执行器 | 16 个位置伺服（直接写目标角）+ 4 个腱执行器（FFJ0/MFJ0/RFJ0/LFJ0，范围 0~3.14） |
| 控制律 | `ctrl = base + amp·sin(2π·freq·t + phase)` |
| 频率 | 0.5 Hz（2 秒一个开合周期） |

### 2.3 幅度与相位（`main.py` 内 `amp()` / `phase()`）
| 关节类别 | 幅度 | 相位 |
|----------|------|------|
| 中/远端屈曲（J3/J4/J5） | 0.55 | 0 |
| 拇指（THJ） | 0.40 | π/2 |
| 手腕（WRJ） | 0.22 | π/4 |
| 小指（LFJ） | 0.30 | π（反相） |
| 其余（近端/掌指） | 0.30 | 0 |

---

## 3. 运行方式

### 3.1 环境依赖
- Ubuntu 20.04 / 22.04 / 24.04（Windows 亦可）
- Python >= 3.10
- MuJoCo 3.x、mujoco-menagerie、imageio + imageio-ffmpeg、numpy

### 3.2 安装依赖
```bash
cd src/manipulator/shadow_hand_mujoco
pip install -r requirements.txt
```

### 3.3 运行仿真

方式一（直接运行，不依赖 ROS）：
```bash
cd src/manipulator/shadow_hand_mujoco
python3 main.py
```
运行成功后，仿真视频自动保存至 `output/shadow_hand_grasp.mp4`；无显示器环境请加 `MUJOCO_GL=osmesa` 前缀。

方式二（catkin 包 + roslaunch，需 ROS 环境）：
```bash
# 将本模块软链到工作空间 src 下，然后
catkin_make
source devel/setup.bash
roslaunch shadow_hand_mujoco main.launch
```

---

## 4. 运行效果

![Shadow Hand 灵巧手动图](https://raw.githubusercontent.com/nanxi666521/ros2/mujoco_shadow_hand/src/manipulator/shadow_hand_mujoco/output/shadow_hand_grasp.gif)

完整视频见 `src/manipulator/shadow_hand_mujoco/output/shadow_hand_grasp.mp4`（8 秒，30 fps，960×544）。

---

## 5. 文件结构

```
shadow_hand_mujoco/
├── main.py            # 仿真与渲染主脚本（入口）
├── common.py          # 公共库（渲染后端 / 相机注入 / 视频导出）
├── requirements.txt
├── package.xml        # catkin 包描述
├── CMakeLists.txt
├── launch/main.launch # roslaunch 启动入口
├── README.md          # 运行环境与步骤说明
└── output/
    ├── shadow_hand_grasp.mp4   # 生成的仿真视频
    ├── shadow_hand_grasp.gif   # 运行效果动图
    └── preview_000.png
```
