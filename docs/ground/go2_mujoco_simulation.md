# 宇树 Go2 四足机器人 MuJoCo 物理仿真

---

## 1. 概述与目标

本模块基于 MuJoCo 物理仿真引擎与官方 Menagerie 模型库中的 `unitree_go2` 四足机器人模型，
实现一个小跑步态控制示例：机器人可在平坦地面上稳定前进，并自动渲染输出仿真视频。

### 核心任务目标
1. **四足小跑步态**：通过足端椭圆轨迹 + 平面逆运动学（IK）反解各腿关节目标角。
2. **力矩电机 PD 控制**：Go2 的执行器为力矩电机，脚本内实现 PD 控制器把关节拉到目标角。
3. **仿真视频自动渲染**：离屏渲染（Offscreen Rendering）自动生成 MP4 视频。

---

## 2. 模型与步态设计

### 2.1 模型来源
- 机器人：`unitree_go2`（宇树 Go2），来自官方 [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) 模型库。
- 场景：Menagerie 自带 `scene.xml`（含地面网格与遮挡剔除），首次运行自动下载缓存。

### 2.2 控制方式
| 项目 | 说明 |
|------|------|
| 执行器 | 12 个力矩电机（ctrlrange ±23.7 N·m） |
| PD 增益 | KP=20, KD=0.5 |
| 足端轨迹 | 椭圆：摆动相抬脚 `LIFT·sin(p)`，支撑相贴地，前后摆动幅度 `A_X` |
| 逆运动学 | 每条腿在矢状面用平面两连杆 IK 求解大腿/小腿目标角 |
| 步态配对 | 对角小跑：FL/RR 一组、FR/RL 一组反相（相位差 π） |

### 2.3 步态参数（`main.py` 顶部）
| 参数 | 值 | 说明 |
|------|-----|------|
| `FREQ` | 1.1 | 步频 Hz |
| `A_X` | 0.10 | 足端前后摆动幅度（m） |
| `LIFT` | 0.042 | 摆动相抬脚高度（m） |
| `KP / KD` | 20.0 / 0.5 | PD 增益 |

---

## 3. 运行方式

### 3.1 环境依赖
- Ubuntu 20.04 / 22.04 / 24.04（Windows 亦可）
- Python >= 3.10
- MuJoCo 3.x、mujoco-menagerie、imageio + imageio-ffmpeg、numpy

### 3.2 安装依赖
```bash
cd src/ground/go2_mujoco
pip install -r requirements.txt
```

### 3.3 运行仿真

方式一（直接运行，不依赖 ROS）：
```bash
cd src/ground/go2_mujoco
python3 main.py
```
运行成功后，仿真视频自动保存至 `output/go2_trot.mp4`；无显示器环境请加 `MUJOCO_GL=osmesa` 前缀。

方式二（catkin 包 + roslaunch，需 ROS 环境）：
```bash
# 将本模块软链到工作空间 src 下，然后
catkin_make
source devel/setup.bash
roslaunch go2_mujoco main.launch
```

---

## 4. 运行效果

![Go2 小跑动图](https://raw.githubusercontent.com/nanxi666521/ros2/mujoco_go2/src/ground/go2_mujoco/output/go2_trot.gif)

完整视频见 `src/ground/go2_mujoco/output/go2_trot.mp4`（6 秒，30 fps，960×544）。

---

## 5. 文件结构

```
go2_mujoco/
├── main.py            # 仿真与渲染主脚本（入口）
├── common.py          # 公共库（渲染后端 / 相机注入 / 视频导出）
├── requirements.txt
├── package.xml        # catkin 包描述
├── CMakeLists.txt
├── launch/main.launch # roslaunch 启动入口
├── README.md          # 运行环境与步骤说明
└── output/
    ├── go2_trot.mp4   # 生成的仿真视频
    ├── go2_trot.gif   # 运行效果动图
    └── preview_000.png
```
