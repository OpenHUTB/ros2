# Shadow Hand 灵巧手 MuJoCo 仿真

## 📖 项目介绍
基于官方 MuJoCo Menagerie 模型库中的 `shadow_hand` 右手灵巧手模型，
通过位置伺服执行器与腱执行器驱动 20 个自由度，让五指 + 手腕做正弦开合摆动
（小指反相、拇指错相），并离屏渲染输出 MP4 仿真视频。不依赖 Gazebo、
不需要写 URDF，适合灵巧手/多指操作入门示例。

## 💻 运行环境
- Ubuntu 20.04 / 22.04 / 24.04（Windows 亦可运行）
- Python >= 3.10
- 依赖版本：mujoco 3.x、mujoco-menagerie、imageio、imageio-ffmpeg、numpy

## 📦 安装依赖
```bash
cd src/manipulator/shadow_hand_mujoco
pip install -r requirements.txt
```
> 模型由 `mujoco-menagerie` 首次运行时自动下载并缓存，无需手动准备。

## 🚀 运行仿真

### 方式一：直接运行脚本（不依赖 ROS）
```bash
cd src/manipulator/shadow_hand_mujoco
python3 main.py            # 输出 output/shadow_hand_grasp.mp4
python3 main.py --preview  # 只渲染 3 帧预览图到 output/preview/
python3 main.py --out 自定义.mp4
```
无显示器 / SSH 环境：`MUJOCO_GL=osmesa python3 main.py`。

### 方式二：作为 catkin 包，用 roslaunch 启动（需 ROS 环境）
```bash
cd 你的工作空间/src
ln -s 本仓库路径/src/manipulator/shadow_hand_mujoco .
cd 你的工作空间
catkin_make
source devel/setup.bash
roslaunch shadow_hand_mujoco main.launch
```

## 🔧 控制原理
- 20 个执行器：16 个位置伺服（直接写目标角）+ 4 个腱执行器（手指 J0，范围 0~3.14）。
- 动作：以初始关节角为基准，每根手指正弦开合 `ctrl = base + amp·sin(2π·freq·t + phase)`。
- 幅度：中/远端屈曲关节（J3/J4/J5）0.55、拇指 0.40、手腕 0.22、其余 0.30；频率 0.5 Hz（2 秒一个循环）。

## 📄 文件结构
```
shadow_hand_mujoco/
├── main.py            # 仿真与渲染主脚本（入口）
├── common.py          # 公共库：渲染后端选择 / 相机注入 / 视频导出
├── requirements.txt
├── package.xml        # catkin 包描述
├── CMakeLists.txt
├── launch/
│   └── main.launch    # roslaunch 启动入口
├── README.md
└── output/
    ├── shadow_hand_grasp.mp4   # 生成的仿真视频
    ├── shadow_hand_grasp.gif   # 运行效果动图
    └── preview_000.png
```
