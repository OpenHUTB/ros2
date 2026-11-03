# 简易人形机器人 MuJoCo 仿真（原地踏步 + 交替摆臂）

## 📖 项目介绍
使用内联 XML 在 MuJoCo 中搭建一个简易人形机器人（躯干 + 头部 + 双臂 + 双腿，4 个旋转关节），
执行器为位置伺服，控制器让双腿交替踏步、双臂交替摆动（相位错开），形成拟行走的节律运动，
并离屏渲染输出 MP4 仿真视频。不依赖 Gazebo、不需要 URDF，适合 MuJoCo 基础建模与运动控制入门示例。

## 💻 运行环境
- Ubuntu 20.04 / 22.04 / 24.04（Windows 亦可运行）
- Python >= 3.10
- 依赖版本：mujoco 3.x、numpy、imageio、imageio-ffmpeg

## 📦 安装依赖
```bash
cd src/humanoid/midterm_humanoid
pip install -r requirements.txt
```

## 🚀 运行仿真

### 方式一：直接运行脚本（不依赖 ROS）
```bash
cd src/humanoid/midterm_humanoid
python3 main.py            # 输出 output/robot_sim.mp4
python3 main.py --preview  # 只渲染 1.5 秒，输出 3 张预览 PNG 检查取景
python3 main.py --out 自定义.mp4
```
无显示器 / SSH 环境：`MUJOCO_GL=osmesa python3 main.py`。

### 方式二：作为 catkin 包，用 roslaunch 启动（需 ROS 环境）
```bash
cd 你的工作空间/src
ln -s 本仓库路径/src/humanoid/midterm_humanoid .
cd 你的工作空间
catkin_make
source devel/setup.bash
roslaunch midterm_humanoid main.launch
```

## 🔧 控制原理
- 4 个关节均为位置伺服执行器（kp=8），直接写目标角，稳定不炸。
- 节律控制：`ctrl = 幅度 × sin(2π·freq·t + 相位)`，freq=0.5 Hz（2 秒一个周期）。
- 双腿交替踏步（相位差 π），双臂交替摆动并与腿部错相，形成协调的拟行走节律。

## 📄 文件结构
```
midterm_humanoid/
├── main.py            # 仿真与渲染主脚本（入口）
├── common.py          # 公共库：渲染后端选择 / 仿真循环 / 视频导出
├── requirements.txt
├── package.xml        # catkin 包描述
├── CMakeLists.txt
├── launch/
│   └── main.launch    # roslaunch 启动入口
├── README.md
└── output/
    ├── robot_sim.mp4   # 生成的仿真视频
    ├── robot_sim.gif   # 运行效果动图
    └── preview_000.png
```
