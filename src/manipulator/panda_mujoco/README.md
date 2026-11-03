# Franka Panda 机械臂 MuJoCo 仿真

## 📖 项目介绍
基于官方 MuJoCo Menagerie 模型库中的 `franka_emika_panda` 七轴机械臂模型，
通过位置伺服执行器直接写目标角，实现多组姿态间的余弦缓动轨迹运动与夹爪开合，
并离屏渲染输出 MP4 仿真视频。不依赖 Gazebo、不需要写 URDF，适合机械臂运动控制入门示例。

## 💻 运行环境
- Ubuntu 20.04 / 22.04 / 24.04（Windows 亦可运行）
- Python >= 3.10
- 依赖版本：mujoco 3.x、mujoco-menagerie、imageio、imageio-ffmpeg、numpy

## 📦 安装依赖
```bash
cd src/manipulator/panda_mujoco
pip install -r requirements.txt
```
> 模型由 `mujoco-menagerie` 首次运行时自动下载并缓存，无需手动准备。

## 🚀 运行仿真

### 方式一：直接运行脚本（不依赖 ROS）
```bash
cd src/manipulator/panda_mujoco
python3 main.py            # 输出 output/panda_motion.mp4
python3 main.py --preview  # 只渲染 3 帧预览图到 output/preview/
python3 main.py --out 自定义.mp4
```
无显示器 / SSH 环境：`MUJOCO_GL=osmesa python3 main.py`。

### 方式二：作为 catkin 包，用 roslaunch 启动（需 ROS 环境）
```bash
cd 你的工作空间/src
ln -s 本仓库路径/src/manipulator/panda_mujoco .
cd 你的工作空间
catkin_make
source devel/setup.bash
roslaunch panda_mujoco main.launch
```

## 🔧 控制原理
- 7 个臂关节为位置伺服执行器（增益 2000~4500），直接把目标角写进 `data.ctrl`，无需 PD。
- 夹爪 actuator8 为腱执行器（0~255），按 8 秒周期开合。
- 轨迹：home 姿态 + 3 组增量位姿共 4 段循环（上抬 → 前伸 → 侧摆 → 归位），余弦缓动平滑过渡。

## 📄 文件结构
```
panda_mujoco/
├── main.py            # 仿真与渲染主脚本（入口）
├── common.py          # 公共库：渲染后端选择 / 相机注入 / 视频导出
├── requirements.txt
├── package.xml        # catkin 包描述
├── CMakeLists.txt
├── launch/
│   └── main.launch    # roslaunch 启动入口
├── README.md
└── output/
    ├── panda_motion.mp4   # 生成的仿真视频
    ├── panda_motion.gif   # 运行效果动图
    └── preview_000.png
```
