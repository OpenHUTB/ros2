# 宇树 Go2 四足 MuJoCo 小跑仿真

## 📖 项目介绍
基于官方 MuJoCo Menagerie 模型库中的 `unitree_go2` 模型，编写自带的 PD 控制器与
「椭圆足端轨迹 + 平面两连杆 IK」步态，让四足机器人稳定向前小跑，并离屏渲染输出 MP4 仿真视频。
不依赖 Gazebo、不需要写 URDF，适合作为机器人运动控制入门示例。

运行效果动图见仓库 `docs/img/ground/go2_mujoco/go2_trot.gif`（完整视频由脚本运行生成）。

## 💻 运行环境
- Ubuntu 20.04 / 22.04 / 24.04（Windows 亦可运行）
- Python >= 3.10
- 依赖版本：mujoco 3.x、mujoco-menagerie、imageio、imageio-ffmpeg、numpy

## 📦 安装依赖
```bash
cd src/ground/go2_mujoco
pip install -r requirements.txt
```
> 模型由 `mujoco-menagerie` 首次运行时自动下载并缓存，无需手动准备。

## 🚀 运行仿真

### 方式一：直接运行脚本（不依赖 ROS）
```bash
cd src/ground/go2_mujoco
python3 main.py            # 输出 output/go2_trot.mp4
python3 main.py --preview  # 只渲染 3 帧预览图到 output/preview/
python3 main.py --duration 3     # 自定义完整视频时长（秒），不填则用默认 6.0
python3 -m unittest discover -s tests -v   # 单元测试：模型结构、执行器、控制器
python3 main.py --out 自定义.mp4
```
无显示器 / SSH 环境：`MUJOCO_GL=osmesa python3 main.py`。

### 方式二：作为 catkin 包，用 roslaunch 启动（需 ROS 环境）
```bash
cd 你的工作空间/src
ln -s 本仓库路径/src/ground/go2_mujoco .
cd 你的工作空间
catkin_make
source devel/setup.bash
roslaunch go2_mujoco main.launch                  # 默认（6.0 秒）
roslaunch go2_mujoco main.launch duration:=3.0    # 自定义时长
roslaunch go2_mujoco main.launch preview:=true    # 只渲染 3 帧预览、不写视频
```
包结构说明：`catkin_python_setup()` + `setup.py` 会把 `src/go2_mujoco/`
安装为 Python 包，`roslaunch` 下 `main.py` 以 `from go2_mujoco.common import ...` 导入公共库；
模块根目录的 `main.py` 是源码树直跑入口（把 `src/` 加入 `sys.path` 后转发到包内入口）。

## 🔧 控制原理
- 12 个执行器为力矩电机（±23.7 N·m），脚本内自带 PD 控制器（KP=20, KD=0.5），
  控制律 τ = KP·(q_d − q) − KD·q̇。
- 步态：足端按椭圆轨迹运动（摆动相抬脚、支撑相贴地），平面 IK 反解大腿/小腿目标角，
  对角配对（FL/RR 与 FR/RL 反相），前进速度约 0.12 m/s。
- 公式与推导、性能实测见教程页 `docs/ground/go2_mujoco_simulation.md`。

## 📄 文件结构
```
go2_mujoco/
├── tests/               # 单元测试（python -m unittest discover -s tests）
├── setup.py             # setuptools 配置（catkin_python_setup 需要）
├── package.xml          # catkin 包描述
├── CMakeLists.txt       # catkin_python_setup + 只安装 main.py 入口
├── requirements.txt
├── .gitignore           # 忽略 output/（运行生成物不入库）
├── launch/
│   └── main.launch      # roslaunch 启动入口
├── README.md
└── src/
    └── go2_mujoco/      # Python 包
        ├── __init__.py
        ├── common.py    # 公共库：渲染后端 / 仿真循环 / 视频导出
        └── main.py      # 仿真与渲染主脚本（入口）
```
`output/`（go2_trot.mp4 / 预览 PNG）为运行生成物，已通过 `.gitignore` 排除，不入版本库。
