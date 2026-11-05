# 三足仿生机器人 MuJoCo 仿真

## 📖 项目介绍
本模块用 MuJoCo 物理引擎搭建三足仿生机器人（躯干 + 3 条腿，共 9 个旋转关节），
9 个关节均由位置伺服执行器驱动，三条腿按 2π/3 相位差交替「抬脚前迈 → 落地支撑」，
形成稳定的三角步态，并离屏渲染输出 MP4 仿真视频。不依赖 Gazebo、不需要写 URDF。

运行效果动图见仓库 `docs/img/ground/tripod_mujoco/sim.gif`（完整视频由脚本运行生成）。

## 💻 运行环境
- Ubuntu 20.04 / 22.04 / 24.04（Windows 亦可运行）
- Python >= 3.10
- 依赖版本：mujoco 3.x、numpy、imageio、imageio-ffmpeg

## 📦 安装依赖
```bash
cd src/ground/tripod_mujoco
pip install -r requirements.txt
```

## 🚀 运行仿真

### 方式一：直接运行脚本（不依赖 ROS）
```bash
cd src/ground/tripod_mujoco
python3 main.py            # 输出 output/sim_video.mp4
python3 main.py --preview  # 只渲染 3 帧预览图到 output/preview/
```
无显示器 / SSH 环境：`MUJOCO_GL=osmesa python3 main.py`。

### 方式二：作为 catkin 包，用 roslaunch 启动（需 ROS 环境）
```bash
cd 你的工作空间/src
ln -s 本仓库路径/src/ground/tripod_mujoco .
cd 你的工作空间
catkin_make
source devel/setup.bash
roslaunch tripod_mujoco main.launch
```
包结构说明：`catkin_python_setup()` + `setup.py` 会把 `src/tripod_mujoco/`
安装为 Python 包，`roslaunch` 下 `main.py` 以 `from tripod_mujoco.common import ...` 导入公共库；
模块根目录的 `main.py` 是源码树直跑入口（把 `src/` 加入 `sys.path` 后转发到包内入口）。

## 🔧 控制原理
- 9 个关节（3×hip_abd / hip_rot / knee）均为位置伺服执行器：每个控制步把目标角
  `qd = base + amp·sin(2π·freq·t + phase)` 写入 `data.ctrl[i]`，由执行器内置的
  kp 增益与关节阻尼决定跟踪特性（脚本不显式计算力矩）。
- 三角步态：三条腿相位差 2π/3，摆动相（sin>0）屈膝抬脚前迈，支撑相伸膝蹬地，
  任意时刻 1 腿摆动、2 腿支撑，步频 0.8 Hz。
- 公式与推导、性能实测见教程页 `docs/ground/tripod_mujoco_simulation.md`。

## 📄 文件结构
```
tripod_mujoco/
├── setup.py             # setuptools 配置（catkin_python_setup 需要）
├── package.xml          # catkin 包描述
├── CMakeLists.txt       # catkin_python_setup + 只安装 main.py 入口
├── requirements.txt
├── .gitignore           # 忽略 output/（运行生成物不入库）
├── launch/
│   └── main.launch      # roslaunch 启动入口
├── README.md
└── src/
    └── tripod_mujoco/   # Python 包
        ├── __init__.py
        ├── common.py    # 公共库：渲染后端 / 仿真循环 / 视频导出
        └── main.py      # 仿真与渲染主脚本（入口，内含模型 XML 与三角步态控制律）
```
`output/`（sim_video.mp4 / 预览 PNG）为运行生成物，已通过 `.gitignore` 排除，不入版本库。
