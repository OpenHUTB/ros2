# 简易人形机器人 MuJoCo 仿真（原地踏步 + 交替摆臂）

## 📖 项目介绍
使用内联 XML 在 MuJoCo 中搭建一个简易人形机器人（躯干 + 头部 + 双臂 + 双腿，4 个旋转关节），
执行器为位置伺服（kp=20），控制器让双腿交替踏步、双臂交替摆动（相位错开），形成拟行走的节律运动，
并离屏渲染输出 MP4 仿真视频。不依赖 Gazebo、不需要 URDF，适合 MuJoCo 基础建模与运动控制入门示例。

运行效果动图见仓库 `docs/img/humanoid/midterm_humanoid/robot_sim.gif`（完整视频由脚本运行生成）。

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
python3 main.py --duration 3     # 自定义完整视频时长（秒），不填则用默认 8.0
python3 -m unittest discover -s tests -v   # 单元测试：模型结构、执行器、控制器
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
roslaunch midterm_humanoid main.launch                  # 默认（8.0 秒）
roslaunch midterm_humanoid main.launch duration:=3.0    # 自定义时长
roslaunch midterm_humanoid main.launch preview:=true    # 只渲染 3 帧预览、不写视频
```
包结构说明：`catkin_python_setup()` + `setup.py` 会把 `src/midterm_humanoid/`
安装为 Python 包，`roslaunch` 下 `main.py` 以 `from midterm_humanoid.common import ...` 导入公共库；
模块根目录的 `main.py` 是源码树直跑入口（把 `src/` 加入 `sys.path` 后转发到包内入口）。

## 🔧 控制原理
- 4 个关节均为位置伺服执行器（kp=20，带关节粘滞阻尼）：每个控制步把目标角写入 `data.ctrl`，
  由执行器内置增益与阻尼决定跟踪特性（脚本不显式计算力矩）。
- 节律控制：`ctrl = 幅度 × sin(2π·freq·t + 相位)`，freq=0.5 Hz（2 秒一个周期）。
- 双腿交替踏步（幅度 0.55 rad、相位差 π），双臂交替摆动（幅度 0.70 rad）并与腿部错相。
- kp 选择依据与实测跟踪误差对比见教程页 `docs/humanoid/midterm_humanoid_simulation.md`。

## 📄 文件结构
```
midterm_humanoid/
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
    └── midterm_humanoid/   # Python 包
        ├── __init__.py
        ├── common.py       # 公共库：渲染后端 / 仿真循环 / 视频导出
        └── main.py         # 仿真与渲染主脚本（入口，内联 XML）
```
`output/`（robot_sim.mp4 / robot_sim.gif / 预览 PNG）为运行生成物，已通过 `.gitignore` 排除，不入版本库。
