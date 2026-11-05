# Franka Panda 机械臂 MuJoCo 仿真

## 📖 项目介绍
基于官方 MuJoCo Menagerie 模型库中的 `franka_emika_panda` 七轴机械臂模型，
通过位置伺服执行器直接写目标角，实现多组姿态间的余弦缓动轨迹运动与夹爪开合，
并离屏渲染输出 MP4 仿真视频。不依赖 Gazebo、不需要写 URDF，适合机械臂运动控制入门示例。

运行效果动图见仓库 `docs/img/manipulator/panda_mujoco/panda_motion.gif`（完整视频由脚本运行生成）。

## 💻 运行环境
- Ubuntu 20.04 / 22.04 / 24.04（Windows 亦可运行）
- Python >= 3.10
- 依赖版本：mujoco 3.x、mujoco-menagerie、imageio、imageio-ffmpeg、numpy、pillow（预览 PNG）

## 📦 安装依赖
```bash
cd src/manipulator/panda_mujoco
pip install -r requirements.txt
```

> Linux 无显示环境（虚拟机 / SSH）用 osmesa 软件渲染，需要系统库：`sudo apt install -y libosmesa6-dev`。
> MP4 由 `imageio-ffmpeg` 自带的 ffmpeg 生成，无需另装；预览 PNG 用 Pillow，已列在 requirements.txt。
> 模型由 `mujoco-menagerie` 首次运行时自动下载并缓存，无需手动准备。
>
> 注意：`mujoco-menagerie` 解包时用了 `tarfile.extractall(filter="data")`，该参数要求 Python >= 3.11.4。
> Python 3.10（Ubuntu 20.04 / 22.04 默认）上首次下载会抛 TypeError；本模块会自动扫描本地缓存复用
> 已有模型，缓存为空时请升级 Python，或手动把模型放到 `~/.cache/mujoco_menagerie/models/`（目录名以
> `<robot>-` 开头）。


## 🚀 运行仿真

### 方式一：直接运行脚本（不依赖 ROS）
```bash
cd src/manipulator/panda_mujoco
python3 main.py            # 输出 output/panda_motion.mp4
python3 main.py --preview  # 只渲染 3 帧预览图到 output/preview/
python3 main.py --duration 3     # 自定义完整视频时长（秒），不填则用默认 8.0
python3 -m unittest discover -s tests -v   # 单元测试：模型结构、执行器、控制器
python3 main.py --out 自定义.mp4
```
Linux 无显示环境默认自动用 osmesa 软件渲染（需先 `sudo apt install -y libosmesa6-dev`），无需手动设 `MUJOCO_GL`；有显卡可用 `MUJOCO_GL=egl` 提速。Windows 自动 wgl，macOS 自动 cgl。

### 方式二：作为 catkin 包，用 roslaunch 启动（需 ROS 环境）
```bash
cd 你的工作空间/src
ln -s 本仓库路径/src/manipulator/panda_mujoco .
cd 你的工作空间
catkin_make
source devel/setup.bash
roslaunch panda_mujoco main.launch                  # 默认（8.0 秒）
roslaunch panda_mujoco main.launch duration:=3.0    # 自定义时长
roslaunch panda_mujoco main.launch preview:=true    # 只渲染 3 帧预览、不写视频
```
包结构说明：`catkin_python_setup()` + `setup.py` 会把 `src/panda_mujoco/`
安装为 Python 包，`roslaunch` 下 `main.py` 以 `from panda_mujoco.common import ...` 导入公共库；
模块根目录的 `main.py` 是源码树直跑入口（把 `src/` 加入 `sys.path` 后转发到包内入口）。

## 🔧 控制原理
- 7 个臂关节为位置伺服执行器（增益 2000~4500），脚本只把目标角写进 `data.ctrl`，
  驱动力由执行器内部驱动模型产生（不是脚本里的 PD 公式）。
- 夹爪 actuator8 为腱执行器（0~255），按 8 秒周期正弦开合。
- 轨迹：home 姿态 + 3 组增量位姿共 4 段循环（上抬 → 前伸 → 侧摆 → 归位），
  段间余弦缓动插值（q_d = q_a + (1−cos(πt/T))/2·(q_b−q_a)），首尾零速度避免力矩冲击。
- 公式与推导、性能实测见教程页 `docs/manipulator/panda_mujoco_simulation.md`。

## 📄 文件结构
```
panda_mujoco/
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
    └── panda_mujoco/    # Python 包
        ├── __init__.py
        ├── common.py    # 公共库：渲染后端 / 仿真循环 / 视频导出
        └── main.py      # 仿真与渲染主脚本（入口）
```
`output/`（panda_motion.mp4 / 预览 PNG）为运行生成物，已通过 `.gitignore` 排除，不入版本库。
