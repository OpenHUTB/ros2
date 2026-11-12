# pi0_mujoco_grasp

基于 MuJoCo 的**单臂抓取与放置**仿真演示：Franka Panda 从桌面抓取红色方块，移动到白色圆盘上方并放置；
主视图 + 腕部相机画中画同步渲染，并提供与 π0 一致的「观测 → 动作块 → 执行」接口（CPU 用脚本化 IK 策略替身）。

## 模块组成

| 文件 | 说明 |
|---|---|
| `main.py` | 演示入口：加载场景、执行抓取/放置、按需录制帧序列 |
| `pi0_mujoco_grasp/pi0_policy_stub.py` | π0 策略接口（CPU 替身）：`obs -> action chunk -> MuJoCo 执行` |
| `models/franka_emika_panda/scene_pickplace.xml` | 桌面 + 红方块 + 白圆盘 + 主相机/腕部相机 |
| `models/franka_emika_panda/panda_pick.xml` | Panda 模型（上游 panda.xml + TCP site + 腕部相机） |
| `models/franka_emika_panda/assets.sha256` | 网格资源校验清单（对应固定上游 commit） |
| `scripts/setup_model.py` | 按固定 commit 下载并校验 Panda 网格资源（约 33 MB） |
| `scripts/make_gif.py` | 帧序列 → 截图 / GIF / MP4 |
| `scripts/record_demo.sh` | 一条命令录制并生成素材 |

## 环境要求

- Ubuntu 20.04，Python 3.8
- `mujoco==2.3.7`（CPU 版即可）、`numpy==1.24.4`、`Pillow==10.4.0`（见 `requirements.txt`）
- 无 GPU 也可运行；渲染使用 MuJoCo GLFW 后端（需要图形会话，虚拟机桌面即可）
- 真实 π0 模型推理需要 GPU（≥ 8 GB 显存）与 openpi 权重/服务；
  本模块在无 GPU 环境下用脚本化 IK 策略替身复现同一套「观测 → 动作块 → 执行」流程。

## 安装

```bash
# 1) Python 依赖（建议虚拟环境）
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# 2) 下载并校验 Panda 网格资源（约 33 MB，固定上游 commit，可用代理镜像）
#    国内网络可设置 PI0_MENAGERIE_BASE 指向镜像（见 scripts/setup_model.py 注释）
python scripts/setup_model.py
```

> **安装为 ROS 2 包（colcon）时**：请先执行第 2 步下载网格，再 `colcon build`，
> 这样 `assets/` 会随 `data_files` 安装到 share 目录；否则安装态启动会提示缺少网格。

## 运行

```bash
# 仅仿真（快速验证）
python main.py

# 渲染 640x480 主视图 + 腕部画中画，并生成截图/GIF/MP4
# XAUTHORITY 按当前 UID 自动查找（非 1000 用户也适用）
export DISPLAY=:0
export XAUTHORITY=${XAUTHORITY:-$(ls /run/user/$(id -u)/gdm/Xauthority 2>/dev/null || echo "$HOME/.Xauthority")}
export MUJOCO_GL=glfw
python main.py --record /tmp/pi0_frames --fps 30
python scripts/make_gif.py --frames /tmp/pi0_frames --out results --fps 30

# 或一键：
bash scripts/record_demo.sh /tmp/pi0_frames
```

ROS 2 下（需 ROS 2 环境；先按上面安装依赖并在源码树下载网格）：

```bash
colcon build --symlink-install --packages-select pi0_mujoco_grasp
source install/setup.bash
ros2 launch pi0_mujoco_grasp main.launch.py                              # 仅仿真
ros2 launch pi0_mujoco_grasp main.launch.py record:=/tmp/pi0_frames fps:=30
ros2 launch pi0_mujoco_grasp main.launch.py policy:=pi0                   # 需要 GPU
```

## 实测结果（VMware 虚拟机，无 GPU）

| 指标 | 数值 |
|---|---|
| 控制方式 | 7 关节位置伺服 + 夹爪（0/255），IK 阻尼最小二乘 |
| IK 位置误差 | 2e-5~5e-5 m（各 waypoint，< 1e-4 m） |
| 抓取放置 | 红色方块从桌面 (0.45, -0.12) 移到白盘 (0.45, 0.12) |
| 放置误差 | 5.1 mm（方块中心到盘心 XY 距离） |
| 渲染 | 640×480 + 腕部 320×240，330 帧（30 fps） |
| 渲染耗时 | 约 20~60 s（CPU 软件渲染，视内存/CPU 而定；1.9 GB VM 实测 54 s） |
| GIF / MP4 | 约 2.9 MB / 0.4 MB |

## 已知限制

- 无 GPU，未运行真实 π0 权重；`pi0_policy_stub.py` 只复现接口与执行链路。
- 场景与相机角度为演示自定义，非逐像素一致。
- 网格资源（约 33 MB）不入库，由 `scripts/setup_model.py` 按固定 commit 下载并校验 SHA256。

## 大模型使用声明

本模块的代码整理、调参与文档由 AI 助手辅助完成；所有命令、截图与数据均在
Ubuntu 20.04 + MuJoCo 2.3.7（CPU）环境中实际运行验证，作者已核对并对内容负责。
