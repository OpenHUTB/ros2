# MuJoCo 城市屋顶光伏覆盖率测量

本模块面向城市建筑屋顶光伏覆盖率测量，使用 MuJoCo 3.12.0 构建轻量四旋翼、下视相机、屋顶光伏阵列与障碍物场景，并生成参数化往复条带航线。统一入口支持场景渲染、同步数据采集、分割与控制训练、局部航点规划、RGB-D 平面地标 SLAM、键盘控制和性能评价。

## 场景

| 名称| 内容|
| --- | --- |
| `sparse` | 6 组稀疏光伏板|
| `dense` | 16 组密集光伏板|
| `occluded` | 12 组光伏板、天窗与两台空调机组|

所有场景仅使用 MJCF 基础几何体，不依赖外部网格、纹理或大型地图。场景源文件位于 `assets/mjcf/`，可由 `tools/generate_scenes.py` 确定性重建。

## 运行

目标环境为 Python 3.10；代码同时在 Python 3.11 上执行兼容性验证。

```powershell
python -m venv .venv
& .\.venv\Scripts\python.exe -m pip install -r .\src\air\mujoco_uav_rooftop_pv_coverage_survey\requirements.txt
& .\.venv\Scripts\python.exe .\src\air\mujoco_uav_rooftop_pv_coverage_survey\main.py --scenario sparse
```

Linux 与 ROS 2 Humble 环境可使用同一入口：

```bash
python3 -m venv .venv
.venv/bin/pip install -r src/air/mujoco_uav_rooftop_pv_coverage_survey/requirements.txt
.venv/bin/python src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --scenario sparse
```

默认输出目录为 `artifacts/scene/`，其中包含 `route.json`、`scene_rgb.png` 与 `summary.json`。可通过 `--config` 和 `--output` 修改配置文件及输出目录。

数据采集模式同步输出 RGB、像素级光伏掩码与专家控制标签：

```powershell
& .\.venv\Scripts\python.exe .\src\air\mujoco_uav_rooftop_pv_coverage_survey\main.py --mode dataset --output .\survey_data
```

`manifest.jsonl` 为每帧记录场景、序列、划分、相机位姿、RGB/掩码相对路径、像素占比，以及横向偏差、航向误差、距转弯点距离、前进速度和偏航角速度。`splits.json` 以完整序列为单位划分 train、validation、test；`quality_report.json` 检查文件完整性、尺寸一致性、二值标签、序列泄漏与重复内容。

## 测试

```powershell
& .\.venv\Scripts\python.exe -m pytest .\src\air\mujoco_uav_rooftop_pv_coverage_survey\tests -q
```

测试覆盖航线边界、条带覆盖、参数校验、三场景差异、四旋翼执行器、下视相机、专家控制符号与限幅、RGB-mask 一致性、划分隔离和 manifest 完整性。

## 接口边界

场景坐标系采用屋顶局部 ENU，航点字段固定为 `x`、`y`、`z`、`yaw`、`speed` 与 `arrival_time`。ROS 2 Humble 控制节点使用 `nav_msgs/Path`、`geometry_msgs/PoseStamped` 和图像话题；数据生成入口可独立于 ROS 运行。

## Docker

在 Ubuntu 22.04 基础的 ROS 2 Humble 容器中运行完整第二天流程：

```bash
docker compose run --rm rooftop-pv
```

容器入口执行 `main.py --mode all`，输出写入挂载的 `artifacts/day2/`。

本模块包含 `package.xml`、`setup.py` 和 launch 文件，可在 ROS 2 Humble 工作空间通过 `colcon build` 安装。

## 神经网络训练与完整流程

从仓库根目录运行完整流程：

```bash
python3 src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode all --output survey_data
```

流程保存 `segmentation.pt`、`control.pt`、`end_to_end.pt`、`planning.pt` 四个训练权重，以及 `day2_summary.json`、RGB/掩码、SLAM 地图、轨迹和 GIF。完整序列分别用于训练、验证和测试。

## 交互键盘与同步 SLAM

```bash
python3 src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode keyboard-interactive --output keyboard_run
python3 src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode slam-nav --output slam_run
```

键盘模式接收查看器事件；W/S 前后运动，A/D 转向，Q/E 小幅转向，空格停车，按键超时后停止。运动采用固定高度速度积分。SLAM 模式通过 RGB-D 彩色地标相对观测和指令里程计联合估计机器人位置与未知地标，并同步导航至目标航点；采用固定高度、固定航向的平面模型。

## ROS 2 安装启动

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mujoco_uav_rooftop_pv_coverage_survey
source install/setup.bash
ros2 launch mujoco_uav_rooftop_pv_coverage_survey rooftop_pv_controller.launch.py
ros2 launch mujoco_uav_rooftop_pv_coverage_survey rooftop_pv_workflow.launch.py mode:=all output:=survey_data
```

控制节点订阅 `Image`、`PoseStamped`、`Path`，发布 `Twist` 和状态；`segmentation_checkpoint`、`planning_checkpoint`、`control_checkpoint` 参数选择分割、局部规划和特征控制权重；`end_to_end_checkpoint` 选择 RGB 到速度模型。所有控制模式要求有效新鲜图像；输入缺失、超时或坐标系不一致时停车。数据生成不要求 ROS；实时消息控制运行于 ROS 2 Humble。通用 workflow launch 可通过 `mode` 参数运行全部数据、训练、SLAM 与键盘模式。

## 运行结果

Ubuntu 22.04 / ROS 2 Humble 容器内 55 项测试通过，完整流程生成三个场景的数据并通过质量审计；安装后的三个 MJCF 场景可在源码目录之外载入，workflow launch 场景启动及 Compose 完整流程运行成功。真实 ROS 消息验收通过，分割、局部规划与控制权重可同时加载；端到端模型直接从 RGB 消息生成有界速度，位姿超时后停车。虚拟显示器中向真实 MuJoCo 查看器发送 W/A/Space 事件，运动、转向、空格停止和超时停止均通过。

完整原理、源码结构、截图与 GIF 见仓库文档 `docs/air/mujoco_uav_rooftop_pv_coverage_survey_day2.md`。
