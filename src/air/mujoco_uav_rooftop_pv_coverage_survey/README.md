# MuJoCo 城市屋顶光伏覆盖率测量

本模块面向城市建筑屋顶光伏覆盖率测量，使用 MuJoCo 3.12.0 构建轻量四旋翼、下视相机、屋顶光伏阵列与障碍物场景，并生成参数化往复条带航线。当前入口可完整载入场景、生成航线、定位相机并输出 RGB 影像与结构化摘要。

## 场景

| 名称 | 内容 |
| --- | --- |
| `sparse` | 6 组稀疏光伏板 |
| `dense` | 16 组密集光伏板 |
| `occluded` | 12 组光伏板、天窗与两台空调机组 |

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

场景坐标系采用屋顶局部 ENU，航点字段固定为 `x`、`y`、`z`、`yaw`、`speed` 与 `arrival_time`。该结构可直接映射到后续 ROS 2 Humble 的 `nav_msgs/Path`、`geometry_msgs/PoseStamped` 和控制话题，而不把 ROS 依赖引入数据生成层。
