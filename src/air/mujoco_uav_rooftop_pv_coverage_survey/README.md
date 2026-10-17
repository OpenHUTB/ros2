# MuJoCo 城市屋顶光伏覆盖率测量 / MuJoCo Urban Rooftop Photovoltaic Coverage Survey

本模块面向城市建筑屋顶光伏覆盖率测量，使用 MuJoCo 3.12.0 构建轻量四旋翼、下视相机、屋顶光伏阵列与障碍物场景，并生成参数化往复条带航线。当前入口可完整载入场景、生成航线、定位相机并输出 RGB 影像与结构化摘要。

This module targets photovoltaic coverage surveys on urban rooftops. It uses MuJoCo 3.12.0 to model a lightweight quadrotor, nadir camera, rooftop photovoltaic arrays, and obstacles, then generates a parameterized boustrophedon route. The current entry point loads a scene, generates the route, positions the camera, and writes an RGB image plus a structured summary.

## 场景 / Scenarios

| 名称 / Name | 内容 / Content |
| --- | --- |
| `sparse` | 6 组稀疏光伏板 / 6 sparse photovoltaic panels |
| `dense` | 16 组密集光伏板 / 16 dense photovoltaic panels |
| `occluded` | 14 组光伏板、天窗与两台空调机组 / 14 panels, one skylight, and two HVAC units |

所有场景仅使用 MJCF 基础几何体，不依赖外部网格、纹理或大型地图。场景源文件位于 `assets/mjcf/`，可由 `tools/generate_scenes.py` 确定性重建。

All scenes use MJCF primitives only, with no external mesh, texture, or large map. Source files are stored in `assets/mjcf/` and can be deterministically rebuilt with `tools/generate_scenes.py`.

## 运行 / Run

目标环境为 Python 3.10；代码同时在 Python 3.11 上执行兼容性验证。

The target environment is Python 3.10; compatibility is also verified with Python 3.11.

```powershell
python -m venv .venv
& .\.venv\Scripts\python.exe -m pip install -r .\src\air\mujoco_uav_rooftop_pv_coverage_survey\requirements.txt
& .\.venv\Scripts\python.exe .\src\air\mujoco_uav_rooftop_pv_coverage_survey\main.py --scenario sparse
```

Linux 与 ROS 2 Humble 环境可使用同一入口：

The same entry point is available on Linux and in a ROS 2 Humble workspace:

```bash
python3 -m venv .venv
.venv/bin/pip install -r src/air/mujoco_uav_rooftop_pv_coverage_survey/requirements.txt
.venv/bin/python src/air/mujoco_uav_rooftop_pv_coverage_survey/main.py --scenario sparse
```

默认输出目录为 `artifacts/scene/`，其中包含 `route.json`、`scene_rgb.png` 与 `summary.json`。可通过 `--config` 和 `--output` 修改配置文件及输出目录。

The default output directory is `artifacts/scene/`, containing `route.json`, `scene_rgb.png`, and `summary.json`. Use `--config` and `--output` to select another configuration or destination.

## 测试 / Tests

```powershell
& .\.venv\Scripts\python.exe -m pytest .\src\air\mujoco_uav_rooftop_pv_coverage_survey\tests -q
```

测试覆盖航线边界、条带覆盖、参数校验、三场景差异、四旋翼执行器与下视相机存在性。

Tests cover route boundaries, strip coverage, parameter validation, differences among the three scenes, quadrotor actuators, and the nadir camera.

## 接口边界 / Interface Boundary

场景坐标系采用屋顶局部 ENU，航点字段固定为 `x`、`y`、`z`、`yaw`、`speed` 与 `arrival_time`。该结构可直接映射到后续 ROS 2 Humble 的 `nav_msgs/Path`、`geometry_msgs/PoseStamped` 和控制话题，而不把 ROS 依赖引入数据生成层。

The scene uses a rooftop-local ENU frame. Waypoints expose stable `x`, `y`, `z`, `yaw`, `speed`, and `arrival_time` fields. This schema maps directly to later ROS 2 Humble `nav_msgs/Path`, `geometry_msgs/PoseStamped`, and control topics without introducing ROS dependencies into the data-generation layer.
