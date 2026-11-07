# carla_lidar_obstacle_detection — 激光雷达 3D 目标检测实验

## 1. 概述

把**语义激光雷达**的点云按「实例」聚成一个个 3D 目标，输出每个目标的类别、质心、包围盒和距离，
画出俯视（BEV）检测图，并与仿真器里真实存在的目标数对账，算出**召回率**。
这是自动驾驶感知栈里「点云 → 目标级输出」的最小可验证实现。

## 2. 与已有示例的区别

| 对比项 | 官方示例 | 本模块 |
|---|---|---|
| 数据 | 只打印点云数量/画个散点 | 解析实例号与类别号，按实例分组做目标级检测 |
| 关联 | 无 | 用实例号把点与 actor 关联（`world.get_actor`），标签与真值天然一致 |
| 稳定性 | 单帧 | 累积 5 帧点云再检测（激光雷达水平角分辨率不足，单帧会大量漏检） |
| 评价 | 无 | 与真值对账算召回率，输出 `PASS/FAIL` |

## 3. 环境要求

- Windows + OpenHUTB 2.10.1 服务端；Python 3.10；`numpy`、`matplotlib`、`PyYAML`
- 客户端 `hutb-2.10.1-cp310` wheel；必须先 `set_files_base_folder()` 再 `get_world()`

## 4. 安装

```bat
python -m pip install D:\hutb\PythonAPI\carla\dist\hutb-2.10.1-cp310-cp310-win_amd64.whl
python -m pip install numpy matplotlib pyyaml
```

## 5. 运行

```bat
:: 终端 1
D:\hutb\CarlaUE4.exe Town10HD_Opt -windowed -carla-rpc-port=2000

:: 终端 2
cd src\ground\carla_lidar_obstacle_detection
python main.py           :: 或 main.bat
```

## 6. 预期输出

```text
[lidar] 已连接 127.0.0.1:2000 | 地图 Carla/Maps/Town10HD_Opt
[lidar] 主车出生于 spawn[11]
[lidar] 布置目标：车 6 辆（8~35 m 内的出生点，候选 6 个）
[lidar] scan 13:  43000 点 -> 检出 7 个目标（车 6 / 人 0）
[lidar] 最后一帧检出 7 个目标（车 6 / 人 0），60 m 内真值 车 6 / 人 0
[lidar] 结论：PASS（检测器稳定输出多目标）
```

## 7. 输出文件

| 文件 | 内容 |
|---|---|
| `docs/bev_detection.png` | 俯视点云 + 每个目标的检测框与「类别/距离」标注 |
| `docs/detections.csv` | 最后一帧目标表：实例号、类别、质心、包围盒、点数、距离 |
| `docs/scans.csv` | 14 帧统计：点数、检出数、车辆数、行人数 |
| `docs/summary.txt` | 召回率、平均每帧检出数等摘要 |

## 8. 目录结构

```text
carla_lidar_obstacle_detection/
├── main.py
├── main.sh / main.bat
├── launch/main.launch
├── config/params.yaml
├── docs/
└── README.md
```

## 9. 已知限制

1. **依赖语义激光雷达的实例号**：普通 `ray_cast` 没有实例信息，需要另做欧氏聚类。
2. 水平角分辨率有限（一圈只有几十个点），单帧对车的召回率不稳定，必须靠多帧累积；实测累积 5 帧后 60 m 内召回率可达 1.0。
3. 没有做跟踪（ID 保持），只做单帧检测统计。
