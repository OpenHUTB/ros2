# carla_ttc_emergency_braking — 自动紧急制动（AEB）实验

## 1. 概述

主车以约 50 km/h 巡航逼近一辆静止前车，程序基于前向目标跟踪计算 **TTC（Time To Collision）**，
在 TTC ≤ 1.6 s 或距离 ≤ 22 m 时输出满制动，最终**停在前车之前不发生碰撞**。
这是自动驾驶安全功能里最基础、也最容易量化的一个闭环：感知 → 决策 → 控制 → 验证。

## 2. 与已有示例的区别

| 对比项 | `PythonAPI/examples/` 里的官方示例 | 本模块 |
|---|---|---|
| 世界与地图 | 手工 `spawn_actor` 摆一个车就结束 | 自动挑选「够直且不穿路口」的路段，主车/前车按车道几何对齐摆放 |
| 传感器 | 相机、激光雷达各写各的 demo | 前向目标跟踪（雷达式）+ 激光雷达独立复核 + RGB 关键帧取证，三路同时工作 |
| 控制 | 大多只发一次 `VehicleControl` | 20 Hz 闭环：巡航 → TTC 判据触发 → 满制动 → 停稳判定 |
| 结论 | 无 | 输出逐帧 CSV + 三联曲线 + `PASS/FAIL` 判定 |

## 3. 环境要求

- Windows 10/11 + **OpenHUTB 2.10.1**（服务端版本字符串 `3ea2130ed`，内核基于 CARLA 0.9.16）
- **Python 3.10**（系统解释器即可）：`numpy`、`matplotlib`、`PyYAML`
- 客户端：`hutb-2.10.1-cp310-cp310-win_amd64.whl`（随仿真器一起分发，**不需要联网下载**）

> ⚠️ 两个必踩的坑：
> 1. `carla==0.9.13`（cp38）连这个服务端会**段错误**，`0.9.15` 会 `std::bad_alloc`，必须用配套的 hutb 客户端或 `carla==0.9.16`。
> 2. 客户端必须先 `client.set_files_base_folder(...)` 再 `get_world()`，否则只会抛一条没有信息的 `RuntimeError`（OpenHUTB 2.10.x 的地图文件按需下载机制）。

## 4. 安装

```bat
:: Windows（wheel 就在仿真器目录里）
python -m pip install D:\hutb\PythonAPI\carla\dist\hutb-2.10.1-cp310-cp310-win_amd64.whl
python -m pip install numpy matplotlib pyyaml
```

## 5. 运行

```bat
:: 终端 1：启动服务端（不要带 settings.json 里的 SimMode:Multirotor，否则会进无人机模式）
D:\hutb\CarlaUE4.exe Town10HD_Opt -windowed -carla-rpc-port=2000

:: 终端 2：跑本实验
cd src\ground\carla_ttc_emergency_braking
python main.py                 :: 或 main.bat / bash main.sh
:: 可选参数：--host 192.168.29.1 --port 2000 --map Town10HD_Opt --no-render
```

## 6. 预期输出

```text
[aeb] 已连接 127.0.0.1:2000 | 服务端版本 3ea2130ed | 地图 Carla/Maps/Town10HD_Opt
[aeb] 自动选择直路段：spawn[75]（方向偏差 0.0°）
[aeb] 前车 vehicle.audi.tt 位于前方 110.5 m
[aeb] 开始实验：巡航油门 0.75，TTC 阈值 1.60s
[aeb] >>> 触发制动 t=4.26s dist=21.3m speed=49.0km/h ttc=1.56s
[aeb] >>> 停稳 t=6.71s 距前车 5.0 m（地面真值 5.0 m）
[aeb] 结论：PASS（制动成功，未发生碰撞）
```

## 7. 输出文件

| 文件 | 内容 |
|---|---|
| `docs/run_log.csv` | 逐帧 `t / speed_kmh / distance_m / lidar_m / gt_distance_m / ttc / braking` |
| `docs/result.png` | 车速、前车距离、TTC 三联曲线（标出阈值线与停稳距离） |
| `docs/frame_*.png` | 关键帧：逼近 / 触发制动 / 停稳 |
| `docs/summary.txt` | 实验摘要（初始距离、制动时刻、停稳距离、最近距离） |

## 8. 目录结构

```text
carla_ttc_emergency_braking/
├── main.py              # 主程序：感知 + 决策 + 控制 + 取证
├── main.sh / main.bat   # 入口（自动挑解释器）
├── launch/main.launch   # ROS1 包装（直连，不需要 bridge）
├── config/params.yaml   # 全部可调参数
├── docs/                # 运行证据（曲线、CSV、关键帧）
└── README.md
```

## 9. 已知限制

1. 前车是**静止**的：没有做相对速度估计，因此 TTC 用自车速度近似；若要跟驰行驶车辆，需要加上目标速度跟踪。
2. 感知用的是「目标列表 + 激光雷达」两层，还没有接入真实视觉检测（YOLO 之类），换到没有真值的目标列表时不可用。
3. 制动是固定 `brake=1.0` 的开关式控制，没有做舒适性（减速度限幅）优化。
