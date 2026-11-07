# airsim_multirotor_waypoint_nav — 无人机航点飞行实验

## 1. 概述

在 OpenHUTB 的「空地一体」（CarlaAir）模式下，用 **AirSim RPC** 接管多旋翼无人机：
起飞 → 依次飞越 4 个航点（25 m×25 m 方形航线，6 m 高度）→ 返回起点 → 降落，
全程记录 NED 位置与速度，最后输出 3D 轨迹、到达误差与耗时。

## 2. 与已有示例的区别

| 对比项 | 官方示例 | 本模块 |
|---|---|---|
| 起控流程 | 直接 `takeoff` 了事 | 显式 `enableApiControl` / `armDisarm`，飞行结束 disarm，避免残留接管状态 |
| 到达判定 | 只 `.join()` 认为到了 | 逐点验收：按 2 m 容差循环采样，直到进入容差或超时，记录**真实到达误差** |
| 取证 | 无 | 3D 轨迹 + 俯视图 + 逐点误差表 + NED 逐帧 CSV |
| 环境 | 假设 AirSim 独立运行 | 处理了 OpenHUTB 特有的「同一个 UE4 进程里 CARLA(2000) + AirSim(41451)」双端口结构 |

## 3. 环境要求

- Windows + OpenHUTB 2.10.1（**必须带 AirSim 插件**，即 `D:\hutb\CarlaUE4\Plugins\HoloOcean`/AirSim 齐全的打包版）
- `D:\hutb\settings.json` 内容为：
  ```json
  {"SettingsVersion": 1.2, "SimMode": "Multirotor"}
  ```
  包里自带 `settings_air.json`，复制成 `settings.json` 即可；**地面实验要把它移走**。
- Python 3.10 + `hutb-2.10.1` 客户端 wheel（内含 `airsim` 客户端）+ `msgpack-rpc-python`
- `msgpack-rpc-python` 只兼容 `tornado<5`，而系统 Python 是 tornado 6（Jupyter 需要）：
  本模块把 **tornado 4.5.3 单独放在 `deps/`**，由 `main.py` 插到 `sys.path` 最前面，互不干扰。

## 4. 安装

```bat
python -m pip install D:\hutb\PythonAPI\carla\dist\hutb-2.10.1-cp310-cp310-win_amd64.whl
python -m pip install msgpack-rpc-python
:: deps/tornado(4.5.3) 已随本模块提供，无需再装
```

## 5. 运行

```bat
:: 终端 1：以空地一体模式启动（服务端会同时开 2000 与 41451）
copy D:\hutb\settings_air.json D:\hutb\settings.json
D:\hutb\CarlaUE4.exe Town10HD_Opt -windowed -carla-rpc-port=2000

:: 终端 2
cd src\air\airsim_multirotor_waypoint_nav
python main.py            :: 或 main.bat / bash main.sh
```

## 6. 预期输出

```text
[uav] AirSim 已连接 127.0.0.1:41451
[uav] 接管控制权
[uav] 起飞到 6.0 m
[uav] 飞往航点 1/4 (25.0, 0.0)
[uav] 航点 1 到达误差 0.11 m，耗时 7.3 s
[uav] 航点 2 到达误差 0.17 m，耗时 7.5 s
[uav] 航点 3 到达误差 0.17 m，耗时 7.5 s
[uav] 航点 4 到达误差 0.13 m，耗时 7.5 s
[uav] 返回起点上方并降落
[uav] 最大到达误差 0.17 m，全程 32.0 s
[uav] 结论：PASS（航点全部到达）
```

## 7. 输出文件

| 文件 | 内容 |
|---|---|
| `docs/trajectory.png` | 3D 轨迹（含航点标注）+ 俯视图（含最大到达误差） |
| `docs/flight_log.csv` | 逐帧 NED 位置 `x/y/z` 与速度 `vx/vy/vz`，带阶段标记 |
| `docs/summary.txt` | 各航点误差/耗时、全程时长、采样点数 |

## 8. 目录结构

```text
airsim_multirotor_waypoint_nav/
├── main.py
├── main.sh / main.bat
├── config/params.yaml   # 航点、速度、容差、超时
├── deps/tornado/        # 仅本模块使用的 tornado 4.5.3
├── docs/
└── README.md
```

## 9. 已知限制

1. 依赖 AirSim 的 `SimpleFlight` 内置飞控，没有接 PX4/MAVLink，因此不支持更真实的动力学。
2. 航点是相对起飞点的固定坐标，没有做避障；城中飞行会穿模。
3. 只在 Town10HD_Opt 上验证；换地图需确认空域足够。
