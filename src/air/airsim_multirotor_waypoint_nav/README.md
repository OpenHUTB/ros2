# airsim_multirotor_waypoint_nav — 无人机航点飞行实验

## 1. 概述

在 OpenHUTB 的空地一体模式下（同一个 UE4 进程里 CARLA 2000 + AirSim 41451），用 **AirSim RPC** 接管多旋翼无人机：
起飞 → 依次飞越 4 个航点（相对起飞点的 25 m×25 m 方形航线，6 m 高度）→ 返回 → 降落。
全程按 0.1 s 采样位置与速度，每个航点用**三维误差（含高度）**判定到达，并做一次稳态偏差修正。

## 2. 与已有示例的区别

| 对比项 | 官方示例 | 本模块 |
|---|---|---|
| 航点语义 | 直接给 NED 绝对坐标 | 配置里是**相对起飞点**的东/北偏移，起飞后记录 `home` 再换算，换地图/出生点可复用同一份参数 |
| 起控流程 | 直接 `takeoff` | `enableApiControl` / `armDisarm` 成对使用，结束交还控制权 |
| 到达判定 | 只看 `join()` 返回 | 三维误差（含高度）+ 连续 3 次命中才算到达；单点超时、总时长上限 |
| 到达精度 | 未定义 | 进入容差后悬停 `settle_time` 秒取中位误差，并对 AirSim 位置控制的稳态偏差做最多两次反向补偿 |
| 采样 | 飞行间隙不记录（原实现只有 7 行日志） | 单线程轮询，全程 0.1 s 一点（本次 459 行） |
| 取证 | 无 | 3D 轨迹 + 俯视图 + 逐点 CSV + summary |

## 3. 环境要求

- Windows + OpenHUTB 2.10.1（**必须带 AirSim 插件**的打包版）
- `D:\hutb\settings.json`：`{"SettingsVersion": 1.2, "SimMode": "Multirotor"}`
  （包里自带 `settings_air.json`，复制成 `settings.json` 即可；地面实验要把它移走）
- Python 3.10+；AirSim 官方/配套 Python 客户端

> **依赖版本敏感**（不满足会直接抛异常，与代码无关）：
> - `msgpack<1.0`：msgpack>=1 去掉了 `Packer(encoding=...)`；
> - `tornado<5`：tornado>=5 去掉了 `tornado.platform.auto` 与旧 IOLoop API；
> - 因此不能把 tornado 打进仓库，只能按版本约束用 pip 安装（见第 4 节）。

## 4. 安装

```bash
# 建议独立虚拟环境，避免把 tornado 4.x 装进系统 Python（会破坏 Jupyter）
python -m venv .venv-air && . .venv-air/Scripts/activate    # Windows
pip install numpy matplotlib pyyaml
pip install msgpack-rpc-python "msgpack<1.0" "tornado<5"
pip install <AirSim 客户端>        # 例如从 OpenHUTB 包里取 airsim 包，或 pip install airsim
```

## 5. 运行

```bat
:: 终端 1：以空地一体模式启动（同时开 2000 与 41451）
copy D:\hutb\settings_air.json D:\hutb\settings.json
D:\hutb\CarlaUE4.exe Town10HD_Opt -windowed -carla-rpc-port=2000

:: 终端 2
cd src\air\airsim_multirotor_waypoint_nav
python main.py --host 127.0.0.1
:: 或 main.bat / bash main.sh；ROS 2 下 ros2 launch airsim_multirotor_waypoint_nav main.launch.py host:=<宿主机IP>
```

## 6. 预期输出

```text
[uav] AirSim 已连接 192.168.29.1:41451
[uav] 接管控制权
[uav] 起飞到 6.0 m
[uav] 起飞点 NED=(9.18, -5.33, -6.54)，航点为相对起飞点偏移 [(25.0, 0.0), (25.0, 25.0), (0.0, 25.0), (0.0, 0.0)]
[uav] 飞往航点 1/4：相对起飞点偏移 (25.0, 0.0)
[uav] 航点 1 三维到达误差 0.53 m（容差 2.0 m），已闭锁到达，耗时 12.6 s
...
[uav] 最大三维到达误差 0.53 m（阈值 0.60 m），全程 73.0 s，采样 459 点
[uav] 结论：PASS（航点全部到达）
```

## 7. 输出文件

| 文件 | 内容 |
|---|---|
| `docs/trajectory.png` | 3D 轨迹（含起飞点与航点）+ 俯视图（标最大三维误差） |
| `docs/flight_log.csv` | 全程逐点：`t / phase / x,y,z / vx,vy,vz / err`（phase 标明 wp、settle、fix、return、landed） |
| `docs/summary.md` | 航点偏移、各点三维误差与耗时、闭锁情况、采样点数、PASS 阈值 |

## 8. 目录结构

```text
airsim_multirotor_waypoint_nav/
├── main.py                  # 便捷入口（等价于 ros2 run <包> <包>）
├── main.sh / main.bat
├── package.xml / setup.py / setup.cfg
├── airsim_multirotor_waypoint_nav/
│   ├── __init__.py
│   └── app.py               # 连接 / 起飞 / 航点飞行 / 到达判定 / 采样 / 取证
├── launch/
│   ├── main.launch          # ROS1 风格（直连，不需要 bridge）
│   └── main.launch.py       # ROS2 launch（Node 调 console_scripts）
├── config/params.yaml       # 航点偏移、容差、采样、判据
├── resource/airsim_multirotor_waypoint_nav
├── test/test_airsim_multirotor_waypoint_nav.py
├── docs/
└── README.md
```

## 9. 已知限制

1. 用 SimpleFlight 内置飞控，未接 PX4/MAVLink，动力学真实度有限。
2. 航点固定、无避障；复杂空域需要叠加规划器。
3. **AirSim 内置位置控制在悬停时有约 1.3 m 稳态偏差**（实测），模块用「悬停 + 一次/两次反向补偿」压到 0.4~0.5 m；
   若需要更高精度，应改为自己写位置外环（速度指令 + PI）。
4. 采样在单线程里做：msgpackrpc 不是线程安全的，用后台线程并发 RPC 会触发 `BufferError`，因此没有用多线程。
5. CARLA 取证截图只在安装配套 hutb 客户端时执行（上游 carla 客户端与 OpenHUTB 2.10.x 协议不兼容，会 abort）。
