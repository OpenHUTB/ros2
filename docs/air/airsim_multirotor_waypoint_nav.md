# 无人机航点飞行（AirSim）

> 模块路径：`src/air/airsim_multirotor_waypoint_nav` ｜ 载体：多旋翼无人机 ｜ 地图：Town10HD_Opt

## 实验目的

验证 OpenHUTB「空地一体」模式下 AirSim 接口的可用性，并实现一条可验收的航点飞行闭环：
起飞 → 方形航线 4 个航点 → 返回 → 降落；全程记录轨迹，逐点量化**三维**到达精度。

## 方法

1. **环境**：`D:\hutb\settings.json` 写入 `{"SettingsVersion":1.2,"SimMode":"Multirotor"}`；
   服务端启动后同一 UE4 进程内同时提供 CARLA RPC(2000) 与 AirSim RPC(41451)。
2. **接口**：`airsim.MultirotorClient`；`enableApiControl(True)` + `armDisarm(True)` 接管，结束交还。
3. **航点语义**：配置里是**相对起飞点**的东/北偏移；起飞后读取当前位置作为 `home`，
   再换算成 NED 绝对目标（`absolute_waypoints()`），换地图或出生点不用改配置。
4. **飞行**：`takeoffAsync` → `moveToZAsync(-h)` → 依次下发 `moveToPositionAsync`，
   **不 join**，由主循环边采样边判到达（单线程轮询，msgpackrpc 非线程安全，用线程会 BufferError）。
5. **验收**：
   - 到达判定：三维误差（含高度）≤ 2.0 m 且**连续 3 次采样**命中；
   - 到达精度：进入容差后 `hoverAsync` 并悬停 1.5 s，取误差中位数；
   - 稳态偏差修正：AirSim 内置位置控制有约 1.3 m 稳态偏差，按实测偏差反向补偿目标点（最多两次）；
   - 超时：单点 45 s、总时长 150 s；
   - PASS：4 个航点全部闭锁到达且最大三维误差 ≤ 0.6 m。
6. **采样**：全程 0.1 s 一点，`flight_log.csv` 记录 phase（wp/ settle / fix / return / landed）。

## 结果

| 航点（相对起飞点偏移） | 三维到达误差 | 是否闭锁 | 耗时 |
|---|---|---|---|
| (25, 0) | 0.53 m | 是 | 12.6 s |
| (25, 25) | 0.41 m | 是 | 12.7 s |
| (0, 25) | 0.45 m | 是 | 12.6 s |
| (0, 0) | 0.39 m | 是 | 12.6 s |

- 全程 73.0 s，采样 **459 点**（0.1 s 间隔），最大三维误差 **0.53 m** ≤ 0.6 m → **PASS**
- 轨迹见 `src/air/airsim_multirotor_waypoint_nav/docs/trajectory.png`，逐点数据见 `flight_log.csv`

## 结论

- 相对起飞点的航点语义让同一份配置在不同出生点/地图上可复用；
- 加入「悬停 + 稳态偏差补偿」后，AirSim 内置位置控制的约 1.3 m 悬停偏差被压到 0.4~0.5 m；
- 单线程轮询采样既满足「飞行中要有轨迹数据」，又避开了 msgpackrpc 的非线程安全问题。

## 局限与改进

1. SimpleFlight 内置飞控，未接 PX4/MAVLink，动力学真实度有限。
2. 航点固定、无避障；复杂空域需叠加规划器。
3. 位置精度依赖补偿，若需更高精度应自己实现位置外环（速度指令 + PI）。
4. 未接入机载相机取证，只有轨迹与状态数据。
