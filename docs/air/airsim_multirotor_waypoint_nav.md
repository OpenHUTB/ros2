# 无人机航点飞行（AirSim）

> 模块路径：`src/air/airsim_multirotor_waypoint_nav` ｜ 载体：多旋翼无人机 ｜ 地图：Town10HD_Opt

## 实验目的

验证 OpenHUTB「空地一体」模式下 AirSim 接口的可用性，并实现一条可验收的航点飞行闭环：
起飞 → 方形航线 4 个航点 → 返回 → 降落，逐点量化到达误差。

## 方法

1. **环境**：`D:\hutb\settings.json` 写入 `{"SettingsVersion":1.2,"SimMode":"Multirotor"}`，
   服务端启动后同一个 UE4 进程内同时提供 CARLA RPC(2000) 与 AirSim RPC(41451)。
2. **接口**：`airsim.MultirotorClient`；`enableApiControl(True)` + `armDisarm(True)` 接管，
   飞行结束 `armDisarm(False)` + `enableApiControl(False)` 交还。
3. **飞行**：`takeoffAsync` → `moveToZAsync(-6)` → 依次 `moveToPositionAsync` 飞 4 个航点
   （(25,0) → (25,25) → (0,25) → (0,0)，NED 坐标，6 m 高，4 m/s）→ 回原点 → `landAsync`。
4. **验收**：每个航点 `.join()` 之后继续采样，直到进入 2 m 容差或超时，
   记录真实到达误差与耗时；输出 PASS/FAIL。

## 结果

| 航点 | 坐标 (x,y) | 到达误差 | 耗时 |
|---|---|---|---|
| W1 | (25, 0) | 0.11 m | 7.3 s |
| W2 | (25, 25) | 0.17 m | 7.5 s |
| W3 | (0, 25) | 0.17 m | 7.5 s |
| W4 | (0, 0) | 0.13 m | 7.5 s |

全程 32.0 s，最大到达误差 **0.17 m**，判定 PASS。
轨迹图见 `src/air/airsim_multirotor_waypoint_nav/docs/trajectory.png`。

## 结论

AirSim 的速度控制接口在 OpenHUTB 打包版上工作正常，方形航线四个角的到达误差均在 0.2 m 以内，
远小于 2 m 的验收容差；说明「起飞—航点—降落」这条链路可以直接作为后续任务（编队、避障、RL）的基座。

## 局限与改进

1. 使用 SimpleFlight 内置飞控，未接 PX4/MAVLink，动力学真实度有限。
2. 航点固定且无避障；复杂空域需要叠加栅格地图/规划器。
3. 未接入机载相机取证，只有轨迹与状态数据。
