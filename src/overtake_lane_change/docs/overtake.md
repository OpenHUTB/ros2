# 超车变道（Overtake）设计文档

## 1. 任务背景
在 ROS 课程作业中，前两次作业分别实现了自动泊车（hybrid A*）与红绿灯启停、ACC 自适应巡航（纵向跟车）。
本次作业选取**超车变道**方向：车辆在高速巡航中遇到慢速前车时，自动完成
"打灯 → 变道 → 超越 → 变回 → 恢复巡航" 的完整驾驶行为，属于典型的**横向 + 纵向联合控制**场景。

## 2. 场景建模
- 三车道直线道路，单车道宽 `lane_width = 3.5 m`，车道中心线位于 `y ∈ {-3.5, 0, 3.5}`；
- 前车（红色）：`x = 80 m` 起，恒速 `8 m/s`（模拟慢车/大车）；
- 本车（蓝色）：`x = 0 m` 起，巡航速度 `15 m/s`；
- 车辆尺寸：长 `4.5 m`、宽 `1.8 m`；所有车距语义为**保险杠间距**（中心距 − 车长）。

## 3. 控制律设计

### 3.1 状态机
```
CRUISE --(保险杠间距 < 40m)--> SIGNAL --(打灯 1s)--> CHANGING
CHANGING --(横向到位 |Δy|<0.08)--> OVERTAKING --(超过前车 15m)--> CHANGING_BACK
CHANGING_BACK --(回到原车道)--> CRUISE
```

### 3.2 纵向控制
- 速度比例控制：`a = k_v · (v_target − v)`，限幅 `a ∈ [−4.0, 2.5] m/s²`；
- 各状态目标速度：
  - `CRUISE`：`v_cruise = 15 m/s`；
  - `SIGNAL / CHANGING`：跟住前车（`v_target = v_lead`），保持间距不缩小；
  - `OVERTAKING / CHANGING_BACK`：`v_overtake = 17 m/s` 加速超越；
  - 回到原车道后恢复 `15 m/s`。

### 3.3 横向控制
- 比例控制横向速度：`v_y = k_lat · (y_target − y)`，限幅 `|v_y| ≤ 2.0 m/s`；
- 目标横向位置：超车道 `y_target = 3.5`，原车道 `y_target = 0`；
- 到位判定 `|y − y_target| < 0.08 m` 后锁定车道，横向速度归零，避免过冲。

### 3.4 安全兜底
任何状态下，若前车保险杠间距 `< 2.0 m`，立即以最大减速度 `d_max = 4.0 m/s²` 制动，
保证仿真全程不发生碰撞。

## 4. 参数表
| 参数 | 值 | 说明 |
|---|---|---|
| lane_width | 3.5 m | 车道宽度 |
| v_cruise | 15.0 m/s | 巡航速度 |
| v_overtake | 17.0 m/s | 超车速度 |
| trigger_dist | 40.0 m | 超车触发保险杠间距 |
| signal_time | 1.0 s | 打灯时长 |
| overtake_margin | 15.0 m | 超越后回道的纵向余量 |
| a_max / d_max | 2.5 / 4.0 m/s² | 加速/减速限幅 |
| v_lat_max | 2.0 m/s | 横向速度限幅 |
| car_len | 4.5 m | 车长 |

## 5. 仿真与验证
- `headless_test.py`：以 0.01s 步长跑 26s，自动检查 7 项指标（见 README 验证表），全部 PASS；
- `overtake_sim.py`：
  - 交互模式（pygame 窗口，ESC 退出）；
  - 录制模式 `--record demo.gif --duration 26`，仅依赖 Pillow（≥7.0），
    不使用 `rounded_rectangle`，兼容虚拟机旧版 Pillow；
- `main.py`：ROS Noetic 节点，订阅 `/lead_distance`、`/lead_speed`，发布 `/cmd_vel`。

## 6. 已知限制
- 仿真为二维（纵向 + 横向），未建模转向曲率与动力学（如侧滑）；
- 超车道空闲判断简化为场景假设（仅两车）；多车场景需增加目标车道占用检测；
- 前车匀速行驶，未处理前车加减速的交互博弈。
