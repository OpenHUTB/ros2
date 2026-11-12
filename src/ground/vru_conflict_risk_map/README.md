# vru_conflict_risk_map

根据自车与行人/儿童/骑行者轨迹生成冲突风险图：输出每条 VRU 的最小距离、
相对运动 TTC 和 `safe/caution/critical` 等级。

模块提供两种互补模式：

- `point`：默认，分析中心点距离与相对运动 TTC；
- `swept`：把车辆和 VRU 视为圆形碰撞体，分析最近接近点（CPA）、最小净空、
  碰撞时间和有限预测时域内的碰撞风险。

## 输入格式

`ego.csv`：

| 字段 | 单位 | 说明 |
|---|---|---|
| `timestamp` | s | 时间戳 |
| `x`, `y` | m | 自车位置 |
| `speed_mps` | m/s | 自车速度；TTC 方向由相邻位置差分确定 |

`vru.csv`：

| 字段 | 说明 |
|---|---|
| `track_id` | VRU 轨迹 ID |
| `class` | `pedestrian` / `child` / `cyclist` |
| `timestamp` | 时间戳，单位 s |
| `x`, `y` | VRU 位置，单位 m |

默认在 50 ms 容差内为每条 VRU 点匹配最近的自车点。未匹配点计入
`skipped_points`；整条轨迹都没有匹配点时计入 `skipped_tracks`。未知类别会
跳过并计入 `invalid_class_points`，不会让整份报告失败。时间轴排序后使用二分
查找，匹配复杂度为 O(N_vru log N_ego)。

## 指标定义

距离定义为匹配时刻自车与 VRU 的平面欧氏距离。速度由相邻轨迹点差分估计，
相对位置记作 `r = p_vru - p_ego`，相对速度记作 `v_rel`。只有当二者正在接近
（`r · v_rel < 0`）时才计算：

```text
TTC = |r|² / (-r · v_rel)
```

不接近时输出 `null`，因此侧向静止 VRU 不会被误判为近期冲突；自车停止时也能
通过 VRU 相对运动检测接近。

`min_distance_m` 与 `min_ttc_s` 分别可能来自不同采样帧，因此报告同时输出
`min_distance_timestamp_s`、`min_ttc_timestamp_s` 和首次达到最终等级的
`risk_timestamp_s`，避免把不同帧指标拼成一个不存在的状态。

| 风险等级 | 条件（满足任一） |
|---|---|
| `critical` | 距离 ≤ 3 m 或 TTC ≤ 2 s |
| `caution` | 距离 ≤ 6 m 或 TTC ≤ 4 s |
| `safe` | 其它有数据情况 |

## 运行

```bash
cd src/ground/vru_conflict_risk_map
python main.py
# 或指定文件：
python main.py --ego data/ego.csv --vru data/vru.csv --out report.json
# 碰撞体/净空分析：
python main.py --mode swept --ego data/ego.csv --vru data/vru.csv
# 生成同一场景的碰撞体与净空示意图（可选 matplotlib）：
python scripts/plot_swept_demo.py
```

默认读取 `data/ego.csv` 与 `data/vru.csv`，输出 `vru_conflict_report.json`。

随包样例包含行人、儿童和骑行者三条轨迹。`counts` 按每条最终轨迹的风险等级
去重计数；`point_counts` 才是逐采样点计数。所有 VRU 都因超时或未知类别被跳过
时，`overall_risk` 为 `no_data`，不会伪装成 `safe`。

## 碰撞体与 CPA 模式

`swept` 模式为自车和 VRU 配置圆形半径。默认自车半径 1.0 m，VRU 半径分别为
行人 0.30 m、儿童 0.25 m、骑行者 0.50 m，可通过 Python API 覆盖。

对相对位置 `r` 和相对速度 `v`：

```text
t_cpa = max(0, -(r·v) / |v|²)
CPA   = |r + v·t_cpa|
净空  = CPA - (自车半径 + VRU 半径)
```

若碰撞体已经重叠，碰撞时间记为 0；否则解析求解
`|r + v·t| = r_combined` 得到未来表面接触时间。默认预测时域为 5 s：

- 碰撞时间 ≤ 2 s：`critical`；
- 碰撞时间 ≤ 5 s：`caution`；
- 无碰撞但 CPA 净空 ≤ 1 m：`caution`；
- 其它为 `safe`。

轨迹输出包括 `combined_radius_m`、`min_clearance_m`、
`closest_approach_distance_m`、`closest_approach_time_s`、
`collision_ttc_s` 和 `collision_within_horizon`。其中
`min_clearance_m` 是预测时域内的最小净空；
`collision_ttc_s` 保留解析得到的原始碰撞时间；是否在预测时域内由
`collision_within_horizon` 明确表示。

`scripts/plot_swept_demo.py` 可读取同一 CSV 并输出碰撞体、净空与 TTC 标注图，
用于人工复核；绘图依赖不进入运行时必需依赖。

## 测试

```bash
python -m unittest discover -s test -p "test_*.py" -v
```

当前 23 项测试覆盖中心点距离/TTC、停止自车与侧向 VRU、碰撞体表面 TTC、
CPA/净空、轨迹去重计数、时间失配、未知类别、跨帧时间戳和随包样例。

## ROS 2

节点参数为 `ego_csv`、`vru_csv`、`analysis_mode`、`ego_radius`、
`caution_clearance` 和 `horizon`，启动后在 `~/report` 话题发布一次 JSON
`String` 并保持节点运行。模块定位为一次性离线轨迹评估，不提供实时重算：

```bash
ros2 launch vru_conflict_risk_map main.launch.py \
  ego_csv:=/path/to/ego.csv vru_csv:=/path/to/vru.csv \
  analysis_mode:=swept ego_radius:=1.0 horizon:=5.0
```
