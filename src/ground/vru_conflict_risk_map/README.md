# vru_conflict_risk_map

根据自车与行人/儿童/骑行者轨迹生成冲突风险图：输出每条 VRU 的最小距离、
相对运动 TTC 和 `safe/caution/critical` 等级。

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
跳过并计入 `invalid_class_points`，不会让整份报告失败。

## 指标定义

距离定义为匹配时刻自车与 VRU 的平面欧氏距离。速度由相邻轨迹点差分估计，
相对位置记作 `r = p_vru - p_ego`，相对速度记作 `v_rel`。只有当二者正在接近
（`r · v_rel < 0`）时才计算：

```text
TTC = |r|² / (-r · v_rel)
```

不接近时输出 `null`，因此侧向静止 VRU 不会被误判为近期冲突；自车停止时也能
通过 VRU 相对运动检测接近。

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
```

默认读取 `data/ego.csv` 与 `data/vru.csv`，输出 `vru_conflict_report.json`。

随包样例包含行人、儿童和骑行者三条轨迹。`counts` 按每条最终轨迹的风险等级
去重计数；`point_counts` 才是逐采样点计数。所有 VRU 都因超时或未知类别被跳过
时，`overall_risk` 为 `no_data`，不会伪装成 `safe`。

## 测试

```bash
python -m unittest discover -s test -p "test_*.py" -v
```

当前 11 项测试覆盖距离阈值、停止自车/迎面 VRU、侧向静止 VRU、轨迹去重计数、
时间失配、未知类别和随包样例。

## ROS 2

节点参数为 `ego_csv` 与 `vru_csv`，启动后在 `~/report` 话题发布一次 JSON
`String` 并保持节点运行。模块定位为一次性离线轨迹评估，不提供实时重算：

```bash
ros2 launch vru_conflict_risk_map main.launch.py \
  ego_csv:=/path/to/ego.csv vru_csv:=/path/to/vru.csv
```
