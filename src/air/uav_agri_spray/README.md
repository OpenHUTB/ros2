# uav_agri_spray — 植保喷洒作业覆盖与参数评估

## Quick Start

```bash
# pip 依赖
pip install numpy mujoco matplotlib pyyaml

# 算法层，秒级，不需要 MuJoCo
python main.py --no-sim

# 全流程：三个 case + 前 6 条扫描线 MuJoCo 真飞闭环
python main.py

# 单个 case
python main.py --case case_field_obstacle
```

源码目录直接运行即可，`main.py` 会自动把同级
[uav_mujoco_sim](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_mujoco_sim)
加入 `sys.path`；装进工作空间后用 `ros2 run uav_agri_spray main` 等价。

colcon 编译：

```bash
colcon build --packages-select uav_agri_spray
source install/setup.bash
ros2 launch uav_agri_spray main.launch.py     # 在线作业进度节点
```

三个运行模式分别是 `--no-sim`（纯算法、秒级）、`--case <name>`（单场景）、
无参数（三场景 + 闭环）。结果自动写入 `results/uav_agri_spray_result.json`，
图写入 `images/`。

## Background

植保无人机按固定喷幅往返作业，田块边界、电线杆与树会把"理想等间距航线"
打乱；喷幅 3 m、重叠率 20% 时相邻行间沉积能不能压到阈值以上，靠拍脑袋
定参数往往要么漏喷、要么白飞。本模块把三件事串起来：

1. 用旋转卡壳求凸多边形最小宽方向，沿该方向布置往返扫描线，行数最少、
   转弯最少；障碍按外扩 5 m 的圆形边界走劣弧绕行，行间转入用半圆弧；
2. 在 1 m 栅格上把每条扫描线当成横向高斯沉积源（σ = 喷幅/4）叠加，
   统计覆盖率、重叠率与漏喷面积；
3. 在喷幅固定（3 m）的前提下扫描 速度 3/4/5 m/s × 重叠率 10/20/30%，
   给出"覆盖率-时长"权衡表并推荐参数；最后抽推荐参数下前 6 条扫描线在
   MuJoCo 里真飞，用真实位置重算沉积，检验计划与现实的偏差。

三个基准田块：50 m×36 m 的矩形田（`case_field_rect`）、56 m×40 m 带电线杆
与树的矩形田（`case_field_obstacle`）、L 形田块拆成两个凸子区
（`case_field_lshape`，南臂 50 m×16 m + 北臂 24 m×24 m）。

## Design

代码按"纯函数 + dataclass 配置"组织，`core/` 只依赖 numpy：

- `core/geometry.py`：多边形面积、点在多边形内、旋转卡壳最小宽方向、
  点到线段距离、直线与凸多边形求交；
- `core/planner.py`：`SprayConfig`（喷幅/重叠率/速度/高度/绕行距离）→
  往返扫描线、障碍绕行弧、半圆转入弧；输出 `SprayPath`（稠密航点 +
  每段所属行号 + 总航程/时长）；`progress_state()` 给 ROS 节点算当前行号
  与已作业面积百分比；
- `core/checker.py`：1 m 栅格离散、按线段/弧段叠加高斯沉积、覆盖率
  （沉积 ≥ 0.5）、重叠率（叠加剂量 ≥ 1.0）、漏喷面积；
- `core/sweep.py`：9 组参数组合的权衡表，覆盖率 ≥ 98% 且时长最短者推荐；
- `core/flyer.py`：MuJoCo 真飞前 N 条扫描线。参考速度前馈经一阶低通
  （τ = 0.5 s）、姿态阻尼 kw = −0.18 抑制分段航迹激励的振荡，记录刚体
  真实位置并按 0.3 m 重采样成加权点源重算实际沉积；
- `ros/spray_node.py`：订阅 `/odom`，2 Hz 发布
  `/uav_agri_spray/status`（JSON：`line_id` / `area_percent` / `position`），
  判定逻辑全部在 `core.planner.progress_state`。

规划约束：扫描线间距 = 喷幅 ×(1 − 重叠率)；行内遇障碍用半径
`r + 5 m` 的圆上劣弧绕行（恒距障碍中心，天然满足安全间隙）；行间转入用
直径 = 有效间距的半圆，落在田块边界之外。

## Verification

`python main.py`（含 MuJoCo 闭环）全流程实测 5.8 s 墙钟，其中真飞
6843 步 / 4.5 s。以下数字与 `results/uav_agri_spray_result.json` 逐项一致。

规划与沉积（默认 4 m/s、20% 重叠，喷幅 3 m → 间距 2.40 m）：

```
case_field_rect        田块 1800.0 m2  15 条扫描线  航程 802.7 m  时长 200.7 s
                       覆盖率 100.0%   重叠率 16.7%  漏喷 0.0 m2   平均沉积 0.779
case_field_obstacle    田块 2240.0 m2  17 条扫描线  航程 1039.3 m 时长 259.8 s
                       覆盖率 94.2%    重叠率 20.3%  漏喷 130.0 m2 平均沉积 1.020
case_field_lshape      田块 1376.0 m2  17 条扫描线  航程 695.4 m  时长 173.9 s
                       覆盖率 100.0%   重叠率 14.2%  漏喷 0.0 m2   平均沉积 0.800
```

障碍田块的 130.0 m² 漏喷是障碍外扩 5 m 的绕行圈本身：杆/树周围不允许喷洒，
这部分面积按规则不进入沉积统计之外的补救，提示实际调度时应把障碍周边
划成补喷区。

参数扫描（`case_field_rect`，时长为 3/4/5 m/s 三档）：

```
重叠 10%   覆盖率 61.1%   漏喷 700.0 m2   时长 234.0 / 175.5 / 140.4 s
重叠 20%   覆盖率 100.0%  漏喷   0.0 m2   时长 267.6 / 200.7 / 160.5 s
重叠 30%   覆盖率 100.0%  漏喷   0.0 m2   时长 301.0 / 225.8 / 180.6 s
推荐       5.0 m/s @ 20%  覆盖率 100.0%   时长 160.5 s
```

10% 重叠下相邻行中点叠加沉积只有 0.36（< 阈值 0.5），出现条带状漏喷；
20% 起中点叠加升到 0.56，覆盖率直接到 100%。推荐组合即"覆盖率 ≥ 98%
里时长最短"的那一档。障碍田块没有任何组合达到 98%（最好 94.4%），
兜底策略退化为"覆盖率最高、同覆盖率下时长最短"，落在 30% 重叠 5 m/s。

MuJoCo 闭环（抽 `case_field_rect` 推荐参数 5.0 m/s @ 20%，飞前 6 条扫描线）：

```
飞行航程 318.8 m   仿真 6843 步   墙钟 4.5 s
全程最大跟踪误差 2.18 m   终点位置误差 0.008 m
计划覆盖率 38.9%  →  真实覆盖率 36.8%   偏差 −2.1 个百分点
```

（覆盖率在整个 1800 m² 栅格上统计，前 6 条线只覆盖条带状区域，所以绝对值
偏低，关键是计划与真飞同口径可比。）真实覆盖率略低于计划：跟踪误差把
部分沉积推到条带外侧，边界上多出的剂量又补回一点；两者差 2.1 个百分点，
说明"按计划航迹喷"这个前提成立，但边界容差要在补喷策略里留余量。

测试：`python -m pytest test/ -q` → **31 项全过（约 1.3 s）**，含 1 项 MuJoCo
冒烟（小田块 12 m×8 m 飞 2 条线，墙钟 < 20 s、终点误差 < 1 m）。

图：`images/coverage_map.png`（沉积热力图 + 计划航线，英文标题）、
`images/coverage_map_case_field_obstacle.png`、
`images/coverage_map_case_field_lshape.png`、
`images/coverage_map_actual.png`（MuJoCo 真飞的实际沉积）。
`python tools/make_figures.py` 可从结果 JSON 重新生成全部图。

## Limitations

- 沉积模型是静态二维高斯叠加，没有风场、雾滴蒸发与高度衰减；真实作业的
  漂移会把沉积整体平移，覆盖率需要按风向再打折。
- 网格 1 m 分辨率与 σ = 0.75 m 同量级，个别组合（如 30% 重叠）的重叠率会
  受栅格相位影响，出现非单调的小幅波动；覆盖率判定不受影响。
- 障碍绕行只对圆形禁区建模，矩形棚舍/沟渠需要先做外接圆近似；绕行圈内的
  面积按"禁喷"处理，不做局部补喷规划。
- 闭环只飞推荐参数下的前 6 条扫描线（时间预算），全田真飞按 4.5 s / 6 条线
  外推约 11 s 墙钟，但未实测。
- 与同目录 [uav_formation_flight](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_formation_flight)
  （编队保持）和 [uav_mujoco_sim](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_mujoco_sim)
  （仿真底座）不同：本模块关心的是"地块怎么扫、喷得够不够、参数怎么选"，
  只用它们共用的四旋翼模型做闭环验证，不涉及多机协同与任务分配。
