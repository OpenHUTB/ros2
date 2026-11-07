# uav_delivery_route — 末端物流多站点配送与换电调度

One depot serves 10–14 farm stations; the UAV plans batched delivery sorties
under a 2.5 kg payload cap and an 18-min battery, swaps batteries at the depot,
and the first sortie is re-flown in MuJoCo to check what the plan is worth in
the real world. 一个配送中心 + 10~14 个收发站的末端物流场景：在 2.5 kg 载重
与 18 min 续航约束下把包裹划分成若干架次，电量不足时回中心换电（90 s），
每站投递悬停 20 s；第一个架次用 MuJoCo 真飞复核计划质量。

## Quick Start

```bash
# 依赖：Python 3.10+，numpy / mujoco / matplotlib / pytest 已装好；
# ROS 2 环境下可 colcon build，纯算法层无需安装即可直接运行
cd src/air/uav_delivery_route
python main.py --no-sim        # 秒级，仅算法层
python main.py                 # 三个 case 全跑，含 MuJoCo 闭环真飞（约 40 s 墙钟）
python main.py --case case_farm_sparse
python -m pytest test/ -q      # 25 项测试全过
python tools/make_figures.py   # 从 results/*.json 生成 images/batches.png
```

结果自动写入 `results/<case>_result.json`；ROS 节点入口是
`ros2 run uav_delivery_route dispatch`（或 `python -m uav_delivery_route.ros.dispatch_node`），
launch 文件在 [launch/main.launch.py](launch/main.launch.py)，
关键参数集中在 [config/uav_delivery_route.yaml](config/uav_delivery_route.yaml)。

## Background

农网/乡村末端配送的特点是站点多而散、单件重量差异大、无人机续航有限。
这个问题里真正难的不是"把 12 个点串起来"，而是三件事互相耦合：
载重上限决定了单次能带多少包裹，电量上限决定了单次能飞多远，而能耗又
随载重变化——带得越多飞得越费电。规划器必须同时把两本账算在一起，
否则会出现"载重允许但电量回不来"的航迹。换电点固定在配送中心，
换一次 90 s，架次划分越碎、换电开销越大，这里存在一个天然的权衡。

## Design

核心思路是"分簇定序、电量切批、真飞复核"，全部实现在
[uav_delivery_route/core/](uav_delivery_route/core/)（只依赖 numpy，无 ROS 依赖）：

- **能耗模型**（[core/energy.py](uav_delivery_route/core/energy.py)）：
  功率对载重线性 `P = 40 W + 18 W/kg × m`，满载 2.5 kg 时 85 W，
  电池容量由"满载巡航 18 min"反推为 91800 J ≈ 25.5 Wh。
  剩余电量低于 **1.25 × 返航所需** 即触发换电决策（安全系数写在
  `RETURN_MARGIN`）。
- **航线构造**（[core/planner.py](uav_delivery_route/core/planner.py)）：
  先把站点按相对配送中心的方位角扇区预分组（`n_sectors`，默认 3），
  簇内最近邻串接成总序列，再做电量/载重双判据切架次——任一站装不下
  或"飞过去 + 悬停后剩余 < 1.25 × 返航所需"就封口回中心换电；
  切批前对簇内序列做 2-opt 改进。输出架次划分、总时长（含换电与悬停）。
- **校验**（[core/checker.py](uav_delivery_route/core/checker.py)）：
  逐架次查续航能耗不超电池容量、任意时刻载重不超 2.5 kg、
  所有站恰好服务一次，输出违反清单——正常应为空。
- **闭环**（[core/flyer.py](uav_delivery_route/core/flyer.py)）：
  抽第一个架次在课程底座
  [uav_mujoco_sim](../uav_mujoco_sim/README.md)（同一份 quadrotor.xml +
  GeometricController）里真飞，参考速度前馈加一阶低通（τ=0.5 s）、
  姿态阻尼 kw=-0.18 抑制分段航迹振荡。统计**刚体积分出来的真实位置**：
  每段实际航程、到站误差（取该站悬停结束时刻）、全程耗时。
- **在线调度**（[ros/dispatch_node.py](uav_delivery_route/ros/dispatch_node.py)）：
  薄壳节点，订阅 odom 与电量话题，判定逻辑复用 core 的 `decide_next`，
  以 JSON 发布下一目标站与建议换电时机到 `/uav_delivery/dispatch`。

三个 case 都在 [core/cases.py](uav_delivery_route/core/cases.py) 里用固定坐标（可复现）：
`case_farm_dense`（站近包裹轻，单架次）、`case_farm_sparse`（0.8–1.6 km 远，
多架次 + 换电）、`case_farm_heavy`（5 件 2.2 kg+ 的近上限重包裹必须一票一架次）。

## Verification

`python main.py` 一次跑完三个 case（闭环墙钟合计约 38 s），checker 违反清单全为空：

- case_farm_dense —— 12 站，1 架次 0 换电
  - 计划：总航程 2638.95 m，任务总时长 533.22 s
  - 真飞第 1 架次：仿真 535.72 s（墙钟 21.2 s），实际航程 2683.39 m
    （比计划多 44.4 m，+1.7%），平均到站误差 0.059 m，最大 0.767 m
- case_farm_sparse —— 12 站，6 架次 5 次换电
  - 计划：总航程 22275.05 m，任务总时长 3165.01 s
  - 真飞第 1 架次（3221.58 m 计划）：仿真 400.45 s（墙钟 14.4 s），
    实际航程 3231.87 m（+10.3 m，+0.3%），平均到站误差 0.261 m，最大 0.782 m
- case_farm_heavy —— 12 站，9 架次 8 次换电（5 件重包裹各占一架次）
  - 计划：总航程 10593.39 m，任务总时长 2137.04 s
  - 真飞第 1 架次（538.52 m 计划）：仿真 82.34 s（墙钟 2.7 s），
    实际航程 545.33 m（+6.8 m，+1.3%），平均到站误差 0.383 m，最大 0.767 m

实际航程普遍略高于计划：几何控制器跟踪分段航迹存在微小超调与绕弯，
量级在 0.3%–1.7%，说明计划层能耗/航程账目与真实飞行一致。

测试（`python -m pytest test/ -q`，**25 项全过**）按 feature 分三个文件：
[test_planner_basic.py](test/test_planner_basic.py)（分簇/最近邻/2-opt/切批/覆盖）、
[test_edge_cases.py](test/test_edge_cases.py)（能耗线性、载重上限、超续航站、换电判据、调度决策）、
[test_fly_smoke.py](test/test_fly_smoke.py)（MuJoCo 小规模冒烟，墙钟 < 3 s）。
架次划分与各架次航程对比图见
[images/batches.png](images/batches.png)，由
[tools/make_figures.py](tools/make_figures.py) 从 results JSON 生成。

## Limitations

- 能耗模型是线性近似，只计载重对功率的一阶影响，未建模风速、温度与
  电池放电曲线；1.25 的返航安全系数吸收了这部分误差，但长航线场景
  建议实测标定 `p_base_w` 与 `load_coef_w_per_kg`。
- 换电点固定在配送中心，未考虑分布式换电柜；架次划分对 `n_sectors`
  与站点几何敏感，极不均匀的扇区分布可能产生长尾架次。
- 闭环只复飞每个 case 的第一个架次，其余架次仍是计划层结论；
  多机协同配送、动态插单与风扰下的重规划不在本模块范围内。
- ROS 节点按单机编排，odom 与电量话题需外部仿真或真机提供。
