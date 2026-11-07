# uav_inspection_cover

输电走廊**无人机巡检覆盖规划**：给定走廊内一排杆塔（二维坐标 + 塔高），
为每座塔生成 2~3 个观察角度的观察点，规划一条从运维站出发、依次拍完所有
观察点的巡检航线，校验覆盖率，并取航线前 4 个航点段在 MuJoCo 里**闭环
真飞**，用刚体积分出的真实位置统计实际航程与到点误差。

> 仿真基于同仓库的 [uav_mujoco_sim](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_mujoco_sim)
> 底座（MuJoCo 刚体四旋翼 + 几何控制器），纯 CPU 运行，不需要 Gazebo。

---

## Quick Start

Windows 源码目录直接跑（不需要 colcon install，`main.py` 会自动挂上
同级的 `uav_mujoco_sim` 路径）：

```bash
# 1) 仅算法层（秒级，不需要 mujoco）
python main.py --no-sim

# 2) 单个 case 的 MuJoCo 闭环真飞
python main.py --case case_grid_short

# 3) 全部 3 个 case + 闭环（约 18 s 墙钟）
python main.py
```

Linux/macOS 用 `./main.sh`，Windows 也可以双击 `main.bat`，参数一致。

编译与安装（ROS 2 环境）：

```bash
colcon build --packages-select uav_inspection_cover
source install/setup.bash
ros2 run uav_inspection_cover main --no-sim      # 算法层
ros2 launch uav_inspection_cover main.launch.py  # 在线进度节点
```

测试：

```bash
python -m pytest test/ -q
# 28 passed in ~6 s（含 2 个 MuJoCo 冒烟测试，各 < 10 s）
```

运行结果自动写入 `results/inspection_result.json`，配图由
`tools/make_figures.py` 从这份 JSON 生成到 `images/route_compare.png`，
README 里的所有数字都与 JSON 一致。

## Background

输电走廊巡检的实际痛点：杆塔多、走廊长，运维无人机如果按"想到哪飞到哪"
的方式飞，会有两类浪费——塔间访问顺序不合理导致的**折返航程**，以及
河流 / 交跨等**不可飞区**逼出来的临时绕行。已有的
[uav_mujoco_sim](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_mujoco_sim)
解决了"怎么飞得稳"（单机物理仿真），没有回答"这条巡检航线值不值得飞、
能不能飞、覆盖全不全"。本模块补的就是这一层：**先算出一条可飞、短、
全覆盖的航线，再用真物理仿真证明它真的飞得下来**。

场景描述（三个 case，均为双排网格走廊）：

```
case_grid_short   6 座塔，运维站在 (300, 220)，无禁飞区
case_grid_long    12 座塔，运维站在 (0, 220)，无禁飞区
case_grid_river   10 座塔，运维站在 (-100, 0)，河道禁飞区 (280,80)-(700,140)
```

每座塔按高度取 2~3 个观察角度（塔高 ≥32 m 取 3 个），观察点 = 塔位 +
60 m 方位角偏移 + 0.7 塔高的观察高度。

## Design

算法分层全部在 `core/`（纯 numpy，无 ROS 依赖）：

- `core/geometry.py` — 观察点生成与可飞性检查。航段到塔身的二维点到
  线段距离 ≥25 m 判定安全；禁飞区按线段-矩形距离 ≥15 m 判定。
- `core/planner.py` — 规划分层：最近邻构造塔访问序 → 2-opt 改进（代价
  函数是**整条航线的实际航程**，含串接与绕行，只有总航程严格下降才
  接受）；塔内观察点按到达角（进入方向为零点、逆时针角度差升序）串接；
  遇不可飞段先在可飞图（40 m 方格网，节点/边都过安全检查）上跑
  Dijkstra，再对整段拼接点做视线简化，去掉微段与锯齿。预计时长 =
  航程 / 8 m/s + 每观察点悬停 6 s。
- `core/checker.py` — 覆盖校验：航路折线进入观察点 15 m 球即算覆盖，
  输出覆盖率与漏检清单。
- `core/flyer.py` — MuJoCo 闭环：取前 4 个航点段，位置目标沿时间线
  推进（转角越大越减速、观察点悬停 6 s），参考速度前馈经一阶低通
  （τ=0.5 s），姿态阻尼 kw=-0.18；统计全部来自刚体积分出的真实位置。
- `uav_inspection_cover/ros/inspect_node.py` — 在线进度节点（薄壳）：
  订阅 `nav_msgs/Odometry`，判定逻辑下沉 core 的纯函数
  `progress_snapshot`，把当前目标观察点下标、目标坐标与进度百分比以
  JSON 发布到 `/uav_inspection_cover/target`。

关键参数集中在 `config/inspect.yaml`（观察点偏移、安全距离、覆盖半径、
可飞图间距、闭环取段数等），`core/geometry.py` 的 `Params` 是同一套
数值的 dataclass 配置对象。

## Verification

`python main.py` 全量运行（3 个 case + 6 次闭环真飞）约 **18.3 s** 墙钟，
结果与 `results/inspection_result.json` 完全一致：

```
case_grid_short   6 塔   规划 2057.0 m / 353.1 s → 2-opt 改进 1873.6 m / 330.2 s（省 8.9%）
case_grid_long   12 塔   规划 4920.3 m / 807.0 s → 2-opt 改进 4096.5 m / 704.1 s（省 16.7%）
case_grid_river  10 塔   规划 4140.2 m / 673.5 s → 2-opt 改进 3679.7 m / 616.0 s（省 11.1%）
```

覆盖率（15 m 球判定，三个 case 改进前后均为）：

```
case_grid_short   100.0%，漏检 0
case_grid_long    100.0%，漏检 0
case_grid_river   100.0%，漏检 0（河道禁飞区由 Dijkstra 绕行保证）
```

MuJoCo 闭环真飞（各取改进前后航线前 4 个航点段）：

```
case_grid_short   基线: 计划 316.4 m → 实飞 330.3 m，到点误差 0.00 m
                  改进: 计划 424.3 m → 实飞 438.2 m，到点误差 0.00 m
case_grid_long    基线: 计划 708.6 m → 实飞 722.1 m，到点误差 0.00 m
                  改进: 计划 682.1 m → 实飞 695.5 m，到点误差 0.00 m
case_grid_river   基线: 计划 705.7 m → 实飞 719.2 m，到点误差 0.00 m
                  改进: 计划 618.0 m → 实飞 632.6 m，到点误差 0.00 m
```

实飞航程比计划长约 2~4%，来自转角处的横向超调，属于真实的
waypoint 飞行行为；到点误差 0.00 m 是因为参考时间线在最后一个航点后
保持 6 s（转角减速 + 末端收敛的模型见 `core/flyer.py`）。

测试 28 项全过，按 feature 组织在三个文件里（B 套风格，不与 core
子模块一一对应）：

```
test/test_planner_basic.py   13 项：观察点几何 / NN 置换性 / 2-opt 不回退且必改进 / 时长公式 / 进度推进
test/test_edge_cases.py      11 项：最终航线逐段可飞 / 河道绕行不出界 / 覆盖漏检清单 / 单塔退化场景
test/test_fly_smoke.py        4 项：MuJoCo 冒烟（2 段真飞 < 10 s）+ 悬停时间线
```

配图 `images/route_compare.png`（tab10 配色 + 虚线基线，英文标题，
dpi=110）由 `tools/make_figures.py --json results/inspection_result.json`
从运行结果重新生成，不含任何手工曲线。

## Limitations

- 覆盖校验目前只判"航路经过观察点 15 m 球"，没有建模相机视场角、
  云台姿态与光照方向，"拍到"与"拍好"之间还有距离。
- 可飞图是 40 m 方格网 + 视线简化，绕行路径接近最短但不是全局最优；
  极窄的禁飞区空档（小于一格间距）可能绕不出路，此时保留直飞段并
  在结果里如实暴露。
- 2-opt 的接受准则用整条航线实际航程评估，塔数更大时（>20）评估
  次数会明显增加，可换成分段缓存或贪心邻域。
- 闭环只真飞前 4 个航点段（约 100~200 s 仿真时间），整条航线的
  全程真飞留给 `uav_mujoco_sim` 的长时仿真场景去做。
- ROS 节点在 Linux + ROS 2 Humble 及以上验证 `ros2 run` / `ros2 launch`
  入口；Windows 上仅验证算法层与仿真闭环。
