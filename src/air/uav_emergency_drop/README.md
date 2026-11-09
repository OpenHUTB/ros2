# uav_emergency_drop — 应急物资投送：投给谁、怎么扔、什么时候按投放键

ROS 2 (ament_python) 模块。场景是灾后道路中断：一片区域里 6~10 个需求点
（坐标、要几份、多着急），一架无人机载货 12 份，要把急救物资尽快扔到
各需求点旁边。要回答三个问题：

1. 载货有限，先送谁？——按紧迫度 × 需求加权，容量约束下贪心装填，
   一架装不完就切成带装载时间的架次序列；
2. 从高度 h、速度 v 平抛，松手之后落点在哪？——解析解给出前抛
   `x = v·√(2h/g)`、风偏 `w·t_fall×0.8`、散布 `σ = 0.15h`；
3. 什么时候按投放键？——沿直飞航段解析求解理想释放点，到达该点时
   用真实位置/速度外推落点并逐项校验（状态有限、速度与航向沿进入段、
   高度接近计划值、落点偏差 ≤ 接受半径），全部达标才发指令；
   提前量超出段长等不可行情形提示降速或绕行。

选点、排架次用规划器，落点用弹道解析解，最后整体在 MuJoCo 里真飞，
拿刚体积分出的真实释放位置喂给弹道模型预测落点，和真实目标对比。

## 快速运行

```bash
# 依赖：python3 + numpy + mujoco + matplotlib（无 ROS 也能跑算法层与闭环）
pip install numpy mujoco matplotlib pytest

python main.py --no-sim           # 仅算法层：选点 + 架次 + 弹道表，秒级
python main.py --case case_aid_wind   # 单个算例（含 MuJoCo 闭环）
python main.py                    # 全部 3 个算例，闭环总墙钟约 5 s
python tools/make_gif.py          # 重录 docs 里的对比动图
python -m pytest test/ -q         # 31 项测试

# ROS 侧（ROS 2 Humble 及以上，Windows 下需 WSL2 或原生 ROS 2 环境）
cd ~/ros2_ws/src && git clone https://github.com/OpenHUTB/ros2.git
cd ~/ros2_ws && colcon build --packages-select uav_emergency_drop \
  && source install/setup.bash
ros2 run uav_emergency_drop drop_node   # 订阅 /uav/odom，落点校验达标发 JSON 投放指令
ros2 launch uav_emergency_drop main.launch.py
```

源码目录直接运行时 `main.py` 会自动把同级 `uav_mujoco_sim` 加进
`sys.path`，不用先 colcon install。

## 效果

三个算例（村庄群 8 点 / 河对岸 6 点绕桥 / 8 m/s 横风 6 点），
每个都真飞并投放 3 次：

| 算例 | 选点 | 送出 | 架次 | 落点偏差 mean / max |
|---|---|---|---|---|
| `case_aid_village`（h=30, v=8） | 8/8 | 23 份 | 2 | 0.02 / 0.05 m |
| `case_aid_river`（河带绕桥） | 6/6 | 17 份 | 2 | 0.04 / 0.06 m |
| `case_aid_wind`（东风 8 m/s, h=20, v=6） | 6/6 | 17 份 | 2 | 0.01 / 0.02 m |

提前量到底值不值这个钱，动图里看得很直：左栏不提前、飞过目标正上方
才松手，落点平均偏出去 12~21 m；右栏按解析解提前 lead 米松手，
落点基本压在目标上（两栏飞的是同一条闭环轨迹，只有投放时刻不同）：

| 算例 | 不提前直投 | 按解析解提前量 |
|---|---|---|
| 村庄群 | 19.8 m | 0.02 m |
| 河对岸（绕桥） | 20.5 m | 0.04 m |
| 8 m/s 横风 | 12.5 m | 0.01 m |

横风那组更直观：12.92 m 的风偏全部由逆风偏移的越顶点补偿，
投放后解析落点回到真实目标。

口径说明：表中落点偏差均为弹道解析模型（质点平抛 + 常值风偏）的
预测偏差，未含散布 σ 与投放物气动，不等同于实物命中精度。

## 代码结构

```text
uav_emergency_drop/
├── core/ballistics.py   平抛解析解：t_fall / 前抛 / 风偏 / σ / 释放区间
├── core/planner.py      贪心装填 + 交换改进，多架次排序，河流禁飞带绕行
├── core/trigger.py      evaluate_drop：真实状态投放判定（节点与闭环共用）
├── core/flyer.py        MuJoCo 闭环：Quadrotor + GeometricController 真飞，
│                        转角切角圆滑，在理想释放点投一次并记录真实释放状态
├── core/cases.py        3 个算例
├── ros/drop_node.py     薄壳节点：订阅 odom，落点校验达标发布 JSON 投放指令
└── tools/make_gif.py    重跑闭环并录制 docs 用对比动图
```

`results/<case>_result.json` 是跑完留下的机读结果（选点、架次、
逐次投放的释放位置/速度/预计落点），数字和上面的表一致。

## 已知局限

- 落点偏差 0.1 m 量级的前提是投放瞬间处于巡航稳态。急转弯后立即投放
  会把跟踪振荡带进弹道——river 算例曾实测 26 m 偏差，所以航线规划
  避免在转角后 20 m 内安排释放点，转角做切角圆滑；
- 全程最大跟踪误差 6~19 m 出现在转角切角瞬态和爬升转巡航的速度阶跃，
  是几何控制器的真实动态；投放窗口内误差 ≤0.26 m，末端悬停 ≤0.10 m；
- 风场按常值风处理，阵风和投放物的气动阻力（当前按质点平抛）没建模；
- 架次航程用固定巡航速度估算，没算爬升/悬停能耗与精确回程时间。

## 和仓库里其他模块的分工

[uav_deconflict](https://github.com/OpenHUTB/ros2) 管多机航迹冲突的
检测与解脱，本模块只关心"单机把货准确扔到地面点"；
[uav_noise_footprint](https://github.com/OpenHUTB/ros2) 管噪声足迹，
本模块管弹道落点。三者共用同一套 `uav_mujoco_sim` 仿真底座。
