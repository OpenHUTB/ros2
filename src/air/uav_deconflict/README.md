# uav_deconflict

低空多机 **4D 航迹冲突检测与解脱**：把每架无人机的任务描述成"起飞时刻 +
航路 + 巡航高度"的 4D 航迹，解析求解两两之间的时空最近接近（CPA），
再用地面等待 / 高度层调整 / 侧向绕行三种手段把冲突消解掉，最后用课程
既有的 `uav_mujoco_sim`（MuJoCo 刚体模型 + 几何控制器）**闭环复飞**，
验证真实飞行中的间隔是否达到标准。

> 仿真器为 MuJoCo 3.x（`uav_mujoco_sim` 底座），纯 CPU 运行，不需要
> Gazebo / Unreal / 独立显卡。4 架同时仿真约 7 倍实时。

---

## 1. 这个模块解决什么问题

城市低空里多架无人机按各自计划飞行，航路交叉、对头、追越都会产生
**间隔侵入**。仓库里已有的模块解决的是别的层面的问题：

| 已有模块 | 解决的问题 | 与本模块的区别 |
| --- | --- | --- |
| [`uav_mujoco_sim`](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_mujoco_sim) | 单机物理仿真底座 | 本模块在其上做多机调度层 |
| [`uav_formation_flight`](https://github.com/OpenHUTB/ros2/tree/master/src/air/uav_formation_flight) | 编队**保持**（队形误差） | 不处理航路间的时空冲突 |
| `no_fly_zone_check` | 静态禁飞区（几何） | 不涉及机机间隔与时间维 |
| `uav_route_cruise` 等航点类 | 单机沿航路飞行 | 假设航路之间不冲突 |

本模块补的是**空域流量管理**这一层：给定一批"都要飞"的任务，产出一份
两两间隔达标的执行计划，并用真物理仿真证明它真的达标。

## 2. 方法

### 2.1 4D 航迹

一条航迹 = 二维航路点 + 巡航高度 + 起飞时刻，运动分三段：

```
垂直爬升(2.5 m/s) → 沿折线平飞(v) → 垂直下降
```

三段都是匀速直线，因此任意时刻的位姿是解析的，两条航迹之间的最近接近
也可以解析求解——这是冲突检测可复现、可写单元测试的前提。

### 2.2 冲突检测（圆柱间隔，解析区间）

低空间隔标准是圆柱：水平 `sep_h=50 m` **且** 垂直 `sep_v=20 m` 必须
同时满足。在每对时间重叠的航段内，相对运动是匀速直线，两个条件分别
化为二次/一次不等式，直接解出侵入时间区间，再求交：

```
|Δp_xy(t)| < 50 m   且   |Δp_z(t)| < 20 m      →  [t_start, t_end]
```

没有采样步长，结果与实现无关（测试里与手算解析解对照）。

### 2.3 冲突解脱（三种手段，按代价择优）

| 手段 | 做法 | 代价模型 |
| --- | --- | --- |
| 地面等待 | 推迟起飞 | 延误 Δt |
| 高度层调整 | 巡航高度 ±15/30/45 m（限制在 30~120 m） | 额外爬升/下降时间 |
| 侧向绕行 | 航段上叠加平滑 S 形横向偏移（余弦剖面） | 额外航程 |

规则：

* `priority` 高的任务不让路（场景里有 priority=10 的医疗急救机）；
* 同级任务里起飞较晚者让（先到先得）；
* 每次为"让路机"枚举全部候选动作，用**整队重新检测**后的
  （冲突数，总代价）评分，只接受严格减少冲突的动作，迭代直到无冲突。

绕行偏移必须用平滑剖面：早期版本只插一个折点，航向在一帧内突变 8.8°，
几何控制器姿态回路被激励成增幅振荡、整机翻转坠落（metro 场景跟踪误差
发散到 2×10^5 m）；改平滑剖面后同一场景最大跟踪误差 3 m。

### 2.4 MuJoCo 闭环复核

用 `uav_mujoco_sim` 的 `quadrotor.xml` + 几何控制器，每架飞机独立实例化，
按解脱后的 4D 航迹真飞，记录刚体积分出的真实位置，再统计实际最小间隔。
参考速度前馈经一阶低通（τ=0.5 s）——分段航迹的速度阶跃会激励姿态回路，
真实飞控同样有轨迹平滑器。

## 3. 目录结构

```
uav_deconflict/
├── main.py / main.sh / main.bat        一键运行入口
├── package.xml / setup.py / setup.cfg  ament_python 包
├── config/params.yaml                  在线监视节点参数
├── launch/main.launch.py               启动间隔监视节点
├── uav_deconflict/
│   ├── core/
│   │   ├── plan.py        4D 航迹（分段线性 + 三种解脱变体）
│   │   ├── conflict.py    圆柱间隔解析检测
│   │   ├── resolve.py     三手段贪心消解
│   │   ├── scenarios.py   四个可复现场景
│   │   ├── sim.py         MuJoCo 多机闭环
│   │   ├── monitor.py     在线判定内核（最近机对 / 圆柱侵入，无 ROS 依赖）
│   │   ├── metrics.py     间隔序列 / 裕度比 / 延误统计
│   │   └── pipeline.py    端到端流水线与报告
│   ├── cli.py             ros2 run 入口
│   └── ros/deconflict_node.py   在线间隔监视（订阅 odom → 发布状态/Marker）
├── test/                  42 项单元测试（含 MuJoCo 冒烟与监视内核）
├── tools/make_figures.py  生成文档配图（全部来自真实运行数据）
└── result_all.json        机读结果
```

## 4. 安装、编译、运行、测试

### 4.1 安装依赖

* ROS 2 Humble（运行 ROS 封装节点时需要；仅跑算法层与仿真则不需要）；
* Python ≥ 3.10、`numpy`；
* `mujoco` 3.x：`pip install mujoco`（纯 CPU 可跑，无需独立显卡）；
* 仿真底座 `uav_mujoco_sim`（同仓库 `src/air/`，模块会自动挂到 `sys.path`）。

### 4.2 编译（ament_python 包）

```bash
cd <仓库根目录>
colcon build --packages-select uav_deconflict
source install/setup.bash
```

### 4.3 运行

```bash
# 源码目录直接运行（自动挂同级 uav_mujoco_sim 到 sys.path）
python main.py                       # 全部场景 + MuJoCo 闭环复飞
python main.py --scenario metro      # 只跑一个场景
python main.py --no-sim              # 仅算法层（秒级）

# 编译装包后
ros2 run uav_deconflict main
ros2 launch uav_deconflict main.launch.py   # 在线间隔监视节点
```

### 4.4 运行结果

`python main.py` 输出端到端报告；机读结果写入 `result_all.json`。
下图全部来自本机真实运行数据（生成脚本：`tools/make_figures.py`）：

![四场景解脱前后对比](../../docs/img/air/uav_deconflict_summary.png)

![metro 场景 MuJoCo 闭环复飞](../../docs/img/air/uav_deconflict_metro.png)

## 5. 实测结果（4 场景 × 基线/解脱后 × MuJoCo 闭环）

间隔标准：水平 50 m / 垂直 20 m。"真飞最小间隔"来自 MuJoCo 刚体积分的
真实位置，不是计划值。

| 场景 | 冲突(前→后) | 解脱动作 | 总延误 | 基线真飞最小间隔 | 解脱后真飞最小间隔 | 侵入采样 |
| --- | --- | --- | --- | --- | --- | --- |
| crossing 十字交叉+斜穿（5 机） | 8 → 0 | 高度×2 + 绕行×1 | 26.3 s（另有提前 24.0 s） | **0.17 m** | **59.1 m** | 1090 → 0 |
| corridor 走廊对头+追越（4 机） | 4 → 0 | 高度×1 + 等待×1 | 34.0 s | **0.24 m** | **54.5 m** | 618 → 0 |
| metro 网格多机（6 机） | 5 → 0 | 等待×2 + 绕行×1 + 高度×1 | 49.3 s | **4.66 m** | **55.3 m** | 767 → 0 |
| slot 固定时隙禁等待（3 机） | 3 → 0 | 绕行×2 | 1.4 s | **28.28 m** | **51.8 m** | 367 → 0 |

真飞最大跟踪误差 3.5~5.5 m（起飞瞬态），全部到达误差 0.00 m。
医疗急救机（priority=10）在所有场景中均未被延误。

完整报告：`python main.py`；机读结果：`result_all.json`。

## 6. 测试

```bash
python -m pytest test/ -q     # 42 项，约 3 s（含 MuJoCo 冒烟）
# 编译装包后也可用
colcon test --packages-select uav_deconflict
```

覆盖：航迹构建与三种解脱变体、冲突区间与手算解析解对照（对头/平行/
切线边界/时序分离）、解脱规则（医疗机保护、禁等待、max_delay 上界、
解不缩短任务）、在线监视内核（最近机对选取、圆柱两轴同时判破、过期
数据剔除、状态字典格式）、MuJoCo 冒烟（真飞最小间隔 ≈ 计划间隔 30 m，
到达误差 < 1 m）。

> ROS 2 节点本身是本机唯一没有实跑的部分：开发机为 Windows，未安装
> ROS 2 发行版。因此把判定逻辑全部下沉到 `core/monitor.py`（无 ROS
> 依赖、10 项单测覆盖），节点只保留订阅/发布这层薄壳，并做了
> `py_compile` 与 AST 结构校验。节点功能等同"每 0.5 s 调一次
> `monitor.closest()` 并发布结果"，逻辑侧的正确性由单测兜住。

## 7. 已知限制

* 圆柱间隔是简化模型，未建模机体尺寸、尾流与通信延迟；
* 贪心消解不保证全局最优（每步可解释、可复现，且不会制造新冲突对）；
* 未考虑风与执行器故障；`slot` 场景禁等待是人为设定的运行约束；
* ROS 节点只做**监视与告警**，不在线改航迹——线上改动需要空域管理方批准，
  这条边界是刻意保留的。

## 8. 作业信息

* 姓名：黄家欣　学号：2535030010
* 仿真器：MuJoCo（经课程底座 `uav_mujoco_sim`，模型与话题契约与其一致）
* 本模块使用大模型辅助编写代码与文档，全部运行结果由本机实测产生。
