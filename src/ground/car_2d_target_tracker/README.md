# car_2d_target_tracker —— 2D 小车目标追踪（离线指标 + 可选可视化）

> 地面载具选题：单轮车在 2D 平面里追踪（可移动的）目标，带静态障碍。
> 重点不是"窗口里看着追上了"，而是给出**可复算的指标**：到达时间、路径长度、
> 与障碍的最小间隙、卡死时间占比；并对照两种控制器说明"原地打转"为什么会卡死。

## 1. 与原来那版的区别

| 项 | 原实现（1 个文件 296 行） | 本版 |
|---|---|---|
| 运行方式 | **只能开 pygame 窗口**（无 pygame / 无显示器直接跑不起来） | 默认**离线批跑**（只依赖 numpy），`--gui` 才需要 pygame |
| 评价方式 | 看窗口截图 | 到达时间 / 路径 / 最小间隙 / 卡死占比，CSV 可复算 |
| 撞墙处理 | 速度清零 + **原地按最大角速度打转** | 保留 baseline 行为作对照；新增 `escape`：沿障碍切向绕行 |
| 碰撞检测 | 只看**新位置**（大步长会穿模） | 沿整段路径扫掠 + 回退到第一个碰撞点之前 |
| 工程 | 无 launch / package.xml / README / 测试 | 全部补齐（22 项单测） |

## 2. 目录结构

```
src/ground/car_2d_target_tracker/
├── main.py                  # 课程约定入口（转发到包内 cli）
├── main.sh / main.bat       # 一键脚本
├── car_2d_target_tracker/
│   ├── cli.py               # headless（默认）/ --gui / --selftest
│   ├── config.py            # 参数与场景定义
│   ├── geometry.py          # 角度、圆面间隙、扫掠碰撞（纯函数）
│   ├── car.py               # 单轮车模型（贴住障碍、不穿模）
│   ├── target.py            # 目标运动（固定 seed 可复现）
│   ├── controller.py        # baseline（原实现）/ escape（绕障改进）
│   ├── metrics.py           # 四项指标（纯函数）
│   ├── runner.py            # 三场景对照 + CSV 导出
│   ├── plotting.py          # matplotlib 可选绘图
│   └── gui.py               # pygame 可选可视化
├── launch/main.launch.py    # ros2 launch 入口
├── test/test_tracker_logic.py
├── package.xml / setup.py / setup.cfg / resource/
└── requirements.txt
```

## 3. 运行环境（实测）

| 组件 | 实测版本 / 说明 |
|---|---|
| 操作系统 | Windows 11 原生（Linux/macOS 同样可用） |
| Python | 3.12.14 |
| numpy | 2.3.5（**必需**） |
| pygame | 2.6.1（**可选**，只有 `--gui` 需要） |
| matplotlib | 3.11.2（可选，只用于出图；缺失时只写 CSV，退出码仍为 0） |
| ROS / 图形界面 | 都不需要 |

```bash
cd src/ground/car_2d_target_tracker
pip install -r requirements.txt      # numpy
pip install pygame matplotlib        # 可选：GUI / 出图
```

## 4. 运行

```bash
python3 main.py                      # 离线批跑：3 场景 × 2 控制器，导出 CSV/PNG
bash main.sh                         # Linux/macOS 包装脚本
main.bat                             # Windows 包装脚本
python3 main.py --selftest           # 22 项单元测试
python3 main.py --gui                # pygame 窗口（无显示器机器可用 SDL_VIDEODRIVER=dummy 冒烟）
python3 main.py --sim-time 60 --seed 7
ros2 launch car_2d_target_tracker main.launch.py mode:=both sim_time:=30
```

产物（默认 `shots/`）：`{场景}_{模式}.csv`（逐帧 `t,x,y,v,omega,dist,clearance`）、
`tracker_metrics.csv`、`tracker_trajectories.png`、`tracker_metrics.png`。

## 5. 实测指标（本机实测，可由 CSV 复算）

判据：到达半径 18 px；"卡死"定义为速度 < 5 px/s；单场景 30 s；目标随机种子 42。

| 场景 | 控制器 | 到达 | 到达时间 | 路径长度 | 最小间隙 | 卡死占比 | 末距 |
|---|---|---|---|---|---|---|---|
| `chase`（追移动目标） | baseline | **否** | — | 1940 px | — | 0.1% | **40.2 px** |
| | escape | 是 | **2.52 s** | 381 px | — | 0.7% | 17.7 px |
| `gate`（穿两障碍间隙） | baseline | 是 | 4.48 s | 682 px | 20.0 px | 0.4% | 17.7 px |
| | escape | 是 | **3.85 s** | 683 px | 20.0 px | 0.4% | 17.0 px |
| `blocked`（障碍正挡路） | baseline | **否** | — | 240 px | 0.0 px | **95.6%** | **440 px** |
| | escape | 是 | **10.27 s** | 737 px | 0.0 px | 54.1% | 18.0 px |

两条可复现的结论：

1. **追移动目标**：原速度律在 120 px 内按距离减速，靠近时速度被压到与 60 px/s 的
   目标同量级，末距卡在 40 px 追不上；改进版把减速半径改成 40 px 后 2.52 s 追上。
2. **障碍正挡路**：baseline 撞上后速度清零、原地打转，95.6% 的时间几乎不动，
   30 s 内始终没到；escape 沿障碍切向绕过去，10.27 s 到达。

![轨迹对照](../../docs/img/ground/car_2d_tracker_trajectories.png)

![指标对照](../../docs/img/ground/car_2d_tracker_metrics.png)

## 6. 调参记录（三处都改在"会出事"的地方）

| 阶段 | 现象 | 原因 / 处理 |
|---|---|---|
| 碰撞检测 | 大步长时车会**穿过**障碍 | 原实现只检查终点。改为沿路径扫掠（步长 ≤ 一个车半径） |
| 回退位置 | 用二分找"最远能走到的位置"反而把车送到障碍另一侧 | 碰撞判定沿路径**非单调**（障碍只占中间一段），二分会跳过它。改为找第一个碰撞样本再回退一格（`block_fraction`） |
| 追击速度 | 追移动目标时末距恒为 40 px | 减速半径 120 px 把速度压到目标同量级；改进版改用 40 px（仅快到才减速） |
| 绕障速度比例 | 0.35 时绕障要 16.9 s | 实测 0.5→12.8 s、0.7→10.5 s、0.85→10.3 s、1.0→10.2 s，取 0.85 兼顾速度与贴墙余量 |

## 7. 常见问题

**Q1：`--gui` 报 `ModuleNotFoundError: pygame`？**
GUI 是可选依赖：`pip install pygame`。无图形界面的机器请用默认离线批跑，指标完全一样。

**Q2：无显示器时想验证 GUI 代码没写坏？**
```bash
SDL_VIDEODRIVER=dummy python3 main.py --gui --frames 80
```
本机就是这样验证的（跑 80 帧后正常退出）。

**Q3：为什么 `blocked` 场景里 escape 的"卡死占比"还有 54%？**
它是贴着障碍慢慢绕，速度低于 5 px/s 的时间不少——这是**能绕过去但要爬墙**的代价。
若要更快可以调大 `escape_speed_ratio`，或换更聪明的局部规划（见限制）。

**Q4：能不能追到 60 px/s 以上的目标？**
能。改进版速度上限 180 px/s，`chase` 场景 2.52 s 就追上；再快的目标需要调大
`max_speed` 或缩短控制周期。

## 8. 已知限制

- 运动学模型（无动力学、无轮胎滑移），角度用一阶积分，不能反映真实车辆的低速转弯特性。
- 绕障是"沿障碍切向 + 卡住翻转方向"的启发式，不保证最优；对**凹形障碍群**（如 U 形）
  可能来回试探，没有全局规划。
- 碰撞把车贴到表面，没有回弹/摩擦；障碍都是圆，没有多边形/动态障碍。
- 指标都在 60 Hz 定步长、30 s 场景内成立；目标运动是匀速直线 + 边界反弹。
- 本机未安装 ROS，`ros2 launch` / `colcon build` 路径未实测（launch 与 setup.py 按
  同仓库其它 ament_python 模块的写法编写）。
