# 轮胎附着系数与制动距离扫掠实验

提高轮胎附着系数能不能一直缩短制动距离？答案取决于**瓶颈在哪一侧**：附着不够时，路面
决定上限；附着够好时，制动器扭矩才是上限，继续换更好的轮胎没有收益。

仓库里 `carla_vehicle_dynamics_test` 已经测了"加速 / 制动 / 稳态转向"的基本性能，但都是
在默认附着系数下各测一次，看不出上面这条分界。本模块在 CARLA/HUTB 里**把 `tire_friction`
当作自变量扫一遍**，每个值都跑一次 50→0 km/h 全力制动，量出制动距离，再用两区段模型和
弹性判据判断当前是"附着受限"还是"制动器受限"。

模块同时提供**离线演示**（不需要模拟器，用已保存的实测数据或合成数据出图），所以审核者
没有模拟器也能复现图表。

## 目录结构

```text
src/ground/tire_friction_braking_sweep/
├── main.py                       # 入口：demo / sim / fit
├── launch/main.launch.py         # ROS 2 启动（ExecuteProcess 调 main.py sim）
├── package.xml / setup.py / setup.cfg
├── requirements.txt              # numpy、Pillow
├── resource/tire_friction_braking_sweep
├── tire_friction_braking_sweep/
│   ├── model.py                  # μg 上限、两区段模型、饱和曲线、制动距离
│   ├── analyze.py                # 数据点、拟合、弹性、瓶颈判定、摘要
│   ├── sim.py                    # CARLA/HUTB 扫掠（延迟导入 carla）
│   ├── visualize.py              # Pillow 2×2 效果图
│   └── cli.py                    # 命令行分派
├── tests/                        # 17 个单元测试
└── results/                      # demo/sim 输出：sim_sweep.json、metrics.json、PNG
```

## 运行环境

| 项目 | 版本 / 说明 |
|---|---|
| 操作系统 | Windows 11（实测） |
| Python | 3.12.14 |
| NumPy / Pillow | 2.3.5 / 12.3.0 |
| 模拟器 | HUTB 2.10.1（`D:\OpenHUTB\hutb`，服务端版本 `3ea2130ed`，地图 `Town10HD`） |
| 客户端 | `PythonAPI/carla/dist/hutb-2.10.1-cp312-cp312-win_amd64.whl`（提供 `carla` 包） |
| ROS 2 | 可选；`launch/main.launch.py` 未在 ROS 2 环境实测 |

## 运行步骤

**只看图（不需要模拟器）**：

```bash
python -m pip install -r src/ground/tire_friction_braking_sweep/requirements.txt
cd src/ground/tire_friction_braking_sweep
python main.py demo          # 读 results/sim_sweep.json 出图；没有就跑合成数据
```

**跑真实扫掠**（先启动模拟器，再执行）：

```bash
python main.py sim --speed 50 --frictions 0.4 0.6 0.8 1.0 1.2 1.6
```

可选参数：`--host`、`--port`、`--vehicle`、`--speed`（制动初速 km/h）、`--out`。

## 运行效果

`results/demo_screenshot.png`（149 KB，1600×1150）：

![轮胎附着系数与制动距离](../img/ground/tire_friction_braking_sweep/demo_screenshot.png)

本次实测（Town10HD，`vehicle.tesla.model3`，50 km/h 全力制动，同步模式 0.05 s 步长）：

| μ | 制动距离 | 等效减速度 | 距离比（实测/μg 上限） | 距离弹性 | 判定 |
|---|---|---|---|---|---|
| 0.40 | 44.99 m | 2.18 m/s² | 1.80 | — | 附着受限 |
| 0.60 | 32.73 m | 3.03 m/s² | 1.94 | −0.545 | 附着受限 |
| 0.80 | 24.52 m | 3.95 m/s² | 1.99 | −0.752 | 附着受限 |
| 1.00 | 21.07 m | 4.68 m/s² | 2.10 | −0.563 | 附着受限 |
| 1.20 | 18.70 m | 5.18 m/s² | 2.27 | −0.562 | 附着受限 |
| 1.60 | 17.86 m | 5.51 m/s² | 2.85 | **−0.134** | **制动器受限** |

结论：

1. μ 从 0.40 提到 1.60，制动距离从 44.99 m 降到 17.86 m（**−60.3%**）；
2. 但收益**急剧衰减**：μ 0.40→0.60 时距离缩短 27%，而 μ 1.20→1.60（附着提高 33%）只再缩短 4.5%；
3. 弹性判据在 μ=1.6 处降到 −0.134（阈值 0.2），说明瓶颈已从轮胎附着转到制动器扭矩；
4. 实测减速度始终低于 `μg` 上限（距离比 1.80–2.85 倍），说明该车型在模拟器里**达不到附着极限**——
   滑移率、载荷转移与制动力分配都会打折，这也是把实测数据与理论线一起画出来的原因。

## 模型与指标定义

**两个上限**：轮胎附着 `a ≤ μg`，制动器 `a ≤ a_brake`，理想减速度取小者
`a_ideal = min(μg, a_brake)`；对应制动距离仍是 `s = v²/(2a)`。

**饱和曲线**（描述实测趋势）：`a(μ) = a_max·μ/(μ + μ½)`，取倒数后对 `(1, 1/μ)` 做线性最小二乘
即可拟合，不需要非线性优化。

**指标**：等效减速度 `a = v²/(2s)`；距离比 `s / (v²/(2μg))`；距离弹性
`(Δs/s)/(Δμ/μ)`——绝对值越小说明继续提高附着越没用。

**瓶颈判据**：弹性绝对值小于 0.2 判为"制动器受限"。第一个点没有前一点，沿用第二个点的判定。

## 参数

| 参数 | 默认值 | 含义 |
|---|---|---|
| `frictions` | 0.4 0.6 0.8 1.0 1.2 1.6 | 扫掠的轮胎附着系数 |
| `speed` | 50 | 制动初速（km/h） |
| `vehicle` | `vehicle.tesla.model3` | 车型 |
| `dt` | 0.05 | 同步模式步长（s） |
| `mass_kg` | 1500 | 整车质量，仅用于理论参考值 |

## 单元测试

```bash
cd src/ground/tire_friction_braking_sweep
python -m unittest discover -s tests -v
```

17 个用例：`μg` 上限与两区段取小、`s = v²/(2a)` 的解析值与平方关系、饱和曲线形状与单调性、
瓶颈切换、数据点派生量、饱和拟合能还原已知参数（a_max/μ½ 误差 < 0.05）、弹性定义与单调衰减、
摘要结构自洽、各类非法输入。

## 已知限制

- **单车型、单地图、单初速**：结论只对这组条件成立，换车型（质量/制动扭矩）分界点会移动；
- **`tire_friction` 是模拟器的集总参数**，不是真实轮胎的滑移特性曲线，测得的"附着受限"是
  模拟器物理模型的表现；
- **`brake=1.0` 全力制动**下，高附着段受制动器扭矩限制，因此饱和出现在 μ≈1.2–1.6，实际车辆
  的分界点取决于制动系统；
- 每个附着系数只跑一次（模拟器确定性好，两次同一 μ 的距离完全一致，实测 24.52 m 可复现），
  但没有做多次重复取均值；
- 合成数据模式下（无模拟器）的输出**不是实测**，`source` 字段会标成 `synthetic`；
- ROS 2 launch 未实测（本机没有 ROS 2）。
