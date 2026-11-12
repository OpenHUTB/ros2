# passability_check

车辆几何通过性核算：由轴距、前/后悬、三个测点的离地高度和车底最低点位置，算出
**接近角 / 离去角 / 纵向通过角**；再把车按真实位姿沿一条纵剖面走一遍，判断**车头、车底、车尾
会不会蹭、蹭多深**。

纯标准库实现（`math` / `json` / `io` / `os`），**不依赖 CARLA、不依赖 MuJoCo、不需要 GPU 与显示器**，
一条命令在终端出结果，也可以在没有任何 ROS 的机器上做回归。

## 这个包解决什么问题

「这台车能不能下这个地库」「我的车过这个减速带会不会拖底」，平时只能靠看参数表里的
接近角/离去角，或者干脆开过去听声音。参数表上的三个角度是**无限长斜面**下的判据，
而真实坡道有长度、有圆角、有排水沟：

* 地库坡道下沿是一个**下凹折角**，卡的是接近角——车头最前点在这条折线上一沉，前唇就蹭了；
* 坡顶转平是一个**上凸折角**，卡的是纵向通过角——车身中部骑在折点上，地板先碰地；
* 减速带的坡面只有 35 cm，比前悬还短，用「无限长斜面」判据会**过于保守**。

本模块把两套判据都算出来：

| 判据 | 依据 | 用途 |
|---|---|---|
| 角度判据 | 车辆几何 + 剖面折角 | 横向比较车型，和参数表对得上 |
| 数值判据 | 沿剖面逐位姿采样车身下缘 | 回答「这一条剖面到底蹭不蹭」，以它为准 |

两者不一致时结论取 `PASS_WITH_NOTE` 并打印 `备注` 说明原因（见 `speed_bump` 那两行）：

| `result` | 含义 |
| --- | --- |
| `PASSED` | 五条断言全过 |
| `PASS_WITH_NOTE` | 数值复核能过，角度判据偏保守（备注说明原因） |
| `FAILED` | 数值判据不通过（托底或余量不足） |

结论**只由数值判据决定**：角度判据是「无限长斜面」的保守近似，不认短坡面，
拿它拉低结论会出现「结论 FAILED、备注说能过」的自相矛盾。

## 目录结构

```text
src/ground/passability_check/
├── package.xml
├── setup.py / setup.cfg
├── README.md
├── main.py                     # 统一入口（仓库约定 main.* 开头）
├── main.bat / main.sh          # Windows / Linux 一键入口
├── config/
│   ├── params.yaml             # ROS 2 节点参数
│   └── scenarios/              # 三条可核算剖面
│       ├── garage_ramp.json    # 地库坡道（20% 上坡，坡顶转平）
│       ├── mountain_crest.json # 山路过顶（10° 上坡接 6° 下坡）
│       └── speed_bump.json     # 小区减速带（高 10 cm，顶宽 70 cm）
├── launch/main.launch.py
├── resource/passability_check  # ament 索引标记
└── passability_check/
    ├── vehicle.py              # 车辆几何 + 三个角度（纯函数）
    ├── terrain.py              # 纵剖面、坡度、折点分类
    ├── clearance.py            # 接地求解、位姿、下缘采样、侵入量
    ├── scenario.py             # JSON 场景读取与校验
    ├── report.py               # 报告文本、字符剖面图、断言、CSV/JSON
    └── node.py                 # ROS 2 节点（只做收发，不算几何）
tests/test_passability_check_local.py
docs/ground/passability_check.md
```

## 三个角度怎么算

一律以「后轴接地点」为原点，x 向前、z 向上，单位米。`L` 轴距，`fo` 前悬，`ro` 后悬，
`hf / hm / hr` 分别是车头、车底最低点、车尾的离地高度，最低点在轴距的 `k` 处（默认 0.5）：

| 量 | 公式 | 轿车默认值 |
|---|---|---|
| 接近角 | `atan2(hf, fo)` | 9.04° |
| 离去角 | `atan2(hr, ro)` | 11.89° |
| 纵向通过角 | `atan2(hm, kL) + atan2(hm, (1-k)L)` | 11.84° |

`k = 0.5` 时纵向通过角退化为教科书里的 `2·atan(2hm / L)`。注意 **最低点越靠中间，纵向通过角越小**
——因为两条切线同时变平，这是最不利的位置。

## 五条断言

| 断言 | 判据 | 不通过意味着 |
|---|---|---|
| 接近角 | `接近角 >= 剖面最大下凹折角` | 下坡道/进沟时前唇先碰地 |
| 离去角 | `离去角 >= 剖面最大下凹折角` | 倒车出坡时后保险杠先碰地 |
| 纵向通过角 | `纵向通过角 >= 剖面最大上凸折角` | 过坡顶、上陡坡转平时地板先碰地 |
| 离地间隙 | `最低离地 >= 场景门槛 + 安全余量` | 减速带、门槛、积雪把车底顶起来 |
| 全程不托底 | `扫描最小余量 >= 安全余量` | 逐位姿采样后确实蹭到了，报告给出位置与深度 |

前三条的「需要多少度」由剖面的折角**自动算出**，不用手填；后两条由场景文件的
`min_clearance_m` / `safety_margin_m` 给出。五条互相独立，失败信息都带「差多少」。

## 场景文件格式

```json
{
  "name": "garage_ramp",
  "title": "地库坡道（20% 上坡，坡顶转平）",
  "note": "坡长 6 m、坡度 11.31°",
  "terrain": [[0.0, 0.0], [3.0, 0.0], [9.0, 1.2], [12.0, 1.2]],
  "min_clearance_m": 0.12,
  "safety_margin_m": 0.0
}
```

`terrain` 是一条 x 严格递增的折线（米）；两端按水平延长，所以车头车尾开出去不会报错。
用 JSON 而不是 YAML：`json` 是标准库，评分机器上不用装 PyYAML。

## 运行

```bash
cd src/ground/passability_check

python3 main.py                                  # 默认车型跑全部场景（报告 + 剖面图）
python3 main.py --vehicle suv                    # 换车型
python3 main.py --vehicle all --quiet            # 车型 × 场景对比表
python3 main.py --scenario garage_ramp --vehicle suv
python3 main.py --json out.json --csv out.csv    # 结构化输出
python3 main.py --list                           # 列出场景
python3 main.py --strict                         # 结论 FAILED 时返回退出码 3（可挂 CI）
python3 main.py --test                           # 自检，不依赖 ROS
```

Windows 下 `main.bat`，Linux 下 `bash main.sh`，参数同上。

## 测试

```bash
cd <仓库根目录>
python3 tests/test_passability_check_local.py     # 181 项：几何、断言、报告、场景、工程约定
python3 tests/test_passability_node_loop.py       # 35 项：节点闭环（进程内 rclpy 兼容层）
python3 -m pytest tests/test_passability_check_local.py tests/test_passability_node_loop.py -q
```

只用标准库，不需要 ROS / 仿真器 / 显示器。覆盖：默认值与常量、参数校验、三个角度（含手算校核）、
剖面查询与折点分类、接地二分求解、侵入量与扫描、五条断言（含每条单独失败）、报告与字符图、
场景文件读写、JSON/CSV 输出、工程文件约定。

## ROS 2 节点

```bash
colcon build --packages-select passability_check
source install/setup.bash
ros2 launch passability_check main.launch.py vehicle:=suv scenario:=garage_ramp
```

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `report` | `std_msgs/String` | 发布（transient_local） | 终端同款报告文本 |
| `summary` | `std_msgs/String` | 发布（transient_local） | 同一份结果的 JSON |
| `evaluate` | `std_msgs/Empty` | 订阅 | 收到一条就重新核算（改完场景文件不用重启） |

```bash
ros2 topic echo /passability_check/report
ros2 topic pub --once /passability_check/evaluate std_msgs/Empty
```

节点收发这一层由 `tests/test_passability_node_loop.py` 用进程内 rclpy 兼容层做闭环验证；
真实 `ros2 launch` 在线运行尚未在装了 ROS 2 的机器上复跑（见「已知限制」）。

## 已验证 / 未验证

已验证（Windows 11 + Python 3.13.14）：

* `main.py` 四个入口（默认、`--vehicle all --quiet`、`--json/--csv`、`--test`）实际运行，退出码正确
* 两个测试文件：181 + 35 项全部通过
* `mkdocs build` 本页无告警（其余页面的缺图告警来自稀疏检出的不完整图片目录）

未验证：

* 真实 `rclpy` 在线运行（`colcon build` + `ros2 launch` + `ros2 topic echo`），开发机无 ROS 2 环境
* 真实车辆实测：三台车的几何参数取自公开参数表，不是实测车

## 已知限制

* 车身下缘用**三节点折线**近似，不建模轮拱、排气、护板细节；真车下缘是曲面，折线模型的偏差在厘米级。
* 轮胎按**接地点**处理（不考虑胎面压缩与胎肩），也不处理单侧车轮上台阶这种**横向**不对称工况。
* 只做**静态几何**判断，不含悬架压缩、载荷导致的车身下沉与俯仰——满载时离地高度会变小，
  需要的话把 `clearance_*` 填成满载值。
* 平面（x-z）二维核算，不涉及车宽方向的车道/通道约束（那属于转弯扫掠，见 `turning_sweep_check`）。

## 与已有模块的关系

| 模块 | 关注点 | 与本模块的区别 |
|---|---|---|
| `turning_sweep_check` | 转弯时车身扫过的圆环宽度 | 算「转过弯要占多宽」；本模块算「过坡/过坎会不会蹭」 |
| `braking_distance_check` | 制动距离与跟车间距 | 纵向动力学；本模块是纯几何 |
| `hill_start_assist` | 坡道起步不溜车 | 控制策略；本模块不控制，只判断几何可行性 |
| `mujoco_car_rollover_guard` | MuJoCo 里跑出来的侧翻风险 | 需要仿真器与横向动力学；本模块离线、纯几何、二维剖面 |
