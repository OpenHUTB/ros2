# 车辆几何通过性核算（接近角 / 离去角 / 纵向通过角）

「这台车能不能下这个地库」「过这个减速带会不会拖底」——平时只能看参数表里的接近角、离去角，
或者干脆开过去听声音。这一页把这件事做成可计算、可断言、可回归的三个角度加一条剖面。

实现放在 `src/ground/passability_check/`，纯标准库，**不需要 CARLA、不需要 MuJoCo、不需要 GPU 与显示器**，
一条命令在终端出结果。

## 1 这一页解决什么问题

车位、坡道这类场景，卡人的不是「马力不够」，而是几个纯几何的量：

* **下坡道底部的下凹折角**卡接近角。车头最前点在折线上一沉，前唇就蹭了——这是地库最常见的刮蹭。
* **坡顶转平的上凸折角**卡纵向通过角。车身中部骑在折点上，地板先碰地——山路、厂区坡道最典型。
* **减速带**高 10 cm、坡面只有 35 cm。它比前悬还短，用「无限长斜面」判据会判得过严，
  必须按真实剖面逐位姿算。

所以本页给了两套判据，并且明确谁说了算：

| 判据 | 依据 | 用途 |
| --- | --- | --- |
| 角度判据 | 车辆几何 + 剖面折角 | 横向比较车型，和参数表上的数字对得上 |
| 数值判据 | 沿剖面逐位姿采样车身下缘 | 回答「这一条剖面到底蹭不蹭」，**以它为准** |

两者不一致时，结论取 `PASS_WITH_NOTE` 并打印一条 `备注` 说明原因（见第 6 节的减速带那两行）。
三档结论的含义：

| `result` | 含义 | 依据 |
| --- | --- | --- |
| `PASSED` | 五条断言全过 | 角度与数值一致 |
| `PASS_WITH_NOTE` | **数值复核能过**，角度判据不通过 | 数值判据为准；备注说明为什么角度判据偏保守 |
| `FAILED` | 数值判据不通过（托底或余量不足） | 数值判据为准 |

早期版本用「五条断言全过」当结论，于是减速带那组会出现「结论 FAILED、备注说以数值复核为准」
的自相矛盾——下游只要读 `result` 就会把「能过」当成「过不去」。现在结论字段只由数值判据决定，
角度判据降级为 `PASS_WITH_NOTE` + 备注，两者不会再打架。

## 2 三个角度怎么算

一律以后轴接地点为原点，`x` 向前、`z` 向上，单位米。`L` 轴距，`fo` 前悬，`ro` 后悬，
`hf / hm / hr` 分别是车头、车底最低点、车尾的离地高度，最低点在轴距的 `k` 处（默认 `k = 0.5`）：

| 量 | 公式 | 轿车默认 | SUV 默认 |
| --- | --- | --- | --- |
| 接近角 | `atan2(hf, fo)` | 9.04° | 14.51° |
| 离去角 | `atan2(hr, ro)` | 11.89° | 16.93° |
| 纵向通过角 | `atan2(hm, kL) + atan2(hm, (1-k)L)` | 11.84° | 17.10° |

`k = 0.5` 时纵向通过角退化为教科书里的 `2·atan(2hm / L)`，两者的数值在测试里逐位比对过
（`C10`）。注意一个反直觉的结论：**车底最低点越靠中间，纵向通过角越小**——两条切线同时变平，
这是最不利的位置；把最低点挪到 `k = 0.35`，同一台车的纵向通过角从 11.84° 变成 12.99°（`C11`）。

## 3 为什么还要数值复核

接近角、离去角、纵向通过角对应的是**无限长斜面**：坡要一直延伸下去，车头才会「沉到底」。
真实坡道有长度。减速带的坡面只有 0.35 m，而前悬 0.85 m——车头还没完全爬上坡面，
前轮已经下坡了，所以按角度算会判「过不去」，实际能过。

数值复核的做法是：让车沿剖面走一遍，每个位姿都把车身下缘的 41 个采样点变换到世界坐标，
和路面高度比较，记下最大侵入量（> 0 就是托底）和它发生的位置。俯仰角由前后两个接地点确定——
给定后轴位置，用二分求出前轴接地位置，使两接地点弦长正好等于轴距。

## 4 五条断言

| 断言 | 判据 | 不通过意味着 |
| --- | --- | --- |
| 接近角 | `接近角 >= 剖面最大下凹折角` | 下坡道、进沟时前唇先碰地 |
| 离去角 | `离去角 >= 剖面最大下凹折角` | 倒车出坡时后保险杠先碰地 |
| 纵向通过角 | `纵向通过角 >= 剖面最大上凸折角` | 过坡顶、上陡坡转平时地板先碰地 |
| 离地间隙 | `最低离地 >= 场景门槛 + 安全余量` | 减速带、门槛、积雪把车底顶起来 |
| 全程不托底 | `扫描最小余量 >= 安全余量` | 逐位姿采样后确实蹭到了，报告给出位置与深度 |

前三条的「需要多少度」由剖面折角**自动算出**，不用手填；后两条由场景文件的
`min_clearance_m` / `safety_margin_m` 给出。五条互相独立，失败信息都带「差多少」。

## 5 运行

```bash
cd src/ground/passability_check

python3 main.py                                   # 默认车型跑全部场景
python3 main.py --vehicle suv --scenario garage_ramp
python3 main.py --vehicle all --quiet             # 车型 × 场景对比表
python3 main.py --json out.json --csv out.csv
python3 main.py --test                            # 自检，不依赖 ROS
```

Windows 下用 `main.bat`，Linux 下用 `bash main.sh`，参数相同。

### 5.1 轿车下 20% 的地库坡道

```text
通过性核算 v0.1.0（纯几何，不依赖仿真器）
车辆：sedan，轴距 2.70 m，前悬 0.88 m，后悬 0.95 m，车长 4.53 m，车宽 1.80 m
离地：车头 0.140 m / 车底 0.140 m（轴距 50% 处）/ 车尾 0.200 m，最低 0.140 m
场景：地库坡道（20% 上坡，坡顶转平）（garage_ramp）
剖面：0.00 m ~ 12.00 m，4 个点，最大上坡 11.31° / 最大下坡 0.00°
折点：x=3.00 m 下凹 11.31°；x=9.00 m 上凸 11.31°
角度：接近角 9.04°，离去角 11.89°，纵向通过角 11.84°
采样：下缘 41 点 × 480 档位姿（步长 0.029 m），俯仰 +0.00° ~ +11.31°
侵入：最大 0.035 m（车头 x=3.90 m），最小余量 -0.035 m
断言：3 通过 / 2 失败（共 5 条）
  [FAIL] 接近角    实测 9.04°，剖面最大下凹折角 11.31°（差 -2.27°）
  [PASS] 离去角    实测 11.89°，反向（倒车）遇到同一条下凹折角 11.31°（差 0.58°）
  [PASS] 纵向通过角  实测 11.84°，剖面最大上凸折角 11.31°（差 0.53°）
  [PASS] 离地间隙   最低离地 0.140 m，要求 >= 0.120 m（差 +0.020 m）
  [FAIL] 全程不托底  最小余量 -0.035 m（要求 >= +0.000 m），最大侵入 0.035 m 位于车头 x=3.90 m
结论：FAILED
```

字符剖面图（`X` 就是车头钻进坡面的那一点，`^` 是坡底的下凹折点）：

```text
                      ooooooooooooooooooooooooooooooooooooooooooooooo
                    ooooooooooooooooooooooooooooooooooooooooooooooooooooo
                 ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
              ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
            ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
            ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
            ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
            ooooooooooooo@@@oooooooooooooooooooooooooooooooooooooo@@@oooooooooooo
            oooooooooooo@@@@@oooooooooooooooooooooooooooooooooooo@@@@@ooooooooooo     ######
            ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo###########
                                oooooooooooooooooooooooooooooooooooooooooooooooooX##########
                                                                   ^#############X##########
############################################################################################
--------------------------------------------------------------------------------------------
窗口 x ∈ [-1.36, 4.64] m，横向一格 0.065 m，纵向一格 0.082 m（纵向放大 1.6×）；后轴在 x=0.32 m，俯仰 +0.09°
图例：# 地面  o 车身  @ 车轮  X 侵入点  ^ 下凹折点  v 上凸折点
```

35 mm 的侵入量，和「前唇下沿蹭掉一层漆」的量级一致。

### 5.2 同一坡道换 SUV

```bash
python3 main.py --vehicle suv --scenario garage_ramp
```

```text
角度：接近角 14.51°，离去角 16.93°，纵向通过角 17.10°
侵入：最大 0.000 m（车头 x=3.85 m），最小余量 +0.050 m
断言：5 通过 / 0 失败（共 5 条）
  [PASS] 接近角    实测 14.51°，剖面最大下凹折角 11.31°（差 3.20°）
  [PASS] 离去角    实测 16.93°，反向（倒车）遇到同一条下凹折角 11.31°（差 5.62°）
  [PASS] 纵向通过角  实测 17.10°，剖面最大上凸折角 11.31°（差 5.79°）
  [PASS] 离地间隙   最低离地 0.200 m，要求 >= 0.120 m（差 +0.080 m）
  [PASS] 全程不托底  最小余量 +0.050 m（要求 >= +0.000 m），最大侵入 0.000 m 位于车头 x=3.85 m
结论：PASSED
```

```text
                     oooooooooooooooooooooooooooooooooooooooooooooooooo
                  oooooooooooooooooooooooooooooooooooooooooooooooooooooooo
               oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
             ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
             ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
             ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
             ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
             ooooooooooo@@@@@oooooooooooooooooooooooooooooooooooo@@@@@oooooooooo          ##
             oooooooooooo@@@oooooooooooooooooooooooooooooooooooooo@@@ooooooooooo   #########
                                            ooooooooooooo                   ################
                                                                   ^ #######################
############################################################################################
--------------------------------------------------------------------------------------------
窗口 x ∈ [-1.36, 4.64] m，横向一格 0.065 m，纵向一格 0.093 m（纵向放大 1.4×）；后轴在 x=0.34 m，俯仰 +0.02°
图例：# 地面  o 车身  @ 车轮  X 侵入点  ^ 下凹折点  v 上凸折点
```

同一个坡道，轿车蹭、SUV 不蹭——差别不在「车贵不贵」，而在接近角 9.04° 对 14.51°，
以及最小余量 -35 mm 对 +50 mm。

### 5.3 山路过顶：车底中部托底

```bash
python3 main.py --vehicle sedan --scenario mountain_crest
```

```text
场景：山路过顶（10° 上坡接 6° 下坡）（mountain_crest）
折点：x=5.00 m 上凸 16.00°
角度：接近角 9.04°，离去角 11.89°，纵向通过角 11.84°
侵入：最大 0.048 m（车底中部 x=4.99 m），最小余量 -0.048 m
断言：3 通过 / 2 失败（共 5 条）
  [PASS] 接近角    实测 9.04°，剖面最大下凹折角 0.00°（差 9.04°）
  [PASS] 离去角    实测 11.89°，反向（倒车）遇到同一条下凹折角 0.00°（差 11.89°）
  [FAIL] 纵向通过角  实测 11.84°，剖面最大上凸折角 16.00°（差 -4.16°）
  [PASS] 离地间隙   最低离地 0.140 m，要求 >= 0.100 m（差 +0.040 m）
  [FAIL] 全程不托底  最小余量 -0.048 m（要求 >= +0.000 m），最大侵入 0.048 m 位于车底中部 x=4.99 m
结论：FAILED
```

```text
                      ooooooooooooooooooooooooooooooooooooooooooooooooooo
                   oooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
                ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
             oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
            ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
            oooooooooooooooooooooooooooooooooooooooooooooooooooooo@@@oooooooooooo
            ooooooooooooo@@@ooooooooooooooooooooooooooooooooooooo@@@@@ooooooooooo
            oooooooooooo@@@@@oooooooooooooooooooooooooooooooooooooooooooooooooooo
            ooooooooooooooooooooooooooooooooooXooooooooooooo####
                            ##################X###############################
                    ########################################################################
           #################################################################################
   #########################################################################################
############################################################################################
窗口 x ∈ [1.96, 7.96] m，横向一格 0.065 m，纵向一格 0.098 m（纵向放大 1.3×）；后轴在 x=3.64 m，俯仰 +2.08°
图例：# 地面  o 车身  @ 车轮  X 侵入点  ^ 下凹折点  v 上凸折点
```

`X` 落在车身中部而不是车头，正是 5.1 与这一节的差别：下凹折角卡车头，上凸折角卡地板。

## 6 车型 × 场景对比表

```bash
python3 main.py --vehicle all --quiet
```

```text
对比表（结论：PASSED = 全部断言通过；PASS_WITH_NOTE = 数值复核能过、角度判据偏保守；FAILED = 数值判据不通过）
scenario,vehicle,approach_deg,departure_deg,breakover_deg,min_clearance_m,max_depth_m,min_margin_m,result
garage_ramp,sedan,9.04,11.89,11.84,0.140,0.035,-0.035,FAILED
garage_ramp,suv,14.51,16.93,17.10,0.200,0.000,0.050,PASSED
mountain_crest,sedan,9.04,11.89,11.84,0.140,0.048,-0.048,FAILED
mountain_crest,suv,14.51,16.93,17.10,0.200,0.000,0.014,PASSED
speed_bump,sedan,9.04,11.89,11.84,0.140,0.000,0.040,PASS_WITH_NOTE
speed_bump,suv,14.51,16.93,17.10,0.200,0.000,0.100,PASS_WITH_NOTE
```

`mountain_crest` 那行 SUV 的最小余量只有 14 mm：折角 16.00° 对纵向通过角 17.10°，
能过但余量已经很薄——这种「擦着过」的情况比 PASSED 两个字有用得多。

减速带那两行值得单独说。两车都是 `PASS_WITH_NOTE`：接近角（9.04° / 14.51°）小于坡面角 15.95°，
但真正的侵入量是 0.000 m、最小余量 +0.04 m / +0.10 m——坡面只有 0.35 m，比前悬还短，
车头还没完全爬上坡就过去了。结论字段因此不写 FAILED，报告末尾会打印：

```text
备注：角度判据（接近角）不通过，但沿这条剖面数值复核显示能过：剖面里最短的坡面只有 0.36 m，
比前悬 0.85 m 还短，车头还没完全爬上坡面就已经过去了，无限长斜面判据在这种短坡面上偏保守，
结论以数值复核为准。
```

这也是本模块保留两套判据的原因：角度判据便于和参数表对照，数值判据才回答「这一条剖面能不能过」。

## 7 场景文件

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

`terrain` 是 x 严格递增的折线（米），两端按水平延长；`min_clearance_m` 是「本场景最低要留多少离地」
（减速带、门槛、积雪），`safety_margin_m` 是额外的安全余量。用 JSON 而不是 YAML，
是因为 `json` 是标准库，跑测试的机器上不需要额外装 PyYAML。

换自己的坡道只要改 `terrain`：折点写坡底、坡顶、平段端点即可，折角由程序自己算。

## 8 ROS 2 节点与参数

```bash
colcon build --packages-select passability_check
source install/setup.bash
ros2 launch passability_check main.launch.py vehicle:=suv scenario:=mountain_crest
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `vehicle` | `sedan` | 车型：`sedan` / `suv` |
| `scenario` | `garage_ramp` | 场景名，对应 `config/scenarios/*.json` |
| `scenario_file` | 空 | 直接给场景 JSON 绝对路径，非空时优先 |
| `samples` | 41 | 车身下缘采样点数 |
| `steps` | 480 | 沿剖面扫描的位姿档数 |
| `safety_margin_m` | -1.0 | 负值表示用场景自带的值 |
| `publish_hz` | 1.0 | 报告重发频率 |

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `report` | `std_msgs/String` | 发布（transient_local） | 终端同款报告文本 |
| `summary` | `std_msgs/String` | 发布（transient_local） | 同一份结果的 JSON |
| `evaluate` | `std_msgs/Empty` | 订阅 | 收到一条就重新核算，改完场景文件不用重启 |

```bash
ros2 topic echo /passability_check/report
ros2 topic pub --once /passability_check/evaluate std_msgs/Empty
```

节点本身不做几何计算，全部调用包里的纯函数，所以离线测试覆盖到的就是节点实际跑的代码。

### 8.1 节点闭环冒烟测试（不需要装 ROS）

只测纯函数测不到「结果到底有没有发出去、收到 `evaluate` 会不会重算」。这一层由
`tests/test_passability_node_loop.py` 覆盖：它用一层进程内的最小 rclpy 兼容层
（`init` / `Node` / `Publisher` / `Subscription` / `Timer` / `Parameter` / `Logger` / `QoSProfile`，
加上 `std_msgs` 的 `String`、`Empty`）把 `PassabilityNode` 真跑起来，话题在进程内投递、
定时器手工触发，逐拍确定，不占端口、不需要 `ros2 daemon`。

```bash
python3 tests/test_passability_node_loop.py
```

```text
通过 35 项，失败 0 项
全部通过
```

覆盖的链路：`参数 → 核算 → 发布 report/summary` → `收到 evaluate 重算并重发` →
`运行中把车型换成 suv，结论从 FAILED 变 PASSED` → `抬高安全余量后同一台车被判定不通过` →
`车型填错时节点进 ERROR 且不重发过期报告` → `publish_hz 建定时器并按周期发布` →
`scenario_file 优先于 scenario`。

**未验证的部分**：真实 `rclpy` 在线运行（`ros2 launch` + `ros2 topic echo`）没有在本机跑过——
开发机是 Windows，没有 ROS 2 环境，节点按 Humble 的 API 编写。这一条在 PR 描述里也如实写明。

## 9 离线测试

```bash
cd <仓库根目录>
python3 tests/test_passability_check_local.py
python3 -m pytest tests/test_passability_check_local.py -q
```

```text
通过 181 项，失败 0 项
全部通过
```

另外还有一个节点闭环测试（见 8.1）：`python3 tests/test_passability_node_loop.py` → 35 项。

只用标准库，不需要 ROS / 仿真器 / 显示器。覆盖 11 组：默认值与常量、参数校验、三个角度
（含手算校核与 `k` 的影响）、剖面查询与折点分类、接地二分求解、侵入量与扫描、五条断言
（含每条单独失败）、报告与字符图、场景文件读写、JSON/CSV 输出、工程文件约定。

## 10 与已有模块的关系

| 模块 | 关注点 | 与本模块的区别 |
| --- | --- | --- |
| `turning_sweep_check` | 转弯时车身扫过的圆环宽度 | 算「转过这个弯要占多宽」；本模块算「过这个坡会不会蹭」 |
| `braking_distance_check` | 制动距离与跟车间距 | 纵向动力学；本模块是纯几何 |
| `hill_start_assist` | 坡道起步不溜车 | 控制策略；本模块不控制，只判断几何可行性 |
| `mujoco_car_rollover_guard` | MuJoCo 里跑出来的侧翻风险 | 需要仿真器与横向动力学；本模块离线、纯几何、二维剖面 |
| 本模块 | 接近角 / 离去角 / 纵向通过角 / 托底 | 一条剖面回答「这台车过不过得去，蹭多深」 |

## 11 常见问题

**接近角是不是 `atan(离地/前悬)` 这么简单？**
是。接近角定义就是「前轮接地点到车头最低点的连线」与地面的夹角，两点坐标分别是 `(0, 0)` 和
`(fo, hf)`，所以 `atan2(hf, fo)`。前提是车头最低点就在最前缘；如果保险杠下方有个更高的唇边，
把 `hf` 换成那个唇边的高度即可。

**为什么纵向通过角用两个 `atan` 相加，而不是 `2·atan(2hm/L)`？**
后者只在车底最低点正好在轴距中点时成立。真车最低点常常偏前（发动机下护板）或偏后，
用两个 `atan` 相加才是通式；`k = 0.5` 时两者数值完全一致，测试 `C10` 逐位比对过。

**扫描步长 0.029 m 够细吗？**
够。侵入量是位置的光滑函数，步长减半（`--steps 960 --samples 81`）后最大侵入量只在小数点后
第三位变化（`F17`/`F18` 覆盖这一点）。

**为什么只算二维剖面？左右轮不一样高怎么办？**
单侧车轮上台阶是横向不对称工况，需要三维建模（四个轮子的接地约束），不在本模块范围内。
本模块处理的是「整车沿一条纵向剖面通过」，也就是地库坡道、减速带、坡顶这些最常见的情形。

**满载后车身下沉，结果还准吗？**
本模块不含悬架压缩，`clearance_*` 填的是静态值。满载时离地高度会减小，把三个 `clearance_*`
换成满载实测值再算一遍即可——角度和侵入量都会跟着变。

## 大模型声明

使用大模型辅助代码编写、测试编写与文档整理，提交内容由本人负责。
