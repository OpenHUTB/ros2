# 基于 IMU 的路面异常检测（坑洼 / 减速带 / 粗糙路面）

`src/ground/imu_road_anomaly_detection/`

用车载 IMU 的**垂向加速度**识别三类路面异常：坑洼（pothole）、减速带
（speed bump）、粗糙路面（rough patch），并把每个事件关联到 GNSS 经纬度，
可用于众包路面病害巡检。检测核心只用 Python 标准库，绘图才需要 Pillow。

## 1. 为什么做这个模块

仓库里已有大量相机 / 激光雷达 / 毫米波雷达感知与端到端控制作业，IMU
通常只是多传感器融合里的一个附属通道，没有**单独以 IMU 为主传感器**的路面
病害检测模块。本模块补上这一方向：不依赖视觉、不依赖光照、不需要标定，
只用 100 Hz 以内的垂向加速度即可给出异常类型、时刻与位置。

## 2. 运行环境

| 项目 | 要求 |
|---|---|
| 检测 / 自测 / 评估 | Python 3.9+，**仅标准库** |
| 绘图 `imu_plot.py` | Pillow（`pip install pillow`），缺了也能用 `--no-plot` 跑完整个流程 |
| 真实数据采集 `imu_collect.py` | CARLA 0.9.x（本机实测 OpenHUTB v2.10.0 / Town10HD）+ `carla` Python 包 |
| 数据 / 文档图片 | 全部随仓库附带 PNG，小于 3 MB |

CARLA 的 Python 包一般不在 PyPI 环境里，用环境变量 `CARLA_PY_PATH` 指向
含 `carla` 包的目录即可；采集脚本也会自动在仓库根目录及其上两级目录里
查找随仿真器附带的 `_hutb_py`，不写死任何本机绝对路径。

## 3. 目录结构

```
imu_road_anomaly_detection/
├── main.py              入口：自测 / 仿真 / 实采 / 离线分析，可直接运行整个模块
├── imu_anomaly.py       检测核心：重力对齐、毛刺剔除、高通、滑窗能量、事件分割、分类
├── imu_sim_gen.py       四分之一车悬架物理模型，生成带真值的仿真 IMU 数据
├── imu_collect.py       CARLA 实采：IMU(100Hz 请求) + GNSS(10Hz) + RGB 参考帧
├── imu_eval.py          与真值的全局最小总时差匹配，输出召回/精确率/分类准确率
├── imu_plot.py          四面板分析图（Pillow）
├── tests/
│   └── test_imu_anomaly.py   单元测试（标准库 unittest，无第三方依赖）
└── data/
    ├── imu_drive.json          带标注的物理仿真数据（40 s，7 个真值事件）
    └── carla_drive.json        CARLA Town10HD 实采数据（44 s，6 个真值事件）
```

运行后 `data/` 下会生成 `sim_results.json` / `carla_results.json`（检测结果与评估
指标）以及 `*_analysis.png` 与（实采时）`scene_rgb.png` / `rgb_frame/`；提交到仓库的
分析图按仓库约定只放在 `docs/img/imu_road_anomaly_detection/`。仓库根 `.gitignore`
会忽略所有 `data/`（原意是排除体积大的行为克隆数据集），本模块用自带的
`.gitignore` **只放行上述 2 个输入数据集 JSON**，其余运行产物（含两个
`*_results.json`）都不入库 —— 结果由程序复跑生成，不再作为提交内容。

## 4. 运行步骤

在 `src/ground/imu_road_anomaly_detection/` 目录下执行：

```bash
# ① 零依赖自检（合成信号，2 个已知事件）
python main.py --self-test

# ② 带真值的物理仿真评估（不需要 CARLA）
python main.py --data data/imu_drive.json --results sim_results.json

# ③ CARLA 实采数据的验证（不需要启动仿真器，用附带数据）
python main.py --data data/carla_drive.json --results carla_results.json

# ④ 重新生成带标注的物理仿真数据并评估
python main.py --simulate --duration 40 --results sim_results.json

# ⑤ 连接运行中的 CARLA 重新采集（需先启动 CARLA 服务端，默认端口 2000）
python main.py --collect --host 127.0.0.1 --port 2000 --duration 44 \
               --results carla_results.json --plot-name carla_analysis.png

# ⑥ 想要纯净平路噪声底、不加激励：
python main.py --collect --no-inject --duration 44 --results carla_results.json
```

单元测试（标准库 `unittest` 与 `pytest` 都能收集，不依赖 carla / numpy / Pillow）：

```bash
python -m unittest discover -s tests -v
python -m pytest tests -q
```

`main.py` 不带参数时会直接分析 `data/imu_drive.json` 并出图，即"一条命令跑完
整个模块"。`--no-plot` 可在没有 Pillow 的环境里跳过绘图。

启动 CARLA 服务端（Windows 示例）：

```
"C:/hutb_windows_v2.10.0/CarlaUE4/Binaries/Win64/CarlaUE4-Win64-Shipping.exe" \
  -quality-level=Low -carla-rpc-port=2000 -nosound -ResX=800 -ResY=600 -windowed
```

## 5. 算法流程

1. **三轴重力投影（静态姿态）**：跳过起始 `baseline_frames` 个样本，取三轴中位数
   作为**固定**重力向量，归一化后对每个样本做 `ax·ux + ay·uy + az·uz` 投影，再减
   投影的中位数 —— 三个轴都真正参与，不是原来的 `az - gz`。中位数对孤立毛刺和
   短暂冲击都不敏感。注意这只是静态姿态估计，动态俯仰/侧倾不补偿。
2. **毛刺剔除**：CARLA 在车辆生成瞬间偶发单个非物理量级读数（实测出现过
   `-12029 m/s²`），把偏离全局中位数超过 200 m/s² 的样本替换为中位数。
3. **高通残差**：减去 `hp_window` 帧的滑动均值，去掉重力与路面纵坡带来的低频漂移。
4. **滑窗能量与 3σ 阈值**：对残差按 `detect_window` 帧、`detect_stride` 步长求平方和；
   去掉能量最高的 10% 后估计基线均值与标准差，阈值 = 均值 + 3σ。
5. **事件分割**：合并相邻超阈窗口，用峰值半高宽（FWHM）修剪边界；对单帧冲击
   保留按**秒**计算的最短时间窗 `min_span_s = 0.10 s`（不能按帧算，否则 50 Hz
   与 100 Hz 数据得到的时长会差一倍）。
6. **特征提取**：峰值幅值、窗口时长、能量、波峰系数、有效冲击时长、Goertzel
   单频点估计的频谱质心（**只有窗长分辨率 `fs/n ≤ 2 Hz` 时才输出**，否则记
   `None`，不拿伪 1 Hz 网格凑数）；同时按最近时刻挂接 GNSS 经纬度。
7. **规则分类**：
   - 幅值 < 5 m/s² → `rough_patch`（持续低幅不平）；
   - 幅值 ≥ 5 m/s² 且时长 ≥ 0.12 s → `speed_bump`（长缓隆起）；
   - 其余强冲击 → `pothole`（短促凹陷 / 回弹）。

所有阈值集中在 `imu_anomaly.py` 的 `DEFAULTS` 字典，便于复现和调参。

## 6. 数据说明（重要）

Town10HD 的路面网格是**几何光滑**的，CARLA 默认不建模坑洼和减速带，
因此"直接开一段路"只能得到噪声底、没有真值。为了在不伪造 IMU 数据流的前提
下拿到带标注的**真实传感器数据**，`imu_collect.py` 在预先声明的时刻给自车
施加标定过的垂向冲量（CARLA 物理引擎的 `add_impulse`）：

| 激励形式 | 建模对象 | 物理含义 |
|---|---|---|
| 单个强冲量 | pothole | 车轮掉入坑内的一次冲击 |
| 相隔 0.32 s 的两个强冲量 | speed_bump | 前轴 / 后轴依次越过减速带 |
| 5 个小幅交替冲量 | rough_patch | 连续短波长的粗糙路面 |

冲量只是**激励**；后续的 IMU、GNSS 读数全部由 CARLA 物理引擎和传感器真实
产生，未做任何信号合成。真值时刻取激励区间的**中心**（检测器报的是振动峰值
时刻，不是激励起始时刻），并记录在 `carla_drive.json` 的 `ground_truth` 中。
`--no-inject` 可以采集完全未加激励的平路数据用于噪声底测试。

因此这份实采数据**不是"没有真值、只做定性验证"的数据**：它带 6 个真值事件，
和仿真数据一样参与第 7 节的定量评估（召回 / 精确率 / 时刻对齐误差 / 分类准确率）。

另一份 `imu_drive.json` 来自 `imu_sim_gen.py` 的**四分之一车悬架模型**
（簧载质量 350 kg、非簧载质量 45 kg、悬架刚度 16 kN/m、减振 2.5 kN·s/m、
轮胎刚度 200 kN/m），由路面位移轮廓激励生成，带 7 个真值事件。它是完全
独立于 CARLA 的第二条验证链路。

> 实测校准：本环境中 `world.tick()` 在同步模式下推进 0.04 s 传感器时间，
> 每 tick 产生 2 个 IMU 样本（间隔 0.02 s），所以尽管请求 `sensor_tick=0.01`
> （100 Hz），实采流的有效采样率是 **50 Hz**。采集脚本按时间戳实测写入
> `scene.sample_rate_hz`，主程序也以实测间隔为准；两套数据分别按 100 Hz 和
> 50 Hz 处理，因此所有时间窗阈值都以秒为单位。

## 7. 结果

| 数据集 | 真值 | 检出 | TP/FN/FP | 召回 | 精确率 | 分类准确率 | 时刻对齐误差 |
|---|---|---|---|---|---|---|---|
| 物理仿真 `imu_drive.json` | 7 | 7 | 7/0/0 | 100% | 100% | 7/7 | 0.104 s |
| CARLA 实采 `carla_drive.json` | 6 | 7 | 6/0/1 | 100% | 85.7% | 6/6 | 0.087 s |

匹配用**全局最小总时差的全局分配**（容差 0.5 s），不是按真值顺序的贪心最近匹配：
贪心在前一个真值抢走"本该属于后一个真值"的检测时，总时差会明显变差。指标由
`imu_eval.py` 自动计算并写入结果 JSON。

表里「时刻对齐误差」不是纯算法误差：真值 = 激励区间**中心**，检测 = **振动峰值**，
两者之间本来就含悬架动力学延迟（簧上固有频率约 1.52 Hz，1/4 周期约 0.13~0.17 s
量级）。频谱质心只在窗长分辨率足够（`fs/n ≤ 2 Hz`）时输出，两套数据的 coverage
都是 **2/7**（7 个事件里只有 2 个窗口长到能分辨），所以 `mean_centroid`（仿真
15.8 Hz / 实采 14.9 Hz）只是这些可解析窗口上的均值，不能读作全体事件的平均。

实采那 1 个误报是路口起步的俯仰瞬态耦合进垂向轴造成的，
保留未做阈值掩盖。运行效果见 `docs/ground/imu_road_anomaly_detection.md`
中的截图与分析图。

## 8. 已知局限

- 规则阈值是在上述两份带标注数据上调出来的，换个车型 / 悬架 / 车速需要重新标定。
- 重力向量是**固定向量**，无法跟随动态俯仰 / 侧倾：纵向加速度会经车身姿态耦合
  进垂向轴，CARLA 实采里 41.00 s 的起步瞬态误报正是这个代价。彻底消除需要姿态角
  或陀螺积分补偿，而不是继续调阈值。
- 只用了垂向加速度，横向颠簸、俯仰角速度尚未参与判断。
- CARLA 实采数据的"异常"是注入激励产生的，不等价于真实道路破损；真车数据需
  要额外的标注与噪声底测试。
- GNSS 只用于上报事件位置，没有做地图匹配。

---

**大模型使用声明**：本模块的代码、文档与图示在 AI 助手辅助下完成，实验数据、
阈值与结论均已由作者在 CARLA / 本地 Python 环境中实际运行核对。作者对提交
内容负全部责任，文责自负。
