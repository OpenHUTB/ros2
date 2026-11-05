# UAV 磁力计干扰自检与航向一致性门控

多旋翼的偏航控制默认「磁力计给出的航向是对的」。但动力线束、电调和大电流回路
会在机体上叠加一个随电流变化的磁场，把磁航向推偏十几度到几十度；贴近铁磁结构
（起落架、电池仓、载荷）则会让磁场模值明显异常。这两种情况在地面检查里往往看不出，
起飞后表现为「航向缓慢跑偏」「定点悬停时原地打转」，严重时触发失控保护。

本模块做两件事：

1. **起飞前电机干扰自检**：怠速与上油门各整机旋转一周采样，比较两次拟合出的
   硬铁偏差与垂直均值，偏移超标就判 `FAIL`，把问题留在地面解决。
2. **飞行中航向一致性门控**：把磁航向与 GNSS 航迹向互检，同时监测水平磁场模值
   相对标定半径的偏离，异常时逐级降级，并在需要时改用 GNSS 航迹向作为航向来源。

## 边界与分工

| 已有模块 | 做什么 | 本模块不做什么 |
| --- | --- | --- |
| `uav_gnss_integrity_monitor`（同批提交） | 定位**输入侧**的运动学门控，输出可信位姿 | 不处理定位，不输出位置 |
| `uav_altitude_command_guard` | 约束**目标高度指令**（限幅、变化率、超时） | 不约束控制指令 |
| `telemetry_diagnostics` | 基于 GRU 的**遥测异常评分** | 不做学习式异常检测 |
| `uav_mission_safety_manager` | **任务层**状态机（航点、暂停、返航） | 不做任务调度、不发起返航 |
| 本模块 | **航向输入侧**的干扰自检与一致性门控，输出推荐航向与来源 | 不输出姿态/推力指令，不代替飞控的罗盘校准 |

本模块只给出「这个航向能不能用、该用哪个来源」的结论，**不产生任何控制指令**。
磁偏角修正只提供参数入口（`declination_deg`），不做地磁模型计算。

## 目录结构

```
src/air/uav_mag_interference_monitor/
├── main.py                              # 离线回放 + 起飞前自检 CLI（零第三方依赖）
├── test_mag_interference_monitor.py     # 37 项单元测试（unittest / pytest 均可）
├── test_ros_loop_smoke.py               # ROS 话题闭环冒烟测试（7 项，双模式，见下）
├── mujoco_mag_verification.py           # MuJoCo 物理仿真验证（需 pip install mujoco）
├── package.xml / setup.py / setup.cfg / resource/
├── launch/uav_mag_interference_monitor.launch.py
└── uav_mag_interference_monitor/
    ├── mag_interference_math.py         # 判据 + 状态机，纯标准库，不依赖 ROS
    └── mag_interference_node.py         # ROS 2 话题封装（需要 rclpy）
```

教程在 [`docs/air/uav_mag_interference_monitor.md`](../../../docs/air/uav_mag_interference_monitor.md)，
离线回放数据在 `docs/evidence/uav_mag_interference_monitor/`。

## 已验证环境

实际运行过命令的环境：

- Windows 11，Python 3.13.14，**未安装** ROS 2 / pytest；MuJoCo 3.17.0（含 NumPy）
  通过 pip 安装，用于仿真验证。
- 本机实际执行：`python main.py`、`python main.py --plot <目录>`、
  `python main.py --export-preflight <目录>`、
  `python main.py --idle-csv ... --spin-csv ...`、
  `python -m unittest -v test_mag_interference_monitor.py`（37/37 通过）、
  `python -m unittest -v test_ros_loop_smoke.py`（7/7 通过，内置兼容层模式）、
  `python mujoco_mag_verification.py`（自检 PASS/FAIL 判别 + 飞行段门控）。

目标部署环境（真实 rclpy 路径的冒烟测试**待在部署机复测**，命令见下节）：

- Ubuntu 22.04 + ROS 2 Humble + Python 3.10；`rclpy`、`sensor_msgs`、
  `std_msgs` 来自系统 ROS 安装，不能用 pip 代替。

## 离线运行（不需要 ROS）

下列命令的执行目录都是模块目录 `src/air/uav_mag_interference_monitor`：

```bash
# 打印回放报告（起飞前自检 + 两段干扰的飞行回放）
python main.py

# 额外输出逐帧 CSV 与两联 SVG 曲线
python main.py --plot ./outputs

# 把回放用的两组自检样本导出成 CSV，便于对照真实日志格式
python main.py --export-preflight ./outputs

# 用真实日志跑起飞前自检：PASS/WARN 退出码 0，FAIL 退出码 3
python main.py --idle-csv idle.csv --spin-csv spin.csv

# 覆盖判据
python main.py --config '{"min_ground_speed": 3.0}'

# 单元测试
python -m unittest -v test_mag_interference_monitor.py
```

真实日志 CSV 只要求包含 `t,mx,my,mz` 四列（µT，秒），多余列会被忽略。
`--plot` 与本地产物不属于提交内容；仓库里保留的 SVG/CSV 是用上述命令实际生成后
作为证据提交的。

## ROS 话题闭环冒烟测试（test_ros_loop_smoke.py）

回放节点发布 `/drone/magnetic_field` 与 `/drone/gnss`，**真实的 `MagInterferenceNode`
原封不动**地订阅处理，测试订阅 `/drone/heading`、`/drone/heading_valid`、
`/drone/mag_status` 做端到端断言。覆盖四条基本路径：

| 路径 | 场景 | 断言要点 |
| --- | --- | --- |
| 正常输入 | 6 s 平飞 | TRUSTED、来源 `mag`、航向≈45°、模值≈100%（验证 T→µT 换算） |
| 异常输入 | 横向干扰推偏航向 / 沿航向抬高模值 / NaN 非有限值 | DEGRADED→UNRELIABLE、切换 `gnss_course`、`non_finite` 降级且兜底可用 |
| 超时行为 | 磁场话题中断 2.5 s（GNSS 正常）与悬停断流（航迹向不可用） | 看门狗 `stream_timeout` 逐拍降级；前者输出航迹向，后者冻结航向并 `valid=False` |
| 恢复行为 | 干扰退去 | 连续 5 帧正常后回 TRUSTED、来源回 `mag` |

双模式（同一份测试与节点代码）：

```bash
# Ubuntu + ROS 2（真实 rclpy / 话题 / 定时器，墙钟驱动，约 50 s）
source /opt/ros/humble/setup.bash
python -m unittest -v test_ros_loop_smoke.py

# 无 ROS 2 的机器：自动回退到内置最小 rclpy 兼容层（纯标准库，逐拍确定）
python -m unittest -v test_ros_loop_smoke.py
```

兼容层只实现节点用到的原语（节点/发布/订阅/定时器/参数/时钟），判据与节点代码
不改一行。本机（Windows，无 ROS 2）实测 7/7 通过，输出见
`docs/evidence/uav_mag_interference_monitor/ros_loop_smoke.txt`。

> 设计修正（由该测试暴露）：节点最初把 GNSS 回调时间当作观测时间，GNSS 正常
> 流动时会不断刷新看门狗的 `last_seen_t`，**磁场话题真正中断时超时判据被掩盖**，
> 旧磁场数据会被当作新观测继续输出 TRUSTED。现已改为观测时间以磁力计自身
> 时间戳为准，缓存过期后不再评估，降级统一交给看门狗处理；该行为由
> `test_mag_stream_timeout_watchdog` 与 `test_timeout_without_course_freezes_heading`
> 两条用例锁定。

## MuJoCo 物理仿真验证（mujoco_mag_verification.py）

```bash
pip install mujoco          # 本模块唯一引入第三方依赖的入口
python mujoco_mag_verification.py            # 打印报告
python mujoco_mag_verification.py --plot DIR # 额外输出 CSV 与 SVG
```

旋转与平飞由 MuJoCo 物理引擎驱动（速度伺服起转的角加速度、非均匀角速度、
稳态误差都体现在采样里），水平地磁读数来自 MuJoCo 的 `magnetometer` 传感器；
MuJoCo 不模拟电磁场，硬铁偏差与油门比例电机磁场按文档化解析模型叠加
（磁力计标准误差模型项）。实测结果：

```text
自检采样：怠速整圈 349 帧（速度伺服，非均匀角速度）
  无干扰组: PASS  偏差偏移  0.00%   注入干扰: FAIL  偏差偏移 30.21%
  圆拟合硬铁估计: (4.00, -2.50) µT（注入真值 4.0, -2.5） 半径 22.00 µT（真值 22.00）
飞行段 328 帧（MuJoCo 平飞）：DEGRADED 10 帧，UNRELIABLE 152 帧，
  切换 GNSS 航迹向 3 次，恢复 2 次；磁航向最大偏差 30.58°，门控输出最大 14.89°
事件: 9.20 DEGRADED(heading_error) → 9.70 UNRELIABLE → 19.50 TRUSTED
      → 26.40 DEGRADED(field_norm) → 26.90 UNRELIABLE → 32.30 TRUSTED
```

与离线回放（解析合成数据）结论一致：事件时间、切换次数、输出误差均吻合，
说明门控行为不依赖数据来源。证据见
`docs/evidence/uav_mag_interference_monitor/mujoco_verification.txt`、
`mujoco_replay.csv` 与 `docs/img/air/uav_mag_interference_monitor/mujoco_replay.svg`。

## ROS 2 构建与运行

在仓库根目录执行：

```bash
colcon build --packages-select uav_mag_interference_monitor
source install/setup.bash
ros2 launch uav_mag_interference_monitor uav_mag_interference_monitor.launch.py
```

话题：

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/drone/magnetic_field` | `sensor_msgs/MagneticField` | 机体系磁场，特斯拉，内部转 µT |
| 订阅 | `/drone/gnss` | `sensor_msgs/NavSatFix` | 用相邻两帧解算地速与航迹向 |
| 发布 | `/drone/heading` | `std_msgs/Float64` | 推荐航向（rad，磁北基准 + 磁偏角） |
| 发布 | `/drone/heading_valid` | `std_msgs/Bool` | 该航向是否可直接用于控制 |
| 发布 | `/drone/mag_status` | `std_msgs/String` | JSON：状态、来源、原因码、偏差、模值百分比 |

节点参数与 `Config` 字段一一对应，另有 `bias_x`、`bias_y`、`expected_radius`
（用自检输出填入）与 `declination_deg`。`expected_radius` 留 0 时会退化为
「首帧模值」当基准，此时必须保证首帧是在无干扰的地面采集的。

## 判据

**起飞前自检**。对整圈旋转样本做水平圆拟合
$(m_x-b_x)^2+(m_y-b_y)^2=r^2$，用正规方程 + 高斯消元求解（纯标准库）。
记怠速组为 $(\mathbf b_0,r_0)$、上油门组为 $(\mathbf b_1,r_1)$，
垂直分量均值分别为 $\bar z_0,\bar z_1$，则偏移量为

$$
\Delta=\sqrt{\lVert \mathbf b_1-\mathbf b_0\rVert^2+(\bar z_1-\bar z_0)^2},
\qquad p=\frac{\Delta}{r_0}\times 100\%
$$

$p>5\%$ 判 `FAIL`，$3\%<p\le 5\%$ 判 `WARN`，否则 `PASS`；拟合残差超过
$4\ \mu\mathrm{T}$ 也判 `FAIL`（说明样本本身不可用，例如旋转不完整）。

> 为什么不做三维球面拟合：整机绕竖轴转一圈时，机体系下的样本只落在一个水平圆上，
> z 分量没有展布，$b_z$ 与半径不可分辨，球面拟合是退化的。这里只估水平偏差，
> 垂直方向用均值差单独计入。实测中球面拟合会把 0.0% 的干净样本算成 2.9% 的偏移。

**飞行中门控**。每帧算三路信号：

- 水平模值偏离：$\left|\ \lVert \mathbf m-\mathbf b\rVert / r_{cal}-1\right| > 15\%$
- 航向偏差（仅当水平地速 $\ge 2\ \mathrm{m/s}$、GNSS 航迹向可信时）：
  $|\mathrm{wrap}(\psi_{mag}-\psi_{course})| > 15^\circ$，超过 $30^\circ$ 记为严重
- 数据流超时：相邻帧间隔 $>1\ \mathrm{s}$

连续异常达到 3 帧进 `DEGRADED`（仍输出磁航向但标记降级），达到 8 帧进
`UNRELIABLE`（改用 GNSS 航迹向；航迹向不可用时冻结最后可信磁航向并置
`valid=False`）；连续 5 帧正常才回到 `TRUSTED`。

## 离线回放结果（实际运行输出）

```text
起飞前电机干扰自检:
  无干扰组: PASS  偏差偏移  0.04% 拟合残差 0.17 µT  原因 ok
  注入干扰: FAIL  偏差偏移 30.15% 拟合残差 0.17 µT  原因 bias_shift_exceeds_limit
飞行段 328 帧：DEGRADED 10 帧，UNRELIABLE 153 帧，切换 GNSS 航迹向 3 次，恢复 2 次
磁航向最大偏差 31.12°；门控输出最大偏差 14.86°
状态变化: 9.20 DEGRADED → 9.70 UNRELIABLE → 19.60 TRUSTED
          → 26.40 DEGRADED → 26.90 UNRELIABLE → 32.30 TRUSTED
```

回放中 8–20 s 注入横向干扰（峰值 13 µT），航向被推偏约 30°；
26–32 s 注入沿航向干扰（18 µT），水平模值抬高约 22%，只触发模值判据；
22.3–23.6 s 丢帧 1.3 s，触发一次 `stale`。门控后输出航向的最大偏差
从 31.12° 降到 14.86°（降级阶段仍输出磁航向，因此不为 0）。

## 已知限制与未验证内容

- 已验证**判据、状态机、自检流程、离线回放**、**ROS 话题闭环（进程内兼容层模式，
  7 项冒烟测试）**与 **MuJoCo 物理仿真**；**未验证**部署机上的真实 rclpy 闭环
  （冒烟测试已提供双模式入口，待在 Ubuntu + ROS 2 环境复测）、真实磁力计与
  电机干扰、倾斜状态下的航向精度。开发机为 Windows 11 + Python 3.13.14。
- 未做倾斜补偿：假设机体近似水平，横滚/俯仰超过约 15° 时磁航向会有明显误差，
  上机前应在其后串联姿态补偿，或只在定高平飞段使用本门控。
- MuJoCo 的 magnetometer 参考磁场为固定水平方向（世界系 `(0,-0.5,0)`，无垂直
  分量），仿真中的垂直地磁与机体固定磁场（硬铁、电机电流）为解析叠加模型，
  不是电磁场仿真。
- 水平圆拟合需要整机（或传感器）绕竖轴旋转足够角度；只在小角度内摆动会导致
  拟合病态，此时会以 `ValueError` 明确失败，而不是给出错误标定值。
- GNSS 航迹向由相邻两帧位置差分得到，低速悬停时不可用，这是把
  `min_ground_speed` 设为 2 m/s 的原因。
- 阈值面向小型多旋翼，实机使用前需按机体、当地地磁倾角与动力系统重新标定。
- 只检测干扰，不补偿干扰：超标时给出结论与兜底来源，不做在线椭球补偿。

## 大模型使用声明

本模块的判据实现、ROS 2 封装、文档、单元测试与 PR 描述使用
WorkBuddy（均衡模型）辅助生成；提交者已逐行审阅、实际运行全部命令并核对输出，
对提交内容负责。
