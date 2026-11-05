# 无人机磁力计干扰自检与航向一致性门控

![离线回放曲线](../img/air/uav_mag_interference_monitor/offline_replay.svg)

> 上图由 `python main.py --plot` **实际运行生成**（矢量图，非截图）：
> 上半部分是航向对比（灰=真实航向/航迹向，红=原始磁航向，蓝=门控输出），
> 下半部分是水平磁场模值占标定半径的百分比；浅红竖带为 DEGRADED / UNRELIABLE 区间。

## 1. 为什么要做这个模块

偏航控制默认磁力计给的航向是对的。实际飞行中，动力线束和电调产生的磁场会随油门
变化，把磁航向推偏十几度到几十度；贴近铁磁结构则会让磁场模值异常。地面检查时
电机没转，这些问题看不出来；起飞后在定点悬停里表现为缓慢跑偏、原地打转，
操作者往往先怀疑控制器参数，最后才发现是罗盘被干扰。

仓库已有的关心里，[目标高度指令约束](./uav_altitude_command_guard.md) 处理目标指令，
[遥测诊断](./telemetry_diagnostics.md) 输出异常评分，
[任务安全状态机](./uav_mission_safety_manager/overview.md) 处理任务层，
另有定位完整性门控处理位置输入；
**航向输入侧**的干扰自检与一致性判断是空缺。

本模块把这件事拆成地面和空中两段：地面用「怠速 vs 上油门」两组整圈样本比较硬铁
偏差，把干扰留在地面解决；空中用 GNSS 航迹向与磁航向互检，配合模值判据，
异常时明确告诉下游「改用航迹向」或「冻结最后可信航向」。
纯计算层只用标准库，所以判据可以脱离 ROS 单测。

## 2. 起飞前：电机干扰自检

整机绕竖轴旋转一周采样，机体系下的水平磁场样本应落在一个圆上：

$$
(m_x-b_x)^2+(m_y-b_y)^2=r^2
$$

展开后关于 $(b_x,b_y,r^2-b_x^2-b_y^2)$ 线性，用正规方程 + 高斯消元求最小二乘解
（纯标准库实现，不引入 NumPy）。怠速组记 $(\mathbf b_0,r_0,\bar z_0)$，
上油门组记 $(\mathbf b_1,r_1,\bar z_1)$，偏移量定义为

$$
\Delta=\sqrt{\lVert\mathbf b_1-\mathbf b_0\rVert^2+(\bar z_1-\bar z_0)^2},
\qquad p=\frac{\Delta}{r_0}\times 100\%
$$

判据：$p>5\%$ → `FAIL`；$3\%<p\le5\%$ → `WARN`；否则 `PASS`。
拟合残差 RMS 超过 $4\ \mu\mathrm{T}$ 也判 `FAIL`——这通常意味着旋转不完整或样本
被尖峰污染，此时应重新采样，而不是相信标定值。

!!! note "为什么是圆拟合而不是球面拟合"
    整机只绕竖轴转一圈时，机体系样本集中在一个水平圆上，$z$ 分量没有展布，
    $b_z$ 与半径不可分辨，三维球面拟合在这个条件下是**退化**的。实现初期用了
    球面拟合，结果把零干扰的干净样本算成 2.86% 的偏移（真值 0%），
    这正是数值病态的体现。改为水平圆拟合 + 垂直均值差之后，干净样本的偏移
    降到 0.04% 以内。

## 3. 飞行中：航向一致性门控

每帧计算三路信号：

| 信号 | 判据 | 典型成因 |
| --- | --- | --- |
| 水平模值偏离 | $\left\lvert \lVert\mathbf m-\mathbf b\rVert/r_{cal}-1\right\rvert>15\%$ | 贴近铁磁结构、载荷变化 |
| 航向偏差 | $\lvert\mathrm{wrap}(\psi_{mag}-\psi_{course})\rvert>15^\circ$（$>30^\circ$ 为严重） | 大电流线束的旋转磁场 |
| 数据流超时 | 相邻帧间隔 $>1\ \mathrm{s}$ | 话题中断、驱动异常 |

其中航向偏差只用在地速 $\ge 2\ \mathrm{m/s}$ 时——悬停时 GNSS 航迹向由噪声主导，
不能当参考。$r_{cal}$ 来自起飞前自检（`expected_radius`），不是「第一帧模值」；
后者在首帧本身被干扰时会把正常数据反向判成异常，实现初期踩过这个坑。
可信状态下 $r_{cal}$ 以 1% 步长缓慢跟随环境变化，但只在模值本身正常时才跟随，
避免把真实干扰"学"进基准。

```mermaid
flowchart TD
    A[INIT 尚无可信航向] -->|连续正常帧| B[TRUSTED 输出磁航向]
    A -->|连续异常 3 帧| C[DEGRADED]
    B -->|连续异常 3 帧| C
    C -->|恢复正常一帧| B
    C -->|连续异常 8 帧| D[UNRELIABLE]
    D -->|航迹向可用| E[输出 GNSS 航迹向 valid=True]
    D -->|航迹向不可用| F[冻结最后可信磁航向 valid=False]
    E -->|连续 5 帧正常| B
    F -->|连续 5 帧正常| B
```

降级不等于失效：`DEGRADED` 仍输出磁航向，只是把状态和原因码一起发出去，
让上层自己决定是否降权；只有 `UNRELIABLE` 才切换航向来源。

## 4. 运行方式

离线（不需要 ROS）：

```bash
cd src/air/uav_mag_interference_monitor
python main.py                                  # 回放报告
python main.py --plot ./outputs                 # CSV + SVG
python main.py --export-preflight ./outputs     # 导出两组自检样本 CSV
python main.py --idle-csv idle.csv --spin-csv spin.csv   # 真实日志自检
python -m unittest -v test_mag_interference_monitor.py   # 37 项单测
```

自检 CLI 的退出码：`PASS` / `WARN` 返回 0，`FAIL` 返回 3，参数或文件错误返回 2，
可以直接串进起飞前检查脚本。

在线（Ubuntu 22.04 + ROS 2 Humble）：

```bash
colcon build --packages-select uav_mag_interference_monitor
source install/setup.bash
ros2 launch uav_mag_interference_monitor uav_mag_interference_monitor.launch.py
```

## 5. 回放结果（实际运行输出）

```text
起飞前电机干扰自检:
  无干扰组: PASS  偏差偏移  0.04% 拟合残差 0.17 µT  原因 ok
  注入干扰: FAIL  偏差偏移 30.15% 拟合残差 0.17 µT  原因 bias_shift_exceeds_limit
飞行段 328 帧：DEGRADED 10 帧，UNRELIABLE 153 帧，切换 GNSS 航迹向 3 次，恢复 2 次
磁航向最大偏差 31.12°；门控输出最大偏差 14.86°
```

回放按时间注入了四类情况：

| 时段 (s) | 注入内容 | 期望 | 实际 |
| --- | --- | --- | --- |
| 0–8 | 无 | TRUSTED | TRUSTED |
| 8–20 | 横向干扰 0→13 µT | 航向偏差超限，降级并切换来源 | 9.20 DEGRADED，9.70 UNRELIABLE |
| 20–26 | 干扰退去 | 连续 5 帧后恢复 | 19.60 TRUSTED |
| 22.3–23.6 | 丢帧 1.3 s | 触发 `stale` | 单帧 `stale`，未升级 |
| 26–32 | 沿航向干扰 18 µT | 模值超限 | 26.40 DEGRADED，26.90 UNRELIABLE |
| 32–34 | 干扰退去 | 恢复 | 32.30 TRUSTED |

## 6. 单元测试

```text
Ran 37 tests in 0.109s
OK
```

覆盖：圆拟合还原水平偏差/半径/垂直均值、样本不足与非有限值处理、
自检三级判定（PASS/WARN/FAIL）、纯垂直偏移判失败、角度归一化、
首帧可信判定、航向偏差计算、低速不比航迹向、DEGRADED/UNRELIABLE 的分级、
无航迹向时冻结最后可信航向、恢复需满连续帧、模值判据、非有限输入、
超时判据、看门狗 `note_timeout` 升级、配置校验、回放端到端指标、
CSV/SVG 生成与 CSV 读写的往返一致性。

## 7. 已知限制

- **未验证** ROS 2 话题闭环、仿真飞行与真实磁力计/电机干扰；开发机为
  Windows 11 + Python 3.13.14，无 ROS 2、无仿真器。
- **未做倾斜补偿**：假设机体近似水平。横滚/俯仰超过约 15° 时磁航向误差明显，
  上机前需串联姿态补偿，或只在定高平飞段启用。
- 只检测、不补偿：超标时给结论与兜底来源，不做在线椭球补偿。
- GNSS 航迹向由相邻位置差分得到，低速时不可用（`min_ground_speed` 的作用）。
- 阈值面向小型多旋翼，实机前需按机体、当地地磁倾角与动力系统重新标定。
- 自检需要整圈旋转，只在小角度内摆动会导致拟合病态，此时明确报错而非给出错误值。

## 8. 大模型使用声明

本模块的判据实现、ROS 2 封装、文档、单元测试与 PR 描述使用
WorkBuddy（均衡模型）辅助生成；提交者已逐行审阅、实际运行全部命令并核对输出，
对提交内容负责。
