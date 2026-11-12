# 连续控制器离散化的四种方法对比

同一台控制器，用前向欧拉、后向欧拉、双线性(Tustin)、零阶保持(ZOH) 实现，效果差多少？
什么时候会失稳？本页用二阶对象 `G(s)=wn²/(s²+2ζwn·s+wn²)`（wn=3 rad/s，ζ=0.4）
给出定量答案。

## 1. 问题背景

数字控制器每个采样周期执行一次，连续律必须离散化。教科书常写"用 Tustin 就行"，
但不同方法的**精度差 1~2 个数量级、稳定性边界完全不同**，这在采样周期偏大的 ECU 上
会直接变成实车抖动。

## 2. 四种方法的公式

| 方法 | F | G |
| --- | --- | --- |
| 前向欧拉 | `I + A·Ts` | `B·Ts` |
| 后向欧拉 | `(I − A·Ts)⁻¹` | `(I − A·Ts)⁻¹·B·Ts` |
| 双线性(Tustin) | `(I − A·Ts/2)⁻¹(I + A·Ts/2)` | `(I − A·Ts/2)⁻¹·B·Ts` |
| 零阶保持(ZOH) | `expm(A·Ts)` | `A⁻¹(F − I)B` |

`expm` 用缩放+泰勒级数+平方实现，`F` 的极点模决定稳定性。

## 3. 精度对比

与连续参考响应的最大绝对误差：

| 采样周期 | 前向欧拉 | 后向欧拉 | Tustin | ZOH |
| --- | --- | --- | --- | --- |
| 0.01 s | 0.01518 | 0.01451 | 0.00006 | 0.00000 |
| 0.05 s | 0.08358 | 0.06662 | 0.00161 | 0.00000 |
| 0.10 s | 0.19087 | 0.12001 | 0.00643 | 0.00000 |
| 0.20 s | 0.49521 | 0.20327 | 0.02601 | 0.00000 |

Tustin 比前向欧拉小 1~2 个数量级；ZOH 在采样点上与连续参考一致（误差 < 2e-13）。

## 4. 稳定性边界

| 方法 | 临界采样周期 | 对应 wn·Ts |
| --- | --- | --- |
| 前向欧拉 | **0.2709 s** | 0.813 |
| 后向欧拉 | 始终稳定 | — |
| Tustin | 始终稳定 | — |
| ZOH | 始终稳定 | — |

前向欧拉是本算例里唯一会失稳的方法：`Ts > 0.27 s` 时闭环极点跑出单位圆，
节点会打印「输出发散，停止本轮」。

## 5. 构建与运行

```bash
cd ~/ros2 && source /opt/ros/humble/setup.bash
colcon build --packages-select control_discretization && source install/setup.bash
ros2 launch control_discretization main.launch.py method:=tustin ts:=0.05
cd src/ground/control_discretization && python3 -m control_discretization.main
```

节点按 `ts` 实时推进离散状态方程，发布 `/control_discretization/y` 与
`/control_discretization/metrics`；把 `method:=forward ts:=0.3` 传进去可以现场看到发散。

## 6. 已知限制

1. 只讨论线性定常二阶对象，不含纯延迟（需要 Padé 或增广状态）；
2. `expm` 是自实现的数值方法，未与 scipy 逐位比对，只用解析解校验；
3. 未讨论定点实现（Q 格式、舍入误差）；
4. 节点用定时器近似真实采样，未接控制器硬件。

## 7. 大模型使用声明

使用大模型辅助生成代码、测试与文档；作者已复核离散化公式与实测数据，并对提交内容负责。
