# control_discretization 连续控制器离散化方法对比

同一个二阶连续对象，用**前向欧拉、后向欧拉、双线性(Tustin)、零阶保持(ZOH)** 四种方法离散化，
在采样周期 0.01~0.2 s 上对比精度与稳定性，并给出各自的临界采样周期。

与仓库已有模块的区别：

- `pid_auto_tuning` 讨论的是**PID 参数怎么整定**，不涉及连续→离散的变换；
- `encoder_quantization_velocity` 讨论测量量化误差，不是控制器的离散化；
- 本模块只关心「同一个连续控制器，用不同离散化方法实现，差多少、什么时候会失稳」。

## 1. 运行环境

- 操作系统：Ubuntu 22.04.5 LTS（实测）
- ROS 版本：ROS 2 Humble
- Python：3.10.12
- 依赖：`rclpy`、`std_msgs`（Humble Desktop 自带）；离线入口与测试只用标准库

## 2. 编译与测试

```bash
cd ~/ros2 && source /opt/ros/humble/setup.bash
colcon build --packages-select control_discretization
source install/setup.bash
colcon test --packages-select control_discretization && colcon test-result --verbose
```

## 3. 运行

### 3.1 离线入口（不需要 ROS）

```bash
cd src/ground/control_discretization
python3 -m control_discretization.main
```

实测（wn=3.0 rad/s，zeta=0.4，仿真 6 s）：

| 采样周期 | 前向欧拉 | 后向欧拉 | Tustin | ZOH | 稳定 |
| --- | --- | --- | --- | --- | --- |
| 0.01 s | 0.01518 | 0.01451 | 0.00006 | 0.00000 | 全部稳定 |
| 0.05 s | 0.08358 | 0.06662 | 0.00161 | 0.00000 | 全部稳定 |
| 0.10 s | 0.19087 | 0.12001 | 0.00643 | 0.00000 | 全部稳定 |
| 0.20 s | 0.49521 | 0.20327 | 0.02601 | 0.00000 | 全部稳定 |

表中数字是「与连续参考响应的最大绝对误差」。

临界采样周期（从 1 ms 扫到 2 s）：

| 方法 | 临界 Ts | 对应 wn·Ts |
| --- | --- | --- |
| 前向欧拉 | 0.2709 s | 0.813 |
| 后向欧拉 | 始终稳定 | — |
| 双线性(Tustin) | 始终稳定 | — |
| 零阶保持(ZOH) | 始终稳定 | — |

ZOH 在采样点上与连续参考的偏差 < 2e-13（理论上应为 0，差异只来自数值积分）。

### 3.2 ROS 2 节点

```bash
ros2 launch control_discretization main.launch.py
# 换方法 / 换采样周期
ros2 launch control_discretization main.launch.py method:=forward ts:=0.3
```

节点按 `ts` 起定时器，实时推进离散状态方程：

- 发布 `/control_discretization/y`（`std_msgs/Float64`）：每拍输出
- 发布 `/control_discretization/metrics`（`std_msgs/String`）：极点、最大误差、稳定性

`ts:=0.3` 配 `method:=forward` 可以看到节点报「输出发散，停止本轮」——这就是前向欧拉的
稳定性上限在实时通道里的表现。

## 4. 离散化公式

| 方法 | F | G |
| --- | --- | --- |
| 前向欧拉 | `I + A·Ts` | `B·Ts` |
| 后向欧拉 | `(I − A·Ts)⁻¹` | `(I − A·Ts)⁻¹·B·Ts` |
| 双线性(Tustin) | `(I − A·Ts/2)⁻¹(I + A·Ts/2)` | `(I − A·Ts/2)⁻¹·B·Ts` |
| 零阶保持(ZOH) | `expm(A·Ts)` | `A⁻¹(F − I)B` |

`expm` 用「缩放 + 泰勒级数 + 平方」实现，不依赖 numpy/scipy。

## 5. 结论

1. 采样周期越小，四种方法都收敛到连续响应；Ts 越小差距越不明显。
2. **前向欧拉是唯一会失稳的**：本对象在 Ts > 0.27 s 时极点跑出单位圆。
3. Tustin 的误差比前向/后向欧拉小 1~2 个数量级，且恒稳定，是控制器实现的首选。
4. ZOH 只在「对象是零阶保持输入」时才是精确解，用错了对象就没有这个性质。

## 6. 简化假设

- 只讨论线性定常二阶对象，不含纯延迟（纯延迟需要 Padé 近似或增广状态）；
- `expm` 是自实现的数值方法，未与 scipy 结果逐位比对，只在测试中校验了几组解析解；
- 未讨论定点实现（DSP 上的 Q 格式、舍入误差），那是另一个量级的问题；
- 节点用定时器近似真实采样，未接入实际控制器硬件。

## 7. 大模型使用声明

使用大模型辅助生成代码框架、单元测试与文档；作者已逐项复核离散化公式、极点计算与
实测数据，并对提交内容与后续维护负责。
