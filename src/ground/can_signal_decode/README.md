# can_signal_decode 车辆 CAN 报文解码与校验

把 8 字节 CAN 载荷按位域还原成物理量，并同时校验 **CRC8(SAE J1850)** 和 **滚动计数器**，
用量化数据说明「不校验就直接用」会造成多大的错误采样。

与仓库已有模块的区别：

- `ros-bridge`、`carlair_ros_bridge` 是把 CARLA/AirSim 的数据搬到 ROS，不涉及 CAN 位域；
- `imu_allan_noise_identification`、`encoder_quantization_velocity` 处理的是传感器噪声与量化；
- 本模块只做**报文层**：位域（Intel/Motorola）、缩放偏移、CRC 与计数器，无仿真器、无外部依赖。

## 1. 运行环境

- 操作系统：Ubuntu 22.04.5 LTS（实测）
- ROS 版本：ROS 2 Humble
- Python：3.10.12
- 依赖：`rclpy`、`std_msgs`（Humble Desktop 自带）；离线入口只用 Python 标准库

## 2. 编译与测试

```bash
cd ~/ros2 && source /opt/ros/humble/setup.bash
colcon build --packages-select can_signal_decode
source install/setup.bash
colcon test --packages-select can_signal_decode && colcon test-result --verbose
```

## 3. 运行

### 3.1 离线入口（不需要 ROS）

```bash
cd src/ground/can_signal_decode
python3 -m can_signal_decode.main
```

实测输出（20 s @ 50 Hz，注入 3% CRC 错误、2% 丢帧、2% 超量程）：

```text
CRC8/SAE-J1850 自检: crc8(b'123456789') = 0x4B（期望 0x4B）

报文校验统计（分母为该报文总帧数）
报文                     帧数   CRC失败    计数跳变    越界帧     通过率
VCU_Status             1000       35        17      28    92.4%
Steering_Status         334        9         7       2    95.2%

被丢弃的帧: 92 / 1334 （6.9%）

通过校验后的信号统计
信号              单位          最小        最大        均值
VehicleSpeed     km/h       35.00     85.00     60.44
MotorTorque      Nm       -120.00    120.00      6.60
Gear                          3.00      5.00      3.92
BrakePressure     bar         0.00     18.00      5.99
SteeringAngle    deg      -210.00    210.00     31.04
```

对比一下：不做校验时这两条报文的 `VehicleSpeed` 极值会被污染到 **403 km/h**、
`Gear` 会跳到 **5**（真值只有 3~5 的整数挡位但会出现非法组合）、`MotorTorque` 出现 −3198 Nm。
这正是 CRC + 计数器必须在解码前检查的原因。

### 3.2 ROS 2 节点

```bash
ros2 launch can_signal_decode main.launch.py
```

- 订阅 `/can/frame`（`std_msgs/UInt8MultiArray`）：前 4 字节帧 ID（小端），后 8 字节载荷
- 发布 `/can/signals`（`std_msgs/Float64MultiArray`）：按信号表顺序的物理量
- 发布 `/can/decode_status`（`std_msgs/String`）：`crc=ok counter=ok range=ok accepted=..` 

手工灌一帧（0x100，车速 60.00 km/h、扭矩 0、4 挡、制动 0，末字节 223 是该帧的 CRC8）：

```bash
ros2 topic pub --once /can/frame std_msgs/msg/UInt8MultiArray \
  "{data: [0,1,0,0, 112,23, 0,0, 4,0, 0,223]}"
ros2 topic echo /can/signals --once
```

把 CRC 字节改成别的值（例如 0）可以看到节点按 `require_crc=true` 整帧丢弃并打日志。

## 4. 报文定义

| 报文 | 信号 | 起始位 | 长度 | 字节序 | 缩放/偏移 | 单位 |
| --- | --- | --- | --- | --- | --- | --- |
| 0x100 VCU_Status | VehicleSpeed | 0 | 16 | little | 0.01 | km/h |
| | MotorTorque | 16 | 16 | little | 0.1（有符号） | Nm |
| | Gear | 32 | 3 | little | 1 | — |
| | BrakePressure | 35 | 8 | little | 0.5 | bar |
| | AliveCounter | 43 | 4 | little | 1 | — |
| 0x101 Steering_Status | SteeringAngle | 7 | 16 | Motorola | 0.1（有符号） | deg |
| | AliveCounter | 32 | 4 | little | 1 | — |

两条报文的 byte7 都是 CRC8/SAE J1850，校验范围为 byte0~byte6。

## 5. 校验规则

| 检查项 | 规则 | 不通过时 |
| --- | --- | --- |
| CRC | `CRC8(payload[0:7]) == payload[7]`，多项式 0x1D、初值 0xFF、结果异或 0xFF | 整帧丢弃 |
| 滚动计数器 | 4 bit，逐帧 +1，15→0 回绕 | 记为丢帧，仍可按 `require_crc` 策略决定是否使用 |
| 量程 | 每个信号有 `minimum`/`maximum` | 记为越界帧 |

## 6. 简化假设

- 只支持标准帧 ID（11 bit）、单帧 8 字节，不涉及 CAN FD、多帧传输（ISO-TP）；
- 位域顺序按 DBC 的 Intel/Motorola 约定实现，未支持 DBC 文件的自动解析；
- 只实现 CRC8/SAE J1850，真实整车还会用到 CRC16、E2E Profile 等机制；
- 故障注入是匀速率的独立随机事件，不模仿真实总线的突发干扰与总线负载；
- 未接入真实 CAN 硬件（socketcan / candump 日志），节点输入是手工构造的字节数组。

## 7. 大模型使用声明

使用 **DeepSeek 的 deepseek-flash 模型**（通过 DeepSeek Harness 客户端调用）辅助完成：
位域（Intel/Motorola）与 CRC8 的实现方案、单元测试用例、README 与教程页初稿、
以及运行输出与统计表格的整理。

所有公式推导、位序约定、CRC8 参数、量程设置与实测数字均由作者逐项复核，并在
Ubuntu 22.04.5 + ROS 2 Humble + Python 3.10.12 上实跑确认（`colcon build`、
`colcon test` 15 项、`ros2 launch`）；作者对提交内容与后续维护负全部责任。
