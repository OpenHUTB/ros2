# 车辆 CAN 报文信号解码与一致性校验

CAN 总线上一条报文只有 8 字节，信号按位域打包，还带 CRC 和滚动计数器。本页给出
位域解码（Intel/Motorola）、CRC8 校验与计数器检查的完整流程，并用注入故障的数据说明
「不校验就直接用」会错到什么程度。

## 1. 为什么需要这一层

整车上 `VehicleSpeed`、`MotorTorque` 这类信号都是按 `起点 + 长度 + 缩放 + 偏移` 打包的，
解析错一位就会把 300 km/h 当成真实车速。总线干扰、ECU 重启、报文丢失都会让数据失真，
所以接收端必须同时做三件事：**位域还原、CRC 校验、计数器连续性检查**。

## 2. 位域约定

```text
把 8 字节报文看成 64 位整数，byte0 的最低位是第 0 位：
- little（Intel）：从 start_bit 起向高位连续取 length 位
- big（Motorola）：start_bit 是信号的最高位，之后同字节内递减、跨字节跳到下一字节最高位
    start_bit=7, length=16 → 7,6,5,4,3,2,1,0,15,14,13,12,11,10,9,8
物理值 = raw * scale + offset（有符号信号按补码还原）
```

## 3. 报文定义

| 报文 | 信号 | 起始位 | 长度 | 字节序 | 缩放 | 量程 |
| --- | --- | --- | --- | --- | --- | --- |
| 0x100 VCU_Status | VehicleSpeed | 0 | 16 | little | 0.01 | 0~250 km/h |
| | MotorTorque | 16 | 16 | little | 0.1（有符号） | ±300 Nm |
| | Gear | 32 | 3 | little | 1 | — |
| | BrakePressure | 35 | 8 | little | 0.5 | 0~127.5 bar |
| | AliveCounter | 43 | 4 | little | 1 | 0~15 |
| 0x101 Steering_Status | SteeringAngle | 7 | 16 | Motorola | 0.1（有符号） | ±780 deg |
| | AliveCounter | 32 | 4 | little | 1 | 0~15 |

两条报文的 byte7 都是 CRC8/SAE J1850，校验范围 byte0~byte6。

## 4. 校验结果

注入 3% CRC 错误、2% 丢帧、2% 超量程后（20 s @ 50 Hz）：

| 报文 | 帧数 | CRC 失败 | 计数器跳变 | 越界帧 | 通过率 |
| --- | --- | --- | --- | --- | --- |
| VCU_Status | 1000 | 35 | 17 | 28 | 92.4% |
| Steering_Status | 334 | 9 | 7 | 2 | 95.2% |

丢弃 92/1334 = 6.9% 的帧之后，信号统计回到真值范围（车速 35~85 km/h、扭矩 ±120 Nm、
挡位 3~5）。**不校验时**同一批数据的极值会被污染成 403 km/h 与 3198 Nm 的异常值。

## 5. 构建与运行

```bash
cd ~/ros2 && source /opt/ros/humble/setup.bash
colcon build --packages-select can_signal_decode && source install/setup.bash
ros2 launch can_signal_decode main.launch.py
cd src/ground/can_signal_decode && python3 -m can_signal_decode.main
```

节点话题：订阅 `/can/frame`（`UInt8MultiArray`，4 字节帧 ID + 8 字节载荷），
发布 `/can/signals`（`Float64MultiArray`）与 `/can/decode_status`（`String`）。

## 6. 已知限制

1. 只支持标准帧（11 bit）与单帧 8 字节，不涉及 CAN FD、ISO-TP 多帧；
2. 位域按 DBC 约定实现，但未支持 DBC 文件解析；
3. 只实现 CRC8/SAE J1850，真实整车还有 CRC16 与 E2E Profile；
4. 故障注入是独立随机事件，不模仿突发干扰与总线负载；
5. 未接入真实 CAN 硬件（socketcan / candump）。

## 7. 大模型使用声明

使用 **DeepSeek 的 deepseek-flash 模型**（通过 DeepSeek Harness 客户端调用）辅助生成
位域/CRC 的实现方案、单元测试与文档初稿。

所有位序约定、CRC 参数与实测数字均由作者逐项复核，并在 Ubuntu 22.04.5 + ROS 2 Humble
上实跑确认（`colcon test` 15 项全部通过）；作者对提交内容负全部责任。
