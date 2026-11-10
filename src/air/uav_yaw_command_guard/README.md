# uav_yaw_command_guard

无人机偏航指令保护模块。它把目标偏航角与当前偏航角之间的差值转换到
`[-pi, pi)`，从而在 `+179°` 与 `-179°` 之间选择 2° 短路径，并按最大偏航角速度
限制 `Twist.angular.z`。目标或当前航向反馈超时后输出零角速度，避免继续追踪失效
指令或使用陈旧航向。

## 运行环境

- Ubuntu 22.04.5 LTS
- ROS 2 Humble
- Python 3.10.12

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_yaw_command_guard
source install/setup.bash
ros2 launch uav_yaw_command_guard main.launch.py duration:=8
```

演示结束时 launch 自动关闭两个节点。`duration:=0` 表示持续运行，需要手动
`Ctrl+C`。

## ROS 2 接口

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/uav/yaw/current` | `std_msgs/Float64` | 输入 | 当前偏航角，弧度 |
| `/uav/yaw/target` | `std_msgs/Float64` | 输入 | 原始目标偏航角，弧度 |
| `/uav/yaw/rate_cmd` | `geometry_msgs/Twist` | 输出 | 只填写 `angular.z`，单位 rad/s |
| `/uav/yaw/guard_status` | `std_msgs/String` | 输出 | 当前角、目标角、最短误差、模式和限幅状态 |

状态 JSON 示例：

```json
{
  "mode": "LIMITED",
  "current_yaw_deg": 45.0,
  "target_yaw_deg": -170.0,
  "error_deg": -145.0,
  "rate_deg_s": -90.0,
  "limited": true
}
```

CSV 默认写入：

```text
/tmp/uav_yaw_command_guard_runtime.csv
```

字段为：

```text
time_s,mode,current_yaw_deg,target_yaw_deg,error_deg,rate_deg_s,limited
```

正常跟踪时 `mode` 为 `TRACKING / LIMITED / ALIGNED`；目标超时为 `HOLD`；
当前航向反馈超时为 `CURRENT_STALE`。后两种模式均输出零角速度。

## 测试

```bash
cd src/air/uav_yaw_command_guard
PYTHONPATH=. python3 -m unittest discover -s tests -v
```

## 离线演示

不启动 ROS 2 时也可以复现相同的核心算法：

```bash
PYTHONPATH=. python3 -m uav_yaw_command_guard.demo --output-dir artifacts/demo
```

离线入口会输出 CSV、JSON 摘要和曲线图，用于检查回绕、限幅与目标保持行为。
若系统没有 Matplotlib，脚本会使用 Pillow 生成同一条曲线的静态 PNG；两者都未
安装时仍会输出 CSV 和 JSON。

本次 Windows / Python 3.14 离线闭环运行 8 秒、20 Hz，共 161 个样本，其中 82 个
样本触发限幅，最大角速度 `90°/s`，最终误差 `0°`。

## 边界

本模块输出的是偏航角速度建议，不直接控制飞控。实际接入时必须确认飞控坐标系、
角速度正方向和安全限值；当前演示不对风扰、执行器延迟或航向估计误差建模。

## 大模型使用声明

使用 OpenAI Codex 辅助完成代码、测试和文档，作者已复核提交内容并对数据真实性
与后续维护负责。
