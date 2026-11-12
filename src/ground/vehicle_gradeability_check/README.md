# vehicle_gradeability_check

车辆爬坡能力与牵引力余量核算模块。它根据整车质量、坡度、车速、滚动阻力、
空气阻力、驱动牵引力上限和轮胎路面附着系数，核算当前坡度所需牵引力、
可传递的最大牵引力以及牵引力余量，并区分 `CLIMBABLE`（可爬坡）、
`POWER_LIMIT`（动力不足）和 `TRACTION_LIMIT`（附着打滑风险）三种状态。
坡度或车速反馈超时后输出零牵引力建议，避免依据陈旧数据继续给油。

## 运行环境

- Ubuntu 22.04.5 LTS
- ROS 2 Humble
- Python 3.10.12

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select vehicle_gradeability_check
source install/setup.bash
ros2 launch vehicle_gradeability_check main.launch.py duration:=10
```

演示结束时 launch 自动关闭两个节点。`duration:=0` 表示持续运行，需要手动
`Ctrl+C`。

## ROS 2 接口

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/vehicle/grade/slope` | `std_msgs/Float64` | 输入 | 当前道路坡度，度 |
| `/vehicle/grade/speed` | `std_msgs/Float64` | 输入 | 当前车速，m/s |
| `/vehicle/grade/traction_cmd` | `geometry_msgs/Twist` | 输出 | 仅填写 `linear.x`，建议牵引力，N |
| `/vehicle/grade/grade_status` | `std_msgs/String` | 输出 | 所需/可用牵引力、余量和状态 JSON |

状态 JSON 示例：

```json
{
  "mode": "POWER_LIMIT",
  "slope_deg": 24.0,
  "speed_mps": 5.0,
  "required_n": 6157.0,
  "drive_limit_n": 4500.0,
  "traction_limit_n": 7998.0,
  "available_n": 4500.0,
  "margin_n": -1657.5,
  "slip_risk": false,
  "power_deficit": true
}
```

CSV 默认写入：

```text
/tmp/vehicle_gradeability_check_runtime.csv
```

字段为：

```text
time_s,mode,slope_deg,speed_mps,required_n,drive_limit_n,traction_limit_n,available_n,margin_n,slip_risk,power_deficit
```

正常爬坡时 `mode` 为 `CLIMBABLE`；驱动力不足为 `POWER_LIMIT`；
附着极限不足为 `TRACTION_LIMIT`；坡度反馈超时为 `SLOPE_STALE`，
车速反馈超时为 `SPEED_STALE`，后三种状态均输出零牵引力建议。

## 测试

```bash
cd src/ground/vehicle_gradeability_check
PYTHONPATH=. python3 -m unittest discover -s tests -v
```

## 离线演示

不启动 ROS 2 时也可以复现相同的核心算法：

```bash
PYTHONPATH=. python3 -m vehicle_gradeability_check.demo --output-dir artifacts/demo
```

离线入口会输出 CSV、JSON 摘要和曲线图，用于检查坡度需求、牵引力余量与
动力不足状态。若系统没有 Matplotlib，脚本会使用 Pillow 生成同一条曲线的
静态 PNG；两者都未安装时仍会输出 CSV 和 JSON。

本次 Windows / Python 3.12 离线闭环运行 10 秒、20 Hz，共 201 个样本，
其中 161 个样本可爬坡、40 个样本动力不足，最小牵引力余量 `-1657.5 N`，
6 m/s 车速下可爬坡度约 `17.0°`。

## 边界

本模块输出的是牵引力建议与状态判断，不直接控制油门或制动。实际接入时
必须确认车辆质量、传动效率、载荷转移、路面附着和坡度估计误差；当前演示
不对坡起步溜车、换挡过程、轮胎侧偏或坡度动态变化建模。

## 大模型使用声明

使用大模型辅助完成代码、测试和文档，作者已复核提交内容并对数据真实性
与后续维护负责。
