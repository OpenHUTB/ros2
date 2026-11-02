# tb3_vfh_avoidance

TurtleBot3 激光雷达 **VFH（向量场直方图）局部避障**（ROS 2 Humble）。

72 扇区极坐标直方图 + 配置空间膨胀（车身半径按角投影）+ 谷值搜索，
内置安全停障、航向平滑与死锁恢复。算法核心纯 Python、无 ROS 依赖，
可在没有 Gazebo 的机器上离线闭环仿真复现全部结果。

## 快速开始

```bash
# 1. 单元测试（17 项，无需 ROS）
python -m pytest test -q

# 2. 离线闭环仿真（VFH vs 朴素三扇区，输出轨迹对比图）
python tools/offline_demo.py

# 3. ROS 2 节点
python main.py                       # 或 ros2 launch tb3_vfh_avoidance avoid.launch.py

# 4. 无 Gazebo 联调（另一终端发布仿真 LaserScan）
python tools/publish_synthetic_scan.py --seconds 16
```

## 话题

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/scan` | `sensor_msgs/LaserScan` | 激光雷达数据（`angle_min=0`，`ranges[0]` 为正前方） |
| 订阅 | `vfh_goal_bearing` | `std_msgs/Float32` | 可选：外部期望航向（弧度，机体系） |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` | 速度指令 |
| 发布 | `vfh_state` | `std_msgs/String` | 决策状态（cruise/avoid/blocked/turn/guard/recovery） |

详细算法说明、实测数据与实现修复记录见
[`docs/ground/tb3_vfh_avoidance.md`](../../../docs/ground/tb3_vfh_avoidance.md)。

## 大模型使用声明

本模块代码与文档使用了大模型工具（WorkBuddy）辅助生成，
作者已逐行核对并实际运行验证，对提交内容负全部责任。
