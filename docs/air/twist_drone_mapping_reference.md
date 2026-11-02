# geometry_msgs/Twist 与无人机机体系速度映射对照表

> ROS 速度指令消息与 OpenHUTB/AirSim NED 机体速度的映射关系, 供桥接类模块查阅。

## 1. 消息定义

```
geometry_msgs/Twist:
    Vector3  linear    # 线速度 (m/s)
    Vector3  angular   # 角速度 (rad/s)
```

## 2. ROS (REP-103) 速度符号约定

| 分量 | 正方向 | 说明 |
| --- | --- | --- |
| `linear.x` | 机头向前 | 前进 |
| `linear.y` | 机体左侧 | 左移 |
| `linear.z` | 向上 | 上升 |
| `angular.z` | 逆时针 (俯视) | 左旋 |

## 3. 映射到 OpenHUTB (NED) 速度

OpenHUTB AirSim 模式 `moveByVelocityAsync(vx, vy, vz, duration, drivetrain, yaw_mode)` 中的速度为 **NED 机体系**: x 前, y 右, z 下。

| ROS Twist 分量 | NED 参数 | 转换 |
| --- | --- | --- |
| `linear.x` | `vx` | 不变 |
| `linear.y` | `vy = -linear.y` | 取反 (左→右) |
| `linear.z` | `vz = -linear.z` | 取反 (上→下) |
| `angular.z` | `yaw_or_rate = -angular.z` | 取反 (逆时针→顺时针), YawMode(is_rate=True) |

## 4. 代码模板

```python
# ENU cmd_vel -> NED 机体系 (桥接节点核心转换)
vx = cmd.linear.x
vy = -cmd.linear.y          # ROS 左移为正 -> NED 右移为正
vz = -cmd.linear.z          # ROS 上升为正 -> NED 下降为正
yaw_rate = -cmd.angular.z   # ROS 逆时针为正 -> NED 顺时针为正

client.moveByVelocityAsync(
    vx, vy, vz, duration=0.2,
    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
    yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate))
```

## 5. 状态反向转换 (无人机 -> ROS)

定位输出 (如 `nav_msgs/Odometry`) 需将模拟器 NED 状态转回 ENU:

```python
# 位置/速度: y、z 取反
odom.pose.position.x = p.x_val
odom.pose.position.y = -p.y_val
odom.pose.position.z = -p.z_val
# 四元数: y、z 分量取反
odom.pose.orientation = (q.w, q.x, -q.y, -q.z)
```

## 6. 常见坑

1. **忘记取反 y/z**: 无人机"指令向左、飞向右", 是 NED/ENU 未转换的最典型症状;
2. **偏航方向**: ROS 逆时针为正 (右手系 z 向上), NED 顺时针为正 (z 向下), 忘记取反会导致原地打转;
3. **绝对位置指令** `moveToPositionAsync` 同样是 NED: 想飞 10m 高要传 `z=-10`;
4. **速度持续时间**: `moveByVelocityAsync` 的 duration 应略大于控制周期, 避免指令间隙掉速。

## 7. 大模型使用声明

本文档使用大模型工具（WorkBuddy）辅助整理，映射关系经仓库模块 ned_enu_transform 与 drone_client 实际运行验证，作者对提交内容负全部责任。
