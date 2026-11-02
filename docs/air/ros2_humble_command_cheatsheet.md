# ROS 2 Humble 常用命令速查表

> 整理自 ROS 2 Humble 官方文档, 供课程实验快速查阅。对应官方教程: <https://docs.ros.org/en/humble/Tutorials.html>

## 1. 环境与工作空间

```bash
source /opt/ros/humble/setup.bash       # 每个终端都要 source (或写入 ~/.bashrc)
printenv ROS_DISTRO                     # 检查环境: 应输出 humble
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws  # 创建工作空间
colcon build                            # 编译
colcon build --packages-select <pkg>    # 只编译某个包
source install/setup.bash               # 编译后叠加工作空间
```

## 2. 节点

```bash
ros2 run <package> <executable>             # 运行节点
ros2 node list                              # 列出运行中的节点
ros2 node info /node_name                   # 查看节点的发布/订阅/服务
ros2 run <pkg> <exec> --ros-args -p key:=value   # 传参数启动
```

## 3. 话题

```bash
ros2 topic list                 # 列出所有话题
ros2 topic list -t              # 附带消息类型
ros2 topic echo /chatter        # 打印话题数据
ros2 topic hz /chatter          # 查看发布频率
ros2 topic bw /image_raw        # 查看带宽
ros2 topic info /chatter        # 话题类型与订阅者数量
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.5}}" -r 10   # 手动发布 (10Hz)
```

## 4. 服务与动作

```bash
ros2 service list                       # 服务列表
ros2 service call /spawn turtlesim/srv/Spawn \
  "{x: 2.0, y: 2.0, name: turtle2}"     # 调用服务
ros2 action list                        # 动作列表
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {...}}"
```

## 5. 参数

```bash
ros2 param list                 # 列出节点参数
ros2 param get /node param_name # 读取参数
ros2 param set /node param_name value   # 动态设置参数
ros2 param dump /node > p.yaml  # 导出参数
```

## 6. 排查工具

```bash
ros2 doctor                             # 环境体检
ros2 interface show geometry_msgs/msg/Twist   # 查看消息定义
ros2 daemon stop && ros2 daemon start   # 发现守护进程异常时重启
export ROS_DOMAIN_ID=42                 # 多组实验隔离 (同网段互不干扰)
```

## 7. 常见问题

| 现象 | 处理 |
| --- | --- |
| `ros2: command not found` | 忘记 source `/opt/ros/humble/setup.bash` |
| 找不到别人的节点 | 两台机器 ROS_DOMAIN_ID 或 DDS 实现不一致 |
| `colcon: command not found` | `sudo apt install python3-colcon-common-extensions` |
| topic echo 无输出 | 检查发布端频率 (`ros2 topic hz`) 与防火墙 |

## 8. 大模型使用声明

本文档使用大模型工具（WorkBuddy）辅助整理，内容对照 ROS 2 Humble 官方文档核实，作者对提交内容负全部责任。
