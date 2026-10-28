# ros2_service_param_demo

ROS 2 **服务（service）与参数（parameter）** 基础实验：用「无人机模式节点」演示模式切换、运行中调参与状态发布。

接口表、常见问题、退出提示说明与截图来源见[完整文档](../../../docs/chap2/ros2_service_param_demo.md)。

本模块是「基于强化学习的四旋翼无人机飞行控制」项目的第二个前置基础实验：`ros2_pubsub_py` 验证了节点之间的单向消息流，本实验补齐**模式切换**（调用方需要拿到成功或失败）与**在线调参**（不重启节点改发布频率）这两项基础设施。只演示 ROS 2 服务与参数机制，不连接仿真器、不发送任何飞控指令、不使用自定义消息。

## 环境与运行

| 项目 | 版本 / 说明 |
|---|---|
| 操作系统 | Ubuntu 22.04 LTS（VMware 虚拟机），宿主为 Windows 11 |
| ROS | ROS 2 Humble |
| Python | 3.10（Ubuntu 22.04 自带，与 Humble 匹配） |
| 依赖 | 仅 ROS 2 自带功能包：`rclpy`、`std_msgs`、`std_srvs`、`launch`、`launch_ros` |

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cp -r <仓库路径>/src/chap2/ros2_service_param_demo ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select ros2_service_param_demo
source install/setup.bash
ros2 launch ros2_service_param_demo service_param_demo.launch.py
```

等价的入口还有 `python3 ~/ros2_ws/src/ros2_service_param_demo/main.py` 和 `ros2 run ros2_service_param_demo uav_mode_node`。

验证接口（另开终端，两套环境都要 source）：

```bash
ros2 service call /uav/set_mode std_srvs/srv/SetBool "{data: true}"
ros2 param set /uav_mode_node publish_rate 5.0
ros2 topic echo /uav/status
```

## 使用声明

使用 ChatGPT / Codex 辅助实现与文档整理；提交者负责审阅、理解并对提交内容负责。
