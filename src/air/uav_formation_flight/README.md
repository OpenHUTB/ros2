# uav_formation_flight

无人机编队飞行实验包：用 Leader-Follower 结构驱动 OpenHUTB（AirSim 接口）中的
三架四旋翼保持给定队形，完成直线巡航、圆形编队与队形切换三类任务，记录全部
载具的真实轨迹，并给出队形保持误差、编队成形时间与队形切换过渡时间等量化指标。

完整实验说明见站点文档：`docs/air/uav_formation_flight.md`

## 文件结构

| 文件 | 说明 |
| --- | --- |
| `scripts/formation_controller.py` | 主节点：三架依次起飞、长机跟踪航点、僚机跟踪队长机位形偏移、记录 CSV 并输出指标 |
| `scripts/plot_formation.py` | 读取日志绘制队形误差曲线、航迹俯视图与指标柱状图 |
| `scripts/fleet_check.py` | 多机连接自检：列出模拟器载具、逐架 RPC 与起降验证 |
| `scripts/lag_probe.py` | 量「速度指令 → 实际速度」的延迟，并判定垂直通道在移动时是否被执行 |
| `scripts/tune_hover.py` | 单机被控对象辨识与位置环增益试跑（不录屏） |
| `scripts/deploy_probe.py` | 单独复现展开阶段，并打印每次碰撞的对象名 |
| `scripts/vehicle_diag.py` | 逐架对比 AirSim 状态估计与真值、量执行机构响应 |
| `config/params.yaml` | 连接参数、飞行参数、PID 增益、队形定义与三套任务 |
| `launch/main.launch` | 启动入口，支持覆盖任务名与 PID 增益 |

## 依赖

- ROS Noetic（`rospy`、`nav_msgs`、`geometry_msgs`、`visualization_msgs`）
- AirSim 1.8.1 模拟器（宿主机）与 `airsim` Python 客户端（客户机）
- 模拟器侧需在 `settings.json` 的 `Vehicles` 中声明三架载具
- `matplotlib`（仅出图脚本需要，缺失时 `sudo apt install -y python3-matplotlib`）

## 快速开始

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rosrun uav_formation_flight fleet_check.py
roslaunch uav_formation_flight main.launch mission:=line_straight
```
