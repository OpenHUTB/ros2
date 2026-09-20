# 基于 ROS 消息解耦的无人机键盘遥控

本文介绍如何通过键盘遥控 Windows 宿主机上 AirSim 仿真环境中的多旋翼无人机。与直接在脚本中调用
AirSim 客户端不同，本方案将**键盘输入**与**仿真器控制**拆分为两个独立的 ROS 节点，二者之间只通过
`/drone/cmd_vel` 话题（`geometry_msgs/Twist`）交换数据，从而实现消息层面的解耦。

代码位于 `src/air/air_teleop/` 目录：

| 文件 | 节点名 | 职责 |
| --- | --- | --- |
| `drone_ros_teleop.py` | `drone_ros_teleop` | 用 `pynput` 监听键盘，按 10 Hz 把当前按键状态发布为 `Twist` 消息 |
| `drone_ros_node.py` | `drone_ros_node` | 订阅 `/drone/cmd_vel`，经 AirSim RPC 驱动无人机 |

## 双节点架构

```mermaid
graph LR
    K[键盘输入] -->|pynput 监听| T[drone_ros_teleop 键盘发布节点]
    T -->|/drone/cmd_vel Twist 10 Hz| B[drone_ros_node 桥接控制节点]
    B -->|AirSim RPC 41451| A[Windows 宿主机 AirSim 模拟器]
```

两个节点都运行在虚拟机的 ROS 主控上，只有桥接节点与宿主机上的 AirSim 发生网络通信。解耦带来的好处：

* 键盘节点不关心仿真器细节，桥接节点不关心输入设备；
* 更换输入方式（摇杆、路径规划器等）时，只需向 `/drone/cmd_vel` 发布同格式的消息；
* 任一节点异常退出都不影响另一节点的安全收尾（详见下文安全机制一节）。

## 坐标系约定

AirSim 使用 **NED 坐标系**：x 轴指向前方、y 轴指向右侧、z 轴指向地面。因此 `/drone/cmd_vel`
中的消息内容按 NED 机体坐标系解释：

| Twist 字段 | 含义 | 备注 |
| --- | --- | --- |
| `linear.x` | 机体 x 轴线速度（m/s） | 正值前进 |
| `linear.y` | 机体 y 轴线速度（m/s） | 正值右移 |
| `linear.z` | 机体 z 轴线速度（m/s） | **负值上升、正值下降** |
| `angular.z` | 偏航角速度（rad/s） | 正值右偏航（顺时针，从上方看） |

## 安装依赖

```shell
# 拉取仓库并进入仓库根目录
git clone https://github.com/OpenHUTB/ros2.git
cd ros2
# 安装两个节点运行所需的依赖（airsim、pynput）
python -m pip install -r src/requirements.txt
```

### 安装 rospy 运行环境

两个节点均基于 `rospy` 编写，若新环境中仅按上述步骤安装依赖，运行节点时可能报错：

```text
ModuleNotFoundError: No module named 'rospy'
```

此时需补齐 ROS 的 Python 运行环境，依次执行：

```shell
sudo apt-get install python3-roslib
pip install rospkg
pip install catkin-tools
source /opt/ros/noetic/setup.bash
python -c "import rospy"   # 无报错即表示 rospy 可用
```

若 AirSim 客户端连接报 numpy 相关错误，请参考[建立虚拟机和空域载具之间的连接](./setup_and_connect.md)
安装兼容版本的 `numpy` 与 `msgpack-rpc-python`。

## 启动步骤

### 1. 启动宿主机上的 AirSim 模拟器

在 Windows 宿主机上运行（以 AbandonedPark 场景为例）：

```bat
cd AbandonedPark\WindowsNoEditor\
AbandonedPark.exe
```

![](../img/air/abandoned_park.png)

### 2. 在虚拟机中启动 ROS 主控

```shell
source /opt/ros/noetic/setup.bash
roscore
```

### 3. 启动桥接控制节点

另开一个终端，`--host` 后面填宿主机（Windows）的 IP 地址（通过 `ipconfig` 查看）：

```shell
source /opt/ros/noetic/setup.bash
python src/air/air_teleop/drone_ros_node.py --host 172.21.108.47
```

!!! 注意
    桥接节点必须能访问宿主机 AirSim RPC 服务的 41451 端口。连接失败时请检查：
    宿主机防火墙是否放行 41451 端口；虚拟机网络模式（NAT/桥接）下能否 `ping` 通宿主机 IP。
    宿主机 IP 一般和这里的 `172.21.108.47` 不一致，以 `ipconfig` 实际输出为准。

### 4. 启动键盘发布节点

再开一个终端：

```shell
source /opt/ros/noetic/setup.bash
python src/air/air_teleop/drone_ros_teleop.py
```

桥接节点收到第一条速度指令后会自动解锁、起飞并进入遥控状态，模拟器窗口中可以看到无人机起飞：

![](../img/air/hello_drone.png)

## 实飞效果

按上述步骤启动后，在键盘节点窗口中按住 `W`/`A`/`S`/`D` 等按键即可遥控无人机，
完整实飞演示见下方动图：

![键盘遥控无人机实飞演示](../img/air/demo.gif)

实飞过程中的模拟器画面截图：

![实飞画面截图](../img/air/test.png)

## 参数配置

桥接节点同时支持命令行参数与 ROS 私有参数，**命令行参数优先级更高**：

| 命令行参数 | 私有参数 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `--host` | `~host` | `127.0.0.1` | 宿主机（AirSim 服务）IP 地址 |
| `--port` | `~port` | `41451` | AirSim RPC 端口 |
| `--vehicle` | `~vehicle` | `""` | 载具名称，空串表示仿真器中的第一架无人机 |
| — | `~timeout` | `0.5` | 无指令超时时间（s），超时自动悬停 |
| — | `~rate` | `10.0` | 控制频率（Hz） |

通过参数服务器设置私有参数的示例（效果与 `--host` 相同）：

```shell
rosparam set /drone_ros_node/host 172.21.108.47
python src/air/air_teleop/drone_ros_node.py
```

## 键位映射

| 按键 | 功能 | NED 指令 |
| :---: | :---: | --- |
| `W` / `S` | 前进 / 后退 | `linear.x = ±3.0 m/s` |
| `A` / `D` | 左移 / 右移 | `linear.y = ∓3.0 m/s` |
| `Space` | 上升 | `linear.z = -2.0 m/s` |
| `Ctrl+P` | 下降 | `linear.z = +2.0 m/s` |
| `J` / `L` | 左偏航 / 右偏航 | `angular.z = ∓30 度/秒` |
| `ESC` | 退出键盘遥控 | 停止发布，桥接节点超时后自动悬停 |

**提示：**

* 按键松开后对应轴速度立即归零（键盘节点每个周期按当前按键集合重新计算）；
* 无任何按键时持续发布全零指令，无人机原地保持；
* 键盘节点需要运行在带图形桌面的终端中，SSH 纯命令行环境下 `pynput` 无法监听键盘。

## 安全机制

* **自动起飞**：桥接节点收到第一条速度指令时解锁电机并爬升到安全高度；
* **超时悬停**：连续 0.5 s 未收到任何指令（键盘节点退出、网络中断等）时自动悬停；
* **安全退出**：桥接节点退出（`Ctrl+C` 或 `rosnode kill`）时，通过 `rospy.on_shutdown`
  回调先悬停、再调用 `enableApiControl(False)` 释放 AirSim API 控制权；
* 键盘节点按 `ESC` 退出后，桥接节点因超时自动悬停，无需手工干预。

## 验证话题

```shell
source /opt/ros/noetic/setup.bash
rostopic list                 # 应能看到 /drone/cmd_vel
rostopic echo /drone/cmd_vel  # 按住 W 键可看到 linear.x = 3.0
```

## 常见问题

* **连接宿主机失败**：检查 `--host` 是否为宿主机 IP、宿主机防火墙是否放行 41451 端口；
* **键盘无响应**：确认键盘节点运行在虚拟机桌面的终端窗口中，且该窗口获得焦点；
* **无人机无动作**：用 `rostopic echo /drone/cmd_vel` 确认话题有消息，并确认桥接节点日志中已打印“已连接 AirSim 服务”。

## 参考

* [建立虚拟机和空域载具之间的连接](./setup_and_connect.md)
* [空域模拟器的 ROS 封装器](./ros_pkgs.md)
* [AirSim 官方 ROS 封装（airsim_ros_pkgs）](https://microsoft.github.io/AirSim/airsim_ros_pkgs/)
