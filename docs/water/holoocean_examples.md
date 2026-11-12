# 水域载具示例


用于与 HoloOcean ROS 2 接口交互的示例节点。

## 示例

| Launch 文件 | 描述 |
|---|---|
| [joy_launch.py](https://github.com/OpenHUTB/ros2/blob/master/src/water/holoocean_examples/holoocean_examples/joy_holoocean.py) | 支持可选摄像头 HUD 的单手柄遥操作 |
| [multi_joy_launch.py](https://github.com/OpenHUTB/ros2/blob/master/src/water/holoocean_examples/launch/multi_joy_launch.py) | 与上文相同，但支持同时使用有线和无线手柄。 |
| [waypoint_launch.py](https://github.com/OpenHUTB/ros2/blob/master/src/water/holoocean_examples/launch/waypoint_launch.py) | 水面船舶的航路点跟踪 |
| [command_launch.py](https://github.com/OpenHUTB/ros2/blob/master/src/water/holoocean_examples/launch/command_launch.py) | 通过福森自动驾驶仪直接指令深度、航向和速度 |

---

## 游戏手柄示例

### 前提条件

如果尚未安装 `joy_linux` ROS 2 软件包，请进行安装：

```bash
sudo apt install ros-$ROS_DISTRO-joy-linux
```

连接您的控制器。确认设备已显示：

```bash
ls /dev/input/js*
```
您可以设置 udev 规则，将特定控制器映射到特定路径。

配置文件默认指向 `js0`。如果您的设备路径不同，请在 config YAML 文件中的 `joy_node`（或在多代理中的`joy_node_wired`）下调整 `dev`。


### 运行

```bash
ros2 launch holoocean_examples joy_launch.py
```

针对多个代理和多个控制器同时进行：

```bash
ros2 launch holoocean_examples multi_joy_launch.py
```

### Agent（代理）的加载方式

节点从配置中读取 `scenario_path`（该值与 `holoocean_node` 使用的值相同），并解析场景 JSON 文件以获取代理列表。您**无需**在 YAML 文件中列出代理，只需指定按钮映射即可。

`agents.buttons` 是一个并行数组，其索引 `i` 对应于场景文件中的第 `i` 个代理。例如，如果场景中定义的代理顺序为 `[auv0, auv1, auv2]`：

```yaml
agents.buttons: [0, 1, 3]   # auv0=A, auv1=B, auv2=Y
agents.default: 'auv0'
```

`agents.default` 用于设定启动时处于活动状态的代理。

---

### 查找按钮与轴的映射关系

查找特定控制器索引的最简单方法是：在按下按钮的同时，输出（echo） joy 话题（topic）的内容：

```bash
# 仅连接一个控制器时运行
ros2 run joy_linux joy_linux_node

# 在另一个终端中运行：
ros2 topic echo /joy
```

`buttons` 数组显示当前被按下的按钮状态（按下为 `1`）。`axes` 数组显示每个模拟输入（摇杆和扳机键）的当前值。

[ROS joy 维基页面](https://wiki.ros.org/joy)也提供了常见控制器的映射表。



### 默认映射（通过 joy_linux 使用 Xbox 控制器）

以下数值与 [joy_config.yaml](https://github.com/OpenHUTB/ros2/blob/master/src/water/holoocean_examples/config/joy_config.yaml)（有线控制器）及 [multi_joy_config.yaml](https://github.com/OpenHUTB/ros2/blob/master/src/water/holoocean_examples/config/multi_joy_config.yaml) 中的默认设置一致。


#### 按钮

| 索引 | Xbox 按钮 | 功能 |
|---|---|---|
| 0 | A | 选择代理 0 |
| 1 | B | 如果存在，选择代理 1 |
| 2 | X | 如果存在，选择代理 2 |
| 3 | Y | 如果存在，选择代理 3 |
| 6 | Back / 选择 | Disarm |
| 7 | Start | Arm |
| 8 | Guide (Xbox logo) | 重置模拟 |

[joy_config.yaml](https://github.com/OpenHUTB/ros2/blob/master/src/water/holoocean_examples/config/joy_config.yaml) 中的无线 Xbox 配置使用了 6、7、8 号按键。某些手柄或驱动程序报告的索引可能不同——请使用 `ros2 topic echo /joy` 进行确认。


#### 轴（Axes）

| 索引 | Xbox 输入 | 功能 |
|---|---|---|
| 0 | 左摇杆 X | 横向移动（平移） |
| 1 | 左摇杆 Y | 前进 / 俯仰（鱼雷） |
| 2 | LT | 相机向下倾斜 |
| 3 | 右摇杆 X | 偏航（BlueROV）/ 偏航（鱼雷） |
| 4 | 右摇杆 Y | 垂直移动（BlueROV）/ 速度（鱼雷） |
| 5 | RT | 相机向上倾斜 |
| 6 | 方向键 X | 横滚微调（BlueROV） |
| 7 | 方向键 Y | 俯仰微调（BlueROV） |

扳机轴（Trigger axes）在静止状态下值为 `1.0`，完全按下时达到 `-1.0`。这是 `joy_linux` 的标准惯例。


### 单代理控制

#### BlueROV2 / 悬停式AUV

| 输入 | 效果 |
|---|---|
| 左摇杆 Y 轴 | 前后推力 |
| 左摇杆 X 轴 | 侧向（平移）推力 |
| 右摇杆 Y 轴 | 垂直推力 |
| 右摇杆 X 轴 | 偏航（Yaw） |
| 十字键 上/下 | 俯仰微调（数值累加，解除待机状态时重置） |
| 十字键 左/右 | 横滚微调（数值累加，解除待机状态时重置） |
| RT / LT | 向上/向下倾斜摄像头 |

#### 水面载具（SurfaceVessel）

| 输入 | 效果 |
|---|---|
| 左摇杆 Y 轴 | 左侧推进器 |
| 右摇杆 Y 轴 | 右侧推进器 |
| RT / LT | 向上/向下倾斜摄像机（若载具配备 CameraSensor） |

#### TorpedoAUV / CougUV

| 输入 | 效果 |
|---|---|
| 左摇杆 Y | 俯仰鳍（Pitch fins） |
| 左摇杆 X | 偏航鳍（Yaw fins） |
| 右摇杆 Y | 推进器速度 |
| RT / LT | 向上/向下倾斜摄像机（若代理配备了摄像机传感器） |

---

### 相机 HUD

`camera_hud` 节点订阅指定代理的 `CameraSensor` 图像和 `DynamicsSensorOdom` 里程计话题，叠加显示驾驶员 HUD，并将结果重新发布至 `<agent>/CameraHUD`。

查看输出：

```bash
ros2 run rqt_image_view rqt_image_view
```

然后，从话题（topic）下拉菜单中选择 `/holoocean/<agent>/CameraHUD`（或相应的代理）。

`agent_name` 参数用于指定 HUD 跟随哪个代理。在 [joy_config.yaml](https://github.com/OpenHUTB/ros2/blob/master/src/water/holoocean_examples/config/joy_config.yaml) 和 [multi_joy_config.yaml](https://github.com/OpenHUTB/ros2/blob/master/src/water/holoocean_examples/config/multi_joy_config.yaml) 文件中，该参数是在 `camera_hud` 节点部分进行设置的。

---

## 配置文件结构

```
config/
  joy_config.yaml         单控制器，单代理或多代理场景
  multi_joy_config.yaml   双控制器、多代理场景
  waypoint_config.yaml    航点跟随参数
  sv_waypoints.yaml       水面舰艇示例的航路点列表
```

所有 YAML 文件均遵循 ROS 2 节点参数规范。每个文件顶部的通配符条目：

```yaml
/holoocean/**:
  ros__parameters:
    relative_path: true
    scenario_path: 'config/my_scenario.json'
```

将 `relative_path` 和 `scenario_path` 应用于 `holoocean` 命名空间下的**所有**节点，因此您只需在一处设置场景。

