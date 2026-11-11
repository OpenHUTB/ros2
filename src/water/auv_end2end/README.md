# auv_end2end 水下机器人仿真功能包

基于 ROS2 + Gazebo 实现的水下机器人仿真功能包，包含**视觉避障**与**矩形巡航**两个功能。

## 功能说明

### 1. 矩形巡航
Gazebo 仿真环境中的水下机器人按预设矩形轨迹自动巡航。

启动方式：
```bash
ros2 launch auv_end2end auv_rect_cruise.launch.py
```

该 launch 会同时启动 Gazebo 仿真与巡航节点。

> 注意：`auv_rect_cruise.launch.py` 与 `auv_sim.launch.py` 都会各自启动 Gazebo 并发布 `/auv/cmd_vel`，请勿同时运行两个 launch，切换前先 Ctrl+C 关闭当前仿真。

### 2. 水下视觉避障

基于单目相机画面检测前方暗色障碍物，自动判断方位并执行横向避让，超时后进入冷却期防止反复触发。

启动方式：

```
ros2 launch auv_end2end auv_sim.launch.py
```

#### 避障逻辑说明

##### 检测区域

- 取相机画面中上部区域（高度 15%~70%）进行灰度检测，剔除海底地面与顶部雾效
- 将画面拆分为左、右两个半区
- 分别统计暗色障碍物像素占比

##### 检测阈值

- 默认用 Otsu 自适应阈值，抗雾效与整体光照变化
- 画面不够「双峰」（类间方差 / 总方差 < 0.8，例如开阔水域整幅灰度相同）时，退回固定阈值 `obstacle_thresh = 22`
- 这样均匀背景会被判为「没有障碍」，而不是被 Otsu 从中间切开后当成「全是障碍」

##### 触发条件

- 单侧障碍物占比超过 22% 时触发避障
- 哪一侧障碍物更多，就向另一侧横向避让
- 两侧占比都低于 18%、且已避让满 0.3 秒时，恢复直线巡航（最小避让时长用于抑制阈值附近抖动）

##### 超时与冷却机制

- 单次最长避让时长为 3.0 秒，防止躲避过远
- 超时后进入 2.0 秒冷却期，期间强制直行
- 冷却期内不接受常规重触发，避免反复横跳；但占比超过 33%（`trigger_ratio × emergency_factor`）时说明障碍显著恶化，会立即中断冷却重新避让
- 说明：障碍持续存在时会形成「避让 3 秒 → 强制直行 2 秒 → 再避让」的循环，这是本作业的教学取舍，面对连续长墙无法绕过

## 环境依赖

- ROS2 Humble
- Gazebo Classic 11.10.2
- Python 3.10+
- OpenCV
- cv_bridge

本包依赖 Gazebo **Classic**（`gazebo` 命令 + `libgazebo_ros_camera.so` 等插件），
而 ROS2 Humble 官方镜像默认搭配的是 Ignition Fortress，直接运行会出现 `gazebo: command not found`。
请先安装 Gazebo Classic 相关包：

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

同时需要 Gazebo 11 本体（`gazebo` 命令来自本体包，不是 ROS 包）：

```bash
sudo apt install gazebo11 libgazebo11-dev
gazebo --version   # 应输出 gazebo, version 11.x
```

## 编译运行

```
cd ~/ros2_ws
colcon build --packages-select auv_end2end
source install/setup.bash
```

编译后执行 `colcon test --packages-select auv_end2end` 可运行单元测试与 flake8 / pep257 代码规范检查（共 22 项，测试只依赖 numpy，不需要启动 Gazebo）。

## 运行与退出

两个 launch 都会启动 Gazebo：巡航加载 `worlds/underwater_cruise.world`（空场景，航路不被墙体阻断），避障加载 `worlds/underwater.world`（8 道错位墙）。单独执行任意一个即可看到仿真画面：

```bash
ros2 launch auv_end2end auv_rect_cruise.launch.py   # 矩形巡航
ros2 launch auv_end2end auv_sim.launch.py           # 视觉避障
```

- 两个 launch 都会向 `/auv/cmd_vel` 发布速度指令，请勿同时运行，切换前先 Ctrl+C 关闭。
- 两个节点均以 `use_sim_time=True` 运行，避让 3 s、冷却 2 s 按 Gazebo 仿真时钟计时。
- Ctrl+C 退出时节点会在 rclpy 关闭 context 之前先发布零速度指令，避免 AUV 保持最后一帧速度继续运动
  （`libgazebo_ros_planar_move.so` 本身没有 cmd_vel 超时）。

## 参数说明

参数全部通过 ROS 参数暴露，launch 中用 `DeclareLaunchArgument` 声明默认值，可在命令行覆盖（无需重新编译）：

```bash
ros2 launch auv_end2end auv_rect_cruise.launch.py side_time:=8.0
ros2 launch auv_end2end auv_sim.launch.py trigger_ratio:=0.3
```

下表为默认值。

避障节点 `auv_obstacle_avoid`：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `obstacle_thresh` | 22 | 固定灰度阈值（0-255），低于该值判为暗色障碍像素；自适应不可用时作为兜底 |
| `adaptive_thresh` | true | true 时用 Otsu 自适应阈值；画面不够双峰时自动退回 `obstacle_thresh` |
| `trigger_ratio` | 0.22 | 触发阈值，单侧暗像素面积占比超过 22% 触发避障 |
| `recover_ratio` | 0.18 | 恢复阈值，双侧占比均低于 18% 恢复直行 |
| `forward_speed` | 4.5 | 直行巡航速度（m/s） |
| `side_speed` | 2.8 | 横向避让速度（m/s） |
| `max_avoid_duration` | 3.0 | 单次最长避让时长（s），超时进入冷却期 |
| `avoid_cooldown` | 2.0 | 冷却时长（s），期间强制直行、禁止常规重触发 |
| `min_avoid_duration` | 0.3 | 最小避让时长（s），防止阈值附近抖动 |
| `emergency_factor` | 1.5 | 冷却期占比超过 `trigger_ratio × 该值` 时立即中断冷却重新避让 |

注意 `obstacle_thresh = 22` 是**灰度阈值**，`trigger_ratio = 0.22` 是**面积占比阈值**，两者不是同一类量。

巡航节点 `auv_rect_cruise`：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `forward_speed` | 2.5 | 前进/后退速度（m/s） |
| `side_speed` | 2.0 | 横向平移速度（m/s） |
| `forward_time` | 10.0 | 直行/后退持续时间（s） |
| `side_time` | 20.0 | 横向平移持续时间（s） |

两个 launch 共用的开关：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gui` | true | 是否启动 `gzclient`；无显示环境设 false |
| `use_rviz` | false | 是否启动 RViz2 并加载 `rviz/auv.rviz` |
| `record_csv` | 空 | 轨迹 CSV 输出路径，留空则不写文件 |

## 节点与话题信息

- 避障节点：`auv_obstacle_avoid`
- 巡航节点：`auv_rect_cruise`
- 轨迹记录节点：`trajectory_recorder`（订阅 `/auv/odom`，发布 `nav_msgs/Path`）
- 相机话题：`/auv/camera/camera/image_raw`
- 速度控制话题：`/auv/cmd_vel`
- 轨迹话题：`/auv/trajectory`
