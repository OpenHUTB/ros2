# 自建四轮小车 Gazebo GMapping 建图实验（ROS Noetic）

姓名：________ 学号：________ 班级：________

本页为独立实验。旧版“第九章”沿用作业编号，不对应本仓库目录：仓库第六章为 SLAM 与自主导航，第九章为 ROS 2。本实验使用 ROS 1 Noetic，不应按仓库第九章查找。源码暂保留原 `src/chap9/urdf_car_slam` 路径，避免破坏既有复现步骤。

本包承接已在 RViz 显示成功的四轮小车，新增 Gazebo Classic 物理驱动、二维激光、里程计和 GMapping。本实验仅覆盖二维仿真建图，不包含自动导航。包名为 urdf_car_slam，与原 urdf_car_demo 分开。

## 一、环境与复制位置

Ubuntu 20.04 + ROS Noetic + Gazebo Classic（通常为 11）。打开普通 Ubuntu 终端，不要激活 AirSim 的 .venv，不要加载 ROS 2 环境。退出之前的 display.launch，避免 robot_description 与 TF 发布器冲突。

```bash
source /opt/ros/noetic/setup.bash
sudo apt update
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins ros-noetic-xacro ros-noetic-robot-state-publisher ros-noetic-gmapping ros-noetic-map-server ros-noetic-rviz ros-noetic-teleop-twist-keyboard liburdfdom-tools
mkdir -p ~/urdf_car_ws/src
```

把本包的 **urdf_car_slam** 文件夹复制到 `~/urdf_car_ws/src/`。可以和 urdf_car_demo 同处工作空间，但不要同时启动两套模型。

```bash
cd ~/urdf_car_ws
catkin_make
source devel/setup.bash
roslaunch urdf_car_slam slam.launch
```

将打开 Gazebo 和 RViz。启动初期等机器人生成、激光与第一张地图出现后再移动；Gazebo 必须处于播放状态。虚拟机性能不足时先关闭其他仿真，或使用 `roslaunch urdf_car_slam slam.launch gui:=false` 保留 Gazebo 后台和 RViz。

## 二、键盘遥控建图

另开 Ubuntu 终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/urdf_car_ws/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=0.15 _turn:=0.4 _repeat_rate:=10.0 _key_timeout:=0.6
```

键盘窗口要保持焦点：`i` 前进，`,` 后退，`j` 左转，`l` 右转，`k` 停止。按键超时后发送零速度；长按或重复按键保持移动。退出遥控前先按 k。

先慢速走直线，再转弯。绕房间探索，经过障碍物的不同侧面，最后回到起点附近。不要贴墙或持续撞障碍。RViz 中白色为已探索可通行区，黑色为障碍，灰色为未知区域。没有自动避障，地图不会代替遥控驾驶。

## 三、保存地图

保持 slam.launch 运行，另开终端：

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/urdf_car_maps
rosrun map_server map_saver -f ~/urdf_car_maps/car_gmapping
ls -lh ~/urdf_car_maps/car_gmapping.*
```

应生成 car_gmapping.pgm 和 car_gmapping.yaml。两者一起提交；yaml 记录分辨率和原点，不能只交图片。若 map_saver 一直等待地图，先排查 /map。查看图片可运行 `xdg-open ~/urdf_car_maps/car_gmapping.pgm`。

## 四、系统结构与坐标系

```text
键盘 → /cmd_vel → Gazebo 四轮差速驱动 → /odom、odom→base_footprint
Gazebo 激光 → /scan
Gazebo 关节状态 → /joint_states → robot_state_publisher → 各部件 TF
/scan + odom TF → slam_gmapping → /map、map→odom
```

完整 TF 主链为 `map → odom → base_footprint → base_link → lidar_link`。原车的雷达只有外形，现在增加 ray 传感器，发布 sensor_msgs/LaserScan。无需给 map→odom 或 odom→base_footprint 添加静态 TF，否则会与 SLAM/驱动冲突。

四轮滑移转向由 gazebo_ros_skid_steer_drive 驱动。此插件使用 Gazebo 世界位姿生成仿真里程计，是便于入门的理想化里程计实验；不是实机编码器里程计性能验证。GMapping 从激光和里程计 TF 建图，没有预先加载 world 的栅格地图。

模型加入了质量、惯量、碰撞几何和轮地接触参数。雷达为 360 个采样、10 Hz、最远 8m，地图分辨率为 0.05m。场景由地面、四面墙和三个障碍物构成，不依赖在线下载模型。

## 五、分层检查与排错

另开终端，先 source /opt/ros/noetic/setup.bash，再检查：

```bash
rostopic hz /clock
rostopic hz /scan
rostopic hz /odom
rostopic hz /joint_states
rostopic echo -n 1 /map/info
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo map lidar_link
```

每个 hz 命令观察几秒后 Ctrl+C，再执行下一条。

| 现象 | 检查 |
| --- | --- |
| xacro 或包找不到 | 检查复制层级、catkin_make 和 source devel/setup.bash |
| Failed to load plugin | 检查 gazebo-plugins 是否安装；需 Classic，不能用 ROS2/Gazebo Sim 替代 |
| /clock 无更新 | Gazebo 是否启动且未暂停 |
| 只有小车没有地图 | 检查 /scan、odom TF、gmapping 日志；等第一帧地图 |
| 车轮缺失或 TF 报错 | 检查 /joint_states；本实验不用 joint_state_publisher_gui 伪造轮子状态 |
| 键盘不响应 | 点击键盘终端；查看 /cmd_vel；同时检查 Gazebo 是否暂停 |
| 持续打滑、地图歪斜 | 降低速度，避免急转和碰撞；记录现象再调摩擦和驱动参数 |
| 缺 map→odom | GMapping 是否收到扫描，扫描时间戳能否找到对应 TF |

## 六、作业验收材料

1. 原 URDF 模型结构及新增激光、驱动插件说明。
2. Gazebo 场景截图。
3. RViz 中 Map + LaserScan + RobotModel 截图。
4. TF 链或话题检查截图。
5. 保存的 pgm/yaml 文件和地图图片。
6. 一段遥控建图录屏，说明探索路线和问题。

实验记录：ROS 版本________，Gazebo 版本________，实际地图分辨率________，地图宽高________，是否回到起点________。

用户已提供 Gazebo 场景、RViz 地图和 GMapping 日志截图：场景及模型已加载，RViz Global Status 为 Ok，GMapping 持续输出 Registering Scans: Done。已确认激光和里程计数据输出、初始地图生成；尚未提供完整探索录屏和保存的 pgm/yaml，不能声称已完成整个场景建图或定量精度验证。

一次联调中 rosnode list 未显示 slam_gmapping；随后手动启动 GMapping 后获得正常截图。若复现时只有激光和里程计没有地图，先检查是否安装 ros-noetic-gmapping 及 launch 终端报错，再在另一个已加载 Noetic 环境的终端执行：

```bash
rosrun gmapping slam_gmapping _base_frame:=base_footprint _odom_frame:=odom _map_frame:=map _maxUrange:=7.5 _maxRange:=8.0 _delta:=0.05 scan:=/scan
```

已有 GMapping 节点时不要重复启动。

## 七、来源与辅助声明

基于此前小车建模作业扩展，由 OpenAI Codex 辅助编写。

- ROS Noetic 驱动插件源码：https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_skid_steer_drive.cpp
- 内容范围：自建小车的 GMapping 仿真建图；章节编号以所用教材目录为准。



## PR #852 复核后的调整与证据补充

- 初始高度改为可配置的 `spawn_z`，默认 0.01 米。模型中轮底在 `base_footprint` 的 z=0 附近；默认只保留 1 厘米生成间隙，减小旧值 0.04 米落地时的冲击。此调整尚未在 Gazebo 重新运行验证。生成后应等待车辆落稳、激光与 TF 正常，再开始遥控；可用 `roslaunch urdf_car_slam slam.launch spawn_z:=0.0` 比较接地表现。
- RViz 默认开启 TF 显示，便于检查 `map → odom → base_footprint → base_link → lidar_link`。启动初期尚无第一帧地图时，map 相关变换可能暂未出现，不要人为补一条静态 map→odom。
- 保留原来的三张截图，并明确它们仅证明初始地图与扫描匹配运行。尚无完整探索后的地图文件，不能当作完整建图验收。

### 在虚拟机生成缺少的实测材料

保持 `slam.launch` 运行，在键盘遥控终端慢速探索房间、经过障碍物的不同侧面，并尽量回到起点附近。先按 `k` 停车，再在另一个 Noetic 终端执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/urdf_car_ws/devel/setup.bash
RUN_DIR="$HOME/urdf_car_maps/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
rosrun map_server map_saver -f "$RUN_DIR/car_gmapping"
rostopic echo -n 1 /map/info > "$RUN_DIR/map_info.yaml"
ls -lh "$RUN_DIR/car_gmapping.pgm" "$RUN_DIR/car_gmapping.yaml"
echo "$RUN_DIR"
```

`map_saver` 等待地图时不要退出仿真；检查 `/map` 是否由正在运行的 GMapping 发布。地图文件来自实时 `/map`，不能把 Gazebo world 平面图替换成建图结果。

补交该目录下的 `car_gmapping.pgm`、`car_gmapping.yaml` 和 `map_info.yaml`，另附探索后的 RViz 截图（Map、LaserScan、RobotModel、TF 和 Global Status 可见）。保留 YAML 对 PGM 的同目录引用。记录探索路线、是否回到起点及异常现象；如有录屏，也一起保留。

当前材料缺口仍然存在：上述运行命令和配置修复不等于生成了实测地图。新版出生高度、TF 显示与完整探索过程均需在 ROS Noetic + Gazebo 中复测。使用 OpenAI Codex 辅助整理本次修改。
