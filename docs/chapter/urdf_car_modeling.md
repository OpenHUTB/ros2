# ROS Noetic：四轮小车 URDF 建模作业

本页归入本站第 4 章“机器人仿真”，内容为四轮小车 URDF/xacro 建模、RViz 模型显示、TF 树检查与关节滑块观察。旧作业材料的教材编号与本站目录不同；源码保留在 `src/chap6/urdf_car_demo`。

## 1. 文件与环境

Ubuntu 20.04 + ROS Noetic，使用系统 ROS 环境。不要激活 AirSim Python 虚拟环境。

```text
urdf_car_demo/
  package.xml
  CMakeLists.txt
  urdf/car.urdf.xacro
  launch/display.launch
  rviz/car.rviz
```

本包是模型显示实验：雷达只是外形，没有激光数据；车轮滑块只改变关节角度，小车不会沿地面行驶。Gazebo 动力学、底盘驱动及导航不在本次范围。

## 2. 安装与编译（Ubuntu 终端）

```bash
source /opt/ros/noetic/setup.bash
sudo apt update
sudo apt install git ros-noetic-catkin ros-noetic-tf ros-noetic-xacro ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher-gui ros-noetic-rviz liburdfdom-tools
mkdir -p ~/urdf_car_ws/src
```

源码位于本仓库 `src/chap6/urdf_car_demo`。首次获取仓库可执行：

```bash
git clone --depth 1 https://github.com/OpenHUTB/ros2.git ~/ros2-course-source
```

已有仓库时跳过克隆，并将下面的 `REPO_DIR` 改成实际仓库根目录。只复制单个包，不要把整个仓库的 `src` 放进 catkin 工作空间。目标位置已有同名包时先保留原文件，避免直接覆盖自己的修改。

```bash
REPO_DIR="$HOME/ros2-course-source"
test -f "$REPO_DIR/src/chap6/urdf_car_demo/package.xml" && \
  cp -a "$REPO_DIR/src/chap6/urdf_car_demo" "$HOME/urdf_car_ws/src/"
```

确认 `~/urdf_car_ws/src/urdf_car_demo/package.xml` 存在后再编译。使用作业压缩包时，也可将解压后的 `urdf_car_demo` 整个文件夹复制到该位置。

```bash
cd ~/urdf_car_ws
catkin_make
source devel/setup.bash
roslaunch urdf_car_demo display.launch
```

launch 会启动 ROS master（如果还没有）、关节滑块、TF 发布节点和 RViz，无需另开 roscore。

## 3. 观察效果

应看到蓝色车体、四个黑色车轮、橙色雷达和白色车头标记。车轮带橙色标记，拖动滑块后能看出转动。勾选 RViz 左侧 TF 可查看坐标系。

RViz 的 Fixed Frame 应为 `base_footprint`。若不显示模型，确认 RobotModel 已勾选且 robot_description 参数存在。

## 4. 实验检查

另开终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/urdf_car_ws/devel/setup.bash
rosrun xacro xacro "$(rospack find urdf_car_demo)/urdf/car.urdf.xacro" > /tmp/car.urdf
check_urdf /tmp/car.urdf
rostopic echo -n 1 /joint_states
rosrun tf tf_echo base_link front_left_wheel_link
```

### check_urdf 用户实测记录

用户在 Ubuntu ROS Noetic 虚拟机中执行了以下命令，并提供了完整终端输出：

```bash
source /opt/ros/noetic/setup.bash
source ~/urdf_car_ws/devel/setup.bash
rosrun xacro xacro "$(rospack find urdf_car_demo)/urdf/car.urdf.xacro" > /tmp/car.urdf
check_urdf /tmp/car.urdf
```

```text
robot name is: student_car
---------- Successfully Parsed XML ---------------
root Link: base_footprint has 1 child(ren)
    child(1):  base_link
        child(1):  front_left_wheel_link
        child(2):  front_marker_link
        child(3):  front_right_wheel_link
        child(4):  lidar_link
        child(5):  rear_left_wheel_link
        child(6):  rear_right_wheel_link
```

结果：展开后的 URDF 已由 `check_urdf` 成功解析；根链接是 `base_footprint`，其子链接为 `base_link`，四个车轮、前标记与雷达链接均出现在解析树中，共 8 个链接。该记录来自用户实际运行，验证了 URDF 解析与树结构。原始输出另存于包内 `validation/check_urdf_output.txt`。

`tf_echo` 会持续输出；观察后按 `Ctrl+C` 返回终端。`check_urdf` 验证静态模型树，`tf_echo` 检查运行中的 TF，二者用途不同。

## 5. 模型原理与修改

- 单位为米、弧度，坐标 x 向前、y 向左、z 向上。
- `link` 表示刚体，`joint` 表示刚体连接。
- 根为 base_footprint，base_link 中心位于地面上方 0.15m。
- 车体长 0.60m、宽 0.36m、高 0.12m；车轮半径 0.10m、宽 0.04m。
- 轮心相对车体中心 z=-0.05m，所以轮底高度为零。
- 圆柱默认沿 z 轴，绕 x 旋转 π/2 后成为横向轮轴；关节轴为 y。
- 四个轮子用 xacro 宏重复生成，轮关节为 continuous；雷达和车头用 fixed 连接。
- 修改 car.urdf.xacro 的颜色或尺寸后，关闭 launch 并重新运行。
- 修改轮半径时也要重新计算轮心位置与车体离地高度，避免轮子悬空或穿地。

可作为个人扩展：改变车体配色、增加摄像头 link，或增加可转动雷达关节，并解释 TF 树变化。不要将单纯改变颜色描述成新的导航算法。

## 6. 已有实验材料

1. Ubuntu 20.04 + ROS Noetic 环境说明。
2. 包结构及 URDF/xacro 模型说明。
3. RViz 整车显示截图。
4. `check_urdf` 成功输出与 8 个链接的模型树。

继续进行动态仿真可参考[自建小车 Gazebo GMapping 建图](urdf_car_gmapping.md)。该实验使用独立包，运行前先在当前 launch 终端按 `Ctrl+C` 关闭模型显示。

本材料由 OpenAI Codex 辅助编写。用户已在 Ubuntu ROS Noetic 环境启动模型显示，实测截图见下方；`check_urdf` 实测输出记录于第 4 节。

## 7. 实测运行截图

![Ubuntu ROS Noetic 下的 URDF 小车 RViz 显示结果](../img/chap6/urdf_car_demo/rviz_car_result.png)

用户提供的运行截图显示，joint_state_publisher_gui、robot_state_publisher 与 RViz 已启动，RViz 的 Global Status 为 Ok，蓝色车体、黑色车轮、橙色雷达外形和白色车头标记正常显示。

本截图记录了模型显示效果。
