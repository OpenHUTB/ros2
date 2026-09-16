# 双海龟画圆

通过 ROS1 自带的 turtlesim、rostopic 和 mimic 节点，观察速度话题、命名空间和话题重映射。模块来自原始 beginner_tutorials 作业，保留两组画圆速度，并整理为独立 ROS 包。

## 运行环境

- 目标环境：Ubuntu 20.04、ROS1 Noetic、有图形桌面。原始作业截图已确认使用 Noetic，整理后的入口尚待在课程虚拟机复测。
- 依赖：catkin、roslaunch、turtlesim、rostopic、geometry_msgs。
- Windows 用于编辑和预览文档；下列命令在 Ubuntu 终端运行。

```bash
source /opt/ros/noetic/setup.bash
sudo apt install ros-noetic-turtlesim ros-noetic-rostopic ros-noetic-roslaunch
```

其他 ROS1 版本需要替换发行版名称，并重新验证。

## 直接运行

将本模块文件夹复制到 Ubuntu，例如 `~/turtlesim_circle_mimic`。这些示例使用现成 ROS 节点，无需编译自定义节点。

```bash
source /opt/ros/noetic/setup.bash
cd ~/turtlesim_circle_mimic
bash main.sh circle
```

按 Ctrl+C 停止后，运行自动跟随：

```bash
bash main.sh mimic
```

入口也支持手动控制第一只海龟：

```bash
bash main.sh mimic auto_drive:=false
```

另开 Ubuntu 终端，执行以下命令并保持该终端获得键盘焦点：

```bash
source /opt/ros/noetic/setup.bash
rosrun turtlesim turtle_teleop_key /turtle1/cmd_vel:=/turtlesim1/turtle1/cmd_vel
```

## 使用 ROS 包和 launch

仅把本模块放入 catkin 工作空间，不要把混有 ROS1/ROS2 的整个仓库一起编译：

```bash
mkdir -p ~/ros_ws/src
cp -r ~/turtlesim_circle_mimic ~/ros_ws/src/
source /opt/ros/noetic/setup.bash
cd ~/ros_ws
catkin_make
source devel/setup.bash
roslaunch turtlesim_circle_mimic two_turtles_circle.launch
```

停止上一个演示后运行：

```bash
roslaunch turtlesim_circle_mimic turtle_mimic.launch
```

`roslaunch` 会在需要时自动启动 ROS master。两种模式使用相同的节点名，应依次运行。

## 验证与提交

画圆模式应出现两个窗口，两只海龟沿不同半径、相反方向持续运动；跟随模式中第二只海龟应跟踪第一只的位置，可能有跟踪误差。更多原理、检查命令和提交事项见仓库 `docs/turtlesim_circle_mimic/README.md`。

提交者已提供原始 beginner_tutorials 画圆截图和整理后模块的自动跟随截图，分别收录在仓库 `docs/turtlesim_circle_mimic/circle_result.png` 与 `mimic_result.png`。跟随截图显示 mimic 进程启动及两个窗口的圆形轨迹。提交前还需记录完整启动命令，复测画圆、手动控制和退出行为，并补充另一台机器的测试记录。package.xml 中维护者与许可证仍沿用原作业占位值，发布前需填写真实维护信息并确认许可。

## 大模型使用声明

本模块使用 OpenAI Codex 辅助整理已有 launch 文件、补充 main.sh 入口、依赖声明及文档。提交者须逐项理解和核验内容，并对最终提交负责。
