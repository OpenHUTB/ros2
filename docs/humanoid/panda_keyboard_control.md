## 通过键盘操控仿真机械臂

### 一、 项目概述

本项目基于 ROS Noetic 与 MoveIt! 框架，实现 Franka Emika Panda 机械臂的 RViz 仿真与键盘运动控制。通过编写 Python 脚本，实现通过键盘按键控制机械臂各关节转动、姿态复位等功能。项目采用 Catkin 工作空间进行编译管理。

### 二、 项目目录结构

text
panda_ros_project/
└── catkin_ws/
    └── src/
        └── panda_keyboard_control/
            ├── CMakeLists.txt               # 编译配置文件
            ├── package.xml                  # 包依赖声明文件
            ├── setup.py                     # Python 脚本安装配置
            └── scripts/
                └── panda_keyboard_control.py  # 键盘控制核心 Python 脚本

### 三、 核心配置文件内容

1. CMakeLists.txt

声明功能包所需的依赖组件，并安装 Python 脚本。

cmake
```bash
cmake_minimum_required(VERSION 3.0.2)
project(panda_keyboard_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  moveit_commander
  moveit_ros_planning_interface
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/panda_keyboard_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

2. package.xml

定义功能包在构建和运行时的依赖关系。

xml
```bash
  panda_keyboard_control
  0.1.0
  任务②：Panda 机械臂 MoveIt 仿真与键盘运动控制
  user
  MIT

  catkin

  rospy
  std_msgs
  moveit_commander
  moveit_ros_planning_interface

  rospy
  std_msgs
  moveit_commander
  moveit_ros_planning_interface
  panda_moveit_config
```
  

3. setup.py

用于确保 Python 脚本能被正确安装到 catkin 工作空间中。

python
```bash
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['panda_keyboard_control'],
    package_dir={'': 'src'}
)

setup(**d)
```

4. panda_keyboard_control.py

实现键盘输入读取，并通过 MoveIt Commander 控制机械臂关节运动。

python
```bash
!/usr/bin/env python3
-- coding: utf-8 --
"""
键盘控制 Panda 机械臂节点（任务②）。

功能：
  1. 初始化 ROS 节点并连接 MoveIt move_group；
  2. 读取键盘输入，控制机械臂 7 个关节分别转动；
  3. 支持复位到初始姿态和安全退出。

按键说明：
  1-7  分别控制 7 个关节转动
  r    复位到初始姿态
  q    退出程序
"""
import sys
import tty
import termios
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory

class PandaKeyboardControl(object):
    def init(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('panda_keyboard_control', anonymous=True)

        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        self.group.set_max_velocity_scaling_factor(0.3)
        self.group.set_max_acceleration_scaling_factor(0.3)

        self.joint_step = 0.1  # 每次关节转动弧度
        rospy.loginfo("已连接到 move_group，规划组: %s", self.group.get_name())
        rospy.loginfo("当前关节角度: %s", self.group.get_current_joint_values())

    def _get_key(self):
        """非阻塞读取单个按键，返回字符。"""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def _move_joint(self, joint_index, delta):
        """对指定关节施加增量并规划执行。"""
        joint_values = self.group.get_current_joint_values()
        joint_values[joint_index] += delta
        self.group.set_joint_value_target(joint_values)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        if plan:
            rospy.loginfo("关节 %d 移动成功", joint_index + 1)
        else:
            rospy.logwarn("关节 %d 规划失败，请检查限位", joint_index + 1)

    def _reset(self):
        """复位到初始姿态。"""
        rospy.loginfo("正在复位到初始姿态...")
        self.group.set_named_target("ready")
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def run(self):
        rospy.loginfo("Panda 机械臂键盘控制：")
        rospy.loginfo("  1-7: 控制对应关节转动")
        rospy.loginfo("  r:   复位到初始姿态")
        rospy.loginfo("  q:   退出程序")

        try:
            while not rospy.is_shutdown():
                key = self._get_key()
                if key == 'q':
                    break
                elif key == 'r':
                    self._reset()
                elif key in [str(i) for i in range(1, 8)]:
                    idx = int(key) - 1
                    self._move_joint(idx, self.joint_step)
        except Exception as e:
            rospy.logerr("运行异常: %s", str(e))
        finally:
            self.group.stop()
            self.group.clear_pose_targets()
            moveit_commander.roscpp_shutdown()
            rospy.loginfo("程序已安全退出")

if name == 'main':
    try:
        PandaKeyboardControl().run()
    except rospy.ROSInterruptException:
        pass
```

### 四、 编译与启动步骤

1. 安装依赖

打开终端，安装 MoveIt 和 Panda 相关功能包：

```bash
sudo apt update
sudo apt install ros-noetic-moveit ros-noetic-panda-moveit-config ros-noetic-franka-description -y
```

2. 编译工作空间

```bash
cd ~/catkin_ws
catkin_make
```

3. 配置环境

在终端中输入以下命令并回车：

```bash
source ~/catkin_ws/devel/setup.bash
```

注意：这一步如果没有任何报错，说明环境配置成功。如果报错“没有那个文件或目录”，请确认你的 catkin_ws 文件夹路径是否正确。

4. 启动仿真

环境配置成功后，打开**第一个终端**，启动 MoveIt 演示环境：

```bash
source /opt/ros/noetic/setup.bash
roslaunch panda_moveit_config demo.launch
```

注意：执行这条命令后，RViz 仿真界面会打开，并显示 Panda 机械臂模型。**必须等待终端输出 `Ready to take commands for planning group panda_arm.` 提示后**，再进行下一步。

5. 运行控制节点

保持第一个终端运行，打开**第二个终端**：

```bash
source ~/catkin_ws/devel/setup.bash
rosrun panda_keyboard_control panda_keyboard_control.py
```

6. 操作说明

在第二个终端中按以下按键控制：
按键   功能   按键   功能
1   关节1转动   5   关节5转动
2   关节2转动   6   关节6转动
3   关节3转动   7   关节7转动
4   关节4转动   r   复位姿态
q   退出程序

### 五、常见问题排查

1. 报错 `Unable to connect to move_group action server`
   - **原因**：脚本启动时 `demo.launch` 尚未完成初始化，或者 `move_group` 节点崩溃。
   - **解决**：确保先运行 `demo.launch` 并看到 "Ready to take commands for planning group panda_arm." 提示后再运行脚本。检查是否有残留进程：`killall -9 roslaunch roscore` 后重试。

2. `robot_state_publisher` 循环重启或退出
   - **原因**：缺少 URDF 描述文件或 Xacro 解析失败。
   - **解决**：重新安装描述包并刷新库缓存：
     bash
     sudo apt install --reinstall ros-noetic-panda-description ros-noetic-geometric-shapes -y
     sudo ldconfig
     
3. 规划失败（Planning Failed）
   - **原因**：目标关节角度超出限位或步长过大导致碰撞检测失败。
   - **解决**：在代码中减小 `self.joint_step`（例如从 0.1 改为 0.05），或降低速度缩放因子。

4. 找不到功能包 `panda_keyboard_control`
   - **原因**：当前终端未 source 工作空间。
   - **解决**：在每个新终端中执行 `source ~/catkin_ws/devel/setup.bash`，或将其写入 `~/.bashrc`。

### 六、运行效果
https://github.com/Tan-aaa/ros2/blob/2d29e8412ba5b4c09d5d085dd515e26a6c15c552/tests/%E6%9C%BA%E6%A2%B0%E8%87%82.gif
