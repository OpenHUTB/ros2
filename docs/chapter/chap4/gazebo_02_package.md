---
title: 步骤 2：工作空间与功能包
---

# 步骤 2：工作空间与功能包

> 前置条件：完成[步骤 1](gazebo_01_environment.md)。
> 本步目标：建立 catkin 工作空间，成功编译并找到 `mybot_gazebo`。
> 预计时间：20-30 分钟。

- [1. 创建 Catkin 工作空间](#workspace)
- [2. 放入功能包](#copy)
- [3. 功能包目录结构](#structure)
- [4. 编译配置解析](#build-config)
- [5. 编译与验证](#build)
- [6. 常见问题](#debug)
- [7. 本步验收](#acceptance)

## 1. 创建 Catkin 工作空间 <span id="workspace"></span>

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

检查工作空间：

```bash
echo $ROS_PACKAGE_PATH
```

输出中应包含：

```text
/home/<用户名>/catkin_ws/src:/opt/ros/noetic/share
```

为了避免每次新开终端都手动加载，可以加入 `~/.bashrc`：

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. 放入功能包 <span id="copy"></span>

在仓库根目录中，把 `mybot_gazebo` 复制到工作空间：

```bash
cp -r src/chap4/mybot_gazebo ~/catkin_ws/src/
```

## 3. 功能包目录结构 <span id="structure"></span>

```text
mybot_gazebo/
├── package.xml
├── CMakeLists.txt
├── LICENSE
├── main.sh
├── README.md
├── launch/
│   └── mybot_gazebo.launch
├── urdf/
│   └── mybot.urdf.xacro
├── worlds/
│   └── mybot_empty.world
└── scripts/
    └── move_square.py
```

各目录职责：

| 路径 | 作用 |
| --- | --- |
| `package.xml` | 声明依赖和包信息 |
| `CMakeLists.txt` | 安装 Python 脚本，安装 launch、urdf、world 文件 |
| `main.sh` | 仓库要求的统一入口，调用 launch 文件启动仿真 |
| `launch/` | 组织 Gazebo、模型、TF 和参数 |
| `urdf/` | 机器人结构和 Gazebo 插件 |
| `worlds/` | Gazebo 世界文件 |
| `scripts/` | 机器人运动控制节点 |

## 4. 编译配置解析 <span id="build-config"></span>

`package.xml` 至少要声明以下依赖：

```xml
<buildtool_depend>catkin</buildtool_depend>

<exec_depend>gazebo_ros</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>rospy</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>xacro</exec_depend>
```

`CMakeLists.txt` 的核心是：

```cmake
catkin_install_python(PROGRAMS
  scripts/move_square.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

因为本功能包只有 Python 脚本和资源文件，不需要 ROS C++ 编译，但 `catkin_make` 仍然负责注册包和生成环境脚本。

## 5. 编译与验证 <span id="build"></span>

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rospack find mybot_gazebo
```

预期输出：

```text
/home/<用户名>/catkin_ws/src/mybot_gazebo
```

再检查脚本是否可执行：

```bash
ls -l ~/catkin_ws/src/mybot_gazebo/scripts/move_square.py
python3 -m py_compile ~/catkin_ws/src/mybot_gazebo/scripts/move_square.py
```

`py_compile` 通过说明 Python 语法正确，但不会检查 ROS 话题和 Gazebo 连接。

也可以使用包内统一入口启动：

```bash
cd ~/catkin_ws/src/mybot_gazebo
./main.sh
```

## 6. 常见问题 <span id="debug"></span>

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `rospack find` 找不到包 | 未 source `devel/setup.bash` | 重新 source 并确认 `ROS_PACKAGE_PATH` |
| `catkin_make` 找不到依赖 | `package.xml` 依赖未安装 | 安装对应 ROS 包 |
| 脚本无法运行 | 没有执行权限 | `chmod +x scripts/*.py` |
| 修改后没有生效 | 没有重新 source | 每次新终端都要 source |
| 复制到了错误位置 | 功能包放进 `devel` 或嵌套目录 | 必须放在 `~/catkin_ws/src/` 下 |

## 7. 本步验收 <span id="acceptance"></span>

- [ ] `~/catkin_ws` 可以正常 `catkin_make`；
- [ ] `rospack find mybot_gazebo` 找到功能包；
- [ ] `launch`、`urdf`、`worlds`、`scripts` 四个目录都在；
- [ ] 能解释 `package.xml`、`CMakeLists.txt` 和 catkin 工作空间的关系；
- [ ] 知道每个终端都要 source 工作空间环境。

---

- 上一步：[环境检查与 Gazebo 验证](gazebo_01_environment.md)
- 返回[教程总览](gazebo_platform_simulation.md)
- 下一步：[步骤 3：URDF 与 xacro 建模](gazebo_03_urdf_model.md)
