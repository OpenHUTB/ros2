# 6.2 mrobot 激光雷达 SLAM 建图 · 完整复现手册

> **环境**：Ubuntu 20.04 + ROS Noetic + Gazebo 11
**来源**：ROS 机器人开发实践（胡春旭）第六章 · 仿真环境搭建 + gmapping SLAM 建图
**适配说明**：源代码为 kinetic 老代码，本文所有命令均已适配 Noetic，并标注了迁移中遇到的每一个坑。

***

## 第 0 步：确认环境

```bash
rosversion -d      # 输出 noetic
```

***

## 第 1 步：准备 catkin 工作空间 + 复制功能包

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
cp -r ~/ros_exploring_src/robot_mrobot/mrobot_gazebo .
cp -r ~/ros_exploring_src/robot_mrobot/mrobot_navigation .
cp -r ~/ros_exploring_src/robot_mrobot/mrobot_teleop .

# 遥控脚本加执行权限（坑①，缺了会报 cannot locate node）
chmod +x ~/catkin_ws/src/mrobot_teleop/scripts/mrobot_teleop.py
```

***

## 第 2 步：安装依赖（必须先装再编译）

```bash
sudo apt update
sudo apt install -y ros-noetic-gmapping ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-move-base-msgs ros-noetic-amcl
```

> **坑②**：`catkin_make` 报 `Could not find the required component 'xxx'`，就是缺依赖；把 `xxx` 拼成 `ros-noetic-xxx` 装上再编译。

### 📷 第 0~2 步实测截图

终端完整记录：`rosversion -d` 输出 `noetic` → 复制三个功能包 → `chmod +x` 加执行权限 → `sudo apt update`（密码连输错 3 次后成功，提示 190 个软件包可升级）。

!\[第0~2步：环境确认+工作空间+依赖更新]([https://aka.doubaocdn.com/s/tOoVjHF0Nl](https://aka.doubaocdn.com/s/tOoVjHF0Nl))

***

## 第 3 步：修功能包的三个已知雷（kinetic 老代码迁移）

### 坑③ —— 给 package.xml 补 gazebo 模型路径导出

让 Gazebo 找得到包内 `models/`。用编辑器打开（注意：package.xml 是**文件**，不能用 `cd` 进）：

```bash
gedit ~/catkin_ws/src/mrobot_gazebo/package.xml
```

在 `<export>` 块中、`</export>` 之前加入一行：

```xml
<export>
    <!-- Other tools can request additional information be placed here -->
    <gazebo_ros gazebo_model_path="${prefix}/models"/>
</export>
```

等价的一条 sed 命令：

```bash
sed -i 's#  </export>#    <gazebo_ros gazebo_model_path="${prefix}/models"/>\n  </export>#' \
  ~/catkin_ws/src/mrobot_gazebo/package.xml
```

### 📷 坑③ 踩坑实录（执行 `cd ~/catkin_ws/src/mrobot_gazebo/package.xml`，终端报错 `bash: cd: ...: 不是目录` —— package.xml 是文件，不是目录，必须用编辑器打开。

!\[坑③：误用 cd 打开 package.xml 报"不是目录"]([https://aka.doubaocdn.com/s/crOqBgagej](https://aka.doubaocdn.com/s/crOqBgagej))

### 📷 坑③ 修复完成（16:17）

用文本编辑器打开 `package.xml`，在 `<export>` 标签内已成功加入 `<gazebo_ros gazebo_model_path="${prefix}/models"/>`（第 58 行附近）。

!\[坑③修复：package.xml 已加入 gazebo_model_path]([https://aka.doubaocdn.com/s/SFwWvfz8qt](https://aka.doubaocdn.com/s/SFwWvfz8qt))

***

### 坑④ —— xacro 宏缺 `xacro:` 前缀（不修会报 `plate_2_link not found`）

```bash
cd ~/catkin_ws/src/mrobot_gazebo/urdf
sed -i \
 -e 's#<wheel #<xacro:wheel #g' \
 -e 's#<caster #<xacro:caster #g' \
 -e 's#<plate #<xacro:plate #g' \
 -e 's#<mrobot_standoff_2in #<xacro:mrobot_standoff_2in #g' \
 -e 's#<cylinder_inertial_matrix #<xacro:cylinder_inertial_matrix #g' \
 -e 's#<sphere_inertial_matrix #<xacro:sphere_inertial_matrix #g' \
 mrobot_body.urdf.xacro

sed -i 's#<mrobot_body/>#<xacro:mrobot_body/>#g' \
 mrobot_with_rplidar.urdf.xacro \
 mrobot_with_kinect.urdf.xacro \
 mrobot_with_camera.urdf.xacro
```

### 坑⑤ —— teleop 脚本 Python2 → Python3 语法

```bash
cd ~/catkin_ws/src/mrobot_teleop/scripts
sed -i '1s|#!/usr/bin/env python$|#!/usr/bin/env python3|' mrobot_teleop.py

# 修 print 语句（含 print e → pass，避免 NameError）
python3 - <<'PYEOF'
p = 'mrobot_teleop.py'
s = open(p, encoding='utf-8').read()
s = s.replace('print msg', 'print(msg)')
s = s.replace('print vels(speed,turn)', 'print(vels(speed,turn))')
s = s.replace('        print e', '        pass')
open(p, 'w', encoding='utf-8').write(s)
print('fixed')
PYEOF
```

### 📷 坑⑤ 修复实录

终端中执行 Python3 批处理脚本，依次替换 `print msg`、`print vels(...)`、`print e` 三处 Py2 写法，输出 `fixed` 表示修复成功；随后进入 `catkin_make` 编译阶段。

!\[坑⑤修复：teleop 脚本 Py2→Py3 替换完成，开始编译]([https://aka.doubaocdn.com/s/bsNE2V5IE2](https://aka.doubaocdn.com/s/bsNE2V5IE2))

***

## 第 4 步：编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

> 看到各包 `100%`、`Build finished` 即成功。

### 📷 第 4 步编译完成

`catkin_make` 依次处理 `mrobot_teleop`、`mrobot_navigation`、`mrobot_gazebo` 三个包，输出 `Configuring done`、`Generating done`、`Build files have been written to: /home/lyd/catkin_ws/build`，随后 `source devel/setup.bash` 成功。

> 注：日志中的 gazebo\_\* 包 deprecation 警告（Gazebo classic 11 已于 2025 年 1 月停止维护）仅为提示，不影响本次仿真使用。

!\[第4步：catkin_make 编译成功并 source 环境]([https://aka.doubaocdn.com/s/0htrDPV9v6](https://aka.doubaocdn.com/s/0htrDPV9v6))

***

## 第 5 步：启动三件套（3 个终端，每个都要先 source）

```bash
# 终端1：Gazebo 仿真环境 + 带激光雷达的 mrobot
source ~/catkin_ws/devel/setup.bash
roslaunch mrobot_gazebo mrobot_laser_nav_gazebo.launch
```

```bash
# 终端2：gmapping 建图（会自动开 RViz）
source ~/catkin_ws/devel/setup.bash
roslaunch mrobot_navigation gmapping_demo.launch
```

```bash
# 终端3：键盘遥控
source ~/catkin_ws/devel/setup.bash
roslaunch mrobot_teleop mrobot_teleop.launch
```

> **坑⑥**：Gazebo 报 `SpawnModel: entity already exists` → 上次没退干净，先清进程再重跑终端1：
`killall -9 gzserver gzclient rosmaster roscore rviz 2>/dev/null`

### 📷 终端1：Gazebo 仿真启动成功

DiffDrive 差速驱动节点正常加载：订阅 `cmd_vel`、发布 `odom` / `joint_states`，`urdf_spawner-6 process has finished cleanly`，小车模型成功生成。

> 日志中的 `missing <publishOdomTF>`、`missing <odometrySource>` 仅为 urdf 参数标签缺省提示，不影响运行，可忽略。

!\[终端1：Gazebo 启动，DiffDrive 差速驱动加载成功]([https://aka.doubaocdn.com/s/MsiPJzFcYm](https://aka.doubaocdn.com/s/MsiPJzFcYm))

### 📷 终端1：Gazebo 仿真世界视图

Gazebo 3D 视窗：灰色地面网格上搭建 `my_room` 白色迷宫墙体，mrobot 小车位于迷宫内部；左侧模型树展开 mrobot 结构（LINKS：base_footprint、wheel_left_link、wheel_right_link；JOINTS：base_to_wheel_left_joint、base_to_wheel_right_joint；PLUGINS：differential_drive_control），底部显示仿真运行参数（FPS 32.07）。

!\[Gazebo 仿真世界：my_room 迷宫 + mrobot 小车]([https://aka.doubaocdn.com/s/WyNQ3nkwvR](https://aka.doubaocdn.com/s/WyNQ3nkwvR))

### 📷 终端2：RViz 建图画面

左侧终端为 Gazebo 启动日志，右侧 RViz 显示 `gmapping.rviz`：Displays 面板勾选 Grid / Odometry / Odometry EKF / RobotModel / Map / LaserScan，可视化区域中**红色激光扫描点 + 栅格地图逐渐生长**，帧率 31fps，一切正常。

!\[终端2：RViz 中激光雷达扫描 + 栅格地图生成]([https://aka.doubaocdn.com/s/kndIUcv38Q](https://aka.doubaocdn.com/s/kndIUcv38Q))

### 📷 终端3：键盘遥控节点启动

`mrobot_teleop.launch` 启动成功，打印 `Control mrobot!` 操作说明与当前速度参数 `currently: speed 0.2 turn 1`，等待键盘指令控制小车运动。

!\[终端3：键盘遥控节点启动成功]([https://aka.doubaocdn.com/s/7HGDoXCwUI](https://aka.doubaocdn.com/s/7HGDoXCwUI))

***

## 第 6 步：遥控建图

鼠标点一下**终端3**获得键盘焦点，按键控制：

| 键 | 动作 |
| --- | --- |
| `i` / `,` | 前进 / 后退 |
| `j` / `l` | 原地左转 / 右转 |
| `k` 或空格 | 停车 |
| `z` / `q` | 减速 / 加速 |

开着车贴墙绕房间走一圈，RViz 里红色激光线 + 灰色地图生长。

**关键概念**：灰色 = 已确认自由空间；黑色 = 未知区域（还没扫到，**不是墙**）；必须开车过去激光照到才会填灰。小车能到达的角落都扫到、回到起点闭合即完成。

### 📷 第 6 步遥控建图实况

Gazebo 与 RViz 同屏：RViz 中红色激光扫描轨迹勾勒出迷宫墙体轮廓，栅格地图实时生长（31 fps）；左侧终端为键盘遥控节点 `mrobot_teleop`（Control mrobot!），通过按键控制小车贴墙遍历环境。

!\[第6步：RViz 激光建图 + 键盘遥控终端同屏]([https://aka.doubaocdn.com/s/XiXduEb8WG](https://aka.doubaocdn.com/s/XiXduEb8WG))

***

## 第 7 步：保存地图

先在终端3按 `Ctrl-C` 停车退出，然后新开终端：

```bash
source ~/catkin_ws/devel/setup.bash
cd ~
rosrun map_server map_saver -f mymap
```

看到 `Done` 即成功，主目录生成两个文件：

- `mymap.pgm` —— 灰度栅格地图图片
- `mymap.yaml` —— 地图元数据（分辨率、原点、阈值等）

### 📷 第 7 步保存成功

`map_saver` 输出：`Received a 608 X 480 map @ 0.050 m/pix`，成功写入 `mymap.pgm` 与 `mymap.yaml`，输出 `Done`，SLAM 建图完整结束。

> 注：实测分辨率为 608 × 480（以截图为准），像素精度 0.050 m/pix。

!\[第7步：map_saver 保存地图成功]([https://aka.doubaocdn.com/s/W4T4jonZMU](https://aka.doubaocdn.com/s/W4T4jonZMU))

***

## 附录：过程截图记录（与正文对应）

| 顺序 | 截图时间 | 对应环节 | 内容 |
| --- | --- | --- | --- |
| 1 | 10-18 15:06 | 第 5 步 | Gazebo 仿真世界视图：my_room 迷宫墙体 + mrobot 小车（模型树展示差速驱动插件），FPS 32.07 |
| 2 | 10-18 15:18 | 第 6 步 | 遥控建图实况：RViz 红色激光扫描迷宫轮廓 + 栅格地图生长（31fps）+ 键盘遥控终端 |
| 3 | 10-18 16:06 | 第 0~2 步 | `rosversion -d`→noetic、复制三个功能包、chmod、`sudo apt update`（密码输错 3 次后成功，190 个包可升级） |
| 4 | 10-18 16:08 | 坑③ | 误用 `cd package.xml` 报"不是目录" |
| 5 | 10-18 16:17 | 坑③ | 文本编辑器修改 package.xml，已加入 `gazebo_ros gazebo_model_path` |
| 6 | 10-18 16:18 | 坑⑤ | Python3 脚本修复 teleop（输出 fixed），进入 catkin_make |
| 7 | 10-18 16:18 | 第 4 步 | catkin_make 编译完成（3 个包），`source devel/setup.bash` |
| 8 | 10-18 16:19 | 第 5 步 | Gazebo 启动，DiffDrive 差速驱动加载，urdf_spawner 正常结束 |
| 9 | 10-18 16:19 | 第 5 步 | RViz 建图画面：红色激光 + 栅格地图生长（31fps） |
| 10 | 10-18 16:20 | 第 5 步 | 键盘遥控节点启动成功（speed 0.2 turn 1） |
| 11 | 10-18 16:21 | 第 7 步 | `map_saver` 保存成功，608 × 480 @ 0.050 m/pix，生成 mymap.pgm / mymap.yaml |

***

## 遇到的坑速查表

| 编号 | 症状 | 根因 | 解决 |
| --- | --- | --- | --- |
| 坑① | `cannot locate node of type [mrobot_teleop/mrobot_teleop.py]` | 脚本无可执行权限 | `chmod +x` |
| 坑② | `Could not find the required component 'xxx'` | 缺 ros-noetic 依赖 | `sudo apt install ros-noetic-xxx` |
| 坑③ | Gazebo 找不到包内 models | package.xml 未导出模型路径 | 加 `<gazebo_ros gazebo_model_path>` |
| 坑④ | `plate_2_link not found` | xacro 宏调用缺 `xacro:` 前缀 | sed 补齐前缀 |
| 坑⑤ | `/usr/bin/env: python 没有那个文件` / `print` 语法报错 | Py2 写法 | shebang 改 python3 + 修 print |
| 坑⑥ | `SpawnModel: entity already exists` | 残留 gzserver/gzclient | `killall -9` 清进程 |

***

## 复现结果总结

1. **环境**：Ubuntu 20.04 + ROS Noetic（1.17.4）+ Gazebo 11，物理机（Lenovo Legion Y7000）原生运行；
2. **迁移**：完成 kinetic → noetic 三处代码兼容性修改（package.xml 模型路径、xacro 前缀、Py2→Py3）；
3. **仿真**：Gazebo 加载带 rplidar 激光雷达的 mrobot 差速小车，DiffDrive 正常发布 `odom`/`joint_states`；
4. **建图**：RViz 中激光扫描 + gmapping 实时生成栅格地图，键盘遥控遍历环境；
5. **保存**：`map_saver` 成功保存 608 × 480、0.050 m/pix 的栅格地图（mymap.pgm + mymap.yaml）。
