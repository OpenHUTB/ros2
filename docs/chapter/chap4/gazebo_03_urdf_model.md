---
title: 步骤 3：URDF 与 xacro 建模
---

# 步骤 3：URDF 与 xacro 建模

> 前置条件：完成[步骤 2](gazebo_02_package.md)。
> 本步目标：理解并验证 `mybot.urdf.xacro`，得到一个结构合法的移动机器人模型。
> 预计时间：60-90 分钟。

- [1. 模型组成](#components)
- [2. 坐标约定](#frames)
- [3. 参数化](#properties)
- [4. 车体与车轮](#body)
- [5. 支撑轮与传感器](#sensors)
- [6. 检查模型](#check)
- [7. 常见问题](#debug)
- [8. 本步验收](#acceptance)

## 1. 模型组成 <span id="components"></span>

`mybot` 由以下部件构成：

| Link | 类型 | 作用 |
| --- | --- | --- |
| `base_link` | 根链接 | 车体本体 |
| `left_wheel_link` | 刚体 | 左驱动轮 |
| `right_wheel_link` | 刚体 | 右驱动轮 |
| `caster_link` | 刚体 | 后部支撑轮 |
| `laser_frame` | 传感器 frame | 2D 激光雷达 |
| `camera_link` | 传感器 frame | 前视 RGB 相机 |

关节：

| Joint | 类型 | 说明 |
| --- | --- | --- |
| `left_wheel_joint` | continuous | 左轮连续旋转 |
| `right_wheel_joint` | continuous | 右轮连续旋转 |
| `caster_joint` | continuous | 支撑轮旋转 |
| `laser_joint` | fixed | 激光固定安装 |
| `camera_joint` | fixed | 相机固定安装 |

URDF 中三种几何标签的含义必须分清：

```xml
<visual>...</visual>    <!-- 只负责显示 -->
<collision>...</collision> <!-- 参与物理碰撞 -->
<inertial>...</inertial>   <!-- 决定质量和转动惯量 -->
```

只写 `visual` 的模型在 rviz 中可能可见，但进入 Gazebo 后会掉落、穿地或无法稳定接触地面。

## 2. 坐标约定 <span id="frames"></span>

本实验固定使用 ROS 常见坐标方向：

```text
x：机器人前方
y：机器人左侧
z：机器人上方
```

因此：

- 车体相机安装在正前方，`camera_joint` 的 x 为正；
- 左右轮通过 y 方向镜像；
- `odom -> base_link` 表示机器人在里程计坐标系中的位姿；
- `base_link -> laser_frame` 与 `base_link -> camera_link` 描述传感器安装位置。

## 3. 参数化 <span id="properties"></span>

xacro 文件首先定义可复用参数：

```xml
<xacro:property name="base_length" value="0.42"/>
<xacro:property name="base_width" value="0.28"/>
<xacro:property name="base_height" value="0.12"/>
<xacro:property name="wheel_radius" value="0.05"/>
<xacro:property name="wheel_width" value="0.03"/>
<xacro:property name="wheel_separation" value="0.28"/>
<xacro:property name="wheel_offset_z" value="-0.07"/>
<xacro:property name="caster_radius" value="0.035"/>
<xacro:property name="caster_offset_z" value="-0.085"/>
```

修改这些参数后，必须同步考虑：

- 车轮是否仍然接触地面；
- 车体是否与地面碰撞；
- 差速插件中的轮径和轮距是否一致；
- Gazebo 生成模型的 `z` 初值是否需要变化。

## 4. 车体与车轮 <span id="body"></span>

车体使用 `box` 几何体：

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="${base_length} ${base_width} ${base_height}"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="${base_length} ${base_width} ${base_height}"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.090" ixy="0" ixz="0"
             iyy="0.120" iyz="0" izz="0.155"/>
  </inertial>
</link>
```

车轮使用 xacro 宏，左右轮共用同一套设置：

```xml
<xacro:macro name="wheel" params="prefix reflect">
  <link name="${prefix}_wheel_link">
    <visual>
      <origin xyz="0 0 0" rpy="1.57079632679 0 0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </visual>
  </link>

  <joint name="${prefix}_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="${prefix}_wheel_link"/>
    <origin xyz="0 ${reflect * wheel_separation / 2.0} ${wheel_offset_z}"/>
    <axis xyz="0 1 0"/>
  </joint>
</xacro:macro>
```

最容易出错的地方：

- 圆柱默认轴是 z，忘记旋转 90 度会让车轮像圆盘一样直立；
- 左右轮镜像方向错误，会让机器人不能直行；
- 关节轴写错，差速插件给速度后车轮不会正确滚动。

## 5. 支撑轮与传感器 <span id="sensors"></span>

支撑轮让两轮机器人保持三点接触：

```xml
<joint name="caster_joint" type="continuous">
  <parent link="base_link"/>
  <child link="caster_link"/>
  <origin xyz="-0.15 0 ${caster_offset_z}"/>
  <axis xyz="0 1 0"/>
</joint>
```

激光和相机：

```xml
<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="laser_frame"/>
  <origin xyz="0 0 0.09"/>
</joint>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.20 0 0.06"/>
</joint>
```

传感器安装位置错误不会立刻让 Gazebo 崩溃，但会让地图、视觉和导航结果全部偏移。

## 6. 检查模型 <span id="check"></span>

展开 xacro：

```bash
xacro ~/catkin_ws/src/mybot_gazebo/urdf/mybot.urdf.xacro > /tmp/mybot.urdf
```

检查结构和链接关系：

```bash
check_urdf /tmp/mybot.urdf
urdf_to_graphiz /tmp/mybot.urdf
```

`check_urdf` 通过只代表结构合法，不代表物理参数正确。物理稳定性要在 Gazebo 中验证。

在 rviz 中查看 TF 和模型：

如果没有安装额外的 URDF 查看工具，本步骤只要求 `check_urdf` 通过；完整模型、TF 和传感器的首次显示安排在[步骤 7](gazebo_07_first_simulation.md)中进行。若环境已安装 `urdf_tutorial`，也可以运行：

```bash
roslaunch urdf_tutorial display.launch model:=/tmp/mybot.urdf
```

## 7. 常见问题 <span id="debug"></span>

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| xacro 展开失败 | XML 语法或 `${}` 表达式错误 | 检查标签是否闭合 |
| `check_urdf` 报错 | link/joint 父子关系不合法 | 检查 joint 的 parent 和 child |
| 模型在 Gazebo 中下陷 | 初始高度或支撑轮高度不对 | 调整 `-z` 和关节 origin |
| 模型抖动 | 质量或惯性矩阵不合理 | 让质量和几何尺寸匹配 |
| 车轮方向错误 | 关节轴或镜像方向错误 | 检查 `axis` 和 `reflect` |
| 相机看不到目标 | 相机朝向错误 | 检查 `camera_joint` 的 origin 和 rpy |

## 8. 本步验收 <span id="acceptance"></span>

- [ ] 能解释 URDF 中 link 和 joint 的层级关系；
- [ ] 能说明 visual、collision 和 inertial 的区别；
- [ ] `xacro` 展开成功；
- [ ] `check_urdf` 通过；
- [ ] 能说出车轮、激光、相机的安装位置和坐标方向。

---

- 上一步：[工作空间与功能包](gazebo_02_package.md)
- 返回[教程总览](gazebo_platform_simulation.md)
- 下一步：[步骤 4：差速驱动与里程计](gazebo_04_diff_drive.md)
