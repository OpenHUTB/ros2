# 工程问题排查记录

本项目在开发过程中遇到并解决了多个**不易定位**的工程问题。
本章完整记录问题现象、定位过程与解决方案，作为工程经验的沉淀。

---

## 10.1 `/cmd_vel` 完全无消息

### 现象

启动 `mode:=laser` 后：

- Gazebo 正常启动，模型成功生成
- `/scan` 有数据（10 Hz）
- 节点日志显示"避障节点启动"
- **但 `/cmd_vel` 完全没有消息**

```
$ rostopic hz /cmd_vel
subscribed to [/cmd_vel]
no new messages
no new messages
no new messages
```

同时车辆静止不动。

### 定位过程

**第一步：确认节点是否存活**

```bash
$ pgrep -af "main.py"
10351 python3 .../main.py __name:=openhutb_car_main

$ rosnode list
/gazebo
/openhutb_car_main
/robot_state_publisher
...
```

节点存活 ✓

**第二步：检查话题连接**

```
[rospy] topic[/cmd_vel] adding connection to [/gazebo], count 0
[rospy] topic[/cmd_vel] adding connection to [/rostopic_...], count 1
[rospy] topic[/cmd_vel] removing connection to /rostopic_...
```

**连接已建立**，说明发布者注册正常，但从未实际 `publish()`。

**第三步：审查代码**

```python
# ❌ 有缺陷的实现
def feed(self, twist_msg):
    """喂入一条新的速度指令，刷新看门狗计时。"""
    with self._lock:
        self._last_cmd = twist_msg
        self._last_cmd_time = rospy.Time.now()
        self._stopped = False
    # ← 这里没有 publish！

def check(self):
    with self._lock:
        if self._stopped:
            return False
        elapsed = (rospy.Time.now() - self._last_cmd_time).to_sec()
        if elapsed > self._timeout:
            self._pub.publish(Twist())    # ← 只有超时才发布
            self._stopped = True
            ...
```

**根因确认**：`feed()` 只把指令缓存到 `self._last_cmd`，
真正的 `publish()` 只在 `check()` 检测到**超时**时才执行。

结果形成死锁逻辑：

```
feed() 更新 _last_cmd_time
   ↓
check() 发现 elapsed < timeout，不超时
   ↓
不 publish
   ↓
车辆静止（从未收到指令）
```

**指令永远不会被发布** —— 只有算法停止调用 `feed()` 导致超时后，
才会发布一个**全零**的停车指令。这与设计意图完全相反。

### 解决方案

`feed()` 必须立即发布：

```python
# ✅ 修复后
def feed(self, twist_msg):
    """喂入一条新的速度指令：刷新计时并立即发布。

    注意：此处必须真正 publish，否则算法计算出的速度只停留在本对象内部，
    底盘控制器永远收不到指令（车辆静止不动）。
    """
    with self._lock:
        self._last_cmd = twist_msg
        self._last_cmd_time = rospy.Time.now()
        self._stopped = False
    self._pub.publish(twist_msg)      # ★ 关键修复
```

### 验证

```
$ rostopic hz /cmd_vel
subscribed to [/cmd_vel]
average rate: 19.715
	min: 0.048s max: 0.067s std dev: 0.00411s window: 19
average rate: 19.874
```

**20 Hz 稳定输出** ✓

### 经验总结

**教训**：看门狗（watchdog）模式的正确语义应该是
"**正常路径直接发布，异常路径覆盖发布**"，
而不是"缓存后由检查器发布"。

设计安全机制时，必须保证**正常路径与异常路径都能正确输出**，
不能因为引入安全逻辑而破坏主功能。

---

## 10.2 `cannot import name 'LaserAvoidNode'`

### 现象

```
[FATAL] [openhutb_car_main] 模块导入失败：
        cannot import name 'LaserAvoidNode' from 'laser_avoid'
        (/home/user/catkin_ws/devel/lib/openhutb_car/laser_avoid.py)
```

`main.py` 无法导入同目录下的 `laser_avoid.py`。

### 定位过程

**第一步：检查文件是否存在**

```bash
$ ls -la devel/lib/openhutb_car/
-rwxr-xr-x 1 user user  577 depth_camera_avoid.py    ← 只有 577 字节！
-rwxr-xr-x 1 user user  570 laser_avoid.py           ← 只有 570 字节！
-rwxr-xr-x 1 user user  563 main.py
```

**文件大小异常** —— 源文件是 14432 字节，devel 里只有 570 字节。

**第二步：查看文件内容**

```bash
$ head -15 devel/lib/openhutb_car/laser_avoid.py
#!/usr/bin/python3
# -*- coding: utf-8 -*-
# generated from catkin/cmake/template/script.py.in
# creates a relay to a python script source file, acting as that file.
# The purpose is that of a symlink
python_script = '/home/user/catkin_ws/src/openhutb_car/scripts/laser_avoid.py'
with open(python_script, 'r') as fh:
    context = {
        '__builtins__': __builtins__,
        '__doc__': None,
        '__file__': python_script,
        '__name__': __name__,
        '__package__': None,
    }
    exec(compile(fh.read(), python_script, 'exec'), context)
```

**这是一个 relay 脚本**（转发器），不是真实代码。

**根因确认**：`catkin_install_python()` 宏会在 devel 空间生成
relay 脚本，其作用是让 devel 中的文件"表现得像"源文件的符号链接 ——
通过 `exec()` 执行源文件内容。

**为什么这会导致导入失败？**

relay 脚本用 `exec(compile(...), context)` 执行源文件，
其中 `context` 字典的 `__name__` 被设为调用者的名字。

当 `main.py` 执行 `from laser_avoid import LaserAvoidNode` 时：

1. Python 找到 `laser_avoid.py`（relay 脚本）
2. 执行 relay 脚本，它在 `context` 中 exec 源文件
3. 源文件中的 `class LaserAvoidNode` 被定义到 `context` 字典中
4. **但 `context` 不是模块的命名空间** —— 类定义没有挂到模块上
5. `import` 语句查找模块的 `LaserAvoidNode` → 找不到 → ImportError

### 解决方案

用 `install(PROGRAMS)` + 自定义 CMake 规则**原样复制**文件，
不经过 relay 机制：

```cmake
# ---- (1) 安装空间：标准安装 ----
install(PROGRAMS
  scripts/main.py
  scripts/teleop_keyboard.py
  scripts/laser_avoid.py
  scripts/depth_camera_avoid.py
  scripts/odom_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---- (2) devel 空间：原样复制为可导入模块 ----
# catkin_make 只把编译产物放进 devel/，install() 规则要到 catkin_make install
# 才生效。为了让 rosrun / roslaunch 在纯 devel 环境下就能找到节点，
# 这里用一个自定义 target 把脚本原样复制到 devel/lib/<pkg>/。
set(_DEVEL_BIN_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(_script
    main.py teleop_keyboard.py laser_avoid.py
    depth_camera_avoid.py odom_recorder.py)

  add_custom_command(OUTPUT ${_DEVEL_BIN_DIR}/${_script}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${_DEVEL_BIN_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/scripts/${_script}
            ${_DEVEL_BIN_DIR}/${_script}
    COMMAND chmod +x ${_DEVEL_BIN_DIR}/${_script}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/${_script}
    COMMENT "Copying ${_script} to devel space"
  )
  list(APPEND _DEVEL_SCRIPTS ${_DEVEL_BIN_DIR}/${_script})
endforeach()

add_custom_target(openhutb_car_scripts ALL DEPENDS ${_DEVEL_SCRIPTS})
```

同时在 `main.py` 中加固模块搜索路径：

```python
import os, sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
```

### 验证

```bash
$ ls -la devel/lib/openhutb_car/
-rwxrwxrwx 1 user user 13707 depth_camera_avoid.py   ← 完整字节数
-rwxrwxrwx 1 user user 14432 laser_avoid.py
-rwxrwxrwx 1 user user  9100 main.py
-rwxrwxrwx 1 user user  9551 odom_recorder.py
-rwxrwxrwx 1 user user  9537 teleop_keyboard.py

$ python3 -c "
from laser_avoid import LaserAvoidNode;      print('laser_avoid     OK')
from teleop_keyboard import TeleopKeyboardNode; print('teleop_keyboard OK')
from depth_camera_avoid import DepthCameraAvoidNode; print('depth_camera    OK')
"
laser_avoid      OK
teleop_keyboard  OK
depth_camera     OK
```

### 经验总结

**教训**：`catkin_install_python()` 适用于**单文件自包含**的节点脚本。
若脚本之间存在 `import` 依赖，必须改用原样复制。

**通用建议**：需要跨脚本导入时，更规范的做法是组织成
Python 包（含 `__init__.py`），用 `catkin_python_setup()` 安装。
本项目为保持结构扁平（便于任务书审阅），采用了复制方案。

---

## 10.3 车轮空转、车辆不动

### 现象

- `/cmd_vel` 正常发布（20 Hz，`linear.x = 0.2`）
- 车轮关节在持续转动
- **但车辆几乎不移动**

实测数据：

```
$ rostopic echo -n 2 /joint_states
name: [left_wheel_joint, right_wheel_joint]
position: [168.925, 168.928]      ← 车轮转了 168 弧度！
velocity: [2.052]                  ← 角速度正常

车体位置: 30 秒仅移动 0.065 m      ← 应该移动约 6 m
```

### 定位过程

**第一步：检查车体高度**

```bash
$ rostopic echo -n 1 /gazebo/model_states | grep -A5 openhutb_car
position:
  x: 0.18
  y: 0.00
  z: 0.00        ← 车体中心高度为 0！
```

**第二步：对照 URDF 设计**

原设计：

```xml
<!-- ❌ 错误设计 -->
<link name="base_link">
  <collision>
    <origin xyz="0 0 ${chassis_height/2}" .../>   <!-- 0.11 -->
    <geometry><box size="0.44 0.38 0.22"/></geometry>
  </collision>
</link>
```

车体碰撞盒中心在 $z = 0.11$，尺寸 0.22，则**底面在 $z = 0$** ——
**正好压在地面上**。

**第三步：确认物理机制**

Gazebo 中，车体碰撞盒与地面接触会产生：

1. **法向力**：支撑车体重量
2. **切向摩擦力**：阻碍相对滑动

当车体底面与地面接触时，切向摩擦直接作用在车体上，
其大小远超驱动轮能提供的牵引力。结果：

```
驱动轮转动 → 轮地摩擦产生推力 → 但车体被地面摩擦"钉住" → 车轮打滑空转
```

**根因确认**：**底盘离地间隙为零**，车体与地面直接接触。

### 解决方案

**修正几何关系**：让车体中心位于车轮顶部之上。

```xml
<!-- ✅ 正确设计 -->
<!--
  车体离地高度（关键几何参数）
  ==================================================================
  车轮半径 0.0975 m，因此车体底面必须高于地面，否则底盘碰撞体
  会与地面直接接触，摩擦力将整车「钉死」，表现为车轮空转但车辆不动。

  这里令车体中心位于 z = wheel_radius + chassis_height/2，
  使车体底面恰好与车轮顶部齐平，底盘离地间隙 = 车轮半径 = 0.0975 m。
-->
<xacro:property name="chassis_z"
                value="${wheel_radius + chassis_height/2}"/>   <!-- 0.2075 -->

<link name="base_link">
  <collision>
    <origin xyz="0 0 ${chassis_z}" rpy="0 0 0"/>    <!-- ← 用 chassis_z -->
    <geometry><box size="0.44 0.38 0.22"/></geometry>
  </collision>
</link>
```

**同时调整传感器高度**：

```xml
<xacro:property name="laser_z"
                value="${chassis_z + chassis_height/2 + 0.03}"/>   <!-- 0.3475 -->
<xacro:property name="camera_z"
                value="${chassis_z + chassis_height/2 - 0.01}"/>   <!-- 0.3075 -->
```

**并给驱动轮设置足够的摩擦系数**：

```xml
<gazebo reference="left_wheel_link">
  <mu1>1.0</mu1>      <!-- 摩擦系数 -->
  <mu2>1.0</mu2>
  <kp>1000000.0</kp>  <!-- 接触刚度 -->
  <kd>100.0</kd>      <!-- 接触阻尼 -->
</gazebo>
```

**同时移除万向轮的碰撞几何**（避免额外拖拽摩擦）：

```xml
<!-- 万向轮保留为视觉件，不参与碰撞计算 -->
<link name="front_caster_link">
  <visual>...</visual>
  <!-- 无 collision 标签 -->
  <inertial>...</inertial>
</link>
```

### 验证

修复前后对比：

| 指标 | 修复前 | 修复后 |
|---|---|---|
| 车体 z 坐标 | 0.000 | 0.2075 |
| 30 秒位移 | 0.065 m | **6.31 m** |
| 路径效率 | — | **0.997** |

### 经验总结

**教训**：URDF 中的几何关系必须**自洽** ——
碰撞体的位置、尺寸与运动学参数（轮半径、轮距）必须一致。

**排查技巧**：当"指令正常但车辆不动"时，优先检查：

1. 车体 z 坐标是否异常（触地）
2. 车轮关节角度是否累积（打滑）
3. 摩擦系数是否过低

**通用原则**：差速小车的底盘必须有**离地间隙**，
且驱动轮摩擦系数要足够高（建议 $\mu \geq 0.8$）。

---

## 10.4 深度相机读数恒为 1.000 m

### 现象

使用 `libgazebo_ros_openni_kinect.so` 插件时，
深度图统计结果异常：

```
left   p15=1.00  p40=1.00  median=1.45
mid    p15=1.00  p40=1.00  median=1.00    ← 全部锁定为 1.0
right  p15=1.00  p40=1.00  median=1.00
```

算法判定"前方 1 m 有障碍"（$< d_{\text{safe}} = 1.2$ m），
车辆持续原地打转，**30 秒仅移动 0.08 m**。

### 定位过程

**第一步：检查哪个区域的像素**

```python
img = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
h, w = img.shape
r0, r1 = int(0.25*h), int(0.75*h)

for name, (c0f, c1f) in [('left',(0.00,0.25)), ('mid',(0.37,0.62)), ('right',(0.75,1.00))]:
    p = img[r0:r1, int(c0f*w):int(c1f*w)]
    v = p[np.isfinite(p) & (p > 0.01)]
    print('%-6s min=%.3f p15=%.3f median=%.3f max=%.3f' %
          (name, v.min(), np.percentile(v,15), np.median(v), v.max()))
```

输出：

```
left   min=0.051  p15=1.000  median=1.296  max=2.460
mid    min=0.051  p15=1.000  median=1.000  max=8.762   ← max 有真实值！
right  min=0.051  p15=1.000  median=1.000  max=9.232
```

**关键发现**：`max` 显示存在 8.762 m 的真实测距，
但 `median` 却是 1.000 —— 说明**大部分像素是同一个固定值**。

**第二步：直方图分析**

```python
hist, edges = np.histogram(v, bins=...)
peak_idx = np.argmax(hist)
ratio = hist[peak_idx] / v.size
print('峰值深度=%.3f  占比=%.1f%%' % ((edges[peak_idx]+edges[peak_idx+1])/2, ratio*100))
```

输出：

```
left   floor 未检出 (7%)
mid    floor = 1.005 (66%)     ← 66% 的像素都是 1.005！
right  floor = 1.005 (63%)
```

**根因确认**：Gazebo 深度相机对**无回波区域**输出一个固定哨兵值
**1.005 m**（而非标准的 `0` 或 `inf`）。

中区看向前方远处，**66% 的区域超出有效范围**，全部返回 1.005。
这个值看起来完全像一个"正常的深度读数"，因此极易被误认为
传感器工作正常。

**为什么百分位统计失效？**

$$
P_{25}(\text{数据}) = 1.005 \quad \text{当且仅当} \quad
\text{至少 75\% 的数据} \geq 1.005
$$

当中区 66% 是 1.005、其余为 1.005~8.762 时，
第 25 百分位必然落在 1.005 这个密集点上。

无论是 p15、p25 还是 p40，结果都是 1.005。

### 解决方案

**第一步：更换插件**

从 `libgazebo_ros_openni_kinect.so` 换成
`libgazebo_ros_depth_camera.so`（不经过点云截断处理）：

```xml
<plugin name="gazebo_ros_depth_camera_controller"
        filename="libgazebo_ros_depth_camera.so">
  <alwaysOn>true</alwaysOn>
  <updateRate>15.0</updateRate>
  <cameraName>camera</cameraName>
  <imageTopicName>rgb/image_raw</imageTopicName>
  <depthImageTopicName>depth/image_raw</depthImageTopicName>
  <pointCloudTopicName>depth/points</pointCloudTopicName>
  <frameName>camera_optical_link</frameName>
</plugin>
```

更换后 `max` 从 1.0 变为 9.23，但仍存在 1.005 地板值。

**第二步：直方图峰值检测（核心方案）**

```python
@staticmethod
def _detect_floor_value(values):
    """检测深度图中的固定地板值（无效标记）。"""
    if values.size < 100:
        return None

    vmin, vmax = float(values.min()), float(values.max())
    if vmax - vmin < FLOOR_BIN_WIDTH:
        return None

    n_bins = min(int((vmax - vmin) / FLOOR_BIN_WIDTH) + 1, 4096)
    hist, edges = np.histogram(values, bins=n_bins)

    peak_idx = int(np.argmax(hist))
    peak_ratio = float(hist[peak_idx]) / float(values.size)

    if peak_ratio >= FLOOR_VALUE_RATIO:      # 0.30
        return float((edges[peak_idx] + edges[peak_idx + 1]) / 2.0)

    return None
```

**第三步：剔除地板值**

```python
floor_value = self._detect_floor_value(valid)
if floor_value is not None:
    keep = np.abs(valid - floor_value) > (FLOOR_BIN_WIDTH * 0.5)
    filtered = valid[keep]
    if filtered.size >= 30:          # 保护：剔除后仍需足够像素
        valid = filtered
```

### 验证

修复后：

```
left   floor 未检出 (7%)     → p25 = 0.855 m
mid    floor = 1.005 (66%)   → 剔除后 p25 = 3.509 m   ← 真实值！
right  floor = 1.005 (63%)   → 剔除后 p25 = 2.640 m
```

行为对比：

| 阶段 | 中区 p25 | 30 秒位移 | 现象 |
|---|---|---|---|
| 修复前 | 1.000 m（假） | **0.08 m** | 原地打转 |
| 修复后 | 3.509 m（真） | **7.40 m** | 正常避障 |

### 经验总结

**教训**：仿真传感器可能**不遵循真实设备的无效值约定**。
Gazebo 深度相机用 1.005 m 表示"无回波"，
这是一个看起来完全合理的数值，极具迷惑性。

**排查技巧**：

1. 打印统计量的**分布**（直方图），而不只是 min/median/max
2. 比较 `median` 与 `max` —— 若差异极大，说明数据分布异常
3. 用 `np.unique()` 查看是否有异常密集的重复值

**通用防御**：处理传感器数据时，除标准的 `0`/`inf`/`nan` 过滤外，
还应加入**数据合理性校验**（如检测固定值聚集）。

---

## 10.5 XML 注释中的连续连字符

### 现象

```
$ xacro car.xacro
XML parsing error: not well-formed (invalid token): line 201, column 35
when processing file: car.xacro
```

### 根因

XML 规范规定：**注释内容中不能出现连续两个或以上的连字符 `--`**。

我写的分节注释使用了 `----` 作为分隔线：

```xml
<!-- ==================== 激光雷达 ---------------------------------------- -->
                                                              ↑↑↑↑
                                                    非法！XML 不允许
```

### 解决方案

注释内的分隔线改用 `=` 或 `~`：

```xml
<!-- ==================== 激光雷达 ======================================== -->
```

### 批量检查脚本

```powershell
$files = Get-ChildItem -Recurse -Include *.xacro,*.launch,*.world,*.xml
foreach ($f in $files) {
  $c = Get-Content $f.FullName -Raw
  $m = [regex]::Matches($c, '<!--[^>]*--[^>]*-->')
  if ($m.Count -gt 0) {
    "FAIL $($f.Name): $($m.Count) 处"
  }
}
```

### 经验总结

**教训**：写 XML 注释时避免使用 `----` 分隔线。
这是一个**反复踩坑**的问题（在 `car.xacro` 和 `main.launch` 中各出现一次）。

**建议**：在项目中使用上述脚本做提交前检查。

---

## 10.6 问题汇总表

| # | 问题 | 根因 | 解决方案 | 影响 |
|---|---|---|---|---|
| 1 | `/cmd_vel` 无消息 | `feed()` 未调用 `publish()` | 在 `feed()` 中直接发布 | **致命**：车辆完全不动 |
| 2 | 模块导入失败 | `catkin_install_python` 生成 relay 脚本 | 改用 `install(PROGRAMS)` + 自定义复制 | **致命**：节点无法启动 |
| 3 | 车轮空转车辆不动 | 底盘离地间隙为零，车体触地 | 车体中心抬高至 `wheel_radius + h/2` | **致命**：车辆无法移动 |
| 4 | 深度读数恒为 1.0 | Gazebo 深度相机地板值（占比 66%） | 直方图峰值检测 + 剔除 | **致命**：相机模式失效 |
| 5 | xacro XML 解析失败 | 注释含连续 `--` | 改用 `=` 分隔线 | 阻塞：无法生成 URDF |
| 6 | 主循环卡死 | `rospy.Rate` 依赖未就绪的仿真时钟 | 改用墙钟时间控制循环 | **致命**：控制回路停止 |
| 7 | 万向轮拖拽 | `fixed` 关节球体与地面持续接触 | 移除碰撞几何，保留为视觉件 | 中等：转向迟滞 |
| 8 | 激光自遮挡 | 激光扫到车体自身 | 邻域连续性校验过滤 | 中等：误判前方有障碍 |

### 问题分类统计

```
致命级（导致功能完全失效）：5 个
  - /cmd_vel 无消息
  - 模块导入失败
  - 车轮空转
  - 深度读数异常
  - 主循环卡死

中等（影响性能）：2 个
  - 万向轮拖拽
  - 激光自遮挡

阻塞（无法继续开发）：1 个
  - XML 解析失败
```

### 排查方法论总结

| 阶段 | 方法 | 本项目应用 |
|---|---|---|
| **现象确认** | 用 `rostopic`/`rosnode` 确认系统状态 | 确认 `/cmd_vel` 无消息 |
| **数据检查** | 打印原始数据，不依赖二手统计 | 发现深度图 66% 是 1.005 |
| **代码审查** | 逐行检查关键路径 | 发现 `feed()` 缺 `publish()` |
| **对照验证** | 与文档/标准实现对照 | 查 catkin 文档发现 relay 机制 |
| **最小复现** | 单独测试可疑组件 | 单独运行 `gzserver` 验证正常 |
| **物理直觉** | 用物理常识判断异常 | 车轮转 168 rad 但车不动 → 打滑 |

**最重要的一条经验**：

> **不要相信"看起来正常"的数据。**
>
> 深度值 1.005 m 看起来完全正常，实际是无效标记；
> `/cmd_vel` 话题连接正常，实际从未发布；
> 车轮正常转动，实际车辆没动。
>
> **每一个"正常"的表象背后，都要用独立的数据去验证。**
