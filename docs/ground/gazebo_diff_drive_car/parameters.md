# 场景与模型参数

本章汇总所有可调参数，便于复现与调参。

---

## 11.1 场景参数

### 11.1.1 场地尺寸

| 项目 | 值 |
|---|---|
| 场地范围 | X: −6 ~ +6 m，Y: −6 ~ +6 m |
| 场地面积 | 12 × 12 = 144 m² |
| 围墙高度 | 1.0 m |
| 围墙厚度 | 0.2 m |
| 地面 | Gazebo 默认 `ground_plane` |

### 11.1.2 障碍物布局

| 名称 | 类型 | 位置 (x, y) | 尺寸 | 颜色 |
|---|---|---|---|---|
| `obstacle_1` | 圆柱 | (2.5, 2.0) | r=0.4, h=1.0 | 红 |
| `obstacle_2` | 圆柱 | (−2.5, 2.0) | r=0.4, h=1.0 | 红 |
| `obstacle_3` | 圆柱 | (0.0, −2.0) | r=0.4, h=1.0 | 红 |
| `obstacle_box_1` | 方块 | (3.5, −3.0) | 1×1×1 m，yaw=0.4 rad | 蓝 |
| `corridor_wall_a` | 墙 | (−4.0, 3.0) | 3.0×0.15×1.0 m | 灰 |
| `corridor_wall_b` | 墙 | (−4.0, 4.5) | 3.0×0.15×1.0 m | 灰 |
| `calibration_board` | 标定板 | (4.5, 4.5) | 2.0×0.1×1.0 m，yaw=0.785 | 绿 |
| `spawn_marker` | 出生标记 | (−5.0, −4.0) | 1×1×0.01 m | 绿（半透明） |

### 11.1.3 场景设计意图

| 元素 | 测试目的 |
|---|---|
| 圆柱障碍 | 测试对不同轮廓障碍的雷达回波与避障转向 |
| 方块障碍（带旋转） | 测试非对称障碍下的避障决策 |
| 走廊墙（两段平行） | 测试狭窄通道通行能力（宽度 1.5 m） |
| 标定板 | 深度相机测距准确性验证 |
| 出生标记 | 指示无碰撞的推荐初始位置 |

### 11.1.4 物理引擎参数

| 参数 | 值 | 说明 |
|---|---|---|
| 引擎类型 | ODE | Gazebo 默认 |
| 最大步长 | 0.001 s | 1 ms |
| 实时更新率 | 1000 Hz | 与步长对应 |
| 实时因子目标 | 1.0 | 尽量实时 |
| 重力 | −9.8 m/s² | 标准重力 |

---

## 11.2 车辆模型参数

### 11.2.1 几何尺寸

| 参数 | xacro 变量 | 值 | 说明 |
|---|---|---|---|
| 车长 | `chassis_length` | 0.44 m | — |
| 车宽 | `chassis_width` | 0.38 m | — |
| 车高 | `chassis_height` | 0.22 m | — |
| 车轮半径 | `wheel_radius` | 0.0975 m | — |
| 轮间距 | `wheel_separation` | 0.335 m | 左右轮中心距 |
| 轮宽 | `wheel_width` | 0.05 m | — |
| 万向轮半径 | `caster_radius` | 0.03 m | — |

### 11.2.2 派生尺寸

| 参数 | 计算式 | 值 |
|---|---|---|
| 车体中心高度 | `wheel_radius + chassis_height/2` | **0.2075 m** |
| 车轮 x 偏移 | `wheel_x` | 0.10 m |
| 车轮 y 偏移 | `wheel_separation/2` | 0.1675 m |
| 车轮 z | `wheel_radius` | 0.0975 m |
| 激光雷达高度 | `chassis_z + chassis_height/2 + 0.03` | 0.3475 m |
| 激光雷达 x | `chassis_length/2 + 0.06` | 0.28 m |
| 相机高度 | `chassis_z + chassis_height/2 − 0.01` | 0.3075 m |
| 相机 x | `chassis_length/2 + 0.02` | 0.24 m |

> **关键**：`chassis_z` 必须使车体底面高于地面，
> 否则会因地面摩擦导致车轮空转。详见
> [工程问题排查 § 10.3](troubleshooting.md#103-车轮空转车辆不动)。

### 11.2.3 质量与惯量

| 部件 | 质量 | 惯量来源 |
|---|---|---|
| `base_link`（车体） | 5.0 kg | 长方体惯量公式 |
| `left_wheel_link` | 0.3 kg | 圆柱惯量公式 |
| `right_wheel_link` | 0.3 kg | 圆柱惯量公式 |
| `front_caster_link` | 0.05 kg | 固定值 |
| `rear_caster_link` | 0.05 kg | 固定值 |
| `laser_link` | 0.16 kg | 固定值 |
| `camera_link` | 0.1 kg | 固定值 |
| **整车** | **≈ 5.96 kg** | — |

**惯量计算公式**：

长方体（质量 $m$，边长 $x, y, z$）：

$$
I_{xx} = \frac{m(y^2 + z^2)}{12}, \quad
I_{yy} = \frac{m(x^2 + z^2)}{12}, \quad
I_{zz} = \frac{m(x^2 + y^2)}{12}
$$

圆柱（质量 $m$，半径 $r$，长 $l$，轴向为 $z$）：

$$
I_{xx} = I_{yy} = \frac{m(3r^2 + l^2)}{12}, \quad
I_{zz} = \frac{m r^2}{2}
$$

### 11.2.4 摩擦与接触参数

| 部件 | $\mu_1$ | $\mu_2$ | $k_p$ | $k_d$ | 说明 |
|---|---|---|---|---|---|
| `base_link` | 0.05 | 0.05 | — | — | 低摩擦（避免车体拖拽） |
| `left_wheel_link` | 1.0 | 1.0 | 1e6 | 100 | 高摩擦（保证牵引力） |
| `right_wheel_link` | 1.0 | 1.0 | 1e6 | 100 | 同上 |

> **注意**：驱动轮摩擦系数必须足够高（建议 $\mu \geq 0.8$），
> 否则车轮会打滑，无法推动车辆。

### 11.2.5 关节参数

| 关节 | 类型 | 轴 | 阻尼 |
|---|---|---|---|
| `base_joint` | fixed | — | — |
| `left_wheel_joint` | continuous | (0, 1, 0) | 0.1 |
| `right_wheel_joint` | continuous | (0, 1, 0) | 0.1 |
| `front_caster_joint` | fixed | — | — |
| `rear_caster_joint` | fixed | — | — |
| `laser_joint` | fixed | — | — |
| `camera_joint` | fixed | — | — |
| `camera_optical_joint` | fixed | — | — |

---

## 11.3 传感器参数

### 11.3.1 激光雷达

| 参数 | 值 |
|---|---|
| 传感器类型 | `ray` |
| 光束数 | 360 |
| 角度范围 | −π ~ +π |
| 角分辨率 | 0.01745 rad（约 1°） |
| 最小量程 | 0.12 m |
| 最大量程 | 5.6 m |
| 距离分辨率 | 0.01 m |
| 更新频率 | 10 Hz |
| 噪声类型 | 高斯 |
| 噪声标准差 | 0.01 m |
| 可视化 | 开启 |
| ROS 话题 | `/scan` |
| frame_id | `laser_link` |

### 11.3.2 深度相机

| 参数 | 值 |
|---|---|
| 传感器类型 | `depth` |
| 分辨率 | 640 × 480 |
| 水平视场角 | 1.3962634 rad（80°） |
| 近裁剪面 | 0.05 m |
| 远裁剪面 | 30.0 m |
| 更新频率 | 15 Hz |
| 噪声类型 | 高斯 |
| 噪声标准差 | 0.007 m |
| 插件 | `libgazebo_ros_depth_camera.so` |
| ROS 话题 | `/camera/depth/image_raw`（32FC1） |
| | `/camera/rgb/image_raw` |
| | `/camera/depth/points` |
| frame_id | `camera_optical_link` |
| `hackBaseline` | 0.07 |

### 11.3.3 底盘控制器插件

| 参数 | 值 |
|---|---|
| 插件 | `libgazebo_ros_diff_drive.so` |
| 更新频率 | 50 Hz |
| 左轮关节 | `left_wheel_joint` |
| 右轮关节 | `right_wheel_joint` |
| 轮间距 | 0.335 m |
| 轮直径 | 0.195 m |
| 轮加速度 | 1.5 |
| 轮扭矩 | 20 N·m |
| 命令话题 | `cmd_vel` |
| 里程计话题 | `odom` |
| 里程计坐标系 | `odom` |
| 底盘坐标系 | `base_footprint` |
| 里程计来源 | 1（世界坐标系） |
| 发布里程计 TF | true |
| 发布轮 TF | false |

---

## 11.4 控制算法参数

### 11.4.1 激光雷达避障

| 参数 | 值 | 说明 |
|---|---|---|
| `D_SAFE` | 1.0 m | 进入避障阈值 |
| `D_CLEAR` | 1.4 m | 退出避障阈值（滞回） |
| `D_DANGER` | 0.45 m | 紧急后退阈值 |
| `V_CRUISE` | 0.35 m/s | 巡航线速度 |
| `V_AVOID` | 0.18 m/s | 避障最大线速度 |
| `V_BACK` | −0.12 m/s | 紧急后退速度 |
| `W_MIN` | 0.45 rad/s | 最小角速度 |
| `W_MAX` | 1.30 rad/s | 最大角速度 |
| `K_OMEGA` | 1.8 | 转向增益 |
| `EMERGENCY_HOLD` | 0.8 s | 紧急状态保持时长 |
| `SELF_OCCLUSION_RANGE` | 0.30 m | 自遮挡判定距离 |
| `MIN_CLOSE_HITS` | 5 | 真实障碍最少光束数 |

### 11.4.2 深度相机避障

| 参数 | 值 | 说明 |
|---|---|---|
| `D_SAFE` | 1.2 m | 中区避障阈值 |
| `D_CLEAR` | 1.6 m | 退出阈值 |
| `D_DANGER` | 0.5 m | 紧急阈值 |
| `V_CRUISE` | 0.30 m/s | 巡航线速度 |
| `V_AVOID` | 0.15 m/s | 避障最大线速度 |
| `V_BACK` | −0.10 m/s | 紧急后退速度 |
| `W_MIN` | 0.40 rad/s | 最小角速度 |
| `W_MAX` | 1.20 rad/s | 最大角速度 |
| `K_OMEGA` | 1.6 | 转向增益 |
| `DEPTH_PERCENTILE` | 25.0 | 深度统计百分位 |
| `FLOOR_VALUE_RATIO` | 0.30 | 地板值判定占比阈值 |
| `FLOOR_BIN_WIDTH` | 0.01 m | 直方图分箱宽度 |
| `MAX_SPIN_DURATION` | 1.5 s | 原地点转超时 |
| `EMERGENCY_HOLD` | 0.8 s | 紧急保持时长 |
| `ROI_LEFT` | [0.00, 0.25] | 左区列范围 |
| `ROI_MID` | [0.37, 0.62] | 中区列范围 |
| `ROI_RIGHT` | [0.75, 1.00] | 右区列范围 |
| `ROI_ROWS` | [0.25, 0.75] | 竖直取样范围 |

### 11.4.3 参数差异对照

| 参数 | 激光 | 相机 | 差异原因 |
|---|---|---|---|
| 安全距离 | 1.0 m | 1.2 m | 相机视场窄，需更大余量 |
| 巡航线速度 | 0.35 m/s | 0.30 m/s | 相机更保守 |
| 避障线速度 | 0.18 m/s | 0.15 m/s | 同上 |
| 最大角速度 | 1.30 rad/s | 1.20 rad/s | 同上 |
| 转向增益 | 1.8 | 1.6 | 相机侧向信息少，降低增益 |

### 11.4.4 键盘遥控

| 参数 | 值 |
|---|---|
| `LINEAR_STEP` | 0.1 m/s |
| `ANGULAR_STEP` | 0.2 rad/s |
| `LINEAR_MAX` | 0.8 m/s |
| `ANGULAR_MAX` | 1.2 rad/s |

### 11.4.5 系统参数

| 参数 | 值 | 说明 |
|---|---|---|
| `CONTROL_RATE` | 20.0 Hz | 控制回路频率 |
| `COMMAND_TIMEOUT` | 1.0 s | 看门狗超时 |

---

## 11.5 轨迹记录参数

| 参数 | 值 | 说明 |
|---|---|---|
| `MIN_SAMPLE_DISTANCE` | 0.02 m | 轨迹抽稀阈值 |
| `MAX_MARKER_POINTS` | 2000 | RViz 可视化最大点数 |
| `RESULT_DIR` | `~/openhutb_car_results` | 结果输出目录 |

---

## 11.6 Launch 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `mode` | `teleop` | 运行模式 |
| `gui` | `true` | Gazebo 图形界面 |
| `headless` | `false` | 无头模式 |
| `paused` | `false` | 暂停启动 |
| `use_sim_time` | `true` | 使用仿真时间 |
| `rviz` | `false` | 启动 RViz |
| `x` | `-5.0` | 初始 X 坐标 |
| `y` | `-4.0` | 初始 Y 坐标 |
| `yaw` | `0.0` | 初始偏航角 |
| `record` | `true` | 记录轨迹 |
| `world_file` | `worlds/car_world.world` | 场景文件 |
| `urdf_file` | `urdf/car.xacro` | 模型文件 |
| `debug` | `false` | Gazebo 调试 |
| `verbose` | `false` | Gazebo 详细输出 |

---

## 11.7 话题与消息类型

### 11.7.1 订阅的话题

| 话题 | 类型 | 使用节点 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `laser_avoid.py` |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | `depth_camera_avoid.py` |
| `/odom` | `nav_msgs/Odometry` | `odom_recorder.py` |
| `/joint_states` | `sensor_msgs/JointState` | `robot_state_publisher` |
| `/clock` | `rosgraph_msgs/Clock` | 全部（sim time） |

### 11.7.2 发布的话题

| 话题 | 类型 | 发布节点 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `main.py`（经看门狗） |
| `/odom_recorder/trail` | `visualization_msgs/Marker` | `odom_recorder.py` |
| `/tf` | `tf2_msgs/TFMessage` | `robot_state_publisher`, diff_drive |

### 11.7.3 提供的服务

| 服务 | 类型 | 提供者 |
|---|---|---|
| `/gazebo/spawn_urdf_model` | `gazebo_msgs/SpawnModel` | `gazebo_ros` |
| `/gazebo/get_physics_properties` | `gazebo_msgs/GetPhysicsProperties` | `gazebo_ros` |
| `/gazebo/set_model_state` | `gazebo_msgs/SetModelState` | `gazebo_ros` |

---

## 11.8 实测性能参考

### 11.8.1 话题频率

| 话题 | 期望频率 | 实测频率 |
|---|---|---|
| `/scan` | 10 Hz | 9.98 ~ 10.02 Hz |
| `/cmd_vel` | 20 Hz | **19.9 ~ 20.3 Hz** |
| `/camera/depth/image_raw` | 15 Hz | 12.7 ~ 14.1 Hz |
| `/odom` | 50 Hz | 48.3 Hz |
| `/clock` | 1000 Hz | 998 Hz |

### 11.8.2 运行性能

| 配置 | 实时因子 | FPS |
|---|---|---|
| `gui:=false` | 0.98 ~ 1.00 | — |
| `gui:=true` | 0.58 ~ 0.85 | 39 ~ 61 |

### 11.8.3 三种模式实测

| 模式 | 路程 (m) | 净位移 (m) | 路径效率 | 均速 (m/s) |
|---|---|---|---|---|
| teleop | 6.34 | 6.31 | 0.997 | 0.108 |
| laser | 18.22 | 9.07 | 0.498 | 0.319 |
| camera | 12.82 | 9.73 | 0.758 | 0.219 |

---

## 11.9 参数调优建议

### 11.9.1 调参优先级

```
① 几何参数（车体高度、轮半径、轮距）
      └─ 错了会导致车辆无法运动，必须首先验证
              ↓
② 摩擦参数（驱动轮 μ、车体 μ）
      └─ 影响牵引力与运动平滑度
              ↓
③ 安全阈值（D_SAFE / D_DANGER）
      └─ 决定避障时机
              ↓
④ 滞回阈值（D_CLEAR = D_SAFE × 1.4）
      └─ 抑制抖动
              ↓
⑤ 控制增益（K_OMEGA / W_MIN / W_MAX）
      └─ 决定转向响应
              ↓
⑥ 速度参数（V_CRUISE / V_AVOID / V_BACK）
            决定行驶效率
```

### 11.9.2 常见调参场景

| 目标 | 调整 |
|---|---|
| 车辆容易撞障碍 | ↑ `D_SAFE`，↓ `V_CRUISE` |
| 车辆过于保守 | ↓ `D_SAFE`，↑ `V_CRUISE` |
| 转向抖动 | ↑ `D_CLEAR`（增大滞回带） |
| 转向迟钝 | ↑ `K_OMEGA`，↑ `W_MAX` |
| 贴着障碍不转开 | ↑ `W_MIN` |
| 前冲-后退振荡 | ↑ `EMERGENCY_HOLD` |
| 原地打转不停 | ↓ `MAX_SPIN_DURATION` |
| 车轮打滑 | ↑ 驱动轮 `mu1`/`mu2` |
| 车体被地面拖住 | ↑ `chassis_z` |

### 11.9.3 约束检查清单

调参后请确认以下约束：

- [ ] `D_CLEAR > D_SAFE > D_DANGER`
- [ ] `V_CRUISE > V_AVOID > 0 > V_BACK`
- [ ] `W_MAX > W_MIN > 0`
- [ ] `chassis_z > wheel_radius`（车体离地）
- [ ] 驱动轮 `mu1 = mu2 >= 0.8`
- [ ] 传感器安装位置在车体轮廓之外
- [ ] `wheel_separation`（URDF）== `wheelSeparation`（插件）
- [ ] `wheelDiameter`（插件）== `2 × wheel_radius`（URDF）

---

## 11.10 配置文件

本项目提供 `config/params.yaml` 作为参数参考文件，
集中记录所有可调参数，便于查阅与版本对比。

```yaml
laser_avoid:
  d_safe: 1.0
  d_clear: 1.4
  d_danger: 0.45
  v_cruise: 0.35
  ...

depth_camera_avoid:
  d_safe: 1.2
  d_clear: 1.6
  depth_percentile: 25.0
  ...
```

> **说明**：当前实现中参数以内联常量形式写在各脚本顶部，
> `params.yaml` 作为**文档与参考**提供。
> 若需运行时动态调参，可将这些参数改由 ROS Parameter Server 读取。
