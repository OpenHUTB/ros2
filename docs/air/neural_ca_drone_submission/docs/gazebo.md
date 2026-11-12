# 接入 Gazebo / 真实无人机

本项目默认自带 **CA-Sim** 仿真器：纯 NumPy + Pillow，CPU 上即可复现，
不依赖 GPU、Gazebo 或 AirSim。这样做是为了让“`git clone` 后一条命令就能跑”
成为硬约束，而不是可选项。

但算法层（`ca_drone/`）与仿真器是解耦的，接口只有两句话：

```
订阅 /camera/depth/image_raw (32FC1)  +  /odom  +  /move_base_simple/goal
发布 /cmd_vel
```

因此换成 Gazebo 只需要替换“发布深度、订阅速度”的桥接节点，
`main_node.py` / `main.py` 中的推理代码一行都不用改。

## 1. Gazebo 侧需要的组件

| 组件 | 推荐 | 说明 |
| --- | --- | --- |
| 世界 | `gazebo_ros` + 自定义 `.world` | 放入与 `ca_drone/sim/world.py` 相同的墙体布局 |
| 机体 | `iris` 或自建 xacro | 带 `libgazebo_ros_camera.so` 深度相机 |
| 控制 | `gazebo_ros` 的 `cmd_vel` 插件 | 把 `/cmd_vel` 转成机体速度 |
| 定位 | `gazebo_ros_p3d` 或 `mavros` | 发布 `/odom` |

## 2. 深度相机参数

```xml
<sensor type="depth" name="depth_cam">
  <update_rate>10</update_rate>
  <camera>
    <horizontal_fov>1.5708</horizontal_fov>
    <image>
      <width>64</width>
      <height>48</height>
      <format>R_FLOAT32</format>
    </image>
    <clip><near>0.2</near><far>20.0</far></clip>
  </camera>
  <plugin name="depth" filename="libgazebo_ros_camera.so">
    <ros><remapping>image_raw:=/camera/depth/image_raw</remapping></ros>
  </plugin>
</sensor>
```

!!! note "分辨率"
    Gazebo 默认发布 640×480；本项目的 `decode_depth(msg, 64, 48, 12.0)`
    假设 64×48。若用默认分辨率，请在 `main.py` 中改为 640×480，
    并把 `ca_drone/mapping.py` 的 `fov` 与 `MAX_RANGE` 对齐。

## 3. 启动顺序

```bash
# 1) Gazebo 世界与机体
roslaunch your_drone_gazebo spawn.launch world:=ca_corridor.world

# 2) 本项目推理节点（不要再启动 sim_bridge）
roslaunch neural_ca_drone_ros main.launch module:=full
```

`main.launch` 默认会启动 `sim_bridge`，接入 Gazebo 时用
`roslaunch neural_ca_drone_ros full.launch`（只启动推理节点）。

## 4. 为什么没有默认提供 Gazebo 世界

- 课程环境里的 Gazebo 11 在无 GPU 的虚拟机上需要 `LIBGL_ALWAYS_SOFTWARE=1`，
  帧率低于 1 Hz，无法用于 10 Hz 闭环；
- Gazebo 的深度图有噪点与缺失，感知网络需要重新标定，否则指标不可比；
- CA-Sim 的射线投射是解析的，深度图无噪声，便于把“算法误差”与
  “传感器噪声”分开分析（感知训练时我们主动加入了噪声与丢点）。

如果课程要求必须在 Gazebo 中演示，按上面的接口替换桥接节点即可；
本仓库的 ROS 包、话题、launch 命名都已按 Gazebo 的习惯设计。
