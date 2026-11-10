# uav_compass_interference_guard

无人机磁罗盘外部干扰与航向漂移监测模块。

## 功能

融合以下信号判断磁罗盘是否受外部干扰：

- 磁强计三轴合模强度偏离当地地磁场标称值；
- 磁航向与 GPS 对地航迹向的差值（仅当对地速度 > 3 m/s 时 GPS 航向可信）；
- 磁航向与 VIO/视觉里程计航向的差值；
- 电机总电流相对滑动均值的突增（间接反映 EMI）；
- 低高度（< 15 m）下近铁磁建筑、车辆、钢筋结构的加权风险。

输出状态：`CALM` / `SUSPECTED` / `BIASED` / `COMPASS_LOST`，带迟滞滤波。

## 构建与运行

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_compass_interference_guard
source install/setup.bash
ros2 launch uav_compass_interference_guard main.launch.py duration:=20
```

CSV 默认写入 `/tmp/uav_compass_runtime.csv`。

## 话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/uav_compass/sample` | `std_msgs/String` | 输入 | 磁航向/GPS 航向/VIO 航向/磁模长/电流/速度/高度 |
| `/uav_compass/assessment` | `std_msgs/String` | 输出 | 风险分、各项偏差和状态 |
| `/uav_compass/advice` | `std_msgs/String` | 输出 | 继续 / 监视 / 融合 GPS+VIO / 弃用磁罗盘 |

## 作者

2535010018
