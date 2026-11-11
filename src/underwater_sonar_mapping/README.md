# 水下声呐占据栅格

该 ROS 2 节点使用二维圆形障碍场景模拟多波束声呐，并把波束空闲区和最近回波融合到占据栅格。机器人位姿来自 `/odom`；未收到里程计时使用 `initial_pose`。模块同时发布可供 RViz 和其他节点读取的栅格地图与回波点云。

## 构建与运行

```bash
colcon build --packages-select underwater_sonar_mapping
source install/setup.bash
ros2 launch underwater_sonar_mapping main.launch.py
```

默认话题为 `/odom`、`/sonar/map` 和 `/sonar/echoes`。可通过 launch 参数设置 `odom_topic`、`map_width`、`map_height`、`resolution`、`beam_count`、`field_of_view`、`max_range` 和 `scan_rate`。

地图坐标从 `(0, 0)` 开始，宽高参数分别限制 `x`、`y` 的正向范围。障碍通过节点参数 `obstacles` 指定为扁平数组，每三个数依次表示圆心 `x`、`y` 和半径；`initial_pose` 使用 `x`、`y`、`yaw` 三元组。地图未知区域为 `-1`，可见空闲区域为 `0`，回波端点为 `100`。

独立验证纯算法：

```bash
python3 -m pytest test/test_sonar_core.py
```
