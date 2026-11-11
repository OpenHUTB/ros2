# uav_gimbal_reacquisition

低空无人机机载云台目标丢失、搜索重捕获与视轴误差评估演示。

```text
目标轨迹/遮挡
      ↓
云台相机投影（FOV、像素误差、LOS）
      ↓
SEARCH → TRACK → LOST → 搜索 → TRACK
      ↓
重捕获时间、跟踪率、LOS RMS、像素质误差
```

环境：Ubuntu 22.04、ROS 2 Humble、Python 3.10。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_gimbal_reacquisition
source install/setup.bash
ros2 launch uav_gimbal_reacquisition main.launch.py
```

查看：

```bash
ros2 topic list | grep uav_gimbal
ros2 topic echo /uav_gimbal/status
ros2 topic echo /uav_gimbal/observation
```

RViz2 添加 `MarkerArray`，订阅 `/uav_gimbal/markers`，Fixed Frame 设为 `map`。节点只在仿真环境中运行，不控制真实云台。

离线复现：

```bash
cd src/air/uav_gimbal_reacquisition
python3 main.py --duration 50 --output-dir artifacts/demo
```

详细说明见 `docs/air/uav_gimbal_reacquisition.md`。
