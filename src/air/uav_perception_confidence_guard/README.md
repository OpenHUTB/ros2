# uav_perception_confidence_guard

ROS 2 runtime perception-health monitor. It combines expected-target detection,
mean confidence, predictive entropy, inference latency, frame freshness and
illumination, then emits `NOMINAL / DEGRADED / UNRELIABLE / BLIND`.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_perception_confidence_guard
source install/setup.bash
ros2 launch uav_perception_confidence_guard main.launch.py duration:=20
```

Topics:

- `/uav_perception/confidence_sample`: simulated detector and frame statistics.
- `/uav_perception/confidence_assessment`: confidence, latency and health risk.
- `/uav_perception/confidence_advice`: margin, hold/return or motion-stop advice.

The monitor does not replace the detector and does not directly command the UAV.
