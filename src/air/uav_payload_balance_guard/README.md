# uav_payload_balance_guard

ROS 2 payload center-of-gravity and trim monitor. It estimates lateral CG shift
from left/right thrust imbalance and combines roll correction, sway rate and
payload utilization into `BALANCED / ADJUST_TRIM / OFFSET_WARNING / UNSTABLE`.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_payload_balance_guard
source install/setup.bash
ros2 launch uav_payload_balance_guard main.launch.py duration:=20
```

Topics:

- `/uav_payload/balance_sample`: simulated dual-side thrust and attitude data.
- `/uav_payload/balance_assessment`: estimated CG offset and balance risk.
- `/uav_payload/balance_advice`: monitor, rebalance or abort recommendation.

The module does not redistribute payload or directly control the aircraft.
