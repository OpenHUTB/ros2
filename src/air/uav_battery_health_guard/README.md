# uav_battery_health_guard

ROS 2 battery-cell imbalance and load-capability monitor. It combines cell
voltage spread, minimum cell voltage, temperature, internal resistance and
discharge current, then emits `HEALTHY / DEGRADED / LIMIT_LOAD / LAND_NOW`.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_battery_health_guard
source install/setup.bash
ros2 launch uav_battery_health_guard main.launch.py duration:=20
```

Topics:

- `/uav_battery/health_sample`: simulated cell voltages and load data.
- `/uav_battery/health_assessment`: imbalance, thermal and resistance risk.
- `/uav_battery/health_advice`: mission-level load and landing recommendation.

The module is advisory and does not control battery relays or aircraft power.
