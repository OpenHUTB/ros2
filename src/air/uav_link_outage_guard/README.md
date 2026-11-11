# uav_link_outage_guard

UAV communication-link degradation and outage protection. The guard combines
RSSI, latency, packet loss, telemetry age and RSSI trend, then emits a stable
`HEALTHY -> DEGRADED -> HOLD -> RETURN -> LINK_LOST` decision with hysteresis.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_link_outage_guard
source install/setup.bash
ros2 launch uav_link_outage_guard main.launch.py duration:=18
```

Topics:

- `/uav_link/quality`: simulated raw radio measurements.
- `/uav_link/assessment`: risk, EWMA values, trend and predicted time to horizon.
- `/uav_link/advice`: mission-level recommendation.

The package does not command a real aircraft. `LINK_LOST` is an assessment state;
integration with a flight controller must provide a separately validated failsafe.
