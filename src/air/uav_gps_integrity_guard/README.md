# uav_gps_integrity_guard

ROS 2 GPS integrity monitor that cross-checks position-derived speed, reported
GPS speed, IMU speed consistency and horizontal accuracy. It emits
`VALID / SUSPECT / REJECTED` plus a navigation recommendation.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_gps_integrity_guard
source install/setup.bash
ros2 launch uav_gps_integrity_guard main.launch.py duration:=18
```

Topics:

- `/uav_gps/integrity_sample`: simulated GPS and IMU speed inputs.
- `/uav_gps/integrity_assessment`: risk and integrity state.
- `/uav_gps/integrity_advice`: `USE_GPS`, `CROSS_CHECK`, or
  `REJECT_GPS_USE_DEADRECKONING`.

The module is an advisory monitor and does not directly switch navigation
sources or command the flight controller.
