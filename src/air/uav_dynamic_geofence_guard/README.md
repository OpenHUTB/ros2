# uav_dynamic_geofence_guard

Predictive warning for moving temporary no-fly zones. The module assumes
constant UAV and zone velocity over a short horizon, finds the predicted
closest approach, and emits `CLEAR / WARNING / CRITICAL / BREACH` with a
recommended avoidance direction.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_dynamic_geofence_guard
source install/setup.bash
ros2 launch uav_dynamic_geofence_guard main.launch.py duration:=18
```

Topics:

- `/uav_geofence/scene`: UAV motion and active moving zones.
- `/uav_geofence/assessment`: per-zone clearance and closest-approach data.
- `/uav_geofence/advice`: limiting zone and unit avoidance direction.

This is an advisory safety layer. It does not replace a certified geofence or
flight-controller failsafe.
