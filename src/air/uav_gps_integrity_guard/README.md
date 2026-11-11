# uav_gps_integrity_guard

ROS 2 GPS integrity monitor that cross-checks position-derived speed, reported
GPS speed, IMU speed consistency and horizontal accuracy. It emits
`VALID / SUSPECT / REJECTED` plus a navigation recommendation.

## Reproducible environment

- Ubuntu 22.04.5 LTS
- ROS 2 Humble
- Python 3.10.12

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_gps_integrity_guard
source install/setup.bash
ros2 launch uav_gps_integrity_guard main.launch.py duration:=15
```

With a positive `duration`, the simulator exits after that duration and the
launch file shuts down both nodes automatically. With the default
`duration:=0.0`, stop the continuously running demo with `Ctrl+C`.

Run the tests from the package source directory:

```bash
PYTHONPATH=. python3 -m unittest discover -s tests -v
```

## ROS 2 topics

- `/uav_gps/integrity_sample`: simulated GPS and IMU speed inputs.
- `/uav_gps/integrity_assessment`: risk and integrity state.
- `/uav_gps/integrity_advice`: `USE_GPS`, `CROSS_CHECK`, or
  `REJECT_GPS_USE_DEADRECKONING`.

`/uav_gps/integrity_sample` accepts strict JSON with exactly these fields:

```json
{
  "timestamp_s": 1.0,
  "x_m": 3.0,
  "y_m": 0.0,
  "z_m": 25.0,
  "reported_speed_mps": 3.0,
  "horizontal_accuracy_m": 0.9,
  "imu_speed_mps": 3.0
}
```

`/uav_gps/integrity_assessment` emits JSON with these fields:

- `state`: `VALID`, `SUSPECT`, or `REJECTED`.
- `risk`: bounded risk score in `[0, 1]`.
- `position_speed_mps`: position-derived speed.
- `speed_residual_mps`: difference between position-derived and reported speed.
- `imu_residual_mps`: difference between reported and IMU speed.
- `accuracy_m`: reported horizontal accuracy.
- `recommendation`: `USE_GPS`, `CROSS_CHECK`, or
  `REJECT_GPS_USE_DEADRECKONING`.

`/uav_gps/integrity_advice` emits `state`, `recommendation`, and `risk`.

The CSV defaults to `/tmp/uav_gps_integrity_runtime.csv` and contains:

```text
time_s,state,risk,position_speed_mps,speed_residual_mps,imu_residual_mps,accuracy_m,recommendation
```

The module is an advisory monitor and does not directly switch navigation
sources or command the flight controller.
