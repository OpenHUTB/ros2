# uav_perception_confidence_guard

ROS 2 runtime perception-health monitor. It combines expected-target detection,
mean confidence, predictive entropy, inference latency, frame freshness and
illumination, then emits `NOMINAL / DEGRADED / UNRELIABLE / BLIND`.

## Reproducible environment

- Ubuntu 22.04.5 LTS
- ROS 2 Humble
- Python 3.10.12

Install declared ROS dependencies from the workspace root:

```bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_perception_confidence_guard
source install/setup.bash
ros2 launch uav_perception_confidence_guard main.launch.py duration:=20
```

`duration` is the simulator run time in seconds. With a positive value, the
simulator exits when that duration is reached and the launch file shuts down
both nodes automatically. With the default `duration:=0.0`, stop the
continuously running demo with `Ctrl+C`.

Run the ten unit tests from the package source directory:

```bash
PYTHONPATH=. python3 -m unittest discover -s tests -v
```

The CSV defaults to `/tmp/uav_perception_confidence_runtime.csv`.

## ROS 2 topics

- `/uav_perception/confidence_sample`: simulated detector and frame statistics.
- `/uav_perception/confidence_assessment`: confidence, latency and health risk.
- `/uav_perception/confidence_advice`: margin, hold/return or motion-stop advice.

The input topic accepts JSON with `timestamp_s`, `frame_id`,
`target_expected`, `detection_count`, `mean_confidence`,
`predictive_entropy`, `latency_ms`, `frame_age_ms`, and `illumination_lux`.

The CSV columns are `time_s`, `state`, `risk`, `mean_confidence`,
`predictive_entropy`, `latency_ms`, `frame_age_ms`, and `recommendation`.

For a missing expected target, the formula contributes
`0.05 * 0.55 = 0.0275`; the state still transitions immediately to `BLIND`.

The monitor does not replace the detector and does not directly command the UAV.
