# uav_control_authority_guard

ROS 2 monitor for reduced control authority, motor saturation and integrator
windup. It compares commanded and achieved roll/pitch rates with actuator
saturation, integrator magnitude and minimum cell voltage.

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
colcon build --symlink-install --packages-select uav_control_authority_guard
source install/setup.bash
ros2 launch uav_control_authority_guard main.launch.py duration:=18
```

`duration` is the simulator run time in seconds. With a positive value, the
simulator exits when that duration is reached and the launch file shuts down
both nodes automatically. With the default `duration:=0.0`, stop the
continuously running demo with `Ctrl+C`.

Run the eight unit tests from the package source directory:

```bash
PYTHONPATH=. python3 -m unittest discover -s tests -v
```

The CSV defaults to `/tmp/uav_control_authority_runtime.csv`.

## ROS 2 topics

- `/uav_control/authority_sample`: simulated command, response and actuator data.
- `/uav_control/authority_assessment`: authority, saturation and windup risk.
- `/uav_control/authority_advice`: maneuver limit or stabilization recommendation.

The input topic accepts JSON with `timestamp_s`,
`commanded_roll_rate_deg_s`, `actual_roll_rate_deg_s`,
`commanded_pitch_rate_deg_s`, `actual_pitch_rate_deg_s`,
`motor_saturation_ratio`, `integrator_ratio`, and `min_cell_voltage_v`.

The CSV columns are `time_s`, `state`, `risk`, `tracking_error_ratio`,
`authority_ratio`, `saturation_ratio`, `integrator_ratio`, and
`recommendation`.

The package is advisory only and does not send arming, attitude or motor commands.
