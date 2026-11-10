# uav_control_authority_guard

ROS 2 monitor for reduced control authority, motor saturation and integrator
windup. It compares commanded and achieved roll/pitch rates with actuator
saturation, integrator magnitude and minimum cell voltage.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_control_authority_guard
source install/setup.bash
ros2 launch uav_control_authority_guard main.launch.py duration:=18
```

Topics:

- `/uav_control/authority_sample`: simulated command, response and actuator data.
- `/uav_control/authority_assessment`: authority, saturation and windup risk.
- `/uav_control/authority_advice`: maneuver limit or stabilization recommendation.

The package is advisory only and does not send arming, attitude or motor commands.
