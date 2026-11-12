# Keyboard Teleoperation Safety

`teleop_control` publishes keyboard velocity commands on `/cmd_vel`. W/S move forward and backward, A/D turn, Space publishes a stop command, and Q exits.

The node automatically publishes zero velocity when no motion key is received within `command_timeout_sec`. It also publishes zero velocity during normal exit and Ctrl-C handling. This watchdog is checked by a short input poll loop, so it continues to run while the keyboard is idle.

The launch file accepts these ROS parameters:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `linear_speed` | `0.5` | Forward and reverse speed in m/s |
| `angular_speed` | `0.5` | Turn speed in rad/s |
| `command_timeout_sec` | `0.5` | Maximum age of a motion command |
| `poll_period_sec` | `0.05` | Keyboard and ROS event polling interval |

Run with an interactive terminal:

```bash
ros2 launch teleop_control main.launch.py
```

Run the pure command-state tests with:

```bash
python3 -m pytest src/robot_tasks/teleop_control/test/test_teleop_logic.py
```
