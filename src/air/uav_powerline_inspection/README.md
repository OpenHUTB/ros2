# UAV power-line inspection route preview

This ROS 2 Python package generates two camera-facing viewpoints for every utility pole. Viewpoints alternate traversal direction to reduce unnecessary repositioning, and the JSON report includes yaw, altitude, route distance, and ideal travel time.

The preview is simulator-independent and does not connect to or arm a vehicle.

```bash
colcon build --packages-select uav_powerline_inspection
source install/setup.bash
ros2 run uav_powerline_inspection preview --pole-count 6 --pole-spacing 45 --bearing-deg 20
```

`--pole-spacing` must be a positive finite distance in metres. Zero, negative,
`NaN`, and infinite values are rejected before a route file is written.

Run the focused tests with:

```bash
python3 -m pytest src/air/uav_powerline_inspection/test -q
```

The focused suite contains 14 planner and CLI checks. The dedicated
`UAV Power-line Inspection Tests` workflow runs these tests, style checks, and
package-data verification whenever this module changes.
