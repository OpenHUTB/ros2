# Power-line mission preflight

This ROS 2 Python package checks a power-line inspection mission JSON before it reaches a simulator or flight stack. It reports blocking errors, non-blocking warnings, route distance, and the longest leg without modifying the source mission.

```bash
colcon build --packages-select powerline_mission_preflight
source install/setup.bash
ros2 run powerline_mission_preflight check \
  src/air/powerline_mission_preflight/config/demo_mission.json
```

Exit code `0` means the mission has no blocking errors. Exit code `2` means the report contains errors.

```bash
python3 -m pytest src/air/powerline_mission_preflight/test -q
```
