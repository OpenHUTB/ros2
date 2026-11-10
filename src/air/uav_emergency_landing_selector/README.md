# uav_emergency_landing_selector

Constraint-aware emergency landing site selection for UAVs. The module filters
sites using slope, roughness, obstacle/person risk, reachable distance and
energy reserve, then ranks the remaining sites with an explainable cost.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select uav_emergency_landing_selector
source install/setup.bash
ros2 launch uav_emergency_landing_selector main.launch.py duration:=15
```

Topics:

- `/uav_landing/sites`: scenario input with UAV state and candidate sites.
- `/uav_landing/selection`: selected point, score and per-site rejection reasons.
- `/uav_landing/status`: compact runtime status.

Offline demo:

```bash
cd src/air/uav_emergency_landing_selector
python3 -m uav_emergency_landing_selector.headless --output-dir /tmp/landing_demo
```

This module does not command a real aircraft. It evaluates published candidate
sites and provides a deterministic recommendation for integration with a flight
manager or emergency supervisor.
