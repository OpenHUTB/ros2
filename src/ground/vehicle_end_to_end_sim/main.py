#!/usr/bin/env python3
"""课程约定入口（薄壳）：转发到包内实现 vehicle_end_to_end_sim.main。

colcon build 只安装 `vehicle_end_to_end_sim/` 目录，包根这份仅做转发，
避免 `ros2 launch` 出现 ModuleNotFoundError。
"""

from vehicle_end_to_end_sim.main import main

if __name__ == "__main__":
    main()
